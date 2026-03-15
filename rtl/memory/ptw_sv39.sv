`timescale 1ns/1ps
`default_nettype none

/**
 * @module ptw_sv39
 * @brief Page Table Walker para Sv39
 *
 * CORREÇÕES APLICADAS:
 * 1. Implementação de writeback dos bits A (Accessed) e D (Dirty) na PTE.
 *    Sem isso, o Linux gera page faults infinitos durante o boot ao tentar
 *    acessar páginas cujas PTEs não têm o bit A setado.
 *
 *    Fluxo adicionado:
 *      S_CHECK_PTE -> S_UPDATE_PTE (se A ou D precisam ser setados)
 *      S_UPDATE_PTE -> S_WAIT_UPDATE -> S_RESP
 *
 * 2. Endereço da PTE preservado em pte_addr_reg para o writeback.
 * 3. pte_reg atualizado com bits A/D antes de ser inserido na TLB,
 *    para que a TLB receba os bits corretos.
 */
module ptw_sv39 #(
    parameter int XLEN       = 64,
    parameter int PADDR_WIDTH = 56,
    parameter int VPN_WIDTH   = 27,
    parameter int PPN_WIDTH   = 44,
    parameter int ASID_WIDTH  = 16
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     ptw_req_valid,
    input  wire [VPN_WIDTH-1:0]     ptw_req_vpn,
    input  wire [ASID_WIDTH-1:0]    ptw_req_asid,
    input  wire                     ptw_req_is_store,
    input  wire                     ptw_req_is_exec,
    input  wire                     ptw_req_for_itlb,
    output logic                    ptw_req_ready,

    input  wire [XLEN-1:0]          satp,

    output logic                    ptw_mem_req,
    output logic [PADDR_WIDTH-1:0]  ptw_mem_addr,
    input  wire                     ptw_mem_resp_valid,
    input  wire [63:0]              ptw_mem_resp_data,
    input  wire                     ptw_mem_resp_err,

    // FIX: adicionados sinais de escrita para A/D writeback
    output logic                    ptw_mem_we,
    output logic [63:0]             ptw_mem_wdata,

    output logic                    ptw_resp_valid,
    output logic [VPN_WIDTH-1:0]    ptw_resp_vpn,
    output logic [PPN_WIDTH-1:0]    ptw_resp_ppn,
    output logic [ASID_WIDTH-1:0]   ptw_resp_asid,
    output logic [1:0]              ptw_resp_page_size,
    output logic                    ptw_resp_page_fault,
    output logic                    ptw_resp_access_fault,
    output logic                    ptw_resp_for_itlb,

    output logic                    ptw_resp_r,
    output logic                    ptw_resp_w,
    output logic                    ptw_resp_x,
    output logic                    ptw_resp_u,
    output logic                    ptw_resp_g,
    output logic                    ptw_resp_a,
    output logic                    ptw_resp_d
);

    // =========================================================================
    // Constantes
    // =========================================================================
    localparam int PAGE_OFFSET_BITS    = 12;
    localparam int PTE_SIZE_BITS       = 3;
    localparam int VPN_BITS_PER_LEVEL  = 9;

    localparam int SATP_MODE_SV39 = 8;

    // Bit positions na PTE
    localparam int PTE_V = 0;
    localparam int PTE_R = 1;
    localparam int PTE_W = 2;
    localparam int PTE_X = 3;
    localparam int PTE_U = 4;
    localparam int PTE_G = 5;
    localparam int PTE_A = 6;
    localparam int PTE_D = 7;

    // =========================================================================
    // FSM States — adicionados S_UPDATE_PTE e S_WAIT_UPDATE
    // 9 estados no total: necessita logic [3:0] (3 bits só comporta 8)
    // =========================================================================
    typedef enum logic [3:0] {
        S_IDLE        = 4'd0,
        S_LEVEL2      = 4'd1,
        S_LEVEL1      = 4'd2,
        S_LEVEL0      = 4'd3,
        S_WAIT_MEM    = 4'd4,
        S_CHECK_PTE   = 4'd5,
        S_UPDATE_PTE  = 4'd6,   // writeback A/D
        S_WAIT_UPDATE = 4'd7,   // aguardar ack do writeback
        S_RESP        = 4'd8
    } state_t;

    state_t state, next_state;

    // =========================================================================
    // Registradores
    // =========================================================================
    logic [VPN_WIDTH-1:0]    vpn_reg;
    logic [ASID_WIDTH-1:0]   asid_reg;
    logic                    is_store_reg;
    logic                    is_exec_reg;
    logic                    for_itlb_reg;
    logic [1:0]              current_level;
    logic [63:0]             pte_reg;
    logic [PPN_WIDTH-1:0]    current_ppn;
    logic                    page_fault;
    logic                    access_fault;

    // FIX: preservar endereço da PTE para writeback
    logic [PADDR_WIDTH-1:0]  pte_addr_reg;

    // =========================================================================
    // SATP Parsing
    // =========================================================================
    wire [3:0]  satp_mode      = satp[63:60];
    wire [15:0] satp_asid_field = satp[59:44];
    wire [43:0] satp_ppn        = satp[43:0];

    wire [8:0] vpn2 = vpn_reg[VPN_WIDTH-1:18];
    wire [8:0] vpn1 = vpn_reg[17:9];
    wire [8:0] vpn0 = vpn_reg[8:0];

    // =========================================================================
    // PTE Parsing
    // =========================================================================
    wire        pte_v   = pte_reg[PTE_V];
    wire        pte_r   = pte_reg[PTE_R];
    wire        pte_w   = pte_reg[PTE_W];
    wire        pte_x   = pte_reg[PTE_X];
    wire        pte_u   = pte_reg[PTE_U];
    wire        pte_g   = pte_reg[PTE_G];
    wire        pte_a   = pte_reg[PTE_A];
    wire        pte_d   = pte_reg[PTE_D];
    wire [43:0] pte_ppn = pte_reg[53:10];

    wire pte_is_pointer = !pte_r && !pte_x;
    wire pte_is_leaf    = pte_r || pte_x;

    wire superpage_misaligned = (current_level == 2 && pte_ppn[17:0] != '0) ||
                                (current_level == 1 && pte_ppn[8:0]  != '0);

    // FIX: determinar se A/D precisam ser escritos
    wire need_set_a  = pte_is_leaf && pte_v && !pte_a;
    wire need_set_d  = pte_is_leaf && pte_v && is_store_reg && !pte_d;
    wire need_update = need_set_a || need_set_d;

    // PTE com bits A/D atualizados
    wire [63:0] pte_updated = pte_reg | (need_set_a ? (64'd1 << PTE_A) : 64'd0)
                                       | (need_set_d ? (64'd1 << PTE_D) : 64'd0);

    // =========================================================================
    // Função: endereço de PTE
    // =========================================================================
    function automatic [PADDR_WIDTH-1:0] make_pte_addr(
        input [PPN_WIDTH-1:0] ppn,
        input [8:0]           vpn_index
    );
        return {{(PADDR_WIDTH-PPN_WIDTH-PAGE_OFFSET_BITS){1'b0}}, ppn, 12'b0} +
               {{(PADDR_WIDTH-9-PTE_SIZE_BITS){1'b0}}, vpn_index, 3'b0};
    endfunction

    // =========================================================================
    // FSM - Próximo Estado
    // =========================================================================
    always_comb begin
        next_state = state;

        case (state)
            S_IDLE: begin
                if (ptw_req_valid && ptw_req_ready) begin
                    if (satp_mode == SATP_MODE_SV39)
                        next_state = S_LEVEL2;
                    else
                        next_state = S_RESP;
                end
            end

            S_LEVEL2, S_LEVEL1, S_LEVEL0:
                next_state = S_WAIT_MEM;

            S_WAIT_MEM: begin
                if (ptw_mem_resp_valid) next_state = S_CHECK_PTE;
                else if (ptw_mem_resp_err) next_state = S_RESP;
            end

            S_CHECK_PTE: begin
                if (!pte_v || (!pte_r && pte_w)) begin
                    // PTE inválida
                    next_state = S_RESP;
                end else if (pte_is_pointer) begin
                    if (current_level == 0)       next_state = S_RESP;
                    else if (current_level == 2)  next_state = S_LEVEL1;
                    else                           next_state = S_LEVEL0;
                end else begin
                    // Leaf PTE
                    if (superpage_misaligned || !pte_a || (is_store_reg && !pte_d)) begin
                        // FIX: se A ou D precisam ser escritos, ir para UPDATE_PTE
                        //      Misaligned superpage ainda é page fault direto
                        if (superpage_misaligned)
                            next_state = S_RESP;   // fault
                        else
                            next_state = S_UPDATE_PTE;
                    end else begin
                        next_state = S_RESP;
                    end
                end
            end

            // FIX: escrever PTE atualizada na memória
            S_UPDATE_PTE:  next_state = S_WAIT_UPDATE;

            S_WAIT_UPDATE: begin
                if (ptw_mem_resp_valid || ptw_mem_resp_err)
                    next_state = S_RESP;
            end

            S_RESP:  next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end

    // =========================================================================
    // Interface de Memória
    // =========================================================================
    always_comb begin
        ptw_mem_req   = 1'b0;
        ptw_mem_addr  = '0;
        ptw_mem_we    = 1'b0;
        ptw_mem_wdata = '0;

        case (state)
            S_LEVEL2: begin
                ptw_mem_req  = 1'b1;
                ptw_mem_addr = make_pte_addr(satp_ppn, vpn2);
            end
            S_LEVEL1: begin
                ptw_mem_req  = 1'b1;
                ptw_mem_addr = make_pte_addr(current_ppn, vpn1);
            end
            S_LEVEL0: begin
                ptw_mem_req  = 1'b1;
                ptw_mem_addr = make_pte_addr(current_ppn, vpn0);
            end
            // FIX: writeback A/D
            S_UPDATE_PTE: begin
                ptw_mem_req   = 1'b1;
                ptw_mem_we    = 1'b1;
                ptw_mem_addr  = pte_addr_reg;
                ptw_mem_wdata = pte_updated;
            end
            default: ;
        endcase
    end

    // =========================================================================
    // Lógica Sequencial
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= S_IDLE;
            ptw_req_ready       <= 1'b1;
            ptw_resp_valid      <= 1'b0;
            ptw_resp_page_fault  <= 1'b0;
            ptw_resp_access_fault <= 1'b0;
            vpn_reg             <= '0;
            asid_reg            <= '0;
            is_store_reg        <= 1'b0;
            is_exec_reg         <= 1'b0;
            for_itlb_reg        <= 1'b0;
            current_level       <= 2'd2;
            pte_reg             <= '0;
            pte_addr_reg        <= '0;
            current_ppn         <= '0;
            page_fault          <= 1'b0;
            access_fault        <= 1'b0;
            ptw_resp_vpn        <= '0;
            ptw_resp_ppn        <= '0;
            ptw_resp_asid       <= '0;
            ptw_resp_page_size  <= '0;
            ptw_resp_for_itlb   <= 1'b0;
            ptw_resp_r          <= 1'b0;
            ptw_resp_w          <= 1'b0;
            ptw_resp_x          <= 1'b0;
            ptw_resp_u          <= 1'b0;
            ptw_resp_g          <= 1'b0;
            ptw_resp_a          <= 1'b0;
            ptw_resp_d          <= 1'b0;
        end else begin
            state <= next_state;

            case (state)
                S_IDLE: begin
                    ptw_resp_valid <= 1'b0;
                    page_fault     <= 1'b0;
                    access_fault   <= 1'b0;

                    if (ptw_req_valid && ptw_req_ready) begin
                        ptw_req_ready <= 1'b0;
                        vpn_reg       <= ptw_req_vpn;
                        asid_reg      <= ptw_req_asid;
                        is_store_reg  <= ptw_req_is_store;
                        is_exec_reg   <= ptw_req_is_exec;
                        for_itlb_reg  <= ptw_req_for_itlb;
                        current_level <= 2'd2;
                    end
                end

                S_WAIT_MEM: begin
                    if (ptw_mem_resp_valid) begin
                        pte_reg     <= ptw_mem_resp_data;
                        current_ppn <= ptw_mem_resp_data[53:10];

                        // FIX: salvar endereço atual da PTE para possível writeback
                        case (current_level)
                            2'd2: pte_addr_reg <= make_pte_addr(satp_ppn,    vpn2);
                            2'd1: pte_addr_reg <= make_pte_addr(current_ppn, vpn1);
                            2'd0: pte_addr_reg <= make_pte_addr(current_ppn, vpn0);
                            default: ;
                        endcase
                    end else if (ptw_mem_resp_err) begin
                        access_fault <= 1'b1;
                    end
                end

                S_CHECK_PTE: begin
                    if (!pte_v || (!pte_r && pte_w)) begin
                        page_fault <= 1'b1;
                    end else if (pte_is_pointer && current_level == 0) begin
                        page_fault <= 1'b1;
                    end else if (pte_is_leaf && superpage_misaligned) begin
                        page_fault <= 1'b1;
                    end

                    if (pte_is_pointer && current_level > 0)
                        current_level <= current_level - 1;
                end

                // FIX: atualizar pte_reg localmente com bits A/D antes do writeback
                //      Isso garante que a resposta para a TLB inclua A e D setados.
                S_UPDATE_PTE: begin
                    pte_reg <= pte_updated;
                end

                S_WAIT_UPDATE: begin
                    // Aguardar ack do writeback; erros são ignorados graciosamente
                    // (o hardware fez o melhor esforço; o software pode tentar de novo)
                end

                S_RESP: begin
                    ptw_resp_valid      <= 1'b1;
                    ptw_req_ready       <= 1'b1;
                    ptw_resp_vpn        <= vpn_reg;
                    ptw_resp_asid       <= asid_reg;
                    ptw_resp_for_itlb   <= for_itlb_reg;
                    ptw_resp_page_fault  <= page_fault;
                    ptw_resp_access_fault <= access_fault;

                    if (!page_fault && !access_fault) begin
                        ptw_resp_ppn        <= pte_ppn;
                        ptw_resp_page_size  <= current_level;
                        // FIX: usar pte_reg já atualizado (com A/D setados se necessário)
                        ptw_resp_r          <= pte_reg[PTE_R];
                        ptw_resp_w          <= pte_reg[PTE_W];
                        ptw_resp_x          <= pte_reg[PTE_X];
                        ptw_resp_u          <= pte_reg[PTE_U];
                        ptw_resp_g          <= pte_reg[PTE_G];
                        ptw_resp_a          <= pte_reg[PTE_A];
                        ptw_resp_d          <= pte_reg[PTE_D];
                    end else begin
                        ptw_resp_ppn        <= '0;
                        ptw_resp_page_size  <= '0;
                        ptw_resp_r          <= 1'b0;
                        ptw_resp_w          <= 1'b0;
                        ptw_resp_x          <= 1'b0;
                        ptw_resp_u          <= 1'b0;
                        ptw_resp_g          <= 1'b0;
                        ptw_resp_a          <= 1'b0;
                        ptw_resp_d          <= 1'b0;
                    end
                end

                default: ;
            endcase
        end
    end

endmodule
