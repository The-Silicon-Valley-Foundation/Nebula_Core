`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_frontend_rvc
 * @brief Frontend com suporte completo a RV64C (instruções comprimidas)
 *
 * CORREÇÕES APLICADAS:
 * 1. RESET_VECTOR corrigido para 0x10000000 (ROM do LiteX, confirmado por regions.ld)
 *    no LiteX com DRAM em 0x40000000 ou 0x80000000 dependendo do target).
 *    *** AJUSTE ESTE VALOR para o endereço de carga do seu OpenSBI. ***
 * 2. Campo rm da instrução decodificada agora é populado para instruções
 *    FP (7'b1010011) e FMA — corrige conformidade IEEE 754.
 * 3. fetch_exception_cause mudado para logic [5:0] para alinhar com
 *    nebula_core_full.sv e evitar truncamento silencioso.
 */
module nebula_frontend_rvc #(
    parameter int XLEN = 64,
    parameter int VADDR_WIDTH = 39,
    parameter int PADDR_WIDTH = 56,
    parameter int VPN_WIDTH = 27,
    parameter int PPN_WIDTH = 44
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     backend_stall,
    input  wire                     backend_flush,
    input  wire                     backend_redirect,
    input  wire [VADDR_WIDTH-1:0]   backend_redirect_pc,

    output frontend_packet_t        frontend_out,
    output logic                    frontend_valid,
    output logic [VADDR_WIDTH-1:0]  fetch_pc_out,

    input  bp_prediction_t          bp_prediction,

    output logic                    icache_req,
    output logic [PADDR_WIDTH-1:0]  icache_addr,
    input  wire                     icache_ready,
    input  wire                     icache_resp_valid,
    input  wire [63:0]              icache_resp_data,
    input  wire                     icache_resp_error,

    output logic                    itlb_req,
    output logic [VPN_WIDTH-1:0]    itlb_vpn,
    input  wire                     itlb_hit,
    input  wire [PPN_WIDTH-1:0]     itlb_ppn,
    input  wire                     itlb_page_fault,
    input  wire                     itlb_access_fault,

    output logic                    ptw_req,
    output logic [VPN_WIDTH-1:0]    ptw_vpn,
    input  wire                     ptw_ready,
    input  wire                     ptw_resp_valid,
    input  wire                     ptw_page_fault,
    input  wire                     ptw_access_fault,

    input  wire                     mmu_enabled,
    input  wire [1:0]               current_priv,

    // FIX 3: tipo alterado para logic [5:0] — consistente com nebula_core_full
    output logic                    fetch_exception,
    output logic [5:0]              fetch_exception_cause,
    output logic [XLEN-1:0]         fetch_exception_value
);

    // =========================================================================
    // FIX 1: Reset Vector
    // Ajuste para o endereço de load do seu OpenSBI/BBL.
    // LiteX típico com Arty-A7 / Digilent: DRAM em 0x40000000
    // LiteX VexRiscV demo: 0x40000000
    // OpenSBI entry point padrão: base da DRAM
    // *** VERIFIQUE seu target.py: "mem_map" ou "SoCCore.mem_map" ***
    // =========================================================================
    localparam logic [VADDR_WIDTH-1:0] RESET_VECTOR = 39'h00_1000_0000; // ROM: ORIGIN=0x10000000 (regions.ld)

    // =========================================================================
    // Fetch Buffer
    // =========================================================================
    logic [79:0] fetch_buffer;
    logic [3:0]  fetch_buffer_valid;
    logic [2:0]  fetch_offset;

    // =========================================================================
    // PC Management
    // =========================================================================
    logic [VADDR_WIDTH-1:0] pc_reg;
    logic [VADDR_WIDTH-1:0] fetch_pc;
    logic [VADDR_WIDTH-1:0] next_pc;
    logic [VADDR_WIDTH-1:0] instr_pc;

    // =========================================================================
    // Instruction Extraction
    // =========================================================================
    logic [31:0] raw_instr;
    logic [15:0] first_halfword;
    logic [15:0] second_halfword;
    logic        is_compressed;
    logic        need_second_half;
    logic        have_full_instr;

    // =========================================================================
    // Compressed Decoder Interface
    // =========================================================================
    logic [15:0] compressed_instr;
    logic [31:0] expanded_instr;
    logic        compressed_valid;
    logic        compressed_illegal;
    logic        is_compressed_out;

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [3:0] {
        S_RESET,
        S_FETCH_REQ,
        S_TLB_LOOKUP,
        S_TLB_WAIT,
        S_ICACHE_REQ,
        S_ICACHE_WAIT,
        S_PROCESS,
        S_DECODE,
        S_STALL,
        S_FLUSH,
        S_EXCEPTION
    } state_t;

    state_t state, next_state;

    logic [PPN_WIDTH-1:0] cached_ppn;
    logic                 tlb_done;

    // =========================================================================
    // Compressed Decoder Instance
    // =========================================================================
    compressed_decoder_rv64 u_decompress (
        .cinstr        (compressed_instr),
        .instr         (expanded_instr),
        .valid         (compressed_valid),
        .illegal       (compressed_illegal),
        .is_compressed (is_compressed_out)
    );

    // =========================================================================
    // Instruction Alignment and Extraction
    // =========================================================================
    always_comb begin
        case (fetch_offset[1:0])
            2'b00: first_halfword = fetch_buffer[15:0];
            2'b01: first_halfword = fetch_buffer[31:16];
            2'b10: first_halfword = fetch_buffer[47:32];
            2'b11: first_halfword = fetch_buffer[63:48];
            default: first_halfword = '0;
        endcase

        case (fetch_offset[1:0])
            2'b00: second_halfword = fetch_buffer[31:16];
            2'b01: second_halfword = fetch_buffer[47:32];
            2'b10: second_halfword = fetch_buffer[63:48];
            2'b11: second_halfword = fetch_buffer[79:64];
            default: second_halfword = '0;
        endcase

        is_compressed    = (first_halfword[1:0] != 2'b11);
        need_second_half = !is_compressed;

        case (fetch_offset[1:0])
            2'b00: have_full_instr = fetch_buffer_valid[0] &&
                                     (is_compressed || fetch_buffer_valid[1]);
            2'b01: have_full_instr = fetch_buffer_valid[1] &&
                                     (is_compressed || fetch_buffer_valid[2]);
            2'b10: have_full_instr = fetch_buffer_valid[2] &&
                                     (is_compressed || fetch_buffer_valid[3]);
            2'b11: have_full_instr = fetch_buffer_valid[3] &&
                                     (is_compressed || (fetch_offset[2] &&
                                      fetch_buffer[79:64] != 16'h0));
            default: have_full_instr = 1'b0;
        endcase

        if (is_compressed) begin
            raw_instr        = {16'h0, first_halfword};
            compressed_instr = first_halfword;
        end else begin
            raw_instr        = {second_halfword, first_halfword};
            compressed_instr = 16'h0;
        end
    end

    // =========================================================================
    // PC Calculation
    // =========================================================================
    always_comb begin
        instr_pc = pc_reg;
        fetch_pc = {pc_reg[VADDR_WIDTH-1:3], 3'b000};
        next_pc  = is_compressed ? (pc_reg + 2) : (pc_reg + 4);
    end

    // =========================================================================
    // Decode Logic
    // =========================================================================
    decoded_instr_t decoded_instr0;
    decoded_instr_t decoded_instr1;
    logic           can_dual_issue;

    logic [31:0] final_instr;
    assign final_instr = is_compressed ? expanded_instr : raw_instr;

    always_comb begin
        decoded_instr0            = '0;
        decoded_instr0.pc         = instr_pc;
        decoded_instr0.is_compressed = is_compressed;
        decoded_instr0.valid      = have_full_instr && (state == S_DECODE);

        if (is_compressed && compressed_illegal) begin
            decoded_instr0.valid = 1'b0;
        end else if (have_full_instr) begin
            decoded_instr0.opcode = final_instr[6:0];
            decoded_instr0.rd     = final_instr[11:7];
            decoded_instr0.funct3 = final_instr[14:12];
            decoded_instr0.rs1    = final_instr[19:15];
            decoded_instr0.rs2    = final_instr[24:20];
            decoded_instr0.rs3    = final_instr[31:27];
            decoded_instr0.funct7 = final_instr[31:25];

            case (final_instr[6:0])
                7'b0110111, 7'b0010111: begin // LUI, AUIPC
                    decoded_instr0.imm   = {{32{final_instr[31]}}, final_instr[31:12], 12'b0};
                    decoded_instr0.is_alu = 1'b1;
                end

                7'b1101111: begin // JAL
                    decoded_instr0.imm = {{43{final_instr[31]}}, final_instr[31],
                                          final_instr[19:12], final_instr[20],
                                          final_instr[30:21], 1'b0};
                    decoded_instr0.is_branch = 1'b1;
                    decoded_instr0.is_jal    = 1'b1;
                end

                7'b1100111: begin // JALR
                    decoded_instr0.imm      = {{52{final_instr[31]}}, final_instr[31:20]};
                    decoded_instr0.is_branch = 1'b1;
                    decoded_instr0.is_jalr   = 1'b1;
                end

                7'b1100011: begin // Branch
                    decoded_instr0.imm = {{51{final_instr[31]}}, final_instr[31],
                                          final_instr[7], final_instr[30:25],
                                          final_instr[11:8], 1'b0};
                    decoded_instr0.is_branch = 1'b1;
                end

                7'b0000011: begin // Load
                    decoded_instr0.imm     = {{52{final_instr[31]}}, final_instr[31:20]};
                    decoded_instr0.is_load = 1'b1;
                end

                7'b0100011: begin // Store
                    decoded_instr0.imm      = {{52{final_instr[31]}}, final_instr[31:25],
                                               final_instr[11:7]};
                    decoded_instr0.is_store = 1'b1;
                end

                7'b0010011, 7'b0011011: begin // ALU-I, ALU-I-W
                    decoded_instr0.imm     = {{52{final_instr[31]}}, final_instr[31:20]};
                    decoded_instr0.is_alu  = 1'b1;
                    decoded_instr0.is_alu_w = (final_instr[6:0] == 7'b0011011);
                end

                7'b0110011, 7'b0111011: begin // ALU-R, ALU-R-W
                    decoded_instr0.is_alu   = 1'b1;
                    decoded_instr0.is_alu_w = (final_instr[6:0] == 7'b0111011);
                    if (final_instr[31:25] == 7'b0000001)
                        decoded_instr0.is_mdu = 1'b1;
                end

                7'b0101111: begin // AMO
                    decoded_instr0.is_amo = 1'b1;
                    decoded_instr0.is_lr  = (final_instr[31:27] == 5'b00010);
                    decoded_instr0.is_sc  = (final_instr[31:27] == 5'b00011);
                end

                7'b0000111: begin // FP Load
                    decoded_instr0.imm         = {{52{final_instr[31]}}, final_instr[31:20]};
                    decoded_instr0.is_fp_load  = 1'b1;
                    decoded_instr0.is_fp       = 1'b1;
                end

                7'b0100111: begin // FP Store
                    decoded_instr0.imm          = {{52{final_instr[31]}}, final_instr[31:25],
                                                   final_instr[11:7]};
                    decoded_instr0.is_fp_store  = 1'b1;
                    decoded_instr0.is_fp        = 1'b1;
                end

                // FIX 2a: FMA — popular campo rm
                7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: begin // FMADD/FMSUB/FNMSUB/FNMADD
                    decoded_instr0.is_fp        = 1'b1;
                    decoded_instr0.is_fma       = 1'b1;
                    decoded_instr0.is_fp_single = (final_instr[26:25] == 2'b00);
                    decoded_instr0.is_fp_double = (final_instr[26:25] == 2'b01);
                    decoded_instr0.rm           = rounding_mode_t'(final_instr[14:12]); // FIX
                end

                // FIX 2b: FP ops — popular campo rm
                7'b1010011: begin
                    decoded_instr0.is_fp        = 1'b1;
                    decoded_instr0.is_fp_single = (final_instr[26:25] == 2'b00);
                    decoded_instr0.is_fp_double = (final_instr[26:25] == 2'b01);
                    decoded_instr0.rm           = rounding_mode_t'(final_instr[14:12]); // FIX
                end

                7'b1110011: begin // SYSTEM
                    decoded_instr0.csr_addr = final_instr[31:20];
                    if (final_instr[14:12] != 3'b000) begin
                        decoded_instr0.is_csr = 1'b1;
                    end else begin
                        case (final_instr[31:20])
                            12'h000: decoded_instr0.is_ecall  = 1'b1;
                            12'h001: decoded_instr0.is_ebreak = 1'b1;
                            12'h302: decoded_instr0.is_mret   = 1'b1;
                            12'h102: decoded_instr0.is_sret   = 1'b1;
                            12'h105: decoded_instr0.is_wfi    = 1'b1;
                            default: ;
                        endcase
                    end
                end

                7'b0001111: begin // FENCE
                    if (final_instr[14:12] == 3'b001)
                        decoded_instr0.is_fence_i = 1'b1;
                    else
                        decoded_instr0.is_fence   = 1'b1;
                end

                default: ;
            endcase

            // SFENCE.VMA
            if (final_instr[31:25] == 7'b0001001 &&
                final_instr[14:12] == 3'b000 &&
                final_instr[6:0]   == 7'b1110011)
                decoded_instr0.is_sfence_vma = 1'b1;
        end

        decoded_instr1 = '0;
        can_dual_issue = 1'b0;
    end

    // =========================================================================
    // FSM Next State
    // =========================================================================
    always_comb begin
        next_state = state;

        case (state)
            S_RESET:     next_state = S_FETCH_REQ;

            S_FETCH_REQ: begin
                if (backend_flush)  next_state = S_FLUSH;
                else if (mmu_enabled) next_state = S_TLB_LOOKUP;
                else                  next_state = S_ICACHE_REQ;
            end

            S_TLB_LOOKUP: begin
                if (backend_flush)       next_state = S_FLUSH;
                else if (itlb_page_fault) next_state = S_EXCEPTION;
                else if (itlb_hit)        next_state = S_ICACHE_REQ;
                else if (ptw_ready)       next_state = S_TLB_WAIT;
            end

            S_TLB_WAIT: begin
                if (backend_flush)    next_state = S_FLUSH;
                else if (ptw_resp_valid) begin
                    if (ptw_page_fault) next_state = S_EXCEPTION;
                    else                next_state = S_TLB_LOOKUP;
                end
            end

            S_ICACHE_REQ: begin
                if (backend_flush)  next_state = S_FLUSH;
                else if (icache_ready) next_state = S_ICACHE_WAIT;
            end

            S_ICACHE_WAIT: begin
                if (backend_flush)         next_state = S_FLUSH;
                else if (icache_resp_valid) begin
                    if (icache_resp_error)  next_state = S_EXCEPTION;
                    else                    next_state = S_PROCESS;
                end
            end

            S_PROCESS: begin
                if (backend_flush)      next_state = S_FLUSH;
                else if (have_full_instr) next_state = S_DECODE;
                else                      next_state = S_FETCH_REQ;
            end

            S_DECODE: begin
                if (backend_flush)   next_state = S_FLUSH;
                else if (backend_stall) next_state = S_STALL;
                else begin
                    if (have_full_instr) next_state = S_DECODE;
                    else                 next_state = S_FETCH_REQ;
                end
            end

            S_STALL: begin
                if (backend_flush)    next_state = S_FLUSH;
                else if (!backend_stall) next_state = S_DECODE;
            end

            S_FLUSH:     next_state = S_FETCH_REQ;
            S_EXCEPTION: begin
                if (backend_flush) next_state = S_FLUSH;
            end

            default: next_state = S_RESET;
        endcase
    end

    // =========================================================================
    // Output Signals
    // =========================================================================
    assign frontend_out.instr0       = decoded_instr0;
    assign frontend_out.instr1       = decoded_instr1;
    assign frontend_out.instr0_valid = decoded_instr0.valid;
    assign frontend_out.instr1_valid = 1'b0;
    assign frontend_out.dual_issue   = 1'b0;

    assign frontend_valid = (state == S_DECODE) && have_full_instr && !backend_stall;
    assign fetch_pc_out   = instr_pc;

    assign icache_req  = (state == S_ICACHE_REQ);
    assign icache_addr = mmu_enabled ?
                         {{(PADDR_WIDTH-PPN_WIDTH-12){1'b0}}, cached_ppn, fetch_pc[11:0]} :
                         {{(PADDR_WIDTH-VADDR_WIDTH){1'b0}}, fetch_pc};

    assign itlb_req = (state == S_TLB_LOOKUP);
    assign itlb_vpn = fetch_pc[VADDR_WIDTH-1:12];

    assign ptw_req = (state == S_TLB_LOOKUP) && !itlb_hit && !itlb_page_fault;
    assign ptw_vpn = fetch_pc[VADDR_WIDTH-1:12];

    // FIX 3: tipo logic [5:0] — cast explícito a partir do enum
    assign fetch_exception        = (state == S_EXCEPTION);
    assign fetch_exception_cause  = (itlb_page_fault || ptw_page_fault) ?
                                    6'(EXC_INSTR_PAGE_FAULT) : 6'(EXC_INSTR_ACCESS_FAULT);
    assign fetch_exception_value  = {{(XLEN-VADDR_WIDTH){pc_reg[VADDR_WIDTH-1]}}, pc_reg};

    // =========================================================================
    // Sequential Logic
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_RESET;
            pc_reg             <= RESET_VECTOR;
            fetch_buffer       <= '0;
            fetch_buffer_valid <= '0;
            fetch_offset       <= '0;
            cached_ppn         <= '0;
            tlb_done           <= 1'b0;
        end else begin
            state <= next_state;

            case (state)
                S_RESET: begin
                    pc_reg             <= RESET_VECTOR;
                    fetch_buffer       <= '0;
                    fetch_buffer_valid <= '0;
                    fetch_offset       <= '0;
                end

                S_FETCH_REQ: begin
                    fetch_offset <= {1'b0, pc_reg[2:1]};
                end

                S_TLB_LOOKUP: begin
                    if (itlb_hit) begin
                        cached_ppn <= itlb_ppn;
                        tlb_done   <= 1'b1;
                    end
                end

                S_ICACHE_WAIT: begin
                    if (icache_resp_valid && !icache_resp_error) begin
                        if (fetch_offset[2])
                            fetch_buffer[79:64] <= fetch_buffer[63:48];
                        fetch_buffer[63:0]  <= icache_resp_data;
                        fetch_buffer_valid  <= 4'b1111;
                    end
                end

                S_DECODE: begin
                    if (!backend_stall && have_full_instr) begin
                        if (backend_redirect) begin
                            pc_reg             <= backend_redirect_pc;
                            fetch_buffer_valid <= '0;
                            fetch_offset       <= '0;
                        end else if (bp_prediction.valid && bp_prediction.taken) begin
                            pc_reg             <= bp_prediction.target;
                            fetch_buffer_valid <= '0;
                            fetch_offset       <= '0;
                        end else begin
                            pc_reg <= next_pc;

                            if (is_compressed)
                                fetch_offset <= fetch_offset + 1;
                            else
                                fetch_offset <= fetch_offset + 2;

                            if (is_compressed) begin
                                case (fetch_offset[1:0])
                                    2'b00: fetch_buffer_valid[0] <= 1'b0;
                                    2'b01: fetch_buffer_valid[1] <= 1'b0;
                                    2'b10: fetch_buffer_valid[2] <= 1'b0;
                                    2'b11: fetch_buffer_valid[3] <= 1'b0;
                                    default: ;
                                endcase
                            end else begin
                                case (fetch_offset[1:0])
                                    2'b00: fetch_buffer_valid[1:0] <= 2'b00;
                                    2'b01: fetch_buffer_valid[2:1] <= 2'b00;
                                    2'b10: fetch_buffer_valid[3:2] <= 2'b00;
                                    2'b11: fetch_buffer_valid[3]   <= 1'b0;
                                    default: ;
                                endcase
                            end
                        end
                    end
                end

                S_FLUSH: begin
                    if (backend_redirect)
                        pc_reg <= backend_redirect_pc;
                    fetch_buffer       <= '0;
                    fetch_buffer_valid <= '0;
                    fetch_offset       <= '0;
                    tlb_done           <= 1'b0;
                end

                default: ;
            endcase
        end
    end

endmodule
