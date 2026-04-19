`timescale 1ns/1ps

/**
 * @module nebula_axi_adapter
 * @brief Adaptador Nebula (512-bit) <-> AXI4 (64-bit)
 *
 * CORREÇÕES APLICADAS:
 *
 * BUG 1 FIX — mem_ready redefinido.
 *   Antes: assign mem_ready = (d_state == IDLE)
 *   O adapter saía de IDLE ao aceitar a req da L2, então mem_ready=0
 *   durante toda a transação AXI. A L2 ficava presa em S_FILL_REQ
 *   esperando mem_ready=1 que nunca vinha → deadlock (estado 8).
 *
 *   Depois: mem_ready pulsa 1 ciclo quando o handshake AR ou AW é aceito
 *   (arready=1 em R_ADDR, awready=1 em W_ADDR). Isso sinaliza para a L2
 *   que a req foi encaminhada → avança S_FILL_REQ → S_FILL_WAIT.
 *
 *   mem_idle = (d_state == IDLE) exposto como port separado para uso
 *   do cluster no bypass MMIO (Bug 3 fix).
 *
 * PORTS m_axi_i_* de saída removidos da interface:
 *   Em simulação Opção A o LiteX não conecta nenhum sinal do canal I.
 *   Expô-los como output ports causava PINMISSING no Verilator.
 *   Os inputs do canal I (arready, rdata, rresp, rlast, rvalid) são
 *   mantidos como ports de entrada para completude — o top os tie-off a 0.
 *   As saídas (arvalid, araddr, rready, etc.) viraram logic internos.
 */
module nebula_axi_adapter #(
    parameter int PADDR_WIDTH  = 56,
    parameter int AXI_ID_WIDTH = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // mem_ready: pulsa quando handshake AXI é aceito (Bug 1 fix)
    // mem_idle:  adapter livre para nova req (Bug 3 fix no cluster)
    output logic mem_ready,
    output logic mem_idle,

    // I-Cache (512-bit) — tie-off em simulação (imem_req=0)
    input  wire                     imem_req,
    input  wire [PADDR_WIDTH-1:0]   imem_addr,
    output logic                    imem_ack,
    output logic [511:0]            imem_data,

    // D-Cache (512-bit)
    input  wire                     dmem_req,
    input  wire                     dmem_we,
    input  wire [PADDR_WIDTH-1:0]   dmem_addr,
    input  wire [511:0]             dmem_wdata,
    input  wire [63:0]              dmem_wstrb,
    input  wire                     dmem_uncached,
    output logic                    dmem_ack,
    output logic [511:0]            dmem_rdata,

    // AXI4 Data (canal D)
    output logic [AXI_ID_WIDTH-1:0] m_axi_d_awid,
    output logic [PADDR_WIDTH-1:0]  m_axi_d_awaddr,
    output logic [7:0]              m_axi_d_awlen,
    output logic [2:0]              m_axi_d_awsize,
    output logic [1:0]              m_axi_d_awburst,
    output logic                    m_axi_d_awvalid,
    input  wire                     m_axi_d_awready,
    output logic [63:0]             m_axi_d_wdata,
    output logic [7:0]              m_axi_d_wstrb,
    output logic                    m_axi_d_wlast,
    output logic                    m_axi_d_wvalid,
    input  wire                     m_axi_d_wready,
    input  wire [AXI_ID_WIDTH-1:0]  m_axi_d_bid,
    input  wire [1:0]               m_axi_d_bresp,
    input  wire                     m_axi_d_bvalid,
    output logic                    m_axi_d_bready,
    output logic [AXI_ID_WIDTH-1:0] m_axi_d_arid,
    output logic [PADDR_WIDTH-1:0]  m_axi_d_araddr,
    output logic [7:0]              m_axi_d_arlen,
    output logic [2:0]              m_axi_d_arsize,
    output logic [1:0]              m_axi_d_arburst,
    output logic                    m_axi_d_arvalid,
    input  wire                     m_axi_d_arready,
    input  wire [63:0]              m_axi_d_rdata,
    input  wire [1:0]               m_axi_d_rresp,
    input  wire                     m_axi_d_rlast,
    input  wire                     m_axi_d_rvalid,
    output logic                    m_axi_d_rready,

    // AXI4 Instruction canal I — apenas inputs (outputs são logic internos)
    // O top faz tie-off desses inputs a 0 em simulação Opção A.
    input  wire                     m_axi_i_arready,
    input  wire [63:0]              m_axi_i_rdata,
    input  wire [1:0]               m_axi_i_rresp,
    input  wire                     m_axi_i_rlast,
    input  wire                     m_axi_i_rvalid
);

    // =========================================================================
    // Constantes AXI
    // =========================================================================
    localparam logic [7:0]  AXI_LEN_BURST  = 8'd7;
    localparam logic [7:0]  AXI_LEN_SINGLE = 8'd0;
    localparam logic [2:0]  AXI_SIZE       = 3'b011;
    localparam logic [1:0]  AXI_BURST      = 2'b01;

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [2:0] {
        IDLE,
        R_ADDR,
        R_DATA,
        W_ADDR,
        W_DATA,
        W_RESP
    } state_t;

    // =========================================================================
    // D-Cache FSM
    // =========================================================================
    state_t         d_state;
    logic [2:0]     d_beat;
    logic [511:0]   d_rbuf;
    logic [511:0]   d_rdata_reg;
    logic [511:0]   d_wbuf_reg;
    logic [63:0]    d_wstrb_reg;
    logic [PADDR_WIDTH-1:0] d_base;
    logic [2:0]     d_target_beat_reg;
    logic           d_uncached_reg;

    // BUG 1 FIX
    assign mem_idle  = (d_state == IDLE);
    assign mem_ready = (d_state == R_ADDR && m_axi_d_arready) ||
                       (d_state == W_ADDR && m_axi_d_awready);

    // Saídas fixas canal D
    assign m_axi_d_arid    = '0;
    assign m_axi_d_awid    = '0;
    assign m_axi_d_arsize  = AXI_SIZE;
    assign m_axi_d_arburst = AXI_BURST;
    assign m_axi_d_awsize  = AXI_SIZE;
    assign m_axi_d_awburst = AXI_BURST;
    assign m_axi_d_arlen   = d_uncached_reg ? AXI_LEN_SINGLE : AXI_LEN_BURST;
    assign m_axi_d_awlen   = d_uncached_reg ? AXI_LEN_SINGLE : AXI_LEN_BURST;

    wire [63:0] cur_wdata = d_uncached_reg
        ? d_wbuf_reg[d_target_beat_reg * 64 +: 64]
        : d_wbuf_reg[d_beat * 64 +: 64];

    wire [7:0] cur_wstrb_raw = d_uncached_reg
        ? d_wstrb_reg[d_target_beat_reg * 8 +: 8]
        : d_wstrb_reg[d_beat * 8 +: 8];

    wire [7:0] cur_wstrb = (!d_uncached_reg && d_wstrb_reg == 64'hFFFFFFFFFFFFFFFF)
        ? 8'hFF : cur_wstrb_raw;

    assign m_axi_d_wdata = cur_wdata;
    assign m_axi_d_wstrb = cur_wstrb;
    assign m_axi_d_wlast = d_uncached_reg ? 1'b1 : (d_beat == 3'd7);
    assign dmem_rdata    = d_rdata_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_state           <= IDLE;
            d_beat            <= '0;
            d_base            <= '0;
            d_rbuf            <= '0;
            d_rdata_reg       <= '0;
            d_wbuf_reg        <= '0;
            d_wstrb_reg       <= '0;
            d_target_beat_reg <= '0;
            d_uncached_reg    <= 1'b0;
            dmem_ack          <= 1'b0;
            m_axi_d_arvalid   <= 1'b0;
            m_axi_d_araddr    <= '0;
            m_axi_d_rready    <= 1'b0;
            m_axi_d_awvalid   <= 1'b0;
            m_axi_d_awaddr    <= '0;
            m_axi_d_wvalid    <= 1'b0;
            m_axi_d_bready    <= 1'b0;
        end else begin
            dmem_ack <= 1'b0;

            case (d_state)

                IDLE: begin
                    d_beat   <= '0;
                    dmem_ack <= 1'b0;
                    if (dmem_req && !dmem_ack) begin
                        if (dmem_we) begin
                            m_axi_d_awaddr    <= dmem_uncached
                                ? dmem_addr
                                : {dmem_addr[PADDR_WIDTH-1:6], 6'b0};
                            m_axi_d_awvalid   <= 1'b1;
                            d_state           <= W_ADDR;
                            d_base            <= {dmem_addr[PADDR_WIDTH-1:6], 6'b0};
                            d_wbuf_reg        <= dmem_wdata;
                            d_wstrb_reg       <= dmem_wstrb;
                            d_target_beat_reg <= dmem_addr[5:3];
                            d_uncached_reg    <= dmem_uncached;
                        end else begin
                            m_axi_d_araddr    <= dmem_uncached
                                ? dmem_addr
                                : {dmem_addr[PADDR_WIDTH-1:6], 6'b0};
                            m_axi_d_arvalid   <= 1'b1;
                            d_uncached_reg    <= dmem_uncached;
                            d_target_beat_reg <= dmem_addr[5:3];
                            d_state           <= R_ADDR;
                        end
                    end
                end

                // BUG 1 FIX: mem_ready=1 neste ciclo quando arready=1.
                R_ADDR: begin
                    if (m_axi_d_arready) begin
                        m_axi_d_arvalid <= 1'b0;
                        m_axi_d_rready  <= 1'b1;
                        d_state         <= R_DATA;
                    end
                end

                R_DATA: begin
                    if (m_axi_d_rvalid) begin
                        d_rbuf[d_beat * 64 +: 64] <= m_axi_d_rdata;

                        if (m_axi_d_rlast) begin
                            m_axi_d_rready <= 1'b0;
                            dmem_ack       <= 1'b1;
                            d_state        <= IDLE;
                            d_beat         <= '0;

                            if (d_uncached_reg) begin
                                d_rdata_reg <= {8{m_axi_d_rdata}};
                            end else begin
                                // d_beat é o índice do último beat.
                                // d_rbuf[d_beat] ainda não foi commitado
                                // (non-blocking) — lido direto de m_axi_d_rdata.
                                d_rdata_reg <= {
                                    (d_beat == 3'd7) ? m_axi_d_rdata : d_rbuf[7*64 +: 64],
                                    (d_beat == 3'd6) ? m_axi_d_rdata : d_rbuf[6*64 +: 64],
                                    (d_beat == 3'd5) ? m_axi_d_rdata : d_rbuf[5*64 +: 64],
                                    (d_beat == 3'd4) ? m_axi_d_rdata : d_rbuf[4*64 +: 64],
                                    (d_beat == 3'd3) ? m_axi_d_rdata : d_rbuf[3*64 +: 64],
                                    (d_beat == 3'd2) ? m_axi_d_rdata : d_rbuf[2*64 +: 64],
                                    (d_beat == 3'd1) ? m_axi_d_rdata : d_rbuf[1*64 +: 64],
                                    (d_beat == 3'd0) ? m_axi_d_rdata : d_rbuf[0*64 +: 64]
                                };
                            end
                        end else begin
                            d_beat <= d_beat + 1;
                        end
                    end
                end

                // BUG 1 FIX: mem_ready=1 neste ciclo quando awready=1.
                W_ADDR: begin
                    if (m_axi_d_awready) begin
                        m_axi_d_awvalid <= 1'b0;
                        m_axi_d_wvalid  <= 1'b1;
                        d_state         <= W_DATA;
                    end
                end

                W_DATA: begin
                    if (m_axi_d_wready) begin
                        if (m_axi_d_wlast) begin
                            m_axi_d_wvalid <= 1'b0;
                            m_axi_d_bready <= 1'b1;
                            d_beat         <= '0;
                            d_state        <= W_RESP;
                        end else begin
                            d_beat <= d_beat + 1;
                        end
                    end
                end

                W_RESP: begin
                    if (m_axi_d_bvalid) begin
                        m_axi_d_bready <= 1'b0;
                        dmem_ack       <= 1'b1;
                        d_state        <= IDLE;
                    end
                end

                default: d_state <= IDLE;
            endcase
        end
    end

    // =========================================================================
    // I-Cache FSM — idle em simulação (imem_req=0 hardcoded no top).
    // Saídas do canal I são logic interno — não expostas como ports.
    // =========================================================================
    state_t         i_state;
    logic [2:0]     i_beat;
    logic [511:0]   i_rbuf;
    logic [511:0]   i_rdata_reg;
    logic [PADDR_WIDTH-1:0] i_base;

    logic [PADDR_WIDTH-1:0]  m_axi_i_araddr_int;
    logic                    m_axi_i_arvalid_int;
    logic                    m_axi_i_rready_int;

    assign imem_data = i_rdata_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_state             <= IDLE;
            i_beat              <= '0;
            i_base              <= '0;
            i_rbuf              <= '0;
            i_rdata_reg         <= '0;
            imem_ack            <= 1'b0;
            m_axi_i_arvalid_int <= 1'b0;
            m_axi_i_araddr_int  <= '0;
            m_axi_i_rready_int  <= 1'b0;
        end else begin
            imem_ack <= 1'b0;

            case (i_state)

                IDLE: begin
                    i_beat <= '0;
                    if (imem_req && d_state == IDLE) begin
                        i_base              <= {imem_addr[PADDR_WIDTH-1:6], 6'b0};
                        m_axi_i_araddr_int  <= {imem_addr[PADDR_WIDTH-1:6], 6'b0};
                        m_axi_i_arvalid_int <= 1'b1;
                        i_state             <= R_ADDR;
                    end
                end

                R_ADDR: begin
                    if (m_axi_i_arready) begin
                        m_axi_i_arvalid_int <= 1'b0;
                        m_axi_i_rready_int  <= 1'b1;
                        i_state             <= R_DATA;
                    end
                end

                R_DATA: begin
                    if (m_axi_i_rvalid) begin
                        i_rbuf[i_beat * 64 +: 64] <= m_axi_i_rdata;

                        if (m_axi_i_rlast) begin
                            m_axi_i_rready_int <= 1'b0;
                            imem_ack           <= 1'b1;
                            i_state            <= IDLE;
                            i_beat             <= '0;
                            i_rdata_reg <= {
                                (i_beat == 3'd7) ? m_axi_i_rdata : i_rbuf[7*64 +: 64],
                                (i_beat == 3'd6) ? m_axi_i_rdata : i_rbuf[6*64 +: 64],
                                (i_beat == 3'd5) ? m_axi_i_rdata : i_rbuf[5*64 +: 64],
                                (i_beat == 3'd4) ? m_axi_i_rdata : i_rbuf[4*64 +: 64],
                                (i_beat == 3'd3) ? m_axi_i_rdata : i_rbuf[3*64 +: 64],
                                (i_beat == 3'd2) ? m_axi_i_rdata : i_rbuf[2*64 +: 64],
                                (i_beat == 3'd1) ? m_axi_i_rdata : i_rbuf[1*64 +: 64],
                                (i_beat == 3'd0) ? m_axi_i_rdata : i_rbuf[0*64 +: 64]
                            };
                        end else begin
                            i_beat <= i_beat + 1;
                        end
                    end
                end

                default: i_state <= IDLE;
            endcase
        end
    end

    // =========================================================================
    // Debug
    // =========================================================================
    logic dbg_arvalid_ant;
    logic dbg_awvalid_ant;
    always_ff @(posedge clk) begin
        dbg_arvalid_ant <= m_axi_d_arvalid;
        dbg_awvalid_ant <= m_axi_d_awvalid;

        if (m_axi_d_arvalid && !dbg_arvalid_ant)
            $display("[NEBULA READ] Endereco: %h", m_axi_d_araddr);

        if (m_axi_d_awvalid && !dbg_awvalid_ant) begin
            $display("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            $display("[NEBULA WRITE] O CPU TENTOU ESCREVER EM: %h", m_axi_d_awaddr);
            $display("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        end
    end
    // synthesis translate_on

endmodule
