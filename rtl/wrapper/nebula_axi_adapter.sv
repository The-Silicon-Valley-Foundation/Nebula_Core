`timescale 1ns/1ps

/**
 * @module nebula_axi_adapter
 * @brief Adaptador Nebula (512-bit) <-> AXI4 (64-bit)
 *
 * CORREÇÕES APLICADAS:
 * 1. Canal I-Cache (imem_*) implementado com FSM paralela — antes estava
 *    com imem_ack=0 e imem_data=0 hardcoded, o que impedia qualquer fetch
 *    de instrução via AXI. Isso travava o núcleo permanentemente.
 *
 * 2. FSM D-Cache: bugs de cnt corrigidos (incremento em R_WAIT_DATA/W_RESP,
 *    base_addr_reg preservado).
 *
 * 3. Arbitração simples: D-Cache tem prioridade sobre I-Cache.
 *    Quando ambos requisitam simultaneamente, D-Cache é atendido primeiro.
 */
module nebula_axi_adapter #(
    parameter int PADDR_WIDTH  = 56,
    parameter int AXI_ID_WIDTH = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // I-Cache (512-bit)
    input  wire                     imem_req,
    input  wire [PADDR_WIDTH-1:0]   imem_addr,
    output logic                    imem_ack,
    output logic [511:0]            imem_data,

    // D-Cache (512-bit)
    input  wire                     dmem_req,
    input  wire                     dmem_we,
    input  wire [PADDR_WIDTH-1:0]   dmem_addr,
    input  wire [511:0]             dmem_wdata,
    output logic                    dmem_ack,
    output logic [511:0]            dmem_rdata,

    // AXI4 Instruction
    output logic [AXI_ID_WIDTH-1:0] m_axi_i_arid,
    output logic [PADDR_WIDTH-1:0]  m_axi_i_araddr,
    output logic [7:0]              m_axi_i_arlen,
    output logic [2:0]              m_axi_i_arsize,
    output logic [1:0]              m_axi_i_arburst,
    output logic                    m_axi_i_arvalid,
    input  wire                     m_axi_i_arready,
    input  wire [63:0]              m_axi_i_rdata,
    input  wire [1:0]               m_axi_i_rresp,
    input  wire                     m_axi_i_rlast,
    input  wire                     m_axi_i_rvalid,
    output logic                    m_axi_i_rready,

    // AXI4 Data
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
    output logic                    m_axi_d_rready
);

    typedef enum logic [3:0] {
        IDLE,
        R_ADDR, R_WAIT_DATA, R_NEXT,
        W_ADDR, W_DATA, W_RESP, W_NEXT
    } state_t;

    // =========================================================================
    // D-Cache FSM
    // =========================================================================
    state_t            d_state;
    logic [3:0]        d_cnt;
    logic [511:0]      d_buf;
    logic [PADDR_WIDTH-1:0] d_base;

    assign m_axi_d_arid    = '0;
    assign m_axi_d_arlen   = 8'd0;
    assign m_axi_d_arsize  = 3'b011;
    assign m_axi_d_arburst = 2'b01;
    assign m_axi_d_awid    = '0;
    assign m_axi_d_awlen   = 8'd0;
    assign m_axi_d_awsize  = 3'b011;
    assign m_axi_d_awburst = 2'b01;
    assign m_axi_d_wstrb   = 8'hFF;
    assign m_axi_d_wlast   = 1'b1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_state         <= IDLE;
            d_cnt           <= '0;
            d_base          <= '0;
            d_buf           <= '0;
            dmem_ack        <= 1'b0;
            dmem_rdata      <= '0;
            m_axi_d_arvalid <= 1'b0;
            m_axi_d_araddr  <= '0;
            m_axi_d_rready  <= 1'b0;
            m_axi_d_awvalid <= 1'b0;
            m_axi_d_awaddr  <= '0;
            m_axi_d_wvalid  <= 1'b0;
            m_axi_d_wdata   <= '0;
            m_axi_d_bready  <= 1'b0;
        end else begin
            dmem_ack <= 1'b0;

            case (d_state)
                IDLE: begin
                    d_cnt <= '0;
                    if (dmem_req) begin
                        d_base <= dmem_addr;
                        if (!dmem_we) begin
                            m_axi_d_araddr  <= dmem_addr;
                            m_axi_d_arvalid <= 1'b1;
                            d_state         <= R_ADDR;
                        end else begin
                            m_axi_d_awaddr  <= dmem_addr;
                            m_axi_d_awvalid <= 1'b1;
                            m_axi_d_wdata   <= dmem_wdata[63:0];
                            d_state         <= W_ADDR;
                        end
                    end
                end
                R_ADDR: begin
                    if (m_axi_d_arready) begin
                        m_axi_d_arvalid <= 1'b0;
                        m_axi_d_rready  <= 1'b1;
                        d_state         <= R_WAIT_DATA;
                    end
                end
                R_WAIT_DATA: begin
                    if (m_axi_d_rvalid) begin
                        m_axi_d_rready  <= 1'b0;
                        d_buf[d_cnt * 64 +: 64] <= m_axi_d_rdata;
                        d_cnt  <= d_cnt + 1;
                        d_state <= R_NEXT;
                    end
                end
                R_NEXT: begin
                    if (d_cnt == 4'd8) begin
                        dmem_rdata <= d_buf;
                        dmem_ack   <= 1'b1;
                        d_state    <= IDLE;
                    end else begin
                        m_axi_d_araddr  <= d_base + {d_cnt, 3'b000};
                        m_axi_d_arvalid <= 1'b1;
                        d_state         <= R_ADDR;
                    end
                end
                W_ADDR: begin
                    if (m_axi_d_awready) begin
                        m_axi_d_awvalid <= 1'b0;
                        m_axi_d_wvalid  <= 1'b1;
                        d_state         <= W_DATA;
                    end
                end
                W_DATA: begin
                    if (m_axi_d_wready) begin
                        m_axi_d_wvalid <= 1'b0;
                        m_axi_d_bready <= 1'b1;
                        d_state        <= W_RESP;
                    end
                end
                W_RESP: begin
                    if (m_axi_d_bvalid) begin
                        m_axi_d_bready <= 1'b0;
                        d_cnt  <= d_cnt + 1;
                        d_state <= W_NEXT;
                    end
                end
                W_NEXT: begin
                    if (d_cnt == 4'd8) begin
                        dmem_ack <= 1'b1;
                        d_state  <= IDLE;
                    end else begin
                        m_axi_d_awaddr  <= d_base + {d_cnt, 3'b000};
                        m_axi_d_wdata   <= dmem_wdata[d_cnt * 64 +: 64];
                        m_axi_d_awvalid <= 1'b1;
                        d_state         <= W_ADDR;
                    end
                end
                default: d_state <= IDLE;
            endcase
        end
    end

    // =========================================================================
    // FIX 1: I-Cache FSM (implementada, não tie-off)
    // =========================================================================
    state_t            i_state;
    logic [3:0]        i_cnt;
    logic [511:0]      i_buf;
    logic [PADDR_WIDTH-1:0] i_base;

    assign m_axi_i_arid    = '0;
    assign m_axi_i_arlen   = 8'd0;
    assign m_axi_i_arsize  = 3'b011;
    assign m_axi_i_arburst = 2'b01;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_state         <= IDLE;
            i_cnt           <= '0;
            i_base          <= '0;
            i_buf           <= '0;
            imem_ack        <= 1'b0;
            imem_data       <= '0;
            m_axi_i_arvalid <= 1'b0;
            m_axi_i_araddr  <= '0;
            m_axi_i_rready  <= 1'b0;
        end else begin
            imem_ack <= 1'b0;

            case (i_state)
                IDLE: begin
                    i_cnt <= '0;
                    // D-Cache tem prioridade — só atende I-Cache se D estiver idle
                    if (imem_req && d_state == IDLE) begin
                        i_base          <= imem_addr;
                        m_axi_i_araddr  <= imem_addr;
                        m_axi_i_arvalid <= 1'b1;
                        i_state         <= R_ADDR;
                    end
                end
                R_ADDR: begin
                    if (m_axi_i_arready) begin
                        m_axi_i_arvalid <= 1'b0;
                        m_axi_i_rready  <= 1'b1;
                        i_state         <= R_WAIT_DATA;
                    end
                end
                R_WAIT_DATA: begin
                    if (m_axi_i_rvalid) begin
                        m_axi_i_rready  <= 1'b0;
                        i_buf[i_cnt * 64 +: 64] <= m_axi_i_rdata;
                        i_cnt   <= i_cnt + 1;
                        i_state <= R_NEXT;
                    end
                end
                R_NEXT: begin
                    if (i_cnt == 4'd8) begin
                        imem_data <= i_buf;
                        imem_ack  <= 1'b1;
                        i_state   <= IDLE;
                    end else begin
                        m_axi_i_araddr  <= i_base + {i_cnt, 3'b000};
                        m_axi_i_arvalid <= 1'b1;
                        i_state         <= R_ADDR;
                    end
                end
                default: i_state <= IDLE;
            endcase
        end
    end

endmodule
