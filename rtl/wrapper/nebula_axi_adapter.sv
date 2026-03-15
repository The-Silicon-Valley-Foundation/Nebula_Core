`timescale 1ns/1ps

/**
 * @module nebula_axi_adapter
 * @brief Adaptador entre interface Nebula (512-bit) e AXI4 (64-bit)
 *
 * CORREÇÕES APLICADAS:
 * 1. Endereço base salvo em registrador (base_addr_reg) para evitar
 *    acumulação incorreta sobre o endereço AXI modificado.
 * 2. cnt incrementado em R_WAIT_DATA (não em R_NEXT) para que o
 *    teste de fim de burst use o valor já atualizado.
 * 3. Write path corrigido: endereço e dados calculados a partir
 *    do base_addr_reg + (cnt * 8).
 */
module nebula_axi_adapter #(
    parameter int PADDR_WIDTH = 56,
    parameter int AXI_ID_WIDTH = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Interface Nebula (Cache - 512 bits)
    input  wire                     imem_req,
    input  wire [PADDR_WIDTH-1:0]   imem_addr,
    output logic                    imem_ack,
    output logic [511:0]            imem_data,

    input  wire                     dmem_req,
    input  wire                     dmem_we,
    input  wire [PADDR_WIDTH-1:0]   dmem_addr,
    input  wire [511:0]             dmem_wdata,
    output logic                    dmem_ack,
    output logic [511:0]            dmem_rdata,

    // Interface AXI4 Master - 64 bits
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
        R_ADDR,
        R_WAIT_DATA,
        R_NEXT,
        W_ADDR,
        W_DATA,
        W_RESP,
        W_NEXT
    } state_t;

    state_t state;

    // FIX 1: cnt agora é incrementado em R_WAIT_DATA / W_RESP
    logic [3:0]              cnt;
    logic [511:0]            data_buf;

    // FIX 2: registrador base para preservar o endereço original
    logic [PADDR_WIDTH-1:0]  base_addr_reg;

    // =========================================================================
    // Configurações AXI Fixas
    // =========================================================================
    // Cada transação é um burst de 1 beat de 64 bits (8 bytes)
    assign m_axi_d_arid    = '0;
    assign m_axi_d_arlen   = 8'd0;       // 1 beat por transação
    assign m_axi_d_arsize  = 3'b011;     // 8 bytes por beat
    assign m_axi_d_arburst = 2'b01;      // INCR

    assign m_axi_d_awid    = '0;
    assign m_axi_d_awlen   = 8'd0;
    assign m_axi_d_awsize  = 3'b011;
    assign m_axi_d_awburst = 2'b01;

    assign m_axi_d_wstrb   = 8'hFF;
    assign m_axi_d_wlast   = 1'b1;       // sempre last (burst de 1)

    // =========================================================================
    // FSM Principal
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= IDLE;
            cnt             <= '0;
            base_addr_reg   <= '0;
            data_buf        <= '0;
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
            // Limpar ack por padrão (pulso de 1 ciclo)
            dmem_ack <= 1'b0;

            case (state)
                // =============================================================
                IDLE: begin
                    cnt <= '0;
                    if (dmem_req) begin
                        // FIX 3: salvar endereço base UMA VEZ
                        base_addr_reg <= dmem_addr;

                        if (!dmem_we) begin
                            // READ: iniciar primeira transação AR
                            m_axi_d_araddr  <= dmem_addr;
                            m_axi_d_arvalid <= 1'b1;
                            state           <= R_ADDR;
                        end else begin
                            // WRITE: iniciar primeira transação AW
                            m_axi_d_awaddr <= dmem_addr;
                            m_axi_d_awvalid <= 1'b1;
                            // Dado do beat 0
                            m_axi_d_wdata  <= dmem_wdata[63:0];
                            state          <= W_ADDR;
                        end
                    end
                end

                // =============================================================
                // READ PATH
                // =============================================================
                R_ADDR: begin
                    if (m_axi_d_arready) begin
                        m_axi_d_arvalid <= 1'b0;
                        m_axi_d_rready  <= 1'b1;
                        state           <= R_WAIT_DATA;
                    end
                end

                R_WAIT_DATA: begin
                    if (m_axi_d_rvalid) begin
                        m_axi_d_rready <= 1'b0;

                        // Armazenar beat atual
                        data_buf[cnt * 64 +: 64] <= m_axi_d_rdata;

                        // FIX 4: incrementar cnt AQUI
                        cnt <= cnt + 1;

                        state <= R_NEXT;
                    end
                end

                R_NEXT: begin
                    // FIX 5: cnt já foi incrementado em R_WAIT_DATA
                    if (cnt == 4'd8) begin
                        // Todos os 8 beats recebidos
                        dmem_rdata <= data_buf;
                        dmem_ack   <= 1'b1;
                        state      <= IDLE;
                    end else begin
                        // FIX 6: próximo endereço = base + cnt * 8
                        m_axi_d_araddr  <= base_addr_reg + {cnt, 3'b000};
                        m_axi_d_arvalid <= 1'b1;
                        state           <= R_ADDR;
                    end
                end

                // =============================================================
                // WRITE PATH
                // =============================================================
                W_ADDR: begin
                    if (m_axi_d_awready) begin
                        m_axi_d_awvalid <= 1'b0;
                        m_axi_d_wvalid  <= 1'b1;
                        state           <= W_DATA;
                    end
                end

                W_DATA: begin
                    if (m_axi_d_wready) begin
                        m_axi_d_wvalid <= 1'b0;
                        m_axi_d_bready <= 1'b1;
                        state          <= W_RESP;
                    end
                end

                W_RESP: begin
                    if (m_axi_d_bvalid) begin
                        m_axi_d_bready <= 1'b0;

                        // FIX 7: incrementar cnt AQUI
                        cnt   <= cnt + 1;
                        state <= W_NEXT;
                    end
                end

                W_NEXT: begin
                    // FIX 8: cnt já foi incrementado em W_RESP
                    if (cnt == 4'd8) begin
                        dmem_ack <= 1'b1;
                        state    <= IDLE;
                    end else begin
                        // FIX 9: endereço e dado calculados a partir da base
                        m_axi_d_awaddr  <= base_addr_reg + {cnt, 3'b000};
                        m_axi_d_wdata   <= dmem_wdata[cnt * 64 +: 64];
                        m_axi_d_awvalid <= 1'b1;
                        state           <= W_ADDR;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    // =========================================================================
    // Tie-offs: I-Cache master não utilizado
    // =========================================================================
    assign m_axi_i_arid    = '0;
    assign m_axi_i_araddr  = '0;
    assign m_axi_i_arlen   = 8'd0;
    assign m_axi_i_arsize  = 3'b000;
    assign m_axi_i_arburst = 2'b00;
    assign m_axi_i_arvalid = 1'b0;
    assign m_axi_i_rready  = 1'b0;
    assign imem_ack        = 1'b0;
    assign imem_data       = '0;

endmodule
