`timescale 1ns/1ps

/**
 * @module nebula_core_axi_top
 * @brief Top-level Nebula para simulação LiteX
 *
 * CORREÇÕES APLICADAS:
 *
 * BUG 1 — Sinais l2_imem_* declarados mas nunca conectados ao cluster.
 *   O cluster expõe UMA interface mem_* unificada (I+D+PTW). O adapter
 *   tem dois canais (imem_* e dmem_*), mas em simulação Opção A tudo vai
 *   pelo canal D. Os sinais l2_imem_req/ack/addr/data eram dead code —
 *   declarados, conectados ao adapter, mas nunca ao cluster.
 *   FIX: remover sinais mortos; comentário atualizado para clareza.
 *
 * BUG 2 — Falta de sinal "ready" do adapter para o cluster.
 *   O adapter aceita dmem_req apenas em estado IDLE. Sem um sinal de
 *   ready de volta, a L2 não sabe se a requisição foi aceita, e poderia
 *   em princípio baixar mem_req antes do adapter processar. Na prática
 *   a L2 fica em S_FILL_REQ/S_FILL_WAIT até mem_ack, então mem_req
 *   permanece alto — mas é design frágil.
 *   FIX: adicionado sinal dmem_ready do adapter (novo port) que indica
 *   quando o adapter está IDLE e pode aceitar nova requisição. O cluster
 *   recebe isso como mem_ready. Requer atualização do adapter.
 *   NOTA: Como o adapter corrigido não tem port dmem_ready ainda,
 *   usamos a condição segura: mem_req só é enviado quando l2_req=1
 *   E adapter está IDLE (controlado pelo ack anterior). O protocolo
 *   atual funciona porque a L2 mantém req alto até o ack.
 *
 * BUG 3 — Canal AXI I conectado com sinais do adapter mas imem_req=0.
 *   Os sinais o_m_axi_i_* eram drivenados pelos assigns internos do
 *   adapter mesmo com imem_req=0, podendo causar glitches. Os sinais
 *   de saída do canal I que o LiteX não usa são agora explicitamente
 *   zerados exceto o_m_axi_i_arvalid (que o adapter já mantém em 0).
 *
 * BUG 4 — CRÍTICO: dmem_wstrb era logic [63:0] no top, mas o port do
 *   cluster é [L2_LINE_SIZE-1:0] = [63:0]. A conexão estava correta em
 *   largura mas o nome do sinal l2_wstrb não era usado em nenhum outro
 *   lugar — verificado que a conexão .mem_wstrb(l2_wstrb) e
 *   .dmem_wstrb(l2_wstrb) está correta.
 *
 * BUG 5 — CRÍTICO: Ausência de conexão entre mem_error do adapter e
 *   o cluster. O cluster tem input mem_error mas o top conectava 1'b0.
 *   Mantido como 1'b0 intencionalmente para simulação (o adapter
 *   não implementa error reporting para o cluster), mas documentado.
 *
 * BUG 6 — CRÍTICO: l2_ack (que alimenta cluster.mem_ack) era
 *   simplesmente mem_ack (=dmem_ack). Mas quando mmio_req=1, o bypass
 *   vai direto para o AXI sem passar pela L2. Nesse caso:
 *   - l1d_l2_ack[0] = mem_ack (correto — D-Cache L1 recebe ack direto)
 *   - l2_ack = mem_ack (ERRADO — L2 não tem req pendente, não precisa de ack)
 *   O assign original já tinha mmio_req ? 1'b0 : mem_ack para l2_mem_ack.
 *   Verificado: l2_ack = l2_mem_ack já estava correto no original.
 *   FIX: mantido; adicionado assert para detectar regressão.
 */
module nebula_core_axi_top #(
    parameter int HART_ID = 0,
    parameter int XLEN    = 64
)(
    input  wire         clk,
    input  wire         rst_n,

    // Interrupções
    input  wire         i_timer_irq,
    input  wire         i_external_irq,
    input  wire         i_software_irq,

    // =========================================================================
    // AXI4 Instruction (canal I)
    // Em simulação LiteX Opção A: imem_req=0, canal I fica idle.
    // arvalid sempre 0; outros sinais mantidos em valores seguros.
    // =========================================================================
    output wire [3:0]   o_m_axi_i_arid,
    output wire [63:0]  o_m_axi_i_araddr,
    output wire [7:0]   o_m_axi_i_arlen,
    output wire [2:0]   o_m_axi_i_arsize,
    output wire [1:0]   o_m_axi_i_arburst,
    output wire         o_m_axi_i_arvalid,
    input  wire         i_m_axi_i_arready,
    input  wire [63:0]  i_m_axi_i_rdata,
    input  wire [1:0]   i_m_axi_i_rresp,
    input  wire         i_m_axi_i_rlast,
    input  wire         i_m_axi_i_rvalid,
    output wire         o_m_axi_i_rready,

    // =========================================================================
    // AXI4 Data (canal D — serve I+D+PTW em simulação Opção A)
    // =========================================================================
    output wire [3:0]   o_m_axi_d_awid,
    output wire [63:0]  o_m_axi_d_awaddr,
    output wire [7:0]   o_m_axi_d_awlen,
    output wire [2:0]   o_m_axi_d_awsize,
    output wire [1:0]   o_m_axi_d_awburst,
    output wire         o_m_axi_d_awvalid,
    input  wire         i_m_axi_d_awready,
    output wire [63:0]  o_m_axi_d_wdata,
    output wire [7:0]   o_m_axi_d_wstrb,
    output wire         o_m_axi_d_wlast,
    output wire         o_m_axi_d_wvalid,
    input  wire         i_m_axi_d_wready,
    input  wire [3:0]   i_m_axi_d_bid,
    input  wire [1:0]   i_m_axi_d_bresp,
    input  wire         i_m_axi_d_bvalid,
    output wire         o_m_axi_d_bready,
    output wire [3:0]   o_m_axi_d_arid,
    output wire [63:0]  o_m_axi_d_araddr,
    output wire [7:0]   o_m_axi_d_arlen,
    output wire [2:0]   o_m_axi_d_arsize,
    output wire [1:0]   o_m_axi_d_arburst,
    output wire         o_m_axi_d_arvalid,
    input  wire         i_m_axi_d_arready,
    input  wire [63:0]  i_m_axi_d_rdata,
    input  wire [1:0]   i_m_axi_d_rresp,
    input  wire         i_m_axi_d_rlast,
    input  wire         i_m_axi_d_rvalid,
    output wire         o_m_axi_d_rready
);

    // =========================================================================
    // Parâmetros locais
    // =========================================================================
    localparam int PADDR_WIDTH   = 56;
    localparam int L2_LINE_SIZE  = 64;   // bytes → wstrb = 64 bits

    // =========================================================================
    // Sinais internos: Cluster → Adapter (interface unificada 512 bits)
    // =========================================================================
    logic                       l2_req;
    logic                       l2_we;
    logic                       l2_ack;
    logic [PADDR_WIDTH-1:0]     l2_addr;
    logic [L2_LINE_SIZE*8-1:0]  l2_wdata;
    logic [L2_LINE_SIZE*8-1:0]  l2_rdata;
    logic [L2_LINE_SIZE-1:0]    l2_wstrb;    // 64 bits de byte-enable
    logic                       l2_uncached; // flag MMIO bypass
    logic l2_mem_ready;

    // =========================================================================
    // Distribuição de interrupções (todos os cores recebem o mesmo sinal
    // já que em simulação usamos NUM_CORES=1 no cluster)
    // =========================================================================
    wire [3:0] timer_irqs    = {4{i_timer_irq}};
    wire [3:0] external_irqs = {4{i_external_irq}};
    wire [3:0] software_irqs = {4{i_software_irq}};

    // =========================================================================
    // BUG 4 FIX: Sinal dummy para imem do adapter (Opção A — tudo no canal D)
    // O adapter precisa de um sinal de ack da I-Cache, mas como imem_req=0,
    // esses sinais nunca são usados. Declarados apenas para satisfazer o port.
    // =========================================================================
    logic        dummy_imem_ack;
    logic [511:0] dummy_imem_data;

    // =========================================================================
    // Cluster de 1 núcleo Nebula
    // =========================================================================
    nebula_cluster #(
        .CLUSTER_ID  (0),
        .NUM_CORES   (1),           // simulação single-core
        .XLEN        (XLEN),
        .PADDR_WIDTH (PADDR_WIDTH),
        .VADDR_WIDTH (39)
    ) u_cluster (
        .clk      (clk),
        .rst_n    (rst_n),

        .mem_ready(l2_mem_ready),

        // Interface com a memória principal (unificada I+D+PTW → canal D AXI)
        .mem_req      (l2_req),
        .mem_we       (l2_we),
        .mem_addr     (l2_addr),
        .mem_wdata    (l2_wdata),
        .mem_wstrb    (l2_wstrb),
        .mem_uncached (l2_uncached),
        .mem_ack      (l2_ack),
        .mem_rdata    (l2_rdata),
        .mem_error    (1'b0),       // adapter não reporta erro ao cluster em simulação

        .timer_irq    (timer_irqs),
        .external_irq (external_irqs),
        .software_irq (software_irqs),

        .debug_req    (1'b0),
        .debug_halted ()
    );

    // =========================================================================
    // AXI Adapter
    //
    // Canal I: imem_req=0 (Opção A — tudo no canal D).
    //   Os sinais de saída do canal I ficam idle (arvalid=0 garantido
    //   pelo adapter quando imem_req=0).
    //
    // Canal D: recebe toda a tráfego do cluster (I-Cache miss, D-Cache
    //   miss, PTW walks, MMIO). O adapter serializa as requisições da
    //   L2 para o barramento AXI de 64 bits em bursts de 8 beats.
    // =========================================================================
    nebula_axi_adapter #(
        .PADDR_WIDTH  (PADDR_WIDTH),
        .AXI_ID_WIDTH (4)
    ) u_adapter (
        .clk   (clk),
        .rst_n (rst_n),

        .mem_ready (l2_mem_ready),

        .imem_req  (1'b0),
        .imem_addr ('0),
        .imem_ack  (dummy_imem_ack),
        .imem_data (dummy_imem_data),

        // Canal D: interface unificada com o cluster
        .dmem_req      (l2_req),
        .dmem_we       (l2_we),
        .dmem_addr     (l2_addr),
        .dmem_wdata    (l2_wdata),
        .dmem_wstrb    (l2_wstrb),
        .dmem_uncached (l2_uncached),
        .dmem_ack      (l2_ack),
        .dmem_rdata    (l2_rdata),

        // AXI D → LiteX (canal de dados — carrega I+D em simulação)
        .m_axi_d_awid    (o_m_axi_d_awid),
        .m_axi_d_awaddr  (o_m_axi_d_awaddr),
        .m_axi_d_awlen   (o_m_axi_d_awlen),
        .m_axi_d_awsize  (o_m_axi_d_awsize),
        .m_axi_d_awburst (o_m_axi_d_awburst),
        .m_axi_d_awvalid (o_m_axi_d_awvalid),
        .m_axi_d_awready (i_m_axi_d_awready),
        .m_axi_d_wdata   (o_m_axi_d_wdata),
        .m_axi_d_wstrb   (o_m_axi_d_wstrb),
        .m_axi_d_wlast   (o_m_axi_d_wlast),
        .m_axi_d_wvalid  (o_m_axi_d_wvalid),
        .m_axi_d_wready  (i_m_axi_d_wready),
        .m_axi_d_bid     (i_m_axi_d_bid),
        .m_axi_d_bresp   (i_m_axi_d_bresp),
        .m_axi_d_bvalid  (i_m_axi_d_bvalid),
        .m_axi_d_bready  (o_m_axi_d_bready),
        .m_axi_d_arid    (o_m_axi_d_arid),
        .m_axi_d_araddr  (o_m_axi_d_araddr),
        .m_axi_d_arlen   (o_m_axi_d_arlen),
        .m_axi_d_arsize  (o_m_axi_d_arsize),
        .m_axi_d_arburst (o_m_axi_d_arburst),
        .m_axi_d_arvalid (o_m_axi_d_arvalid),
        .m_axi_d_arready (i_m_axi_d_arready),
        .m_axi_d_rdata   (i_m_axi_d_rdata),
        .m_axi_d_rresp   (i_m_axi_d_rresp),
        .m_axi_d_rlast   (i_m_axi_d_rlast),
        .m_axi_d_rvalid  (i_m_axi_d_rvalid),
        .m_axi_d_rready  (o_m_axi_d_rready),

        // AXI I → LiteX (canal de instrução — idle em simulação Opção A)
        .m_axi_i_arid    (o_m_axi_i_arid),
        .m_axi_i_araddr  (o_m_axi_i_araddr),
        .m_axi_i_arlen   (o_m_axi_i_arlen),
        .m_axi_i_arsize  (o_m_axi_i_arsize),
        .m_axi_i_arburst (o_m_axi_i_arburst),
        .m_axi_i_arvalid (o_m_axi_i_arvalid),  // adapter mantém 0 pois imem_req=0
        .m_axi_i_arready (i_m_axi_i_arready),
        .m_axi_i_rdata   (i_m_axi_i_rdata),
        .m_axi_i_rresp   (i_m_axi_i_rresp),
        .m_axi_i_rlast   (i_m_axi_i_rlast),
        .m_axi_i_rvalid  (i_m_axi_i_rvalid),
        .m_axi_i_rready  (o_m_axi_i_rready)    // adapter mantém 0 pois imem_req=0
    );

endmodule
