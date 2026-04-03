`timescale 1ns/1ps
`default_nettype none

/**
 * @module tlb_sv39
 * @brief TLB para Sv39
 *
 * CORREÇÕES APLICADAS:
 * 1. BUG CRÍTICO — bits A/D: o código anterior setava lookup_page_fault=1
 *    quando !found_entry.a ou (!d && store), causando loop infinito no boot
 *    do Linux. O hardware com writeback A/D (ptw_sv39.sv já implementado)
 *    deve operar assim:
 *      - Se A/D precisam ser setados → invalida a entrada no TLB (dá miss)
 *        para que o PTW faça o walk, atualize a PTE na memória e reinserira
 *        a entrada com A/D corretos.
 *      - NÃO gera page fault por ausência de A/D — isso era comportamento
 *        de software-managed A/D, não hardware-managed.
 *    FIX: quando need_set_a ou need_set_d, lookup_hit=0 (forçar miss),
 *    e a invalidação ocorre no always_ff.
 *
 * 2. access_fault não propaga lookup_page_fault — page_fault é sinalizado
 *    apenas para violações reais de permissão (R/W/X/U).
 */
module tlb_sv39 #(
    parameter int VPN_WIDTH  = 27,
    parameter int PPN_WIDTH  = 44,
    parameter int TLB_ENTRIES = 64,
    parameter int ASID_WIDTH = 16
)(
    input  wire clk,
    input  wire rst_n,

    // Lookup
    input  wire                     lookup_valid,
    input  wire [VPN_WIDTH-1:0]     lookup_vpn,
    input  wire [ASID_WIDTH-1:0]    lookup_asid,
    input  wire [1:0]               lookup_priv,
    input  wire                     lookup_is_store,
    input  wire                     lookup_is_exec,
    input  wire                     mstatus_sum,
    input  wire                     mstatus_mxr,

    output logic                    lookup_hit,
    output logic [PPN_WIDTH-1:0]    lookup_ppn,
    output logic                    lookup_page_fault,
    output logic                    access_fault,

    output logic                    need_set_a,
    output logic                    need_set_d,
    output logic [VPN_WIDTH-1:0]    fault_vpn,

    // Insert
    input  wire                     insert_valid,
    input  wire [VPN_WIDTH-1:0]     insert_vpn,
    input  wire [PPN_WIDTH-1:0]     insert_ppn,
    input  wire [ASID_WIDTH-1:0]    insert_asid,
    input  wire [1:0]               insert_page_size,
    input  wire                     insert_r,
    input  wire                     insert_w,
    input  wire                     insert_x,
    input  wire                     insert_u,
    input  wire                     insert_g,
    input  wire                     insert_a,
    input  wire                     insert_d,

    // Invalidate
    input  wire                     invalidate_all,
    input  wire                     invalidate_by_asid,
    input  wire                     invalidate_by_addr,
    input  wire                     invalidate_by_both,
    input  wire [ASID_WIDTH-1:0]    invalidate_asid,
    input  wire [VPN_WIDTH-1:0]     invalidate_vpn
);

    localparam PRIV_U = 2'b00;
    localparam PRIV_S = 2'b01;
    localparam PRIV_M = 2'b11;

    typedef struct packed {
        logic                   valid;
        logic [VPN_WIDTH-1:0]   vpn;
        logic [PPN_WIDTH-1:0]   ppn;
        logic [ASID_WIDTH-1:0]  asid;
        logic [1:0]             page_size;
        logic                   r, w, x, u, g, a, d;
    } tlb_entry_t;

    tlb_entry_t entries [0:TLB_ENTRIES-1];
    logic [$clog2(TLB_ENTRIES)-1:0] replace_ptr;

    function automatic logic vpn_match(
        input [VPN_WIDTH-1:0] v1, v2,
        input [1:0] ps
    );
        case (ps)
            2'b00: return (v1 == v2);
            2'b01: return (v1[VPN_WIDTH-1:9] == v2[VPN_WIDTH-1:9]);
            2'b10: return (v1[VPN_WIDTH-1:18] == v2[VPN_WIDTH-1:18]);
            default: return (v1 == v2);
        endcase
    endfunction

    function automatic [PPN_WIDTH-1:0] build_ppn(
        input [PPN_WIDTH-1:0] ep,
        input [VPN_WIDTH-1:0] vpn,
        input [1:0] ps
    );
        case (ps)
            2'b00: return ep;
            2'b01: return {ep[PPN_WIDTH-1:9],  vpn[8:0]};
            2'b10: return {ep[PPN_WIDTH-1:18], vpn[17:0]};
            default: return ep;
        endcase
    endfunction

    // =========================================================================
    // Lookup (combinacional)
    // =========================================================================
    logic                          found;
    logic [$clog2(TLB_ENTRIES)-1:0] found_idx;
    tlb_entry_t                    found_entry;

    // Sinal para invalidar entrada com A/D desatualizado (registrado)
    logic                          need_invalidate;
    logic [$clog2(TLB_ENTRIES)-1:0] invalidate_idx;

    always_comb begin
        lookup_hit        = 1'b0;
        lookup_ppn        = '0;
        lookup_page_fault = 1'b0;
        access_fault      = 1'b0;
        need_set_a        = 1'b0;
        need_set_d        = 1'b0;
        fault_vpn         = lookup_vpn;
        found             = 1'b0;
        found_idx         = '0;
        found_entry       = '0;
        need_invalidate   = 1'b0;
        invalidate_idx    = '0;

        if (lookup_valid) begin
            for (int i = 0; i < TLB_ENTRIES; i++) begin
                if (entries[i].valid &&
                    vpn_match(entries[i].vpn, lookup_vpn, entries[i].page_size) &&
                    (entries[i].g || entries[i].asid == lookup_asid)) begin
                    found       = 1'b1;
                    found_idx   = i[$clog2(TLB_ENTRIES)-1:0];
                    found_entry = entries[i];
                end
            end

            if (found) begin
                // Verificar permissões
                if (lookup_priv == PRIV_U && !found_entry.u) begin
                    access_fault = 1'b1;
                end else if (lookup_priv == PRIV_S && found_entry.u && !mstatus_sum) begin
                    access_fault = 1'b1;
                end else if (lookup_is_store && !found_entry.w) begin
                    access_fault = 1'b1;
                end else if (lookup_is_exec && !found_entry.x) begin
                    access_fault = 1'b1;
                end else if (!lookup_is_store && !lookup_is_exec &&
                             !found_entry.r && !(found_entry.x && mstatus_mxr)) begin
                    access_fault = 1'b1;
                end

                if (access_fault) begin
                    // Violação de permissão = page fault
                    lookup_page_fault = 1'b1;
                    lookup_hit        = 1'b0;  // não retorna PPN em caso de fault
                end else begin
                    // FIX 1: verificar A/D APÓS permissões passarem
                    // Se A/D precisam ser setados, dar miss (não page fault)
                    // para que o PTW walk aconteça e atualize a PTE.
                    if (!found_entry.a || (lookup_is_store && !found_entry.d)) begin
                        need_set_a      = !found_entry.a;
                        need_set_d      = lookup_is_store && !found_entry.d;
                        // Miss: não hit, sem fault — PTW vai atualizar
                        lookup_hit        = 1'b0;
                        need_invalidate   = 1'b1;
                        invalidate_idx    = found_idx;
                    end else begin
                        // Hit válido com A/D corretos
                        lookup_hit = 1'b1;
                        lookup_ppn = build_ppn(found_entry.ppn, lookup_vpn, found_entry.page_size);
                    end
                end
            end
            // Miss (found=0): PTW walk necessário, sem sinalizar fault
        end
    end

    // =========================================================================
    // Sequential
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < TLB_ENTRIES; i++)
                entries[i].valid <= 1'b0;
            replace_ptr <= '0;
        end else begin
            // Invalidação por A/D desatualizado (forçar PTW walk)
            if (need_invalidate)
                entries[invalidate_idx].valid <= 1'b0;

            // SFENCE.VMA
            if (invalidate_all) begin
                for (int i = 0; i < TLB_ENTRIES; i++)
                    entries[i].valid <= 1'b0;
            end else if (invalidate_by_asid) begin
                for (int i = 0; i < TLB_ENTRIES; i++) begin
                    if (entries[i].valid && !entries[i].g &&
                        entries[i].asid == invalidate_asid)
                        entries[i].valid <= 1'b0;
                end
            end else if (invalidate_by_addr) begin
                for (int i = 0; i < TLB_ENTRIES; i++) begin
                    if (entries[i].valid &&
                        vpn_match(entries[i].vpn, invalidate_vpn, entries[i].page_size))
                        entries[i].valid <= 1'b0;
                end
            end else if (invalidate_by_both) begin
                for (int i = 0; i < TLB_ENTRIES; i++) begin
                    if (entries[i].valid && !entries[i].g &&
                        entries[i].asid == invalidate_asid &&
                        vpn_match(entries[i].vpn, invalidate_vpn, entries[i].page_size))
                        entries[i].valid <= 1'b0;
                end
            end else if (insert_valid) begin
                entries[replace_ptr].valid     <= 1'b1;
                entries[replace_ptr].vpn       <= insert_vpn;
                entries[replace_ptr].ppn       <= insert_ppn;
                entries[replace_ptr].asid      <= insert_asid;
                entries[replace_ptr].page_size <= insert_page_size;
                entries[replace_ptr].r         <= insert_r;
                entries[replace_ptr].w         <= insert_w;
                entries[replace_ptr].x         <= insert_x;
                entries[replace_ptr].u         <= insert_u;
                entries[replace_ptr].g         <= insert_g;
                entries[replace_ptr].a         <= insert_a;
                entries[replace_ptr].d         <= insert_d;

                if (replace_ptr == TLB_ENTRIES - 1)
                    replace_ptr <= '0;
                else
                    replace_ptr <= replace_ptr + 1;
            end
        end
    end

endmodule
