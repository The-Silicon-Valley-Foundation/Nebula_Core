`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module l2_cache
 * @brief Cache L2 Compartilhado - 512KB, 8-way, 4 bancos
 *
 * CORREÇÕES APLICADAS:
 * 1. Declarações "logic" dentro de always_ff (S_HIT, S_SNOOP_REQ,
 *    S_SNOOP_WAIT, S_FILL_DONE) removidas — ilegal em SystemVerilog
 *    para síntese. Promovidas a sinais de módulo indexados por banco.
 *    Verilator >= 5.036 e Vivado rejeitavam BLKSEQ nestes casos.
 *
 * 2. Assignments blocking (=) dentro de always_ff no reset convertidos
 *    para non-blocking (<=).
 */
module l2_cache #(
    parameter int NUM_CORES  = 4,
    parameter int PADDR_WIDTH = 56,
    parameter int LINE_SIZE  = 64,
    parameter int NUM_WAYS   = 8,
    parameter int SIZE_KB    = 512,
    parameter int NUM_BANKS  = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  l2_req_t                 l1_req  [NUM_CORES],
    output l2_resp_t                l1_resp [NUM_CORES],

    output snoop_req_t              snoop_req  [NUM_CORES],
    input  snoop_resp_t             snoop_resp [NUM_CORES],

    output logic                    mem_req,
    output logic                    mem_we,
    output logic [PADDR_WIDTH-1:0]  mem_addr,
    output logic [LINE_SIZE*8-1:0]  mem_wdata,
    input  wire                     mem_ack,
    input  wire [LINE_SIZE*8-1:0]   mem_rdata,
    input  wire                     mem_error,
    input  logic                    mem_ready
);

    localparam int TOTAL_SETS    = (SIZE_KB * 1024) / (NUM_WAYS * LINE_SIZE);
    localparam int SETS_PER_BANK = TOTAL_SETS / NUM_BANKS;
    localparam int OFFSET_BITS   = $clog2(LINE_SIZE);
    localparam int BANK_BITS     = $clog2(NUM_BANKS);
    localparam int INDEX_BITS    = $clog2(SETS_PER_BANK);
    localparam int TAG_BITS      = PADDR_WIDTH - INDEX_BITS - BANK_BITS - OFFSET_BITS;
    localparam int LINE_BITS     = LINE_SIZE * 8;

    typedef struct packed {
        logic                   valid;
        logic                   dirty;
        cache_state_t           state;
        logic [TAG_BITS-1:0]    tag;
        logic [NUM_CORES-1:0]   sharers;
    } l2_tag_t;

    l2_tag_t              tag_array  [NUM_BANKS][SETS_PER_BANK][NUM_WAYS];
    logic [LINE_BITS-1:0] data_array [NUM_BANKS][SETS_PER_BANK][NUM_WAYS];
    logic [6:0]           plru_bits  [NUM_BANKS][SETS_PER_BANK];
    logic [$clog2(NUM_BANKS)-1:0] mem_owner;

    typedef enum logic [3:0] {
        S_IDLE, S_SELECT, S_TAG_CHECK, S_HIT, S_SNOOP_REQ, S_SNOOP_WAIT,
        S_EVICT_WB, S_EVICT_WAIT, S_FILL_REQ, S_FILL_WAIT, S_FILL_DONE, S_RESPOND
    } state_t;

    state_t state [NUM_BANKS];

    logic [$clog2(NUM_CORES)-1:0] selected_core [NUM_BANKS];
    l2_req_t     req_reg       [NUM_BANKS];
    logic [LINE_BITS-1:0] line_buf [NUM_BANKS];
    logic [NUM_CORES-1:0] snoop_pend [NUM_BANKS];
    logic [LINE_BITS-1:0] snoop_data_buf [NUM_BANKS];
    logic                 snoop_has_data [NUM_BANKS];

    logic [NUM_WAYS-1:0] hit_way     [NUM_BANKS];
    logic                cache_hit   [NUM_BANKS];
    logic [2:0]          hit_way_idx [NUM_BANKS];
    logic [2:0]          victim_way  [NUM_BANKS];

    // FIX 1: sinais extraídos dos always_ff para nível de módulo
    // Indexados por banco para evitar conflito de nomes
    logic [INDEX_BITS-1:0] b_req_index [NUM_BANKS];
    logic [TAG_BITS-1:0]   b_req_tag   [NUM_BANKS];

    // Para S_HIT: sinais computados combinacionalmente
    logic [NUM_CORES-1:0]  b_others      [NUM_BANKS];
    logic                  b_has_others  [NUM_BANKS];
    // Para S_SNOOP_REQ
    logic                  b_snoop_done  [NUM_BANKS];

    function automatic logic [BANK_BITS-1:0] get_bank(input [PADDR_WIDTH-1:0] addr);
        return addr[OFFSET_BITS +: BANK_BITS];
    endfunction

    function automatic logic [INDEX_BITS-1:0] get_index(input [PADDR_WIDTH-1:0] addr);
        return addr[OFFSET_BITS + BANK_BITS +: INDEX_BITS];
    endfunction

    function automatic logic [TAG_BITS-1:0] get_tag(input [PADDR_WIDTH-1:0] addr);
        return addr[PADDR_WIDTH-1 -: TAG_BITS];
    endfunction

    function automatic logic [2:0] get_victim_8way(input logic [6:0] plru);
        if (!plru[0]) begin
            if (!plru[1]) return plru[3] ? 3'd0 : 3'd1;
            else          return plru[4] ? 3'd2 : 3'd3;
        end else begin
            if (!plru[2]) return plru[5] ? 3'd4 : 3'd5;
            else          return plru[6] ? 3'd6 : 3'd7;
        end
    endfunction

    function automatic logic [6:0] update_plru_8way(input logic [6:0] plru, input logic [2:0] way);
        logic [6:0] p;
        p = plru;
        p[0] = way[2];
        if (!way[2]) begin p[1] = way[1]; p[way[1] ? 4 : 3] = way[0]; end
        else         begin p[2] = way[1]; p[way[1] ? 6 : 5] = way[0]; end
        return p;
    endfunction

    // Per-bank combinacional
    genvar b;
    generate
        for (b = 0; b < NUM_BANKS; b++) begin : bank_gen

            assign b_req_index[b] = get_index(req_reg[b].addr);
            assign b_req_tag[b]   = get_tag(req_reg[b].addr);

            // Tag compare
            always_comb begin
                hit_way[b]     = '0;
                cache_hit[b]   = 1'b0;
                hit_way_idx[b] = '0;

                for (int w = 0; w < NUM_WAYS; w++) begin
                    if (tag_array[b][b_req_index[b]][w].valid &&
                        tag_array[b][b_req_index[b]][w].tag == b_req_tag[b]) begin
                        hit_way[b][w]  = 1'b1;
                        cache_hit[b]   = 1'b1;
                        hit_way_idx[b] = w[2:0];
                    end
                end
                victim_way[b] = get_victim_8way(plru_bits[b][b_req_index[b]]);
            end

            // FIX 1: computar b_others e b_has_others combinacionalmente
            always_comb begin
                b_others[b]     = tag_array[b][b_req_index[b]][hit_way_idx[b]].sharers &
                                  ~(NUM_CORES'(1) << req_reg[b].core_id);
                b_has_others[b] = |b_others[b];
            end

            // FIX 1: b_snoop_done combinacional
            always_comb begin
                b_snoop_done[b] = 1'b1;
                for (int c = 0; c < NUM_CORES; c++) begin
                    if (snoop_pend[b][c] && !snoop_resp[c].valid)
                        b_snoop_done[b] = 1'b0;
                end
            end

            // State machine (sem declarações logic internas)
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    state[b]          <= S_IDLE;
                    selected_core[b]  <= '0;
                    req_reg[b]        <= '0;
                    line_buf[b]       <= '0;
                    snoop_pend[b]     <= '0;
                    snoop_data_buf[b] <= '0;
                    snoop_has_data[b] <= 1'b0;

                    for (int s = 0; s < SETS_PER_BANK; s++) begin
                        for (int w = 0; w < NUM_WAYS; w++) begin
                            // FIX 2: usar <= não =
                            tag_array[b][s][w] <= '0;
                        end
                        plru_bits[b][s] <= '0;
                    end
                end else begin
                    case (state[b])
                        S_IDLE: begin
                            for (int c = 0; c < NUM_CORES; c++) begin
                                if (l1_req[c].valid &&
                                    get_bank(l1_req[c].addr) == b[BANK_BITS-1:0]) begin
                                    selected_core[b] <= c[$clog2(NUM_CORES)-1:0];
                                    req_reg[b]       <= l1_req[c];
                                    state[b]         <= S_TAG_CHECK;
                                    break;
                                end
                            end
                        end

                        S_TAG_CHECK: begin
                            if (cache_hit[b]) begin
                                state[b] <= S_HIT;
                            end else begin
                                if (tag_array[b][b_req_index[b]][victim_way[b]].valid &&
                                    tag_array[b][b_req_index[b]][victim_way[b]].dirty)
                                    state[b] <= S_EVICT_WB;
                                else
                                    state[b] <= S_FILL_REQ;
                            end
                        end

                        // FIX 1: S_HIT sem declarações logic internas
                        S_HIT: begin
                            if (req_reg[b].is_write || req_reg[b].upgrade) begin
                                if (b_has_others[b]) begin
                                    snoop_pend[b] <= b_others[b];
                                    state[b]      <= S_SNOOP_REQ;
                                end else begin
                                    tag_array[b][b_req_index[b]][hit_way_idx[b]].state   <= CACHE_MODIFIED;
                                    tag_array[b][b_req_index[b]][hit_way_idx[b]].dirty   <= 1'b1;
                                    tag_array[b][b_req_index[b]][hit_way_idx[b]].sharers <=
                                        (NUM_CORES'(1) << req_reg[b].core_id);
                                    if (req_reg[b].is_write)
                                        data_array[b][b_req_index[b]][hit_way_idx[b]] <= req_reg[b].wdata;
                                    state[b] <= S_RESPOND;
                                end
                            end else begin
                                tag_array[b][b_req_index[b]][hit_way_idx[b]].sharers[req_reg[b].core_id] <= 1'b1;
                                if (tag_array[b][b_req_index[b]][hit_way_idx[b]].state == CACHE_MODIFIED ||
                                    tag_array[b][b_req_index[b]][hit_way_idx[b]].state == CACHE_EXCLUSIVE)
                                    tag_array[b][b_req_index[b]][hit_way_idx[b]].state <= CACHE_SHARED;
                                line_buf[b] <= data_array[b][b_req_index[b]][hit_way_idx[b]];
                                state[b]    <= S_RESPOND;
                            end
                            plru_bits[b][b_req_index[b]] <=
                                update_plru_8way(plru_bits[b][b_req_index[b]], hit_way_idx[b]);
                        end

                        // FIX 1: S_SNOOP_REQ sem declarações logic internas
                        S_SNOOP_REQ: begin
                            for (int c = 0; c < NUM_CORES; c++) begin
                                if (snoop_resp[c].valid && snoop_resp[c].has_data) begin
                                    snoop_has_data[b] <= 1'b1;
                                    snoop_data_buf[b] <= snoop_resp[c].data;
                                end
                            end
                            if (b_snoop_done[b]) state[b] <= S_SNOOP_WAIT;
                        end

                        // FIX 1: S_SNOOP_WAIT sem declarações logic internas
                        S_SNOOP_WAIT: begin
                            tag_array[b][b_req_index[b]][hit_way_idx[b]].sharers <=
                                (NUM_CORES'(1) << req_reg[b].core_id);
                            tag_array[b][b_req_index[b]][hit_way_idx[b]].state   <= CACHE_MODIFIED;
                            tag_array[b][b_req_index[b]][hit_way_idx[b]].dirty   <= 1'b1;

                            if (snoop_has_data[b])
                                data_array[b][b_req_index[b]][hit_way_idx[b]] <= snoop_data_buf[b];
                            if (req_reg[b].is_write)
                                data_array[b][b_req_index[b]][hit_way_idx[b]] <= req_reg[b].wdata;

                            snoop_pend[b]     <= '0;
                            snoop_has_data[b] <= 1'b0;
                            state[b]          <= S_RESPOND;
                        end

                        S_EVICT_WB: begin
                            line_buf[b] <= data_array[b][b_req_index[b]][victim_way[b]];
                            state[b]    <= S_EVICT_WAIT;
                        end

                        S_EVICT_WAIT: begin
                            if (mem_ack) begin
                                tag_array[b][b_req_index[b]][victim_way[b]].valid <= 1'b0;
                                tag_array[b][b_req_index[b]][victim_way[b]].dirty <= 1'b0;
                                state[b] <= S_FILL_REQ;
                            end
                        end

                        S_FILL_REQ: begin
                            if (mem_ready && bus_busy && bus_owner == b[$clog2(NUM_BANKS)-1:0])
                                state[b] <= S_FILL_WAIT;
                        end

                        S_FILL_WAIT: begin
                            if (mem_ack && !mem_error) begin
                                line_buf[b] <= mem_rdata;
                                state[b]    <= S_FILL_DONE;
                            end else if (mem_error) begin
                                state[b] <= S_RESPOND;
                            end
                        end

                        // FIX 1: S_FILL_DONE sem declarações logic internas
                        S_FILL_DONE: begin
                            tag_array[b][b_req_index[b]][victim_way[b]].valid   <= 1'b1;
                            tag_array[b][b_req_index[b]][victim_way[b]].tag     <= b_req_tag[b];
                            tag_array[b][b_req_index[b]][victim_way[b]].sharers <=
                                (NUM_CORES'(1) << req_reg[b].core_id);

                            if (req_reg[b].is_write) begin
                                tag_array[b][b_req_index[b]][victim_way[b]].state <= CACHE_MODIFIED;
                                tag_array[b][b_req_index[b]][victim_way[b]].dirty <= 1'b1;
                                data_array[b][b_req_index[b]][victim_way[b]]      <= req_reg[b].wdata;
                            end else begin
                                tag_array[b][b_req_index[b]][victim_way[b]].state <= CACHE_EXCLUSIVE;
                                tag_array[b][b_req_index[b]][victim_way[b]].dirty <= 1'b0;
                                data_array[b][b_req_index[b]][victim_way[b]]      <= line_buf[b];
                            end

                            plru_bits[b][b_req_index[b]] <=
                                update_plru_8way(plru_bits[b][b_req_index[b]], victim_way[b]);
                            state[b] <= S_RESPOND;
                        end

                        S_RESPOND: state[b] <= S_IDLE;
                        default:   state[b] <= S_IDLE;
                    endcase
                end
            end
        end
    endgenerate

    // Response
    always_comb begin
        for (int c = 0; c < NUM_CORES; c++) begin
            l1_resp[c] = '0;
            for (int bi = 0; bi < NUM_BANKS; bi++) begin
                if (state[bi] == S_RESPOND &&
                    selected_core[bi] == c[$clog2(NUM_CORES)-1:0]) begin
                    l1_resp[c].valid    = 1'b1;
                    l1_resp[c].core_id  = c[$clog2(NUM_CORES)-1:0];
                    l1_resp[c].is_ifetch = req_reg[bi].is_ifetch;
                    l1_resp[c].rdata    = line_buf[bi];
                    l1_resp[c].state    = req_reg[bi].is_write ? CACHE_MODIFIED : CACHE_SHARED;
                end
            end
        end
    end

    // Snoop
    always_comb begin
        for (int c = 0; c < NUM_CORES; c++) begin
            snoop_req[c] = '0;
            for (int bi = 0; bi < NUM_BANKS; bi++) begin
                if (state[bi] == S_SNOOP_REQ && snoop_pend[bi][c]) begin
                    snoop_req[c].valid     = 1'b1;
                    snoop_req[c].op        = req_reg[bi].is_write ? COH_INVALIDATE : COH_READ;
                    snoop_req[c].addr      = req_reg[bi].addr;
                    snoop_req[c].requester = selected_core[bi];
                end
            end
        end
    end

    // Memory interface
    always_comb begin
        mem_req   = 1'b0;
        mem_we    = 1'b0;
        mem_addr  = '0;
        mem_wdata = '0;

        if (bus_busy) begin
            // Só o dono do barramento fala com a memória
            if (state[bus_owner] == S_EVICT_WAIT) begin
                mem_req   = 1'b1;
                mem_we    = 1'b1;
                mem_addr  = {tag_array[bus_owner][get_index(req_reg[bus_owner].addr)][victim_way[bus_owner]].tag,
                            get_index(req_reg[bus_owner].addr),
                            bus_owner[BANK_BITS-1:0],
                            {OFFSET_BITS{1'b0}}};
                mem_wdata = line_buf[bus_owner];
            end else if (state[bus_owner] == S_FILL_REQ ||
                        state[bus_owner] == S_FILL_WAIT) begin
                mem_req  = 1'b1;
                mem_we   = 1'b0;
                mem_addr = {req_reg[bus_owner].addr[PADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
            end
        end
    end

    // =========================================================================
    // Árbitro de barramento — garante acesso exclusivo ao mem_req/mem_ack
    // =========================================================================
    logic                        bus_busy;
    logic [$clog2(NUM_BANKS)-1:0] bus_owner;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bus_busy  <= 1'b0;
            bus_owner <= '0;
        end else begin
            if (!bus_busy) begin
                // Conceder o barramento ao primeiro banco que precisar
                for (int bi = 0; bi < NUM_BANKS; bi++) begin
                    if (state[bi] == S_EVICT_WAIT || state[bi] == S_FILL_REQ) begin
                        bus_busy  <= 1'b1;
                        bus_owner <= bi[$clog2(NUM_BANKS)-1:0];
                        break;
                    end
                end
            end else begin
                // Liberar quando o banco dono terminar (sai de EVICT_WAIT ou FILL_WAIT)
                if (state[bus_owner] != S_EVICT_WAIT &&
                    state[bus_owner] != S_FILL_REQ   &&
                    state[bus_owner] != S_FILL_WAIT) begin
                    bus_busy <= 1'b0;
                end
            end
        end
    end

endmodule
