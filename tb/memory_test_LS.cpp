#include <iostream>
#include "Vnebula_core_full.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

// Simulador de RAM Simples (8KB = 128 linhas de 64 bytes)
uint32_t data_ram[128][16] = {0}; 

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    Vnebula_core_full* top = new Vnebula_core_full;
    VerilatedVcdC* tfp = new VerilatedVcdC;

    top->trace(tfp, 99);
    tfp->open("dump.vcd");

    /*
     * PROGRAMA DE TESTE: STRESS SUPERESCALAR
     * ----------------------------------------------------
     * 0: LUI x1, 1           | 1: ADDI x2, x0, 42
     * 2: SW x2, 0(x1)        | 3: LW x3, 0(x1)
     * 4: ADDI x4, x3, 10     | 5: SD x4, 8(x1)
     * 6: LD x5, 8(x1)        | 7: ADDI x6, x5, 100  <-- PERIGO: Exige o x5 antes do LD terminar!
     * 8: ADD x7, x6, x1      | 9: SUB x8, x6, x2    <-- Exige o x6 acabado de calcular!
     * 10/11 até 15: NOPs apenas para terminar a simulação e gravar a onda.
     */
    uint32_t INSTRUCTIONS[16] = {
        0x000010b7, 0x02a00113, // 0/1
        0x0020a023, 0x0000a183, // 2/3
        0x00a18213, 0x0040b423, // 4/5
        0x0080b283, 0x06428313, // 6/7: LD x5, 8(x1) | ADDI x6, x5, 100
        0x001303b3, 0x40230433, // 8/9: ADD x7, x6, x1 | SUB x8, x6, x2
        0x00000013, 0x00000013, // 10/11
        0x00000013, 0x00000013, // 12/13
        0x00000013, 0x00000013  // 14/15
    };

    top->clk = 0;
    top->rst_n = 0;
    top->imem_ack = 0;
    top->dmem_ack = 0;

    int imem_wait = 0;
    int dmem_wait = 0;
    
    // Latches para desacoplar a memória do FSM do Core
    bool ireq_latched = false;
    bool dreq_latched = false;
    uint64_t latched_daddr = 0;
    bool latched_dwe = false;

    // Roda por 200 meios-ciclos (100 ciclos de clock)
    for (int i = 0; i < 150000; i++) {
        top->clk = !top->clk;
        if (main_time > 10) top->rst_n = 1;

        top->eval();

        if (top->clk == 1 && top->rst_n == 1) {
            // ==========================================
            // LÓGICA DO I-CACHE (Instruções)
            // ==========================================
            if (top->imem_req) ireq_latched = true;

            if (ireq_latched && !top->imem_ack) {
                if (imem_wait == 0) imem_wait = 2; // 2 ciclos de latência
                else {
                    imem_wait--;
                    if (imem_wait == 0) {
                        top->imem_ack = 1;
                        ireq_latched = false;
                        for(int j=0; j<16; j++) top->imem_data[j] = INSTRUCTIONS[j];
                    }
                }
            } else if (top->imem_ack) {
                top->imem_ack = 0;
            }

            // ==========================================
            // LÓGICA DO D-CACHE (Dados - Load/Store)
            // ==========================================
            if (top->dmem_req) {
                dreq_latched = true;
                latched_daddr = top->dmem_addr;
                latched_dwe = top->dmem_we;
            }

            if (dreq_latched && !top->dmem_ack) {
                if (dmem_wait == 0) dmem_wait = 3; // 3 ciclos de latência para a RAM
                else {
                    dmem_wait--;
                    if (dmem_wait == 0) {
                        top->dmem_ack = 1;
                        dreq_latched = false;
                        
                        // Calcula qual linha de cache L1 foi solicitada (64 bytes = 16 words)
                        uint32_t line_idx = (latched_daddr / 64) % 128;
                        
                        if (latched_dwe) {
                            // WRITEBACK: L1 faz eviction e grava na RAM
                            for(int j=0; j<16; j++) {
                                data_ram[line_idx][j] = top->dmem_wdata[j];
                            }
                        } else {
                            // REFILL: L1 deu miss e pede os dados à RAM
                            for(int j=0; j<16; j++) {
                                top->dmem_rdata[j] = data_ram[line_idx][j];
                            }
                        }
                    }
                }
            } else if (top->dmem_ack) {
                top->dmem_ack = 0;
            }
        }

        tfp->dump(main_time);
        main_time++;
    }

    top->final();
    tfp->close();
    delete top;
    delete tfp;
    
    std::cout << "Teste de Memoria concluido! Ficheiro dump.vcd gerado." << std::endl;
    return 0;
}