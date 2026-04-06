#!/usr/bin/env python3
import os
from migen import *
from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig
from litex.soc.cores.cpu import CPUS
from litex.soc.cores.cpu import CPU, CPU_GCC_TRIPLE_RISCV64
from litex.soc.interconnect import axi
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

# 1. Definição do CPU para o ecossistema LiteX
class NebulaCPU(CPU):
    name                 = "nebula"
    data_width           = 64
    endianness           = "little"
    gcc_triple           = CPU_GCC_TRIPLE_RISCV64
    linker_output_format = "elf64-littleriscv"
    nop                  = "nop"
    io_regions           = {0x80000000: 0x80000000} # Região MMIO para periféricos
    family               = "riscv"
    category             = "softcore"
    variants             = ["standard"]

    @property
    def mem_map(self):
        return {
            "rom":      0x00000000,
            "sram":     0x10000000,
            "main_ram": 0x40000000,
            "csr":      0x82000000,
        }

    @property
    def gcc_flags(self):
        # rv64imafdc = RV64GC
        # lp64d = Long/Pointers 64-bit, Double-precision FPU
        return "-march=rv64imafdc_zicsr_zifencei -mabi=lp64d -D__nebula__"

    def __init__(self, platform, variant="standard"):
        super().__init__()
        self.platform = platform
        self.reset    = Signal()
        self.reset_address = 0x00000000
        
        # Barramento AXI de 64 bits do Nebula Core
        self.axi      = axi.AXIInterface(data_width=64, address_width=32, id_width=4)
        self.periph_buses = [self.axi]
        self.memory_buses = []

    def do_finalize(self):
        # 1. Obter o caminho absoluto para a pasta 'rtl'
        import os
        rtl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rtl"))

        # 2. FORÇAR a compilação do Package PRIMEIRO
        pkg_file = os.path.join(rtl_dir, "common/nebula_pkg.sv") 
        self.platform.add_source(pkg_file)

        # 3. Depois carrega o resto do processador
        self.platform.add_source_dir(rtl_dir)

        # 4. Mapeamento físico do Wrapper AXI do núcleo
        self.specials += Instance("nebula_core_axi_top",
            i_clk            = ClockSignal("sys"),
            i_rst_n          = ~(ResetSignal("sys") | self.reset),
            
            # --- Interrupções (Tie-off para evitar PinMissing) ---
            i_i_timer_irq    = 0,
            i_i_external_irq = 0,
            i_i_software_irq = 0,

            # --- AXI Instruction (Tie-off das entradas para evitar PinMissing) ---
            i_i_m_axi_i_arready = 0,
            i_i_m_axi_i_rdata   = 0,
            i_i_m_axi_i_rresp   = 0,
            i_i_m_axi_i_rlast   = 0,
            i_i_m_axi_i_rvalid  = 0,

            # --- AXI Data (Ligado ao LiteX) ---
            # AW (Address Write)
            o_o_m_axi_d_awvalid = self.axi.aw.valid,
            i_i_m_axi_d_awready = self.axi.aw.ready,
            o_o_m_axi_d_awaddr  = self.axi.aw.addr,
            o_o_m_axi_d_awid    = self.axi.aw.id,
            o_o_m_axi_d_awlen   = self.axi.aw.len,
            o_o_m_axi_d_awsize  = self.axi.aw.size,
            o_o_m_axi_d_awburst = self.axi.aw.burst,
            
            # W (Write Data)
            o_o_m_axi_d_wvalid  = self.axi.w.valid,
            i_i_m_axi_d_wready  = self.axi.w.ready,
            o_o_m_axi_d_wdata   = self.axi.w.data,
            o_o_m_axi_d_wstrb   = self.axi.w.strb,
            o_o_m_axi_d_wlast   = self.axi.w.last,
            
            # B (Write Response)
            i_i_m_axi_d_bvalid  = self.axi.b.valid,
            o_o_m_axi_d_bready  = self.axi.b.ready,
            i_i_m_axi_d_bresp   = self.axi.b.resp,
            i_i_m_axi_d_bid     = self.axi.b.id,
            
            # AR (Address Read)
            o_o_m_axi_d_arvalid = self.axi.ar.valid,
            i_i_m_axi_d_arready = self.axi.ar.ready,
            o_o_m_axi_d_araddr  = self.axi.ar.addr,
            o_o_m_axi_d_arid    = self.axi.ar.id,
            o_o_m_axi_d_arlen   = self.axi.ar.len,
            o_o_m_axi_d_arsize  = self.axi.ar.size,
            o_o_m_axi_d_arburst = self.axi.ar.burst,
            
            # R (Read Data)
            i_i_m_axi_d_rvalid  = self.axi.r.valid,
            o_o_m_axi_d_rready  = self.axi.r.ready,
            i_i_m_axi_d_rdata   = self.axi.r.data,
            i_i_m_axi_d_rresp   = self.axi.r.resp,
            i_i_m_axi_d_rlast   = self.axi.r.last
        )

# Regista o CPU no ambiente LiteX
CPUS[NebulaCPU.name] = NebulaCPU

# Sinais virtuais necessários para o Simulador LiteX
_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),
        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),
]

# Módulo que cria o Domínio de Relógio do sistema
class CRG(Module):
    def __init__(self, sys_clk, sys_rst):
        self.clock_domains.cd_sys = ClockDomain()
        self.comb += [
            self.cd_sys.clk.eq(sys_clk),
            self.cd_sys.rst.eq(sys_rst)
        ]

# 2. Definição do SoC Completo
class NebulaSimSoC(SoCCore):
    def __init__(self):
        platform = SimPlatform("sim", _io)
        sys_clk_freq = int(1e6)

        # Cria a Motherboard com ROM, RAM e UART Virtual
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            cpu_type="nebula",
            integrated_rom_size=0x8000,
            integrated_main_ram_size=0x100000, # 1MB de RAM para testes
            uart_name="sim"
        )
        self.add_constant("ROM_BOOT_ADDRESS", self.mem_map["rom"])

        sys_clk = platform.request("sys_clk")
        sys_rst = platform.request("sys_rst")
        self.submodules.crg = CRG(sys_clk, sys_rst)

# 3. Compilação
if __name__ == "__main__":
    soc = NebulaSimSoC()
    builder = Builder(soc, compile_software=True, compile_gateware=True)
    
    # Prepara o Verilator e roda
    sim_config = SimConfig(default_clk="sys_clk")
    sim_config.add_module("serial2console", "serial")
    builder.build(sim_config=sim_config, run=True)
    soc.add_constant("SIM_TRACE", 1) # Ativa logs extras do LiteX