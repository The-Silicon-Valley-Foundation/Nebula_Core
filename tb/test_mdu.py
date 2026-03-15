"""
test_mdu.py — Testbench cocotb para mdu_rv64

Usa cocotb-test (pip install cocotb-test) como runner pytest.
Cobre todas as 10 operações da extensão RV64M + variantes W.
"""

import os
import pytest
import ctypes
from cocotb_test.simulator import run

# ---------------------------------------------------------------------------
# Constantes funct3
# ---------------------------------------------------------------------------
F3_MUL    = 0b000
F3_MULH   = 0b001
F3_MULHSU = 0b010
F3_MULHU  = 0b011
F3_DIV    = 0b100
F3_DIVU   = 0b101
F3_REM    = 0b110
F3_REMU   = 0b111

MASK64    = (1 << 64) - 1
MASK32    = (1 << 32) - 1
MIN_INT64 = -(1 << 63)
MIN_INT32 = -(1 << 31)

RTL_DIR = os.path.abspath(os.environ.get("RTL_DIR", "../rtl"))

# ---------------------------------------------------------------------------
# Modelos de referência
# ---------------------------------------------------------------------------
def s64(v): return ctypes.c_int64(v & MASK64).value
def s32(v): return ctypes.c_int32(v & MASK32).value
def sext32(v): return ctypes.c_int64(ctypes.c_int32(v & MASK32).value).value & MASK64

def ref_mul(a,b):    return (s64(a)*s64(b)) & MASK64
def ref_mulh(a,b):   return ((s64(a)*s64(b)) >> 64) & MASK64
def ref_mulhsu(a,b): return ((s64(a)*(b&MASK64)) >> 64) & MASK64
def ref_mulhu(a,b):  return (((a&MASK64)*(b&MASK64)) >> 64) & MASK64
def ref_div(a,b):
    a,b = s64(a),s64(b)
    if b==0: return MASK64
    if a==MIN_INT64 and b==-1: return MIN_INT64 & MASK64
    return int(a/b) & MASK64
def ref_divu(a,b):
    a,b = a&MASK64, b&MASK64
    return MASK64 if b==0 else (a//b)&MASK64
def ref_rem(a,b):
    ra,rb = s64(a),s64(b)
    if rb==0: return a&MASK64
    if ra==MIN_INT64 and rb==-1: return 0
    return int(ra - int(ra/rb)*rb) & MASK64
def ref_remu(a,b):
    a,b = a&MASK64, b&MASK64
    return a&MASK64 if b==0 else (a%b)&MASK64
def ref_mulw(a,b):  return sext32((s32(a)*s32(b))&MASK32)
def ref_divw(a,b):
    ra,rb = s32(a),s32(b)
    if rb==0: return MASK64
    if ra==MIN_INT32 and rb==-1: return sext32(MIN_INT32&MASK32)
    return sext32(int(ra/rb)&MASK32)
def ref_divuw(a,b):
    a,b = a&MASK32, b&MASK32
    return MASK64 if b==0 else sext32((a//b)&MASK32)
def ref_remw(a,b):
    ra,rb = s32(a),s32(b)
    if rb==0: return sext32(a&MASK32)
    if ra==MIN_INT32 and rb==-1: return 0
    return sext32(int(ra - int(ra/rb)*rb)&MASK32)
def ref_remuw(a,b):
    a,b = a&MASK32, b&MASK32
    return sext32(a&MASK32) if b==0 else sext32((a%b)&MASK32)

# ---------------------------------------------------------------------------
# Lógica cocotb (importada condicionalmente — só quando o sim está rodando)
# ---------------------------------------------------------------------------
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

async def _reset(dut):
    dut.rst_n.value = 0
    dut.req_valid.value = 0
    dut.rs1_data.value = 0
    dut.rs2_data.value = 0
    dut.funct3.value = 0
    dut.is_word_op.value = 0
    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

async def _send(dut, rs1, rs2, f3, word=False):
    t = 0
    while not dut.req_ready.value:
        await RisingEdge(dut.clk)
        t += 1
        assert t < 200, "Timeout req_ready"
    dut.req_valid.value  = 1
    dut.rs1_data.value   = rs1 & MASK64
    dut.rs2_data.value   = rs2 & MASK64
    dut.funct3.value     = f3
    dut.is_word_op.value = int(word)
    await RisingEdge(dut.clk)
    dut.req_valid.value  = 0
    t = 0
    while not dut.resp_valid.value:
        await RisingEdge(dut.clk)
        t += 1
        assert t < 400, "Timeout resp_valid"
    r = int(dut.result.value)
    await RisingEdge(dut.clk)
    return r

def _chk(dut, name, rs1, rs2, got, exp):
    exp = exp & MASK64
    assert got == exp, (
        f"\n[FAIL] {name}\n"
        f"  rs1={rs1&MASK64:#018x}  rs2={rs2&MASK64:#018x}\n"
        f"  got={got:#018x}  expected={exp:#018x}\n"
    )
    dut._log.info(f"[PASS] {name}")

# Vetores
MV = [(0,0),(1,1),(2,3),(-1&MASK64,1),(-1&MASK64,-1&MASK64),
      (0x7FFFFFFFFFFFFFFF,2),(0x8000000000000000,1),
      (0x8000000000000000,-1&MASK64),(0xDEADBEEFCAFEBABE,0x0102030405060708)]
DV = [(10,3),(-10&MASK64,3),(10,-3&MASK64),(-10&MASK64,-3&MASK64),
      (0,5),(7,0),(0x8000000000000000,MASK64),(0x7FFFFFFFFFFFFFFF,1),(100,7)]
WV = [(5,3),(-5&MASK64,3),(0x80000000,MASK64),(0,0),(MASK32,2)]

@cocotb.test()
async def test_mul(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in MV:
        _chk(dut,"MUL",a,b, await _send(dut,a,b,F3_MUL), ref_mul(a,b))

@cocotb.test()
async def test_mulh(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in MV:
        _chk(dut,"MULH",a,b, await _send(dut,a,b,F3_MULH), ref_mulh(a,b))

@cocotb.test()
async def test_mulhsu(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in MV:
        _chk(dut,"MULHSU",a,b, await _send(dut,a,b,F3_MULHSU), ref_mulhsu(a,b))

@cocotb.test()
async def test_mulhu(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in MV:
        _chk(dut,"MULHU",a,b, await _send(dut,a,b,F3_MULHU), ref_mulhu(a,b))

@cocotb.test()
async def test_div(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in DV:
        _chk(dut,"DIV",a,b, await _send(dut,a,b,F3_DIV), ref_div(a,b))

@cocotb.test()
async def test_divu(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in DV:
        _chk(dut,"DIVU",a,b, await _send(dut,a,b,F3_DIVU), ref_divu(a,b))

@cocotb.test()
async def test_rem(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in DV:
        _chk(dut,"REM",a,b, await _send(dut,a,b,F3_REM), ref_rem(a,b))

@cocotb.test()
async def test_remu(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in DV:
        _chk(dut,"REMU",a,b, await _send(dut,a,b,F3_REMU), ref_remu(a,b))

@cocotb.test()
async def test_mulw(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in WV:
        _chk(dut,"MULW",a,b, await _send(dut,a,b,F3_MUL,word=True), ref_mulw(a,b))

@cocotb.test()
async def test_divw(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in WV:
        _chk(dut,"DIVW",a,b, await _send(dut,a,b,F3_DIV,word=True), ref_divw(a,b))

@cocotb.test()
async def test_divuw(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in WV:
        _chk(dut,"DIVUW",a,b, await _send(dut,a,b,F3_DIVU,word=True), ref_divuw(a,b))

@cocotb.test()
async def test_remw(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in WV:
        _chk(dut,"REMW",a,b, await _send(dut,a,b,F3_REM,word=True), ref_remw(a,b))

@cocotb.test()
async def test_remuw(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    for a,b in WV:
        _chk(dut,"REMUW",a,b, await _send(dut,a,b,F3_REMU,word=True), ref_remuw(a,b))

@cocotb.test()
async def test_back_to_back(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    ops = [
        (7,3,F3_MUL,False,ref_mul(7,3)),
        (100,7,F3_DIV,False,ref_div(100,7)),
        (100,7,F3_REM,False,ref_rem(100,7)),
        (-5&MASK64,2,F3_MUL,False,ref_mul(-5&MASK64,2)),
        (10,3,F3_MUL,True,ref_mulw(10,3)),
    ]
    for a,b,f3,w,exp in ops:
        _chk(dut,f"B2B f3={f3}",a,b, await _send(dut,a,b,f3,w), exp)

@cocotb.test()
async def test_reset_mid_op(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await _reset(dut)
    # Disparar divisão (64 ciclos)
    dut.req_valid.value=1; dut.rs1_data.value=100; dut.rs2_data.value=7
    dut.funct3.value=F3_DIV; dut.is_word_op.value=0
    await RisingEdge(dut.clk)
    dut.req_valid.value=0
    for _ in range(10): await RisingEdge(dut.clk)
    # Reset
    dut.rst_n.value=0
    for _ in range(3): await RisingEdge(dut.clk)
    dut.rst_n.value=1
    await RisingEdge(dut.clk)
    # Nova operação deve funcionar
    _chk(dut,"pós-reset",20,4, await _send(dut,20,4,F3_MUL), ref_mul(20,4))

# ---------------------------------------------------------------------------
# Entrada pytest — cocotb-test
# ---------------------------------------------------------------------------
def test_mdu_all():
    """Ponto de entrada pytest: compila e executa todos os @cocotb.test() acima."""
    run(
        verilog_sources=[
            os.path.join(RTL_DIR, "common", "nebula_pkg.sv"),
            os.path.join(RTL_DIR, "backend", "mdu_rv64.sv"),
        ],
        toplevel="mdu_rv64",
        module="test_mdu",          # este próprio arquivo
        simulator="verilator",
        extra_args=[
            "--timing", "--assert",
            "-Wall", "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND",
            "-Wno-ENUMVALUE", "-Wno-UNUSED",
        ],
        waves=os.environ.get("WAVES", "0") == "1",
    )
