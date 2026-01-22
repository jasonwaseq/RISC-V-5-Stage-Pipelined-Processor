"""
Cocotb testbench for simplified RISC-V 5-Stage Pipelined Processor
Tests basic RV32I instructions (8-register version)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# Simple instruction encoders for 8-register processor
def encode_i(rd, rs1, funct3, imm, opcode):
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def encode_r(rd, rs1, rs2, funct3, opcode):
    return (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def encode_s(rs1, rs2, funct3, imm, opcode):
    return ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | ((imm & 0x1F) << 7) | opcode

def encode_b(rs1, rs2, funct3, imm, opcode):
    return (((imm >> 12) & 1) << 31) | (((imm >> 5) & 0x3F) << 25) | (rs2 << 20) | \
           (rs1 << 15) | (funct3 << 12) | (((imm >> 1) & 0xF) << 8) | (((imm >> 11) & 1) << 7) | opcode

# Common instructions
ADDI = lambda rd, rs1, imm: encode_i(rd, rs1, 0b000, imm, 0b0010011)
ADD = lambda rd, rs1, rs2: encode_r(rd, rs1, rs2, 0b000, 0b0110011)
AND = lambda rd, rs1, rs2: encode_r(rd, rs1, rs2, 0b111, 0b0110011)
OR = lambda rd, rs1, rs2: encode_r(rd, rs1, rs2, 0b110, 0b0110011)
XOR = lambda rd, rs1, rs2: encode_r(rd, rs1, rs2, 0b100, 0b0110011)
SW = lambda rs2, rs1, imm: encode_s(rs1, rs2, 0b010, imm, 0b0100011)
LW = lambda rd, rs1, imm: encode_i(rd, rs1, 0b010, imm, 0b0000011)
BEQ = lambda rs1, rs2, imm: encode_b(rs1, rs2, 0b000, imm, 0b1100011)
NOP = lambda: ADDI(0, 0, 0)


async def setup_clock(dut):
    clock = Clock(dut.clk, 83.33, units="ns")  # 12 MHz
    cocotb.start_soon(clock.start())


@cocotb.test()
async def test_addi(dut):
    """Test ADDI instruction"""
    await setup_clock(dut)
    
    # Program: ADDI x1, x0, 42
    program = [ADDI(1, 0, 42), NOP(), NOP(), NOP()]
    
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    
    # Load program and run
    for i, instr in enumerate(program):
        addr = i * 4
        dut.imem_data.value = instr
        await RisingEdge(dut.clk)
    
    # Run 10 cycles to let instruction complete
    for _ in range(10):
        dut.imem_data.value = NOP()
        await RisingEdge(dut.clk)
    
    # Check result (x1 should be 42)
    dut._log.info(f"Test ADDI: x1 = {dut.regs[1].value}")


@cocotb.test()
async def test_add(dut):
    """Test ADD instruction"""
    await setup_clock(dut)
    
    # Program: ADDI x1, 10; ADDI x2, 5; ADD x3, x1, x2
    program = [
        ADDI(1, 0, 10),
        ADDI(2, 0, 5),
        ADD(3, 1, 2),
        NOP(), NOP()
    ]
    
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    
    for instr in program:
        dut.imem_data.value = instr
        await RisingEdge(dut.clk)
    
    for _ in range(10):
        dut.imem_data.value = NOP()
        await RisingEdge(dut.clk)
    
    dut._log.info(f"Test ADD: x1={dut.regs[1].value}, x2={dut.regs[2].value}, x3={dut.regs[3].value}")


@cocotb.test()
async def test_load_store(dut):
    """Test load and store"""
    await setup_clock(dut)
    
    # Program: ADDI x1, 55; SW x1, 0(x0); LW x2, 0(x0)
    program = [
        ADDI(1, 0, 0x55),
        NOP(),
        SW(1, 0, 0),
        NOP(), NOP(),
        LW(2, 0, 0),
        NOP(), NOP(), NOP()
    ]
    
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    
    dmem = {}
    for instr in program:
        dut.imem_data.value = instr
        
        if int(dut.dmem_we.value):
            dmem[0] = int(dut.dmem_wdata.value)
        dut.dmem_rdata.value = dmem.get(0, 0)
        
        await RisingEdge(dut.clk)
    
    for _ in range(10):
        dut.imem_data.value = NOP()
        dut.dmem_rdata.value = dmem.get(0, 0)
        await RisingEdge(dut.clk)
    
    dut._log.info(f"Test Load/Store: x1={dut.regs[1].value}, x2={dut.regs[2].value}")


@cocotb.test()
async def test_branch(dut):
    """Test branch instruction"""
    await setup_clock(dut)
    
    # Program: ADDI x1, 5; ADDI x2, 5; BEQ x1, x2, +8; ADDI x3, 1; ADDI x4, 99
    program = [
        ADDI(1, 0, 5),      # 0x00
        ADDI(2, 0, 5),      # 0x04
        BEQ(1, 2, 8),       # 0x08: branch to PC+8
        ADDI(3, 0, 1),      # 0x0C: skipped
        ADDI(4, 0, 99),     # 0x10: branch target
        NOP(), NOP()
    ]
    
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    
    for instr in program:
        dut.imem_data.value = instr
        await RisingEdge(dut.clk)
    
    for _ in range(15):
        dut.imem_data.value = NOP()
        await RisingEdge(dut.clk)
    
    dut._log.info(f"Test Branch: x1={dut.regs[1].value}, x2={dut.regs[2].value}, x3={dut.regs[3].value}, x4={dut.regs[4].value}")


@cocotb.test()
async def test_bitwise(dut):
    """Test bitwise operations"""
    await setup_clock(dut)
    
    # Program: ADDI x1, 0b1010; ADDI x2, 0b0101; AND x3; OR x4; XOR x5
    program = [
        ADDI(1, 0, 0b1010),
        ADDI(2, 0, 0b0101),
        AND(3, 1, 2),
        OR(4, 1, 2),
        XOR(5, 1, 2),
        NOP(), NOP(), NOP()
    ]
    
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    
    for instr in program:
        dut.imem_data.value = instr
        await RisingEdge(dut.clk)
    
    for _ in range(12):
        dut.imem_data.value = NOP()
        await RisingEdge(dut.clk)
    
    dut._log.info(f"Test Bitwise: x3(AND)={dut.regs[3].value:04b}, x4(OR)={dut.regs[4].value:04b}, x5(XOR)={dut.regs[5].value:04b}")
