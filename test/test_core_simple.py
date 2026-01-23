import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

@cocotb.test()
async def test_simple_addi(dut):
    """Test simple ADDI instruction"""
    clock = Clock(dut.clk, 83, units="ns")  # 12 MHz
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.rst_n.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    
    # Write instruction ADDI x1, x0, 10 (0x00A00093) to address 0
    # Manually initialize memory for simulation
    dut.imem_data.value = 0x00A00093
    
    # Release reset
    dut.rst_n.value = 1
    
    # Let pipeline run
    for i in range(10):
        await RisingEdge(dut.clk)
        dut._log.info(f"Cycle {i}: PC={int(dut.imem_addr.value)}, instr={hex(int(dut.imem_data.value))}")
    
    # Check register x1
    dut.dbg_reg_addr.value = 1
    await RisingEdge(dut.clk)
    x1_val = int(dut.dbg_reg_data.value)
    dut._log.info(f"x1 = {x1_val}")
    assert x1_val == 10, f"Expected x1=10, got {x1_val}"
