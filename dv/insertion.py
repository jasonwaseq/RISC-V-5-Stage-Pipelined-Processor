import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, FallingEdge
from cocotb.types import LogicArray
import random

class ReadyValidDriver:
    """Driver for ready-valid interface"""
    def __init__(self, clk, data, valid, ready):
        self.clk = clk
        self.data = data
        self.valid = valid
        self.ready = ready
        
    async def send(self, value):
        """Send a single value with ready-valid handshake"""
        self.data.value = value
        self.valid.value = 1
        
        # Wait for handshake
        while True:
            await RisingEdge(self.clk)
            if self.ready.value == 1:
                break
        
        # Deassert valid after handshake
        self.valid.value = 0
        
    async def send_all(self, values, delay_range=(0, 3)):
        """Send multiple values with optional random delays"""
        for val in values:
            await self.send(val)
            # Optional random delay between transactions
            if delay_range[1] > 0:
                delay = random.randint(delay_range[0], delay_range[1])
                if delay > 0:
                    await ClockCycles(self.clk, delay)

class ReadyValidMonitor:
    """Monitor for ready-valid interface"""
    def __init__(self, clk, data, valid, ready):
        self.clk = clk
        self.data = data
        self.valid = valid
        self.ready = ready
        
    async def receive(self):
        """Receive a single value with ready-valid handshake"""
        self.ready.value = 1
        
        # Wait for valid
        while True:
            await RisingEdge(self.clk)
            if self.valid.value == 1:
                value = int(self.data.value)
                return value
                
    async def receive_all(self, count, delay_range=(0, 3)):
        """Receive multiple values with optional random delays"""
        values = []
        for _ in range(count):
            val = await self.receive()
            values.append(val)
            # Optional random delay between transactions
            if delay_range[1] > 0:
                delay = random.randint(delay_range[0], delay_range[1])
                if delay > 0:
                    self.ready.value = 0
                    await ClockCycles(self.clk, delay)
        
        self.ready.value = 0
        return values

async def reset_dut(dut):
    """Reset the DUT"""
    dut.reset_i.value = 1
    dut.valid_i.value = 0
    dut.ready_i.value = 0
    dut.data_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    dut.reset_i.value = 0
    await ClockCycles(dut.clk_i, 2)

@cocotb.test()
async def test_insertion_sort_basic(dut):
    """Test basic insertion sort with simple sequence"""
    
    # Start clock
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    # Test data: unsorted list
    test_data = [5, 2, 8, 1, 9, 3, 7, 4, 6, 0]
    expected = sorted(test_data)
    
    dut._log.info(f"Input data: {test_data}")
    dut._log.info(f"Expected output: {expected}")
    
    # Create driver and monitor
    driver = ReadyValidDriver(dut.clk_i, dut.data_i, dut.valid_i, dut.ready_o)
    monitor = ReadyValidMonitor(dut.clk_i, dut.data_o, dut.valid_o, dut.ready_i)
    
    # Send data
    send_task = cocotb.start_soon(driver.send_all(test_data, delay_range=(0, 0)))
    
    # Receive data
    receive_task = cocotb.start_soon(monitor.receive_all(len(test_data), delay_range=(0, 0)))
    
    # Wait for both to complete
    await send_task
    result = await receive_task
    
    dut._log.info(f"Received output: {result}")
    
    # Check result
    assert result == expected, f"Sort failed! Got {result}, expected {expected}"
    dut._log.info("✓ Basic sort test passed!")

@cocotb.test()
async def test_insertion_sort_random_delays(dut):
    """Test insertion sort with random delays on ready/valid"""
    
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    # Random test data
    test_data = [random.randint(0, 255) for _ in range(10)]
    expected = sorted(test_data)
    
    dut._log.info(f"Input data: {test_data}")
    dut._log.info(f"Expected output: {expected}")
    
    driver = ReadyValidDriver(dut.clk_i, dut.data_i, dut.valid_i, dut.ready_o)
    monitor = ReadyValidMonitor(dut.clk_i, dut.data_o, dut.valid_o, dut.ready_i)
    
    # Send with random delays
    send_task = cocotb.start_soon(driver.send_all(test_data, delay_range=(0, 5)))
    
    # Receive with random delays (backpressure)
    receive_task = cocotb.start_soon(monitor.receive_all(len(test_data), delay_range=(0, 5)))
    
    await send_task
    result = await receive_task
    
    dut._log.info(f"Received output: {result}")
    
    assert result == expected, f"Sort failed! Got {result}, expected {expected}"
    dut._log.info("✓ Random delays test passed!")

@cocotb.test()
async def test_insertion_sort_already_sorted(dut):
    """Test with already sorted data"""
    
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    test_data = list(range(10))  # Already sorted
    expected = test_data.copy()
    
    dut._log.info(f"Input data (already sorted): {test_data}")
    
    driver = ReadyValidDriver(dut.clk_i, dut.data_i, dut.valid_i, dut.ready_o)
    monitor = ReadyValidMonitor(dut.clk_i, dut.data_o, dut.valid_o, dut.ready_i)
    
    send_task = cocotb.start_soon(driver.send_all(test_data, delay_range=(0, 0)))
    receive_task = cocotb.start_soon(monitor.receive_all(len(test_data), delay_range=(0, 0)))
    
    await send_task
    result = await receive_task
    
    dut._log.info(f"Received output: {result}")
    
    assert result == expected, f"Sort failed! Got {result}, expected {expected}"
    dut._log.info("✓ Already sorted test passed!")

@cocotb.test()
async def test_insertion_sort_reverse_sorted(dut):
    """Test with reverse sorted data (worst case)"""
    
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    test_data = list(range(9, -1, -1))  # Reverse sorted
    expected = sorted(test_data)
    
    dut._log.info(f"Input data (reverse sorted): {test_data}")
    
    driver = ReadyValidDriver(dut.clk_i, dut.data_i, dut.valid_i, dut.ready_o)
    monitor = ReadyValidMonitor(dut.clk_i, dut.data_o, dut.valid_o, dut.ready_i)
    
    send_task = cocotb.start_soon(driver.send_all(test_data, delay_range=(0, 0)))
    receive_task = cocotb.start_soon(monitor.receive_all(len(test_data), delay_range=(0, 0)))
    
    await send_task
    result = await receive_task
    
    dut._log.info(f"Received output: {result}")
    
    assert result == expected, f"Sort failed! Got {result}, expected {expected}"
    dut._log.info("✓ Reverse sorted test passed!")

@cocotb.test()
async def test_insertion_sort_duplicates(dut):
    """Test with duplicate values"""
    
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    test_data = [5, 2, 5, 1, 2, 3, 5, 4, 2, 1]
    expected = sorted(test_data)
    
    dut._log.info(f"Input data (with duplicates): {test_data}")
    
    driver = ReadyValidDriver(dut.clk_i, dut.data_i, dut.valid_i, dut.ready_o)
    monitor = ReadyValidMonitor(dut.clk_i, dut.data_o, dut.valid_o, dut.ready_i)
    
    send_task = cocotb.start_soon(driver.send_all(test_data, delay_range=(0, 0)))
    receive_task = cocotb.start_soon(monitor.receive_all(len(test_data), delay_range=(0, 0)))
    
    await send_task
    result = await receive_task
    
    dut._log.info(f"Received output: {result}")
    
    assert result == expected, f"Sort failed! Got {result}, expected {expected}"
    dut._log.info("✓ Duplicates test passed!")

@cocotb.test()
async def test_insertion_sort_all_same(dut):
    """Test with all identical values"""
    
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    test_data = [42] * 10
    expected = test_data.copy()
    
    dut._log.info(f"Input data (all same): {test_data}")
    
    driver = ReadyValidDriver(dut.clk_i, dut.data_i, dut.valid_i, dut.ready_o)
    monitor = ReadyValidMonitor(dut.clk_i, dut.data_o, dut.valid_o, dut.ready_i)
    
    send_task = cocotb.start_soon(driver.send_all(test_data, delay_range=(0, 0)))
    receive_task = cocotb.start_soon(monitor.receive_all(len(test_data), delay_range=(0, 0)))
    
    await send_task
    result = await receive_task
    
    dut._log.info(f"Received output: {result}")
    
    assert result == expected, f"Sort failed! Got {result}, expected {expected}"
    dut._log.info("✓ All same values test passed!")

@cocotb.test()
async def test_insertion_sort_multiple_runs(dut):
    """Test multiple consecutive sorting operations"""
    
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    for run in range(3):
        test_data = [random.randint(0, 255) for _ in range(10)]
        expected = sorted(test_data)
        
        dut._log.info(f"Run {run+1} - Input data: {test_data}")
        
        driver = ReadyValidDriver(dut.clk_i, dut.data_i, dut.valid_i, dut.ready_o)
        monitor = ReadyValidMonitor(dut.clk_i, dut.data_o, dut.valid_o, dut.ready_i)
        
        send_task = cocotb.start_soon(driver.send_all(test_data, delay_range=(0, 2)))
        receive_task = cocotb.start_soon(monitor.receive_all(len(test_data), delay_range=(0, 2)))
        
        await send_task
        result = await receive_task
        
        dut._log.info(f"Run {run+1} - Received output: {result}")
        
        assert result == expected, f"Run {run+1} failed! Got {result}, expected {expected}"
        
        # Small delay between runs
        await ClockCycles(dut.clk_i, 10)
    
    dut._log.info("✓ Multiple runs test passed!")

@cocotb.test()
async def test_insertion_sort_edge_values(dut):
    """Test with edge values (0 and 255)"""
    
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    test_data = [255, 0, 128, 255, 1, 254, 0, 127, 200, 50]
    expected = sorted(test_data)
    
    dut._log.info(f"Input data (edge values): {test_data}")
    
    driver = ReadyValidDriver(dut.clk_i, dut.data_i, dut.valid_i, dut.ready_o)
    monitor = ReadyValidMonitor(dut.clk_i, dut.data_o, dut.valid_o, dut.ready_i)
    
    send_task = cocotb.start_soon(driver.send_all(test_data, delay_range=(0, 0)))
    receive_task = cocotb.start_soon(monitor.receive_all(len(test_data), delay_range=(0, 0)))
    
    await send_task
    result = await receive_task
    
    dut._log.info(f"Received output: {result}")
    
    assert result == expected, f"Sort failed! Got {result}, expected {expected}"
    dut._log.info("✓ Edge values test passed!")