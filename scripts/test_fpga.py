#!/usr/bin/env python3
"""
RISC-V FPGA Test Script
Tests the processor on iCEBreaker FPGA via UART

Commands:
  0xAA <addr_hi> <addr_lo> <d3> <d2> <d1> <d0> : Write instruction to memory
  0xBB <reg>                                   : Read register value (returns 4 bytes)
  0xCC                                         : Run processor
  0xDD                                         : Halt processor
  0xEE                                         : Reset processor
"""

import serial
import serial.tools.list_ports
import time
import argparse
import sys

# RV32I instruction encoding helpers
def r_type(funct7, rs2, rs1, funct3, rd, opcode=0b0110011):
    return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def i_type(imm, rs1, funct3, rd, opcode):
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def ADDI(rd, rs1, imm):  return i_type(imm, rs1, 0b000, rd, 0b0010011)
def ADD(rd, rs1, rs2):   return r_type(0b0000000, rs2, rs1, 0b000, rd)
def NOP():               return ADDI(0, 0, 0)


class RISCVTester:
    """RISC-V FPGA tester via UART"""
    
    CMD_WRITE_INSTR = 0xAA
    CMD_READ_REG    = 0xBB
    CMD_RUN         = 0xCC
    CMD_HALT        = 0xDD
    CMD_RESET       = 0xEE
    
    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.1)  # Allow time for connection to stabilize
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
    
    def close(self):
        self.ser.close()
    
    def write_instruction(self, addr, instr):
        """Write a 32-bit instruction to memory at specified address"""
        cmd = bytes([
            self.CMD_WRITE_INSTR,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            (instr >> 24) & 0xFF,
            (instr >> 16) & 0xFF,
            (instr >> 8) & 0xFF,
            instr & 0xFF
        ])
        self.ser.write(cmd)
        # Wait for ACK
        ack = self.ser.read(1)
        if len(ack) == 0:
            raise TimeoutError("Timeout waiting for write ACK")
        if ack[0] != self.CMD_WRITE_INSTR:
            raise ValueError(f"Unexpected ACK: {hex(ack[0])}")
        return True
    
    def read_register(self, reg):
        """Read a register value (returns 32-bit integer)"""
        cmd = bytes([self.CMD_READ_REG, reg & 0x1F])
        self.ser.write(cmd)
        # Read 4 bytes response
        data = self.ser.read(4)
        if len(data) != 4:
            raise TimeoutError(f"Timeout reading register (got {len(data)} bytes)")
        return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
    
    def run_processor(self):
        """Start the processor"""
        self.ser.write(bytes([self.CMD_RUN]))
        ack = self.ser.read(1)
        if len(ack) == 0:
            raise TimeoutError("Timeout waiting for run ACK")
        return ack[0] == self.CMD_RUN
    
    def halt_processor(self):
        """Halt the processor"""
        self.ser.write(bytes([self.CMD_HALT]))
        ack = self.ser.read(1)
        if len(ack) == 0:
            raise TimeoutError("Timeout waiting for halt ACK")
        return ack[0] == self.CMD_HALT
    
    def reset_processor(self):
        """Reset the processor"""
        self.ser.write(bytes([self.CMD_RESET]))
        ack = self.ser.read(1)
        if len(ack) == 0:
            raise TimeoutError("Timeout waiting for reset ACK")
        return ack[0] == self.CMD_RESET
    
    def load_program(self, program, start_addr=0):
        """Load a list of instructions into memory"""
        for i, instr in enumerate(program):
            addr = start_addr + i * 4
            self.write_instruction(addr, instr)
            print(f"  [{addr:04X}] = {instr:08X}")


def find_icebreaker():
    """Find iCEBreaker serial port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'iCEBreaker' in port.description or 'FTDI' in port.description:
            return port.device
        # Linux often shows it as ttyUSB
        if 'ttyUSB' in port.device or 'ttyACM' in port.device:
            return port.device
    return None


def test_basic_alu(tester):
    """Test basic ALU operations"""
    print("\n=== Test: Basic ALU Operations ===")
    
    # Reset and halt
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    # Program:
    # ADDI x1, x0, 10   # x1 = 10
    # ADDI x2, x0, 25   # x2 = 25
    # ADD  x3, x1, x2   # x3 = 35
    program = [
        ADDI(1, 0, 10),
        ADDI(2, 0, 25),
        ADD(3, 1, 2),
        NOP(), NOP(), NOP(), NOP(), NOP()
    ]
    
    print("Loading program...")
    tester.load_program(program)
    
    print("Running processor...")
    tester.run_processor()
    time.sleep(0.1)  # Let it run
    tester.halt_processor()
    
    print("Reading registers...")
    x1 = tester.read_register(1)
    x2 = tester.read_register(2)
    x3 = tester.read_register(3)
    
    print(f"  x1 = {x1} (expected 10)")
    print(f"  x2 = {x2} (expected 25)")
    print(f"  x3 = {x3} (expected 35)")
    
    passed = (x1 == 10) and (x2 == 25) and (x3 == 35)
    print(f"Result: {'PASS' if passed else 'FAIL'}")
    return passed


def test_forwarding(tester):
    """Test data forwarding"""
    print("\n=== Test: Data Forwarding ===")
    
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    # Back-to-back dependencies
    program = [
        ADDI(1, 0, 5),     # x1 = 5
        ADDI(2, 1, 10),    # x2 = x1 + 10 = 15
        ADDI(3, 2, 20),    # x3 = x2 + 20 = 35
        NOP(), NOP(), NOP(), NOP(), NOP()
    ]
    
    print("Loading program...")
    tester.load_program(program)
    
    print("Running processor...")
    tester.run_processor()
    time.sleep(0.1)
    tester.halt_processor()
    
    x1 = tester.read_register(1)
    x2 = tester.read_register(2)
    x3 = tester.read_register(3)
    
    print(f"  x1 = {x1} (expected 5)")
    print(f"  x2 = {x2} (expected 15)")
    print(f"  x3 = {x3} (expected 35)")
    
    passed = (x1 == 5) and (x2 == 15) and (x3 == 35)
    print(f"Result: {'PASS' if passed else 'FAIL'}")
    return passed


def test_all_registers(tester):
    """Test writing to all registers"""
    print("\n=== Test: All Registers ===")
    
    tester.reset_processor()
    time.sleep(0.01)
    tester.halt_processor()
    
    # Write unique value to each register (x1-x31)
    program = [ADDI(i, 0, i * 10) for i in range(1, 16)]
    program.extend([NOP()] * 8)
    
    print("Loading program...")
    tester.load_program(program)
    
    print("Running processor...")
    tester.run_processor()
    time.sleep(0.2)
    tester.halt_processor()
    
    print("Checking registers...")
    passed = True
    for i in range(1, 16):
        val = tester.read_register(i)
        expected = i * 10
        status = "OK" if val == expected else "FAIL"
        print(f"  x{i:2d} = {val:5d} (expected {expected:5d}) [{status}]")
        if val != expected:
            passed = False
    
    # x0 should always be 0
    x0 = tester.read_register(0)
    print(f"  x0  = {x0:5d} (expected 0) [{'OK' if x0 == 0 else 'FAIL'}]")
    if x0 != 0:
        passed = False
    
    print(f"Result: {'PASS' if passed else 'FAIL'}")
    return passed


def interactive_mode(tester):
    """Interactive command mode"""
    print("\n=== Interactive Mode ===")
    print("Commands:")
    print("  w <addr> <instr>  - Write instruction (hex)")
    print("  r <reg>           - Read register")
    print("  run               - Run processor")
    print("  halt              - Halt processor")
    print("  reset             - Reset processor")
    print("  q                 - Quit")
    print()
    
    while True:
        try:
            cmd = input("> ").strip().split()
            if not cmd:
                continue
            
            if cmd[0] == 'q':
                break
            elif cmd[0] == 'w' and len(cmd) == 3:
                addr = int(cmd[1], 16)
                instr = int(cmd[2], 16)
                tester.write_instruction(addr, instr)
                print(f"Wrote {instr:08X} to address {addr:04X}")
            elif cmd[0] == 'r' and len(cmd) == 2:
                reg = int(cmd[1])
                val = tester.read_register(reg)
                print(f"x{reg} = {val} (0x{val:08X})")
            elif cmd[0] == 'run':
                tester.run_processor()
                print("Processor running")
            elif cmd[0] == 'halt':
                tester.halt_processor()
                print("Processor halted")
            elif cmd[0] == 'reset':
                tester.reset_processor()
                print("Processor reset")
            else:
                print("Unknown command")
        except Exception as e:
            print(f"Error: {e}")


def main():
    parser = argparse.ArgumentParser(description='RISC-V FPGA Tester')
    parser.add_argument('port', nargs='?', default=None, help='Serial port (auto-detect if not specified)')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('-i', '--interactive', action='store_true', help='Interactive mode')
    parser.add_argument('-t', '--test', choices=['all', 'alu', 'forward', 'regs'],
                        default='all', help='Test to run')
    args = parser.parse_args()
    
    # Find port
    port = args.port or find_icebreaker()
    if not port:
        print("Error: Could not find iCEBreaker. Specify port with -p")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        sys.exit(1)
    
    print(f"Connecting to {port} at {args.baud} baud...")
    
    try:
        tester = RISCVTester(port, args.baud)
        
        if args.interactive:
            interactive_mode(tester)
        else:
            print("\n" + "=" * 50)
            print("RISC-V 5-Stage Pipeline FPGA Test")
            print("=" * 50)
            
            results = []
            
            if args.test in ['all', 'alu']:
                results.append(('Basic ALU', test_basic_alu(tester)))
            
            if args.test in ['all', 'forward']:
                results.append(('Forwarding', test_forwarding(tester)))
            
            if args.test in ['all', 'regs']:
                results.append(('All Registers', test_all_registers(tester)))
            
            print("\n" + "=" * 50)
            print("Summary:")
            print("=" * 50)
            all_passed = True
            for name, passed in results:
                status = "PASS" if passed else "FAIL"
                print(f"  {name}: {status}")
                if not passed:
                    all_passed = False
            
            print()
            if all_passed:
                print("All tests PASSED!")
            else:
                print("Some tests FAILED!")
                sys.exit(1)
        
        tester.close()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(0)


if __name__ == '__main__':
    main()
