#!/usr/bin/env python3
"""
Debug script for RISC-V FPGA - minimal test
"""
import serial
import time
import sys

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
    
    print(f"Opening {port}...")
    ser = serial.Serial(port, 115200, timeout=2.0)
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    # Reset processor
    print("\n1. Reset processor (0xEE)...")
    ser.write(bytes([0xEE]))
    ack = ser.read(1)
    print(f"   ACK: {ack.hex() if ack else 'TIMEOUT'}")
    
    time.sleep(0.05)
    
    # Halt processor (keep it stopped while we write)
    print("\n2. Halt processor (0xDD)...")
    ser.write(bytes([0xDD]))
    ack = ser.read(1)
    print(f"   ACK: {ack.hex() if ack else 'TIMEOUT'}")
    
    time.sleep(0.05)
    
    # Write instruction: ADDI x1, x0, 42 at address 0
    # Encoding: imm[11:0] | rs1 | funct3 | rd | opcode
    # ADDI x1, x0, 42 = 42<<20 | 0<<15 | 0<<12 | 1<<7 | 0x13
    # = 0x02A00093
    instr = 0x02A00093
    addr = 0x0000
    
    print(f"\n3. Write instruction 0x{instr:08X} to addr 0x{addr:04X}...")
    cmd = bytes([
        0xAA,                      # Write command
        (addr >> 8) & 0xFF,        # Address high
        addr & 0xFF,               # Address low
        (instr >> 24) & 0xFF,      # Data byte 3 (MSB)
        (instr >> 16) & 0xFF,      # Data byte 2
        (instr >> 8) & 0xFF,       # Data byte 1
        instr & 0xFF               # Data byte 0 (LSB)
    ])
    print(f"   Sending: {cmd.hex()}")
    ser.write(cmd)
    ack = ser.read(1)
    print(f"   ACK: {ack.hex() if ack else 'TIMEOUT'}")
    
    time.sleep(0.05)
    
    # Write NOP instructions at subsequent addresses
    nop = 0x00000013
    for offset in range(4, 32, 4):
        cmd = bytes([
            0xAA,
            (offset >> 8) & 0xFF,
            offset & 0xFF,
            (nop >> 24) & 0xFF,
            (nop >> 16) & 0xFF,
            (nop >> 8) & 0xFF,
            nop & 0xFF
        ])
        ser.write(cmd)
        ack = ser.read(1)
        if not ack or ack[0] != 0xAA:
            print(f"   Write NOP at {offset}: FAILED")
            return
    print(f"   Wrote NOPs at addresses 4-28")
    
    # Read register x1 before running (should be 0)
    print("\n4. Read x1 before running...")
    ser.write(bytes([0xBB, 0x01]))
    data = ser.read(4)
    if len(data) == 4:
        val = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
        print(f"   x1 = {val} (0x{val:08X}) - raw bytes: {data.hex()}")
    else:
        print(f"   TIMEOUT (got {len(data)} bytes: {data.hex() if data else 'none'})")
    
    # Run processor
    print("\n5. Run processor (0xCC)...")
    ser.write(bytes([0xCC]))
    ack = ser.read(1)
    print(f"   ACK: {ack.hex() if ack else 'TIMEOUT'}")
    
    # Wait for processor to execute
    print("\n6. Waiting 200ms for execution...")
    time.sleep(0.2)
    
    # Halt processor
    print("\n7. Halt processor (0xDD)...")
    ser.write(bytes([0xDD]))
    ack = ser.read(1)
    print(f"   ACK: {ack.hex() if ack else 'TIMEOUT'}")
    
    time.sleep(0.05)
    
    # Read register x1 (should be 42)
    print("\n8. Read x1 after running...")
    ser.write(bytes([0xBB, 0x01]))
    data = ser.read(4)
    if len(data) == 4:
        val = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
        print(f"   x1 = {val} (0x{val:08X}) - raw bytes: {data.hex()}")
        if val == 42:
            print("   ✓ SUCCESS!")
        else:
            print(f"   ✗ FAIL - expected 42")
    else:
        print(f"   TIMEOUT (got {len(data)} bytes: {data.hex() if data else 'none'})")
    
    # Also read x0 (should always be 0)
    print("\n9. Read x0 (should be 0)...")
    ser.write(bytes([0xBB, 0x00]))
    data = ser.read(4)
    if len(data) == 4:
        val = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
        print(f"   x0 = {val} (0x{val:08X})")
    else:
        print(f"   TIMEOUT (got {len(data)} bytes)")
    
    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
