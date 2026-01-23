# FPGA RISC-V 5-Stage Pipelined Processor

A complete RISC-V 5-stage pipelined processor implementation designed for FPGA deployment on the iCEBreaker (iCE40UP5K) board. This project implements the classic textbook pipeline architecture with Fetch, Decode, Execute, Memory, and Writeback stages, supporting the RV32I base integer instruction set. The processor features proper pipeline hazard handling through data forwarding and automatic stall insertion, enabling efficient execution of back-to-back dependent instructions. A UART-based interface allows real-time program loading, register inspection, and processor control at 115200 baud, making it ideal for educational purposes and embedded applications. The design includes comprehensive testing infrastructure with both simulation testbenches and hardware validation scripts, along with an interactive mode that enables users to assemble and execute RISC-V instructions directly on the FPGA. With a compact footprint of approximately 2500 LUTs, the processor demonstrates efficient resource utilization while maintaining full pipeline functionality and includes 15 example programs showcasing arithmetic, logical, shift, comparison, and data hazard scenarios.

## Features

- **5-stage pipeline**: Fetch (F), Decode (D), Execute (X), Memory (M), Writeback (W)
- **RV32I base instruction set**: All integer instructions supported
- **Data forwarding**: Reduces stalls for data hazards
- **Load-use hazard detection**: Automatic stall insertion
- **Branch handling**: Flush on taken branch
- **UART interface**: Load programs and read registers via serial

## Supported Instructions

| Type | Instructions |
|------|-------------|
| R-type | ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU |
| I-type | ADDI, ANDI, ORI, XORI, SLTI, SLTIU, SLLI, SRLI, SRAI |
| Load | LB, LH, LW, LBU, LHU |
| Store | SB, SH, SW |
| Branch | BEQ, BNE, BLT, BGE, BLTU, BGEU |
| Jump | JAL, JALR |
| Upper | LUI, AUIPC |

## Project Structure

```
├── rtl/                    # RTL source files
│   ├── riscv_core.sv       # Processor core (all 5 stages)
│   ├── riscv_top.sv        # Top-level with UART interface
│   ├── memory.sv           # Synchronous RAM module
│   ├── uart.v              # UART module
│   ├── uart_tx.v           # UART transmitter
│   └── uart_rx.v           # UART receiver
├── test/                   # Testbench
│   └── test_riscv.py       # Cocotb testbench
├── scripts/                # Utility scripts
│   └── test_fpga.py        # FPGA test via UART
├── synth/                  # Synthesis files
│   └── icebreaker.pcf      # Pin constraints
├── Makefile                # Build automation
└── README.md
```

## Pipeline Architecture

```
┌─────┐   ┌─────┐   ┌─────┐   ┌─────┐   ┌─────┐
│  F  │──▶│  D  │──▶│  X  │──▶│  M  │──▶│  W  │
└─────┘   └─────┘   └─────┘   └─────┘   └─────┘
 Fetch    Decode   Execute   Memory   Writeback
```

**Fetch (F)**: Read instruction from memory, update PC  
**Decode (D)**: Decode instruction, read registers, generate immediate  
**Execute (X)**: ALU operation, branch decision, forwarding  
**Memory (M)**: Load/store data memory access  
**Writeback (W)**: Write result back to register file

## Requirements

### Simulation
- Python 3.8+
- cocotb
- Icarus Verilog or Verilator

### Synthesis
- Yosys
- nextpnr-ice40
- icestorm tools

### FPGA Testing
- iCEBreaker FPGA board
- pyserial (`pip install pyserial`)

## Quick Start

### Simulation

```bash
# Run all cocotb tests
make sim

# Run specific test
make sim TESTCASE=test_addi
```

### Synthesis

```bash
# Build bitstream
make build

# Program FPGA
make prog
```

### FPGA Testing

```bash
# Run all hardware tests
python scripts/test_fpga.py

# Interactive mode
python scripts/test_fpga.py -i

# Specify port
python scripts/test_fpga.py -p /dev/ttyUSB0
```

## UART Protocol

The FPGA communicates at 115200 baud (8N1).

| Command | Format | Description |
|---------|--------|-------------|
| Write Instr | `0xAA <addr_hi> <addr_lo> <d3> <d2> <d1> <d0>` | Write 32-bit instruction |
| Read Reg | `0xBB <reg>` | Read register (returns 4 bytes) |
| Run | `0xCC` | Start processor |
| Halt | `0xDD` | Stop processor |
| Reset | `0xEE` | Reset processor |

## Resource Usage (iCE40UP5K)

| Resource | Used | Available |
|----------|------|-----------|
| LUTs | ~2500 | 5280 |
| FFs | ~800 | 5280 |
| BRAM | 4 | 30 |

## Example Program

Simple program that adds two numbers:

```assembly
# x1 = 10, x2 = 25, x3 = x1 + x2 = 35
addi x1, x0, 10     # 0x00A00093
addi x2, x0, 25     # 0x01900113
add  x3, x1, x2     # 0x002081B3
```

Load via UART:
```python
tester.write_instruction(0x00, 0x00A00093)
tester.write_instruction(0x04, 0x01900113)
tester.write_instruction(0x08, 0x002081B3)
tester.run_processor()
# After execution: x3 = 35
```

## License

MIT License
