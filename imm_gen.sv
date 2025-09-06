module imm_gen (
    input  logic [31:0] instr,
    output logic [31:0] imm_out
);
    
    logic [6:0] opcode;
    
    assign opcode = instr[6:0];
    
    always_comb begin
        case (opcode)
            riscv_pkg::OP_LOAD:     // I-type
                imm_out = {{20{instr[31]}}, instr[31:20]};
            riscv_pkg::OP_OP_IMM:   // I-type
                imm_out = {{20{instr[31]}}, instr[31:20]};
            riscv_pkg::OP_STORE:    // S-type
                imm_out = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            riscv_pkg::OP_BRANCH:   // B-type
                imm_out = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
            riscv_pkg::OP_JAL:      // J-type
                imm_out = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
            riscv_pkg::OP_JALR:     // I-type
                imm_out = {{20{instr[31]}}, instr[31:20]};
            riscv_pkg::OP_LUI:      // U-type
                imm_out = {instr[31:12], 12'b0};
            riscv_pkg::OP_AUIPC:    // U-type
                imm_out = {instr[31:12], 12'b0};
            default:
                imm_out = 32'b0;
        endcase
    end
    
endmodule