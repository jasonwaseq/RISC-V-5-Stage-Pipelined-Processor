module imm_gen (
    instr,
    imm_out
);
    input [31:0] instr;
    output [31:0] imm_out;
    reg [31:0] imm_out;

    wire [6:0] opcode;
    assign opcode = instr[6:0];

    // Opcode parameters
    parameter OP_LOAD   = 7'b0000011;
    parameter OP_STORE  = 7'b0100011;
    parameter OP_BRANCH = 7'b1100011;
    parameter OP_JALR   = 7'b1100111;
    parameter OP_JAL    = 7'b1101111;
    parameter OP_OP_IMM = 7'b0010011;
    parameter OP_OP     = 7'b0110011;
    parameter OP_LUI    = 7'b0110111;
    parameter OP_AUIPC  = 7'b0010111;

    always @* begin
        case (opcode)
            OP_LOAD:     // I-type
                imm_out = {{20{instr[31]}}, instr[31:20]};
            OP_OP_IMM:   // I-type
                imm_out = {{20{instr[31]}}, instr[31:20]};
            OP_STORE:    // S-type
                imm_out = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            OP_BRANCH:   // B-type
                imm_out = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
            OP_JAL:      // J-type
                imm_out = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
            OP_JALR:     // I-type
                imm_out = {{20{instr[31]}}, instr[31:20]};
            OP_LUI:      // U-type
                imm_out = {instr[31:12], 12'b0};
            OP_AUIPC:    // U-type
                imm_out = {instr[31:12], 12'b0};
            default:
                imm_out = 32'b0;
        endcase
    end
endmodule