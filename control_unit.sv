module control_unit (
    opcode,
    funct3,
    funct7,
    reg_write,
    mem_to_reg,
    mem_write,
    mem_read,
    branch,
    alu_src,
    alu_src_b,
    jump,
    alu_op,
    funct3_out
);
    input [6:0] opcode;
    input [2:0] funct3;
    input [6:0] funct7;
    output reg reg_write, mem_to_reg, mem_write, mem_read, branch, alu_src;
    output reg [1:0] alu_src_b, jump;
    output reg [3:0] alu_op;
    output reg [2:0] funct3_out;

    // Parameters for opcodes and ALU ops
    parameter OP_LOAD   = 7'b0000011;
    parameter OP_STORE  = 7'b0100011;
    parameter OP_BRANCH = 7'b1100011;
    parameter OP_JALR   = 7'b1100111;
    parameter OP_JAL    = 7'b1101111;
    parameter OP_OP_IMM = 7'b0010011;
    parameter OP_OP     = 7'b0110011;
    parameter OP_LUI    = 7'b0110111;
    parameter OP_AUIPC  = 7'b0010111;

    parameter ALU_ADD  = 4'b0000;
    parameter ALU_SUB  = 4'b0001;
    parameter ALU_AND  = 4'b0010;
    parameter ALU_OR   = 4'b0011;
    parameter ALU_XOR  = 4'b0100;
    parameter ALU_SLL  = 4'b0101;
    parameter ALU_SRL  = 4'b0110;
    parameter ALU_SRA  = 4'b0111;
    parameter ALU_SLT  = 4'b1000;
    parameter ALU_SLTU = 4'b1001;
    parameter ALU_LUI  = 4'b1010;
    parameter ALU_BEQ  = 4'b1011;
    parameter ALU_BNE  = 4'b1100;
    parameter ALU_BLT  = 4'b1101;
    parameter ALU_BGE  = 4'b1110;
    parameter ALU_BLTU = 4'b1111;

    always @* begin
        // Default values
        reg_write = 0;
        mem_to_reg = 0;
        mem_write = 0;
        mem_read = 0;
        branch = 0;
        alu_src = 0;
        alu_src_b = 2'b00;
        jump = 2'b00;
        alu_op = ALU_ADD;
        funct3_out = funct3;

        case (opcode)
            OP_LOAD: begin
                reg_write = 1;
                mem_to_reg = 1;
                mem_read = 1;
                alu_src = 1;
                alu_op = ALU_ADD;
            end
            OP_STORE: begin
                mem_write = 1;
                alu_src = 1;
                alu_op = ALU_ADD;
            end
            OP_BRANCH: begin
                branch = 1;
                case (funct3)
                    3'b000: alu_op = ALU_BEQ;
                    3'b001: alu_op = ALU_BNE;
                    3'b100: alu_op = ALU_BLT;
                    3'b101: alu_op = ALU_BGE;
                    3'b110: alu_op = ALU_BLTU;
                    3'b111: alu_op = ALU_BGE;
                endcase
            end
            OP_JALR: begin
                reg_write = 1;
                jump = 2'b10;
                alu_src = 1;
            end
            OP_JAL: begin
                reg_write = 1;
                jump = 2'b01;
            end
            OP_OP_IMM: begin
                reg_write = 1;
                alu_src = 1;
                case (funct3)
                    3'b000: alu_op = ALU_ADD;
                    3'b010: alu_op = ALU_SLT;
                    3'b011: alu_op = ALU_SLTU;
                    3'b100: alu_op = ALU_XOR;
                    3'b110: alu_op = ALU_OR;
                    3'b111: alu_op = ALU_AND;
                    3'b001: alu_op = ALU_SLL;
                    3'b101: alu_op = (funct7[5] ? ALU_SRA : ALU_SRL);
                endcase
            end
            OP_OP: begin
                reg_write = 1;
                case ({funct7[5], funct3})
                    4'b0000: alu_op = ALU_ADD;
                    4'b1000: alu_op = ALU_SUB;
                    4'b0001: alu_op = ALU_SLL;
                    4'b0010: alu_op = ALU_SLT;
                    4'b0011: alu_op = ALU_SLTU;
                    4'b0100: alu_op = ALU_XOR;
                    4'b0101: alu_op = ALU_SRL;
                    4'b1101: alu_op = ALU_SRA;
                    4'b0110: alu_op = ALU_OR;
                    4'b0111: alu_op = ALU_AND;
                endcase
            end
            OP_LUI: begin
                reg_write = 1;
                alu_src_b = 2'b10;
                alu_op = ALU_LUI;
            end
            OP_AUIPC: begin
                reg_write = 1;
                alu_src_b = 2'b01;
                alu_op = ALU_ADD;
            end
        endcase
    end
endmodule