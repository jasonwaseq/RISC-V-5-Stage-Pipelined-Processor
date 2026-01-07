module branch_unit (
    pc,
    rs1_data,
    rs2_data,
    alu_op,
    imm,
    branch_taken,
    target_addr
);
    input [31:0] pc;
    input [31:0] rs1_data;
    input [31:0] rs2_data;
    input [3:0] alu_op;
    input [31:0] imm;
    output branch_taken;
    output [31:0] target_addr;
    reg branch_taken;
    wire [31:0] target_addr;

    // ALU op parameters
    parameter ALU_BEQ  = 4'b1011;
    parameter ALU_BNE  = 4'b1100;
    parameter ALU_BLT  = 4'b1101;
    parameter ALU_BGE  = 4'b1110;
    parameter ALU_BLTU = 4'b1111;

    wire zero, lt, ltu;
    assign zero = (rs1_data == rs2_data);
    assign lt = ($signed(rs1_data) < $signed(rs2_data));
    assign ltu = (rs1_data < rs2_data);

    always @* begin
        branch_taken = 0;
        case (alu_op)
            ALU_BEQ:  branch_taken = zero;
            ALU_BNE:  branch_taken = ~zero;
            ALU_BLT:  branch_taken = lt;
            ALU_BGE:  branch_taken = ~lt;
            ALU_BLTU: branch_taken = ltu;
            ALU_BGE:  branch_taken = ~ltu;
            default:  branch_taken = 0;
        endcase
    end

    assign target_addr = pc + imm;
endmodule