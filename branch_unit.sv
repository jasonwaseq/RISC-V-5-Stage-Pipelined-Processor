module branch_unit (
    input  logic [31:0] pc,
    input  logic [31:0] rs1_data,
    input  logic [31:0] rs2_data,
    input  riscv_pkg::alu_op_t alu_op,
    input  logic [31:0] imm,
    output logic        branch_taken,
    output logic [31:0] target_addr
);
    
    import riscv_pkg::*;
    
    logic zero;
    logic lt, ltu;
    
    assign zero = (rs1_data == rs2_data);
    assign lt = $signed(rs1_data) < $signed(rs2_data);
    assign ltu = rs1_data < rs2_data;
    
    always_comb begin
        branch_taken = 1'b0;
        case (alu_op)
            ALU_BEQ:  branch_taken = zero;
            ALU_BNE:  branch_taken = !zero;
            ALU_BLT:  branch_taken = lt;
            ALU_BGE:  branch_taken = !lt;
            ALU_BLTU: branch_taken = ltu;
            ALU_BGE:  branch_taken = !ltu;
            default:  branch_taken = 1'b0;
        endcase
    end
    
    assign target_addr = pc + imm;
    
endmodule