module alu (
    input  logic [31:0] a,
    input  logic [31:0] b,
    input  riscv_pkg::alu_op_t alu_op,
    output logic [31:0] result,
    output logic        zero
);
    
    import riscv_pkg::*;
    
    logic [31:0] b_actual;
    logic [31:0] diff;
    logic        lt, ltu;
    
    assign b_actual = (alu_op == ALU_SUB) ? ~b + 1 : b;
    assign diff = a + b_actual;
    assign lt = $signed(a) < $signed(b);
    assign ltu = a < b;
    
    always_comb begin
        case (alu_op)
            ALU_ADD:  result = a + b;
            ALU_SUB:  result = a - b;
            ALU_AND:  result = a & b;
            ALU_OR:   result = a | b;
            ALU_XOR:  result = a ^ b;
            ALU_SLL:  result = a << b[4:0];
            ALU_SRL:  result = a >> b[4:0];
            ALU_SRA:  result = $signed(a) >>> b[4:0];
            ALU_SLT:  result = {31'b0, lt};
            ALU_SLTU: result = {31'b0, ltu};
            ALU_LUI:  result = b;
            default:  result = 32'b0;
        endcase
    end
    
    assign zero = (result == 32'b0);
    
endmodule