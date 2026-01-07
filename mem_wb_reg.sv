module mem_wb_reg (
    clk,
    rst_n,
    // Flattened control signals
    reg_write_in,
    mem_to_reg_in,
    mem_write_in,
    mem_read_in,
    branch_in,
    alu_src_in,
    alu_src_b_in,
    jump_in,
    alu_op_in,
    funct3_in,
    alu_result_in,
    mem_data_in,
    rd_addr_in,
    pc_plus_4_in,
    // Outputs
    reg_write_out,
    mem_to_reg_out,
    mem_write_out,
    mem_read_out,
    branch_out,
    alu_src_out,
    alu_src_b_out,
    jump_out,
    alu_op_out,
    funct3_out,
    alu_result_out,
    mem_data_out,
    rd_addr_out,
    pc_plus_4_out
);
    input clk;
    input rst_n;
    input reg_write_in, mem_to_reg_in, mem_write_in, mem_read_in, branch_in, alu_src_in;
    input [1:0] alu_src_b_in, jump_in;
    input [3:0] alu_op_in;
    input [2:0] funct3_in;
    input [31:0] alu_result_in, mem_data_in, pc_plus_4_in;
    input [4:0] rd_addr_in;
    output reg_write_out, mem_to_reg_out, mem_write_out, mem_read_out, branch_out, alu_src_out;
    output [1:0] alu_src_b_out, jump_out;
    output [3:0] alu_op_out;
    output [2:0] funct3_out;
    output [31:0] alu_result_out, mem_data_out, pc_plus_4_out;
    output [4:0] rd_addr_out;
    reg reg_write_out, mem_to_reg_out, mem_write_out, mem_read_out, branch_out, alu_src_out;
    reg [1:0] alu_src_b_out, jump_out;
    reg [3:0] alu_op_out;
    reg [2:0] funct3_out;
    reg [31:0] alu_result_out, mem_data_out, pc_plus_4_out;
    reg [4:0] rd_addr_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_write_out <= 0;
            mem_to_reg_out <= 0;
            mem_write_out <= 0;
            mem_read_out <= 0;
            branch_out <= 0;
            alu_src_out <= 0;
            alu_src_b_out <= 0;
            jump_out <= 0;
            alu_op_out <= 0;
            funct3_out <= 0;
            alu_result_out <= 0;
            mem_data_out <= 0;
            rd_addr_out <= 0;
            pc_plus_4_out <= 0;
        end else begin
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
            mem_write_out <= mem_write_in;
            mem_read_out <= mem_read_in;
            branch_out <= branch_in;
            alu_src_out <= alu_src_in;
            alu_src_b_out <= alu_src_b_in;
            jump_out <= jump_in;
            alu_op_out <= alu_op_in;
            funct3_out <= funct3_in;
            alu_result_out <= alu_result_in;
            mem_data_out <= mem_data_in;
            rd_addr_out <= rd_addr_in;
            pc_plus_4_out <= pc_plus_4_in;
        end
    end
endmodule