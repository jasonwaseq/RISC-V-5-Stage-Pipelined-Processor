module if_id_reg (
    clk,
    rst_n,
    stall,
    flush,
    pc_in,
    instr_in,
    pc_out,
    instr_out
);
    input clk;
    input rst_n;
    input stall;
    input flush;
    input [31:0] pc_in;
    input [31:0] instr_in;
    output [31:0] pc_out;
    output [31:0] instr_out;
    reg [31:0] pc_out;
    reg [31:0] instr_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc_out <= 32'b0;
            instr_out <= 32'b0;
        end else if (flush) begin
            pc_out <= 32'b0;
            instr_out <= 32'b0;
        end else if (!stall) begin
            pc_out <= pc_in;
            instr_out <= instr_in;
        end
    end
endmodule