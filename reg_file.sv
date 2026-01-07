module reg_file (
    clk,
    rst_n,
    we,
    addr_rs1,
    addr_rs2,
    addr_rd,
    data_rd,
    data_rs1,
    data_rs2
);
    input clk;
    input rst_n;
    input we;
    input [4:0] addr_rs1;
    input [4:0] addr_rs2;
    input [4:0] addr_rd;
    input [31:0] data_rd;
    output [31:0] data_rs1;
    output [31:0] data_rs2;
    reg [31:0] registers [0:31];
    wire [31:0] data_rs1;
    wire [31:0] data_rs2;

    assign data_rs1 = (addr_rs1 == 0) ? 32'b0 : registers[addr_rs1];
    assign data_rs2 = (addr_rs2 == 0) ? 32'b0 : registers[addr_rs2];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (we && (addr_rd != 0)) begin
            registers[addr_rd] <= data_rd;
        end
    end
endmodule