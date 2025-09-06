module reg_file (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        we,
    input  logic [4:0]  addr_rs1,
    input  logic [4:0]  addr_rs2,
    input  logic [4:0]  addr_rd,
    input  logic [31:0] data_rd,
    output logic [31:0] data_rs1,
    output logic [31:0] data_rs2
);
    
    logic [31:0] registers [0:31];
    
    // Read ports
    assign data_rs1 = (addr_rs1 == 0) ? 32'b0 : registers[addr_rs1];
    assign data_rs2 = (addr_rs2 == 0) ? 32'b0 : registers[addr_rs2];
    
    // Write port
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 32; i++) begin
                registers[i] <= 32'b0;
            end
        end else if (we && (addr_rd != 0)) begin
            registers[addr_rd] <= data_rd;
        end
    end
    
endmodule