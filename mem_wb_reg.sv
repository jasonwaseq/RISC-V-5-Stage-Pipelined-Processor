module mem_wb_reg (
    input  logic        clk,
    input  logic        rst_n,
    input  riscv_pkg::ctrl_signals_t ctrl_in,
    input  logic [31:0] alu_result_in,
    input  logic [31:0] mem_data_in,
    input  logic [4:0]  rd_addr_in,
    input  logic [31:0] pc_plus_4_in,
    output riscv_pkg::ctrl_signals_t ctrl_out,
    output logic [31:0] alu_result_out,
    output logic [31:0] mem_data_out,
    output logic [4:0]  rd_addr_out,
    output logic [31:0] pc_plus_4_out
);
    
    import riscv_pkg::*;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_out <= '0;
            alu_result_out <= 32'b0;
            mem_data_out <= 32'b0;
            rd_addr_out <= 5'b0;
            pc_plus_4_out <= 32'b0;
        end else begin
            ctrl_out <= ctrl_in;
            alu_result_out <= alu_result_in;
            mem_data_out <= mem_data_in;
            rd_addr_out <= rd_addr_in;
            pc_plus_4_out <= pc_plus_4_in;
        end
    end
    
endmodule