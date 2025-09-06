module riscv_top (
    input  logic        clk,
    input  logic        rst_n,
    output logic [31:0] pc,
    output logic        mem_write,
    output logic        mem_read
);
    
    // Memory interfaces
    logic [31:0] instr_data;
    logic [31:0] mem_data_in;
    logic [31:0] mem_addr;
    logic [31:0] mem_data_out;
    
    // Instruction memory (simplified)
    logic [31:0] instr_mem [0:1023];
    assign instr_data = instr_mem[pc[11:2]];
    
    // Data memory (simplified)
    logic [31:0] data_mem [0:1023];
    always_ff @(posedge clk) begin
        if (mem_write) begin
            data_mem[mem_addr[11:2]] <= mem_data_out;
        end
    end
    assign mem_data_in = data_mem[mem_addr[11:2]];
    
    // RISC-V core
    riscv_core core (
        .clk(clk),
        .rst_n(rst_n),
        .instr_data(instr_data),
        .mem_data_in(mem_data_in),
        .pc(pc),
        .instr_read(),
        .mem_addr(mem_addr),
        .mem_data_out(mem_data_out),
        .mem_write(mem_write),
        .mem_read(mem_read)
    );
    
endmodule