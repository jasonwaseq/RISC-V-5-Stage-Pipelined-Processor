`timescale 1ns / 1ps

module tb_riscv;
    
    // Clock and reset
    logic clk;
    logic rst_n;
    
    // DUT interfaces
    logic [31:0] pc;
    logic        mem_write;
    logic        mem_read;
    logic [31:0] mem_addr;
    logic [31:0] mem_data_out;
    
    // Memory interfaces
    logic [31:0] instr_data;
    logic [31:0] mem_data_in;
    
    // Test control
    int cycle_count;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Reset generation
    initial begin
        rst_n = 0;
        #20 rst_n = 1;
    end
    
    // Simple instruction memory (direct array)
    logic [31:0] instr_mem [0:1023];
    initial begin
        // Initialize with NOPs (addi x0, x0, 0)
        for (int i = 0; i < 1024; i++) begin
            instr_mem[i] = 32'h00000013;
        end
        
        // Simple test program: ALU operations
        // addi x1, x0, 5
        instr_mem[0] = 32'h00500093;
        // addi x2, x0, 3  
        instr_mem[1] = 32'h00300113;
        // add x3, x1, x2
        instr_mem[2] = 32'h002081b3;
        // addi x4, x0, 10 (test completion)
        instr_mem[3] = 32'h00a00213;
    end
    
    assign instr_data = instr_mem[pc[11:2]];
    
    // Simple data memory
    logic [31:0] data_mem [0:1023];
    initial begin
        for (int i = 0; i < 1024; i++) begin
            data_mem[i] = 32'h0;
        end
    end
    
    always @(posedge clk) begin
        if (mem_write) begin
            data_mem[mem_addr[11:2]] <= mem_data_out;
        end
    end
    
    assign mem_data_in = data_mem[mem_addr[11:2]];
    
    // Instantiate DUT
    riscv_core dut (
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
    
    // Monitor
    always @(posedge clk) begin
        if (rst_n) begin
            cycle_count++;
            
            // Monitor register writes
            if (dut.reg_write && dut.wb_rd_addr != 0) begin
                $display("Cycle %0d: Write x%0d = 0x%08x", 
                        cycle_count, dut.wb_rd_addr, dut.write_data);
                
                // Check for test completion
                if (dut.wb_rd_addr == 4 && dut.write_data == 32'h0000000a) begin
                    $display("TEST COMPLETED SUCCESSFULLY!");
                    $display("Total cycles: %0d", cycle_count);
                    #100;
                    $finish;
                end
            end
            
            // Check for timeout
            if (cycle_count > 100) begin
                $display("TEST TIMEOUT!");
                $finish;
            end
        end
    end
    
    // Waveform dumping
    initial begin
        $dumpfile("riscv_tb.vcd");
        $dumpvars(0, tb_riscv);
        #1000;
        $display("SIMULATION TIMEOUT");
        $finish;
    end
    
    initial begin
        $display("Starting RISC-V Simulation...");
        #1000;
        $display("Simulation completed");
        $finish;
    end
    
endmodule