// Simple synchronous RAM with single read/write port
// Used for both instruction and data memory

module memory #(
    parameter DEPTH = 256,    // Number of 32-bit words
    parameter INIT_FILE = ""  // Optional hex file for initialization
) (
    input  logic        clk,
    input  logic [31:0] addr,
    input  logic [31:0] wdata,
    output logic [31:0] rdata,
    input  logic        we,
    input  logic [3:0]  be       // Byte enables
);

    localparam ADDR_BITS = $clog2(DEPTH);
    
    logic [31:0] mem [0:DEPTH-1];
    logic [ADDR_BITS-1:0] word_addr;
    
    assign word_addr = addr[ADDR_BITS+1:2];  // Word-aligned addressing
    
    // Initialize memory
    initial begin
        for (int i = 0; i < DEPTH; i++) mem[i] = 32'd0;
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end
    
    // Synchronous read and write
    always_ff @(posedge clk) begin
        if (we) begin
            if (be[0]) mem[word_addr][7:0]   <= wdata[7:0];
            if (be[1]) mem[word_addr][15:8]  <= wdata[15:8];
            if (be[2]) mem[word_addr][23:16] <= wdata[23:16];
            if (be[3]) mem[word_addr][31:24] <= wdata[31:24];
        end
        rdata <= mem[word_addr];
    end

endmodule
