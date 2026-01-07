`timescale 1ns/1ps
module sorter_tb;
  localparam int WIDTH = 16;
  localparam int NMAX  = 16;

  logic clk, rst;
  initial begin clk = 0; forever #5 clk = ~clk; end
  initial begin rst = 1; #50 rst = 0; end

  // DUT
  logic [WIDTH-1:0] s_tdata;
  logic             s_tvalid, s_tready, s_tlast;
  logic [WIDTH-1:0] m_tdata;
  logic             m_tvalid, m_tready, m_tlast;
  logic             busy;

  radix_sort_10b #(
    .WIDTH_P(WIDTH),
    .RADIX_BITS_P(10),
    .N_MAX_P(NMAX)
  ) dut (
    .clk_i(clk),
    .reset_i(rst),
    .s_tdata(s_tdata),
    .s_tvalid(s_tvalid),
    .s_tready(s_tready),
    .s_tlast(s_tlast),
    .m_tdata(m_tdata),
    .m_tvalid(m_tvalid),
    .m_tready(m_tready),
    .m_tlast(m_tlast),
    .busy_o(busy)
  );

  // Stimulus
  int N = 10;
  int unsigned values [0:N-1];
  initial begin
    values[0]=10'h1A3; values[1]=10'h000; values[2]=10'h3FF; values[3]=10'h155; values[4]=10'h02A;
    values[5]=10'h200; values[6]=10'h0F0; values[7]=10'h001; values[8]=10'h3FE; values[9]=10'h100;

    @(negedge rst);
    // Drive input stream
    s_tvalid = 0; s_tlast = 0; m_tready = 1;
    @(posedge clk);
    for (int i=0; i<N; i++) begin
      s_tdata  = {6'b0, values[i]};
      s_tvalid = 1;
      s_tlast  = (i==N-1);
      wait(s_tready);
      @(posedge clk);
      s_tvalid = 0; s_tlast = 0;
      @(posedge clk);
    end

    // Collect outputs
    int unsigned got [0:N-1];
    int idx = 0;
    while (idx < N) begin
      if (m_tvalid) begin
        got[idx] = m_tdata[9:0];
        idx++;
      end
      @(posedge clk);
    end

    // Check sorted
    for (int i=1; i<N; i++) begin
      if (got[i-1] > got[i]) begin
        $error("Not sorted at %0d: %0d > %0d", i, got[i-1], got[i]);
      end
    end

    $display("Sorted output:");
    for (int i=0; i<N; i++) $display("%0d", got[i]);
    #100 $finish;
  end

endmodule
