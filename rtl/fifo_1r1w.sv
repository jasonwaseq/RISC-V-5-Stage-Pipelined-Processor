module fifo_1r1w
  #(parameter [31:0] width_p = 8
   // Note: Not depth_p! depth_p should be 1<<depth_log2_p
   ,parameter [31:0] depth_log2_p = 8
   )
  (input [0:0] clk_i
  ,input [0:0] reset_i

  ,input [width_p - 1:0] data_i
  ,input [0:0] valid_i
  ,output [0:0] ready_o 

  ,output [0:0] valid_o 
  ,output [width_p - 1:0] data_o 
  ,input [0:0] ready_i
  );

  logic [depth_log2_p-1:0] wr_ptr_r, rd_ptr_r;
  logic [depth_log2_p:0] count_r; 

  logic wr_en;
  assign wr_en = valid_i & ready_o;
  logic rd_en;
  assign rd_en = valid_o & ready_i;

  assign ready_o = (count_r < (1 << depth_log2_p));
  assign valid_o = (count_r > 0);

  logic [width_p-1:0] rd_data_w;
  ram_1r1w_async #(
    .width_p(width_p),
    .depth_p(1 << depth_log2_p),
    .filename_p("")
  ) ram (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .wr_valid_i(wr_en),
    .wr_data_i(data_i),
    .wr_addr_i(wr_ptr_r),
    .rd_addr_i(rd_ptr_r),
    .rd_data_o(rd_data_w)
  );

  assign data_o = rd_data_w;

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      wr_ptr_r <= '0;
      rd_ptr_r <= '0;
      count_r  <= '0;
    end
    else begin
      if (wr_en)
        wr_ptr_r <= wr_ptr_r + 1;
      if (rd_en)
        rd_ptr_r <= rd_ptr_r + 1;
      case ({wr_en, rd_en})
        2'b10: count_r <= count_r + 1;
        2'b01: count_r <= count_r - 1;
        default: count_r <= count_r;
      endcase
    end
  end

endmodule
