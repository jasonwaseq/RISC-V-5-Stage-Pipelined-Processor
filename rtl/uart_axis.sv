
module uart_axis
   #(parameter N_MAX_P = 256)
   (
      input  logic clk_i,
      input  logic reset_i,
      input  logic rx_serial_i,
      output logic tx_serial_o,
      input  logic [15:0] prescale_i,
      output logic [5:1] led_o
   );

   // UART byte stream
   localparam int DATA_W = 8;
   wire [DATA_W-1:0] m_axis_uart_tdata; // RX bytes from host
   wire              m_axis_uart_tvalid;
   wire              m_axis_uart_tready;

   wire [DATA_W-1:0] s_axis_uart_tdata; // TX bytes to host
   wire              s_axis_uart_tvalid;
   wire              s_axis_uart_tready;

   // Instantiate UART
   uart #(
      .DATA_WIDTH(DATA_W)
   ) uart_inst (
      .clk(clk_i),
      .rst(reset_i),
      .s_axis_tready(s_axis_uart_tready),
      .s_axis_tvalid(s_axis_uart_tvalid),
      .s_axis_tdata(s_axis_uart_tdata),
      .m_axis_tready(m_axis_uart_tready),
      .m_axis_tvalid(m_axis_uart_tvalid),
      .m_axis_tdata(m_axis_uart_tdata),
      .rxd(rx_serial_i),
      .txd(tx_serial_o),
      .tx_busy(),
      .rx_busy(),
      .rx_overrun_error(),
      .rx_frame_error(),
      .prescale(prescale_i)
   );

   // Sorter (16-bit stream, lower 10 bits used)
   logic [15:0] sort_s_tdata;
   logic        sort_s_tvalid, sort_s_tready, sort_s_tlast;
   logic [15:0] sort_m_tdata;
   logic        sort_m_tvalid, sort_m_tready, sort_m_tlast;
   logic        sort_busy;

   radix_sort_10b #(
      .WIDTH_P(16),
      .RADIX_BITS_P(10),
      .N_MAX_P(N_MAX_P)
   ) sorter (
      .clk_i(clk_i),
      .reset_i(reset_i),
      .s_tdata(sort_s_tdata),
      .s_tvalid(sort_s_tvalid),
      .s_tready(sort_s_tready),
      .s_tlast(sort_s_tlast),
      .m_tdata(sort_m_tdata),
      .m_tvalid(sort_m_tvalid),
      .m_tready(sort_m_tready),
      .m_tlast(sort_m_tlast),
      .busy_o(sort_busy)
   );

   // Protocol state machine
   typedef enum logic [2:0] {
      RX_COUNT,
      RX_DATA,
      WAIT_SORT,
      TX_COUNT,
      TX_DATA
   } proto_e;

   proto_e proto_r, proto_n;

   // Counters and buffers
   logic [15:0] count_r;
   logic [15:0] count_sent_r;
   logic [15:0] words_received_r;
   logic [0:0]  count_byte_phase_r; // 0 -> LSB, 1 -> MSB
   logic [0:0]  data_byte_phase_r;  // 0 -> LSB, 1 -> MSB
   logic [15:0] in_word_r;
   logic [15:0] out_word_r;
   logic [0:0]  out_byte_phase_r;   // 0 -> LSB, 1 -> MSB

   // LED status: simple heartbeat on activity
   assign led_o[1] = (proto_r != RX_COUNT);
   assign led_o[5:2] = 4'b0000;

   // UART RX ready when consuming header/data
   assign m_axis_uart_tready = (proto_r == RX_COUNT) || (proto_r == RX_DATA);

   // Drive sorter input stream
   assign sort_s_tvalid = (proto_r == RX_DATA) && (data_byte_phase_r == 1'b1) && m_axis_uart_tvalid && m_axis_uart_tready;
   assign sort_s_tdata  = in_word_r;
   assign sort_s_tlast  = sort_s_tvalid && (words_received_r == count_r);

   // Drive sorter output ready when we have consumed current out_word_r
   assign sort_m_tready = (proto_r == TX_DATA) && (out_byte_phase_r == 1'b1) && s_axis_uart_tready && s_axis_uart_tvalid;

   // UART TX interface
   assign s_axis_uart_tvalid = (proto_r == TX_COUNT) || (proto_r == TX_DATA);
   assign s_axis_uart_tdata  = (proto_r == TX_COUNT) ? (count_byte_phase_r ? count_r[15:8] : count_r[7:0])
                                                                            : (out_byte_phase_r   ? out_word_r[15:8]  : out_word_r[7:0]);

   // Sequential control
   always_ff @(posedge clk_i) begin
      if (reset_i) begin
         proto_r            <= RX_COUNT;
         count_r            <= 16'd0;
         count_sent_r       <= 16'd0;
         words_received_r   <= 16'd0;
         count_byte_phase_r <= 1'b0;
         data_byte_phase_r  <= 1'b0;
         in_word_r          <= 16'd0;
         out_word_r         <= 16'd0;
         out_byte_phase_r   <= 1'b0;
      end else begin
         proto_r <= proto_n;

         case (proto_r)
            RX_COUNT: begin
               if (m_axis_uart_tvalid && m_axis_uart_tready) begin
                  if (!count_byte_phase_r) begin
                     count_r[7:0]      <= m_axis_uart_tdata;
                     count_byte_phase_r <= 1'b1;
                  end else begin
                     count_r[15:8]     <= m_axis_uart_tdata;
                     count_byte_phase_r <= 1'b0;
                     words_received_r   <= 16'd0;
                  end
               end
            end

            RX_DATA: begin
               if (m_axis_uart_tvalid && m_axis_uart_tready) begin
                  if (!data_byte_phase_r) begin
                     in_word_r[7:0]     <= m_axis_uart_tdata;
                     data_byte_phase_r   <= 1'b1;
                  end else begin
                     in_word_r[15:8]    <= m_axis_uart_tdata;
                     data_byte_phase_r   <= 1'b0;
                     // issue to sorter on this cycle via sort_s_tvalid
                     words_received_r    <= words_received_r + 1;
                  end
               end
            end

            WAIT_SORT: begin
               // Wait for first sorted word to become valid
               if (sort_m_tvalid) begin
                  out_word_r       <= sort_m_tdata;
                  out_byte_phase_r <= 1'b0;
               end
            end

            TX_COUNT: begin
               if (s_axis_uart_tready) begin
                  if (!count_byte_phase_r) begin
                     count_byte_phase_r <= 1'b1;
                  end else begin
                     count_byte_phase_r <= 1'b0;
                     count_sent_r       <= count_r;
                  end
               end
            end

            TX_DATA: begin
               if (s_axis_uart_tready) begin
                  if (!out_byte_phase_r) begin
                     // sent LSB, next MSB
                     out_byte_phase_r <= 1'b1;
                  end else begin
                     // sent MSB, request next word when available
                     out_byte_phase_r <= 1'b0;
                     if (sort_m_tvalid) begin
                        out_word_r <= sort_m_tdata;
                     end
                     if (count_sent_r != 16'd0) begin
                        count_sent_r <= count_sent_r - 1;
                     end
                  end
               end
            end
         endcase
      end
   end

   // Next-state control
   always_comb begin
      proto_n = proto_r;
      unique case (proto_r)
         RX_COUNT: begin
            if (!count_byte_phase_r && (m_axis_uart_tvalid && m_axis_uart_tready)) begin
               // got LSB
               proto_n = RX_COUNT;
            end
            if (count_byte_phase_r && (m_axis_uart_tvalid && m_axis_uart_tready)) begin
               // got MSB, move to data
               proto_n = (count_r == 16'd0) ? RX_COUNT : RX_DATA;
            end
         end
         RX_DATA: begin
            if ((m_axis_uart_tvalid && m_axis_uart_tready) && data_byte_phase_r && (words_received_r + 1 == count_r)) begin
               // final word assembled and pushed into sorter
               proto_n = WAIT_SORT;
            end
         end
         WAIT_SORT: begin
            if (sort_m_tvalid) begin
               proto_n = TX_COUNT;
            end
         end
         TX_COUNT: begin
            if (s_axis_uart_tready && count_byte_phase_r) begin
               // after sending MSB of count
               proto_n = TX_DATA;
            end
         end
         TX_DATA: begin
            if ((count_sent_r == 16'd1) && s_axis_uart_tready && out_byte_phase_r) begin
               // last MSB sent
               proto_n = RX_COUNT;
            end
         end
         default: proto_n = RX_COUNT;
      endcase
   end

endmodule
