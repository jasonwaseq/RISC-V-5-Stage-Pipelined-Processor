module radix_sort_10b
  #(parameter WIDTH_P = 16,
    parameter RADIX_BITS_P = 10,
    parameter N_MAX_P = 256,
    parameter ADDR_W_P = $clog2(N_MAX_P)
    )
  (
    input  logic        clk_i,
    input  logic        reset_i,

    // Input stream (collect array)
    input  logic [WIDTH_P-1:0] s_tdata,
    input  logic               s_tvalid,
    output logic               s_tready,
    input  logic               s_tlast,

    // Output stream (sorted array)
    output logic [WIDTH_P-1:0] m_tdata,
    output logic               m_tvalid,
    input  logic               m_tready,
    output logic               m_tlast,

    // Status
    output logic               busy_o
  );

  typedef enum logic [2:0] {
    ST_IDLE_COLLECT,
    ST_SORT_COUNT,
    ST_SORT_WRITE,
    ST_OUTPUT
  } state_e;

  state_e state_r, state_n;

  // Array length
  logic [ADDR_W_P:0] n_r;  // can hold N_MAX

  // Double buffers
  logic [ADDR_W_P-1:0] wr_addr_a_w, wr_addr_b_w;
  logic [ADDR_W_P-1:0] rd_addr_a_w, rd_addr_b_w;
  logic                wr_valid_a_w, wr_valid_b_w;
  logic [WIDTH_P-1:0]  wr_data_a_w, wr_data_b_w;
  logic [WIDTH_P-1:0]  rd_data_a_w, rd_data_b_w;

  // Select current src/dst
  logic src_sel_r; // 0->A src, 1->B src

  // Iteration indices
  logic [RADIX_BITS_P-1:0] bit_idx_r;
  logic [ADDR_W_P:0]       idx_r;       // 0..N
  logic [ADDR_W_P:0]       zeros_r;     // count of zeros in current bit
  logic [ADDR_W_P:0]       pos_zero_r;
  logic [ADDR_W_P:0]       pos_one_r;

  // Input collection
  assign s_tready = (state_r == ST_IDLE_COLLECT);

  // Busy
  assign busy_o = (state_r != ST_IDLE_COLLECT) && (state_r != ST_OUTPUT || m_tvalid);

  // RAM A
  ram_1r1w_async #(
    .width_p(WIDTH_P),
    .depth_p(N_MAX_P),
    .init_p(0),
    .filename_p("")
  ) ram_a (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .wr_valid_i(wr_valid_a_w),
    .wr_data_i(wr_data_a_w),
    .wr_addr_i(wr_addr_a_w),
    .rd_addr_i(rd_addr_a_w),
    .rd_data_o(rd_data_a_w)
  );

  // RAM B
  ram_1r1w_async #(
    .width_p(WIDTH_P),
    .depth_p(N_MAX_P),
    .init_p(0),
    .filename_p("")
  ) ram_b (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .wr_valid_i(wr_valid_b_w),
    .wr_data_i(wr_data_b_w),
    .wr_addr_i(wr_addr_b_w),
    .rd_addr_i(rd_addr_b_w),
    .rd_data_o(rd_data_b_w)
  );

  // Default RAM control
  always_comb begin
    // Defaults
    wr_valid_a_w = 1'b0;
    wr_valid_b_w = 1'b0;
    wr_addr_a_w  = '0;
    wr_addr_b_w  = '0;
    wr_data_a_w  = '0;
    wr_data_b_w  = '0;
    rd_addr_a_w  = '0;
    rd_addr_b_w  = '0;

    // Input collect writes go to source RAM selection for first load (we define A as initial source)
    if (state_r == ST_IDLE_COLLECT && s_tvalid && s_tready) begin
      wr_valid_a_w = 1'b1;
      wr_addr_a_w  = n_r[ADDR_W_P-1:0];
      wr_data_a_w  = s_tdata;
    end

    // Sorting phases: set read address to idx_r of src, write address to positions in dst
    if (state_r == ST_SORT_COUNT) begin
      // Read src at idx
      if (!src_sel_r) begin
        rd_addr_a_w = idx_r[ADDR_W_P-1:0];
      end else begin
        rd_addr_b_w = idx_r[ADDR_W_P-1:0];
      end
    end

    if (state_r == ST_SORT_WRITE) begin
      // Read src at idx
      if (!src_sel_r) begin
        rd_addr_a_w = idx_r[ADDR_W_P-1:0];
      end else begin
        rd_addr_b_w = idx_r[ADDR_W_P-1:0];
      end
      // Write dst at pos_zero/pos_one
      if (!src_sel_r) begin
        // src A -> dst B
        wr_valid_b_w = 1'b1;
        wr_addr_b_w  = ( (rd_data_a_w[bit_idx_r]) ? pos_one_r[ADDR_W_P-1:0] : pos_zero_r[ADDR_W_P-1:0] );
        wr_data_b_w  = rd_data_a_w;
      end else begin
        // src B -> dst A
        wr_valid_a_w = 1'b1;
        wr_addr_a_w  = ( (rd_data_b_w[bit_idx_r]) ? pos_one_r[ADDR_W_P-1:0] : pos_zero_r[ADDR_W_P-1:0] );
        wr_data_a_w  = rd_data_b_w;
      end
    end

    if (state_r == ST_OUTPUT) begin
      // Read from current src to stream out
      if (!src_sel_r) begin
        rd_addr_a_w = idx_r[ADDR_W_P-1:0];
      end else begin
        rd_addr_b_w = idx_r[ADDR_W_P-1:0];
      end
    end
  end

  // Output stream
  logic [WIDTH_P-1:0] cur_src_data;
  assign cur_src_data = (!src_sel_r) ? rd_data_a_w : rd_data_b_w;

  assign m_tdata  = cur_src_data;
  assign m_tvalid = (state_r == ST_OUTPUT);
  assign m_tlast  = (state_r == ST_OUTPUT) && (idx_r == n_r - 1);

  // Sequential control
  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      state_r     <= ST_IDLE_COLLECT;
      n_r         <= '0;
      src_sel_r   <= 1'b0; // A is initial source
      bit_idx_r   <= '0;
      idx_r       <= '0;
      zeros_r     <= '0;
      pos_zero_r  <= '0;
      pos_one_r   <= '0;
    end else begin
      state_r <= state_n;

      case (state_r)
        ST_IDLE_COLLECT: begin
          if (s_tvalid && s_tready) begin
            // write happens in comb block; increment count
            n_r <= n_r + 1;
            if (s_tlast) begin
              // done collecting
              bit_idx_r  <= '0;
              idx_r      <= '0;
              zeros_r    <= '0;
            end
          end
        end

        ST_SORT_COUNT: begin
          // count zeros on current bit
          if (idx_r < n_r) begin
            // read address set; rd_data available combinationally
            if (!src_sel_r) begin
              zeros_r <= zeros_r + (rd_data_a_w[bit_idx_r] ? 0 : 1);
            end else begin
              zeros_r <= zeros_r + (rd_data_b_w[bit_idx_r] ? 0 : 1);
            end
            idx_r   <= idx_r + 1;
          end else begin
            // done counting; setup write phase positions
            pos_zero_r <= '0;
            pos_one_r  <= zeros_r;
            idx_r      <= '0;
          end
        end

        ST_SORT_WRITE: begin
          if (idx_r < n_r) begin
            // write happens in comb based on current read data
            // advance positions
            if (!src_sel_r) begin
              if (rd_data_a_w[bit_idx_r]) begin
                pos_one_r <= pos_one_r + 1;
              end else begin
                pos_zero_r <= pos_zero_r + 1;
              end
            end else begin
              if (rd_data_b_w[bit_idx_r]) begin
                pos_one_r <= pos_one_r + 1;
              end else begin
                pos_zero_r <= pos_zero_r + 1;
              end
            end
            idx_r <= idx_r + 1;
          end else begin
            // finish this bit
            idx_r     <= '0;
            zeros_r   <= '0;
            bit_idx_r <= bit_idx_r + 1;
            src_sel_r <= ~src_sel_r; // swap src/dst
          end
        end

        ST_OUTPUT: begin
          if (m_tvalid && m_tready) begin
            if (idx_r < n_r) begin
              idx_r <= idx_r + 1;
            end
          end
        end
      endcase
    end
  end

  // Next-state logic
  always_comb begin
    state_n = state_r;
    unique case (state_r)
      ST_IDLE_COLLECT: begin
        if (s_tvalid && s_tready && s_tlast) begin
          state_n = (n_r == 0) ? ST_OUTPUT : ST_SORT_COUNT;
        end
      end
      ST_SORT_COUNT: begin
        if (idx_r == n_r) begin
          state_n = ST_SORT_WRITE;
        end
      end
      ST_SORT_WRITE: begin
        if (idx_r == n_r) begin
          if (bit_idx_r == RADIX_BITS_P-1) begin
            // last bit just completed, prepare to output
            state_n = ST_OUTPUT;
            // After last bit, src_sel_r already swapped; it now points to final sorted buffer
          end else begin
            state_n = ST_SORT_COUNT;
          end
        end
      end
      ST_OUTPUT: begin
        if (m_tvalid && m_tready && (idx_r == n_r-1)) begin
          // streamed all values, go idle for next batch
          state_n = ST_IDLE_COLLECT;
          // reset n in sequential
        end
      end
      default: state_n = ST_IDLE_COLLECT;
    endcase
  end

  // Reset N when finishing output
  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      // already handled
    end else if (state_r == ST_OUTPUT && m_tvalid && m_tready && (idx_r == n_r-1)) begin
      n_r <= '0;
      bit_idx_r <= '0;
      idx_r <= '0;
      zeros_r <= '0;
      pos_zero_r <= '0;
      pos_one_r <= '0;
      // Leave src_sel_r as is (points to final buffer), next load writes into A again
      src_sel_r <= 1'b0;
    end
  end

endmodule
