module uart_controller_tb;
    logic clk, rst;

    // DUT signals
    logic rxd;
    logic txd;

    // Wires between RX and TX
    logic [7:0] data_rx;
    logic       valid_rx;
    logic       ready_rx;

    // Clock gen
    initial clk = 0;
    always #5 clk = ~clk; // 100 MHz

    // Reset
    initial begin
        rst = 1;
        #50 rst = 0;
    end

    // Instantiate RX
    uart_rx #(.DATA_WIDTH(8)) u_rx (
        .clk(clk),
        .rst(rst),
        .rxd(rxd),
        .m_axis_tdata(data_rx),
        .m_axis_tvalid(valid_rx),
        .m_axis_tready(ready_rx),
        .busy(),
        .overrun_error(),
        .frame_error(),
        .prescale(16'd87) // adjust for baud rate
    );

    // Instantiate TX (driven by RX outputs)
    uart_tx #(.DATA_WIDTH(8)) u_tx (
        .clk(clk),
        .rst(rst),
        .s_axis_tdata(data_rx),
        .s_axis_tvalid(valid_rx),
        .s_axis_tready(ready_rx),
        .txd(txd),
        .busy(),
        .prescale(16'd87)
    );

    // Testbench driver: send bytes into RX
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, uart_controller_tb);

        wait(!rst);

        send_byte(8'h55);
        send_byte(8'hA3);
        send_byte(8'hFF);

        // Try sending too fast (backpressure test)
        send_byte(8'h11);
        send_byte(8'h22);
        send_byte(8'h33);

        #5000 $finish;
    end

    // Task to bit-bang serial data into RX
    task send_byte(input [7:0] b);
        integer i;
        begin
            // Start bit
            rxd = 0;
            #(87*16); // prescale ticks

            // Data bits (LSB first)
            for (i = 0; i < 8; i++) begin
                rxd = b[i];
                #(87*16);
            end

            // Stop bit
            rxd = 1;
            #(87*16);
        end
    endtask

    // Monitor what comes back from TX
    initial begin
        forever begin
            @(negedge txd); // detect start bit
            # (87*8); // wait ~half a byte
            $display("[%0t] TX line active", $time);
        end
    end
endmodule
