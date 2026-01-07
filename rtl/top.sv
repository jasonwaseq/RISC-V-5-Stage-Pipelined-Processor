// Top-level module: UART + Radix Sorter for 10-bit values
module top (
    input  logic       clk,
    input  logic       rst,
    // UART pins
    input  logic       rxd,
    output logic       txd,
    // Config
    input  logic [15:0] prescale
);

    // Integrate uart_axis which handles protocol + sorter
    logic [5:1] leds;
    uart_axis #(
        .N_MAX_P(256)
    ) uart_sort (
        .clk_i(clk),
        .reset_i(rst),
        .rx_serial_i(rxd),
        .tx_serial_o(txd),
        .prescale_i(prescale),
        .led_o(leds)
    );

endmodule