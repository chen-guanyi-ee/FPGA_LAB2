`timescale 1ns/1ps
/********************************************************
    NEXYS_A7 top  (RSA256 engine + optional VGA data-flow view)

    ENABLE_VGA = 1 : 3D data-flow display on the VGA port,
                     SW0 / SW1 slow the core down so you can watch it.
    ENABLE_VGA = 0 : identical to the plain RSA build (for the slice
                     ranking); VGA pins are just driven low.

    Compare with your template's NEXYS_A7.sv: if it has extra
    logic (reset sync, LEDs ...), keep that and only add the
    VGA / SW parts below.
**********************************************************/
module NEXYS_A7 #(
    parameter bit ENABLE_VGA = 1'b1
)(
    input  logic       CLK100MHZ,
    input  logic       BTNC,
    input  logic       UART_TXD_IN,
    output logic       UART_RXD_OUT,
    input  logic [1:0] SW,
    output logic [3:0] VGA_R,
    output logic [3:0] VGA_G,
    output logic [3:0] VGA_B,
    output logic       VGA_HS,
    output logic       VGA_VS
);

logic [2:0] dbg_state;
logic [6:0] dbg_byte_count;
logic [7:0] dbg_mul_bit, dbg_round_bit;
logic       dbg_mul_by_y, dbg_rx_busy, dbg_rx_valid, dbg_tx_busy, dbg_tx_start;
logic       dbg_n_we, dbg_d_we, dbg_y_we;
logic [1:0] slow;

Rsa256Wrapper rsa_wrapper (
    .i_clk            (CLK100MHZ),
    .i_rst            (BTNC),
    .rx               (UART_TXD_IN),
    .tx               (UART_RXD_OUT),
    .i_slow           (slow),
    .o_dbg_state      (dbg_state),
    .o_dbg_byte_count (dbg_byte_count),
    .o_dbg_mul_bit    (dbg_mul_bit),
    .o_dbg_round_bit  (dbg_round_bit),
    .o_dbg_mul_by_y   (dbg_mul_by_y),
    .o_dbg_rx_busy    (dbg_rx_busy),
    .o_dbg_rx_valid   (dbg_rx_valid),
    .o_dbg_tx_busy    (dbg_tx_busy),
    .o_dbg_tx_start   (dbg_tx_start),
    .o_dbg_n_we       (dbg_n_we),
    .o_dbg_d_we       (dbg_d_we),
    .o_dbg_y_we       (dbg_y_we)
);

generate
if (ENABLE_VGA) begin : g_vga
    // slide switches are asynchronous: two-flop synchroniser
    (* ASYNC_REG = "TRUE" *) logic [1:0] sw_meta_r, sw_sync_r;
    always_ff @(posedge CLK100MHZ) begin
        sw_meta_r <= SW;
        sw_sync_r <= sw_meta_r;
    end
    assign slow = sw_sync_r;

    VgaFlowViz viz (
        .i_clk        (CLK100MHZ),
        .i_rst        (BTNC),
        .i_state      (dbg_state),
        .i_byte_count (dbg_byte_count),
        .i_mul_bit    (dbg_mul_bit),
        .i_round_bit  (dbg_round_bit),
        .i_mul_by_y   (dbg_mul_by_y),
        .i_rx_busy    (dbg_rx_busy),
        .i_rx_valid   (dbg_rx_valid),
        .i_tx_busy    (dbg_tx_busy),
        .i_tx_start   (dbg_tx_start),
        .i_n_we       (dbg_n_we),
        .i_d_we       (dbg_d_we),
        .i_y_we       (dbg_y_we),
        .o_vga_r      (VGA_R),
        .o_vga_g      (VGA_G),
        .o_vga_b      (VGA_B),
        .o_vga_hs     (VGA_HS),
        .o_vga_vs     (VGA_VS)
    );
end else begin : g_no_vga
    assign slow   = 2'b00;          // core always full speed
    assign VGA_R  = 4'h0;
    assign VGA_G  = 4'h0;
    assign VGA_B  = 4'h0;
    assign VGA_HS = 1'b1;
    assign VGA_VS = 1'b1;
end
endgenerate

endmodule
