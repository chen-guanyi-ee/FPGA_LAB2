`timescale 1ns/1ps

module tb;
    localparam time CLK_PERIOD = 10ns;
    localparam time UART_BIT   = 8680ns;

    localparam logic [255:0] N = 256'hCA3586E7EA485F3B0A222A4C79F7DD12E85388ECCDEE4035940D774C029CF831;
    localparam logic [255:0] D = 256'hB6ACE0B14720169839B15FD13326CF1A1829BEAFC37BB937BEC8802FBCF46BD9;
    localparam logic [255:0] Y = 256'hC6B662ECB173C53CC7BB4212057F9C0BA283E000B98C9DCF5FEAEE7D6C933DFB;
    localparam logic [255:0] M = 256'h005468652076616C7565206F662050492069733A0A332E313431353932363533;

    logic clk, rst, rx, tx;
    logic [767:0] received_ndy;

    Rsa256Wrapper dut (
        .i_clk(clk), .i_rst(rst), .rx(rx), .tx(tx)
    );

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // The integrated core reuses/rotates ndy_r immediately after RX.
    always @(posedge dut.finish_r)
        received_ndy = dut.ndy_r;

    task automatic uart_send_byte(input logic [7:0] data);
        rx = 1'b0;
        #(UART_BIT);
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) begin
            rx = data[bit_idx];
            #(UART_BIT);
        end
        rx = 1'b1;
        #(UART_BIT);
    endtask

    task automatic uart_send_256(input logic [255:0] data);
        for (int byte_idx = 31; byte_idx >= 0; byte_idx--)
            uart_send_byte(data[byte_idx*8 +: 8]);
    endtask

    task automatic uart_receive_byte(output logic [7:0] data);
        @(negedge tx);
        #(UART_BIT + UART_BIT/2);
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) begin
            data[bit_idx] = tx;
            #(UART_BIT);
        end
        if (tx !== 1'b1)
            $fatal(1, "UART stop-bit error at time %0t", $time);
    endtask

    initial begin : test_sequence
        logic [7:0] received;
        logic [7:0] expected;

        rx = 1'b1;
        rst = 1'b1;
        #(10*CLK_PERIOD);
        @(negedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // Lecture flow: N (32 B), d (32 B), then y (32 B), MSB byte first.
        uart_send_256(N);
        uart_send_256(D);
        uart_send_256(Y);

        fork
            begin
                wait (dut.finish_r === 1'b1);
            end
            begin
                #(2_000_000);
                $fatal(1, "finish_r was not asserted after 96 received bytes");
            end
        join_any
        disable fork;

        #1ns;
        if (received_ndy !== {N, D, Y})
            $fatal(1, "RX data/order mismatch\nexpected=%h\nactual  =%h", {N,D,Y}, received_ndy);
        if (dut.uart.rx_read_count != 96)
            $fatal(1, "Expected 96 RX FIFO reads, got %0d", dut.uart.rx_read_count);


        // Lab behavior: skip the leading 00 and send m[247:0] (31 bytes).
        for (int byte_idx = 30; byte_idx >= 0; byte_idx--) begin
            uart_receive_byte(received);
            expected = M[byte_idx*8 +: 8];
            if (received !== expected)
                $fatal(1, "TX byte %0d mismatch: expected %02h, got %02h",
                       30-byte_idx, expected, received);
        end

        wait (dut.state_r == 4'd9);
        if (dut.uart.tx_write_count != 31)
            $fatal(1, "Expected 31 TX FIFO writes, got %0d", dut.uart.tx_write_count);
        $display("PASS: 96-byte RX, N/d/y order, status checks, and 31-byte TX verified.");
        $finish;
    end

    initial begin
        #(200_000_000);
        $fatal(1, "Simulation timeout");
    end
endmodule


// Behavioral AXI UART Lite used only by this standalone wrapper test.
// Define USE_XILINX_UARTLITE when compiling the generated Vivado IP model.
`ifndef USE_XILINX_UARTLITE
module axi_uartlite_0 (
    input  logic s_axi_aclk, s_axi_aresetn,
    input  logic [3:0] s_axi_awaddr,
    input  logic s_axi_awvalid,
    output logic s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0] s_axi_wstrb,
    input  logic s_axi_wvalid,
    output logic s_axi_wready,
    output logic [1:0] s_axi_bresp,
    output logic s_axi_bvalid,
    input  logic s_axi_bready,
    input  logic [3:0] s_axi_araddr,
    input  logic s_axi_arvalid,
    output logic s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0] s_axi_rresp,
    output logic s_axi_rvalid,
    input  logic s_axi_rready,
    input  logic rx,
    output logic tx
);
    localparam time UART_BIT = 8680ns;

    logic [7:0] rx_mem [0:255];
    logic [7:0] tx_mem [0:255];
    integer rx_wr_ptr, rx_rd_ptr, tx_wr_ptr, tx_rd_ptr;
    integer rx_read_count, tx_write_count;
    logic [31:0] cycle_r;
    logic [3:0] awaddr_r, last_araddr_r;
    logic [31:0] wdata_r, last_status_r;
    logic aw_captured_r, w_captured_r;

    wire aw_fire = s_axi_awvalid && s_axi_awready;
    wire w_fire  = s_axi_wvalid && s_axi_wready;
    wire [3:0] write_addr = aw_fire ? s_axi_awaddr : awaddr_r;
    wire [31:0] write_data = w_fire ? s_axi_wdata : wdata_r;
    wire rx_empty = (rx_wr_ptr == rx_rd_ptr);
    wire rx_full  = ((rx_wr_ptr - rx_rd_ptr) >= 16);
    wire tx_empty = (tx_wr_ptr == tx_rd_ptr);
    wire tx_full  = ((tx_wr_ptr - tx_rd_ptr) >= 16);
    wire [31:0] status_word = {28'b0, tx_full, tx_empty, rx_full, !rx_empty};

    // Stagger address/data READY to exercise independent AXI handshakes.
    always_comb begin
        s_axi_arready = s_axi_aresetn && !s_axi_rvalid && cycle_r[1];
        s_axi_awready = s_axi_aresetn && !s_axi_bvalid &&
                        !aw_captured_r && cycle_r[0];
        s_axi_wready  = s_axi_aresetn && !s_axi_bvalid &&
                        !w_captured_r && !cycle_r[0];
    end

    initial begin
        rx_wr_ptr = 0;
        tx_rd_ptr = 0;
        tx = 1'b1;
    end

    // UART 115200, 8 data bits, no parity, one stop bit.
    initial begin : uart_rx_model
        logic [7:0] received;
        forever begin
            @(negedge rx);
            if (s_axi_aresetn) begin
                #(UART_BIT/2);
                if (rx === 1'b0) begin
                    for (int bit_idx = 0; bit_idx < 8; bit_idx++) begin
                        #(UART_BIT);
                        received[bit_idx] = rx;
                    end
                    #(UART_BIT);
                    if (rx !== 1'b1)
                        $fatal(1, "UART model received an invalid stop bit");
                    if ((rx_wr_ptr - rx_rd_ptr) >= 16)
                        $fatal(1, "UART RX FIFO overflow");
                    rx_mem[rx_wr_ptr & 8'hff] = received;
                    rx_wr_ptr = rx_wr_ptr + 1;
                end
            end
        end
    end

    initial begin : uart_tx_model
        logic [7:0] sending;
        forever begin
            wait (s_axi_aresetn && (tx_rd_ptr < tx_wr_ptr));
            sending = tx_mem[tx_rd_ptr & 8'hff];
            tx_rd_ptr = tx_rd_ptr + 1;
            tx = 1'b0;
            #(UART_BIT);
            for (int bit_idx = 0; bit_idx < 8; bit_idx++) begin
                tx = sending[bit_idx];
                #(UART_BIT);
            end
            tx = 1'b1;
            #(UART_BIT);
        end
    end

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            cycle_r <= 0;
            rx_rd_ptr <= 0;
            tx_wr_ptr <= 0;
            rx_read_count <= 0;
            tx_write_count <= 0;
            s_axi_rdata <= 0;
            s_axi_rresp <= 0;
            s_axi_rvalid <= 0;
            s_axi_bresp <= 0;
            s_axi_bvalid <= 0;
            awaddr_r <= 0;
            wdata_r <= 0;
            aw_captured_r <= 0;
            w_captured_r <= 0;
            last_araddr_r <= 4'hf;
            last_status_r <= 0;
        end else begin
            cycle_r <= cycle_r + 1'b1;

            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 1'b0;

            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp <= 2'b00;
                case (s_axi_araddr)
                    4'h8: begin
                        s_axi_rdata <= status_word;
                        last_status_r <= status_word;
                    end
                    4'h0: begin
                        if ((last_araddr_r != 4'h8) || !last_status_r[0])
                            $fatal(1, "RX FIFO read without RX-valid status");
                        if (rx_empty)
                            $fatal(1, "Read attempted on empty RX FIFO");
                        s_axi_rdata <= {24'b0, rx_mem[rx_rd_ptr & 8'hff]};
                        rx_rd_ptr <= rx_rd_ptr + 1;
                        rx_read_count <= rx_read_count + 1;
                    end
                    default: begin
                        s_axi_rdata <= 0;
                        s_axi_rresp <= 2'b10;
                    end
                endcase
                last_araddr_r <= s_axi_araddr;
            end

            if (aw_fire) begin
                awaddr_r <= s_axi_awaddr;
                aw_captured_r <= 1'b1;
            end
            if (w_fire) begin
                wdata_r <= s_axi_wdata;
                w_captured_r <= 1'b1;
            end

            if (!s_axi_bvalid && (aw_captured_r || aw_fire) &&
                (w_captured_r || w_fire)) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp <= 2'b00;
                if (write_addr != 4'h4) begin
                    s_axi_bresp <= 2'b10;
                end else if ((last_araddr_r != 4'h8) || last_status_r[3]) begin
                    $fatal(1, "TX FIFO write without TX-not-full status");
                end else if (tx_full) begin
                    $fatal(1, "Write attempted on full TX FIFO");
                end else begin
                    tx_mem[tx_wr_ptr & 8'hff] <= write_data[7:0];
                    tx_wr_ptr <= tx_wr_ptr + 1;
                    tx_write_count <= tx_write_count + 1;
                end
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
                aw_captured_r <= 1'b0;
                w_captured_r <= 1'b0;
            end
        end
    end
endmodule
`endif
