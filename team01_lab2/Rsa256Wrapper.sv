`timescale 1ns/1ps

module Rsa256Wrapper (
    input           i_clk,
    input           i_rst,
    input           rx,
	input           finish_calc,
	output          tx,
    output logic    finish_r
);

//AXI command
localparam	AXI_IDLE 	        = 2'b00;
localparam	AXI_R		        = 2'b10;
localparam	AXI_W		        = 2'b01;

localparam   RX_BASE             = 0*4;
localparam   TX_BASE             = 1*4;
localparam   STATUS_BASE         = 2*4;
localparam   CTRL_BASE           = 3*4;

localparam   RX_OK_BIT           = 0;
localparam   TX_FULL_BIT         = 3;

localparam   S_STATUS_ADDR       = 4'd0;
localparam   S_STATUS_DATA       = 4'd1;
localparam   S_RX_ADDR           = 4'd2;
localparam   S_RX_DATA           = 4'd3;
localparam   S_WAIT_CALC         = 4'd4;
localparam   S_TX_STATUS_ADDR    = 4'd5;
localparam   S_TX_STATUS_DATA    = 4'd6;
localparam   S_TX_DATA           = 4'd7;
localparam   S_TX_RESP           = 4'd8;
localparam   S_FINISH            = 4'd9;

/********************************************************
    Input buffer: N || d || y
**********************************************************/
(* keep = "yes" *) logic [767:0] ndy_r;
logic [6:0] byte_seq_r;
logic [255:0] m;

wire [255:0] n = ndy_r[767:512];
wire [255:0] d = ndy_r[511:256];
wire [255:0] y = ndy_r[255:0];

/********************************************************
    UART
**********************************************************/
//write address
logic[3:0]      s_axi_awaddr;
logic           s_axi_awvalid;
logic           s_axi_awready;
//write data
logic[31:0]     s_axi_wdata;
logic           s_axi_wvalid;
logic           s_axi_wready;
//write response
logic[1:0]      s_axi_bresp;
logic           s_axi_bvalid;
logic           s_axi_bready;
//read address
logic[3:0]     	s_axi_araddr  ;  //input 	master read address
logic         	s_axi_arvalid ;  //input    master read address valid
logic          	s_axi_arready ;  //output	slave ready to receive read address
//read data
logic[31:0]    	s_axi_rdata   ;  //output	slave read data
logic[1:0]    	s_axi_rresp   ;  //output	slave read response
logic          	s_axi_rvalid  ;  //output	slave read data valid
logic         	s_axi_rready  ;  //input 	master ready to receive read data

axi_uartlite_0 uart 
(
  .s_axi_aclk(i_clk),             // input logic s_axi_aclk
  .s_axi_aresetn(!i_rst),         // input logic s_axi_aresetn
  .s_axi_awaddr(s_axi_awaddr),
  .s_axi_awvalid(s_axi_awvalid),
  .s_axi_awready(s_axi_awready),
  .s_axi_wdata(s_axi_wdata),
  .s_axi_wstrb(4'b1111),
  .s_axi_wvalid(s_axi_wvalid),
  .s_axi_wready(s_axi_wready),
  .s_axi_bresp(s_axi_bresp),
  .s_axi_bvalid(s_axi_bvalid),
  .s_axi_bready(s_axi_bready),
  .s_axi_araddr(s_axi_araddr),    // input logic [3 : 0] s_axi_araddr
  .s_axi_arvalid(s_axi_arvalid),  // input logic s_axi_arvalid
  .s_axi_arready(s_axi_arready),  // output logic s_axi_arready
  .s_axi_rdata(s_axi_rdata),      // output logic [31 : 0] s_axi_rdata
  .s_axi_rresp(s_axi_rresp),      // output logic [1 : 0] s_axi_rresp
  .s_axi_rvalid(s_axi_rvalid),    // output logic s_axi_rvalid
  .s_axi_rready(s_axi_rready),    // input logic s_axi_rready
  .rx(rx),                        // input logic rx
  .tx(tx)                         // output logic tx
); 


logic [3:0] state_r;
logic aw_done_r, w_done_r;

always_comb begin
    s_axi_awaddr  = TX_BASE;
    s_axi_awvalid = (state_r == S_TX_DATA) && !aw_done_r;
    s_axi_wdata   = {24'b0, ndy_r[247:240]};
    s_axi_wvalid  = (state_r == S_TX_DATA) && !w_done_r;
    s_axi_bready  = (state_r == S_TX_RESP);

    s_axi_araddr  = (state_r == S_RX_ADDR) ? RX_BASE : STATUS_BASE;
    s_axi_arvalid = (state_r == S_STATUS_ADDR) || (state_r == S_RX_ADDR) ||
                    (state_r == S_TX_STATUS_ADDR);
    s_axi_rready  = (state_r == S_STATUS_DATA) || (state_r == S_RX_DATA) ||
                    (state_r == S_TX_STATUS_DATA);
end

always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        ndy_r      <= 768'b0;
        byte_seq_r <= 7'h01;
        finish_r   <= 1'b0;
        state_r    <= S_STATUS_ADDR;
        aw_done_r  <= 1'b0;
        w_done_r   <= 1'b0;
    end else begin
        case (state_r)
            S_STATUS_ADDR: begin
                if (s_axi_arready)
                    state_r <= S_STATUS_DATA;
            end

            S_STATUS_DATA: begin
                if (s_axi_rvalid)
                    state_r <= s_axi_rdata[RX_OK_BIT] ? S_RX_ADDR : S_STATUS_ADDR;
            end

            S_RX_ADDR: begin
                if (s_axi_arready)
                    state_r <= S_RX_DATA;
            end

            S_RX_DATA: begin
                if (s_axi_rvalid) begin
                    ndy_r <= {ndy_r[759:0], s_axi_rdata[7:0]};

                    // 7-bit LFSR sequence, value 7'h25 occurs before byte 96.
                    if (byte_seq_r == 7'h25) begin
                        finish_r <= 1'b1;
                        state_r  <= S_WAIT_CALC;
                    end else begin
                        byte_seq_r <= {byte_seq_r[5:0],
                                       byte_seq_r[6] ^ byte_seq_r[5]};
                        state_r <= S_STATUS_ADDR;
                    end
                end
            end

            S_WAIT_CALC: begin
                if (finish_calc) begin
                    ndy_r[255:0] <= m;
                    byte_seq_r   <= 7'h01;
                    state_r      <= S_TX_STATUS_ADDR;
                end
            end

            S_TX_STATUS_ADDR: begin
                if (s_axi_arready)
                    state_r <= S_TX_STATUS_DATA;
            end

            S_TX_STATUS_DATA: begin
                if (s_axi_rvalid) begin
                    if (s_axi_rdata[TX_FULL_BIT]) begin
                        state_r <= S_TX_STATUS_ADDR;
                    end else begin
                        aw_done_r <= 1'b0;
                        w_done_r  <= 1'b0;
                        state_r   <= S_TX_DATA;
                    end
                end
            end

            S_TX_DATA: begin
                if (s_axi_awready)
                    aw_done_r <= 1'b1;
                if (s_axi_wready)
                    w_done_r <= 1'b1;

                if ((aw_done_r || s_axi_awready) &&
                    (w_done_r || s_axi_wready))
                    state_r <= S_TX_RESP;
            end

            S_TX_RESP: begin
                if (s_axi_bvalid) begin
                    ndy_r[255:0] <= {ndy_r[247:0], 8'b0};

                    // The same LFSR value 7'h45 occurs before byte 31.
                    if (byte_seq_r == 7'h45) begin
                        state_r <= S_FINISH;
                    end else begin
                        byte_seq_r <= {byte_seq_r[5:0],
                                       byte_seq_r[6] ^ byte_seq_r[5]};
                        state_r <= S_TX_STATUS_ADDR;
                    end
                end
            end

            default: state_r <= S_FINISH;
        endcase
    end
end

endmodule
