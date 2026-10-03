`timescale 1ns/1ps

module Rsa256Wrapper (
    input           i_clk,
    input           i_rst,
    input           rx,
	output          tx
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
logic [767:0] ndy_r;
logic [6:0] byte_seq_r;
logic [255:0] m;
logic finish_r;
logic finish_calc;

/********************************************************
    RSA core control

    MSB-first square-and-multiply.  The received buffer is
    also the storage for N, d and y, so no n_r/d_r/y_r copy
    is required:
      ndy_r[767:512] : N (byte-rotated during modulo)
      ndy_r[511:256] : d (shifted left after each round)
      ndy_r[255:0]   : y (constant until TX starts)

    For each modular multiplication, A and B preserve the
    two operands and m itself is cleared and used as the
    partial sum.  There is no separate 256-bit accumulator.
**********************************************************/
localparam logic [3:0] C_IDLE      = 4'd0;
localparam logic [3:0] C_ROUND     = 4'd1;
localparam logic [3:0] C_START_SQ  = 4'd2;
localparam logic [3:0] C_START_MUL = 4'd3;
localparam logic [3:0] C_MUL_ADD   = 4'd4;
localparam logic [3:0] C_MUL_SUB   = 4'd5;
localparam logic [3:0] C_MUL_DONE  = 4'd6;
localparam logic [3:0] C_DONE      = 4'd7;
localparam logic [3:0] C_HOLD      = 4'd8;

// LFSR values immediately before operation 32 and 256.
localparam logic [5:0] C_BYTE_LAST = 6'h24;
localparam logic [8:0] C_BIT_LAST  = 9'h150;

logic [3:0] core_state_r;
logic [255:0] a_r, b_r;
logic [5:0] core_byte_seq_r;
logic [8:0] mul_seq_r, round_seq_r;
logic mul_by_y_r;
logic add_carry_m_r, add_carry_b_r;
logic sub_carry_m_r, sub_carry_b_r;
logic ge_m_r, ge_b_r;
logic need_sub_m_r, need_sub_b_r;
logic lane_b_r; // 0: operate on m, 1: operate on B

function automatic logic [5:0] core_lfsr6_next(input logic [5:0] q);
    core_lfsr6_next = {q[4:0], q[5] ^ q[4]};
endfunction

function automatic logic [8:0] core_lfsr9_next(input logic [8:0] q);
    core_lfsr9_next = {q[7:0], q[8] ^ q[4]};
endfunction

logic [8:0] adder_lhs, adder_rhs;
logic adder_cin;

always_comb begin
    adder_lhs = lane_b_r ? {1'b0, b_r[7:0]} :
                           {1'b0, m[7:0]};
    if (core_state_r == C_MUL_SUB) begin
        adder_rhs = {1'b0, ~ndy_r[519:512]};
        adder_cin = lane_b_r ? sub_carry_b_r : sub_carry_m_r;
    end else begin
        adder_rhs = lane_b_r ? {1'b0, b_r[7:0]} :
                    (a_r[0] ? {1'b0, b_r[7:0]} : 9'd0);
        adder_cin = lane_b_r ? add_carry_b_r : add_carry_m_r;
    end
end

// One byte-wide carry chain and comparator are time-shared by m and B.
wire [8:0] byte_sum = adder_lhs + adder_rhs + adder_cin;
wire ge_lane_next = (byte_sum[7:0] > ndy_r[519:512]) ? 1'b1 :
                    (byte_sum[7:0] < ndy_r[519:512]) ? 1'b0 :
                    (lane_b_r ? ge_b_r : ge_m_r);

assign finish_calc = (core_state_r == C_DONE);

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

// These are combinational AXI wires, not flip-flops.
assign s_axi_awaddr  = TX_BASE;
assign s_axi_awvalid = (state_r == S_TX_DATA) && !aw_done_r;
assign s_axi_wdata   = {24'b0, m[247:240]};
assign s_axi_wvalid  = (state_r == S_TX_DATA) && !w_done_r;
assign s_axi_bready  = (state_r == S_TX_RESP);

assign s_axi_araddr  = (state_r == S_RX_ADDR) ? RX_BASE : STATUS_BASE;
assign s_axi_arvalid = (state_r == S_STATUS_ADDR) ||
                       (state_r == S_RX_ADDR) ||
                       (state_r == S_TX_STATUS_ADDR);
assign s_axi_rready  = (state_r == S_STATUS_DATA) ||
                       (state_r == S_RX_DATA) ||
                       (state_r == S_TX_STATUS_DATA);

always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        ndy_r      <= 768'b0;
        byte_seq_r <= 7'h01;
        finish_r   <= 1'b0;
        state_r    <= S_STATUS_ADDR;
        aw_done_r  <= 1'b0;
        w_done_r   <= 1'b0;

        core_state_r     <= C_IDLE;
        m                <= 256'b0;
        a_r              <= 256'b0;
        b_r              <= 256'b0;
        core_byte_seq_r  <= 6'h01;
        mul_seq_r        <= 9'h001;
        round_seq_r      <= 9'h001;
        mul_by_y_r       <= 1'b0;
        add_carry_m_r    <= 1'b0;
        add_carry_b_r    <= 1'b0;
        sub_carry_m_r    <= 1'b1;
        sub_carry_b_r    <= 1'b1;
        ge_m_r           <= 1'b1;
        ge_b_r           <= 1'b1;
        need_sub_m_r     <= 1'b0;
        need_sub_b_r     <= 1'b0;
        lane_b_r         <= 1'b0;
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
                    m <= {m[247:0], 8'b0};

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

        case (core_state_r)
            C_IDLE: begin
                if (finish_r) begin
                    m           <= 256'd1;
                    round_seq_r <= 9'h001;
                    core_state_r <= C_ROUND;
                end
            end

            C_ROUND: begin
                // Process d from bit 255 to bit 0.
                core_state_r <= C_START_SQ;
            end

            C_START_SQ: begin
                // m = m*m mod N: operands survive while m becomes acc.
                a_r             <= m;
                b_r             <= m;
                m               <= 256'b0;
                mul_by_y_r      <= 1'b0;
                mul_seq_r       <= 9'h001;
                core_byte_seq_r <= 6'h01;
                add_carry_m_r   <= 1'b0;
                add_carry_b_r   <= 1'b0;
                ge_m_r          <= 1'b1;
                ge_b_r          <= 1'b1;
                need_sub_m_r    <= 1'b0;
                need_sub_b_r    <= 1'b0;
                lane_b_r        <= 1'b0;
                core_state_r    <= C_MUL_ADD;
            end

            C_START_MUL: begin
                // m = m*y mod N; y stays in ndy_r[255:0].
                a_r             <= m;
                b_r             <= ndy_r[255:0];
                m               <= 256'b0;
                mul_by_y_r      <= 1'b1;
                mul_seq_r       <= 9'h001;
                core_byte_seq_r <= 6'h01;
                add_carry_m_r   <= 1'b0;
                add_carry_b_r   <= 1'b0;
                ge_m_r          <= 1'b1;
                ge_b_r          <= 1'b1;
                need_sub_m_r    <= 1'b0;
                need_sub_b_r    <= 1'b0;
                lane_b_r        <= 1'b0;
                core_state_r    <= C_MUL_ADD;
            end

            C_MUL_ADD: begin
                // One shared adder: m lane first, B lane second.
                if (!lane_b_r) begin
                    m <= {byte_sum[7:0], m[255:8]};
                    add_carry_m_r <= byte_sum[8];
                    ge_m_r <= ge_lane_next;
                    lane_b_r <= 1'b1;
                end else begin
                    b_r <= {byte_sum[7:0], b_r[255:8]};
                    ndy_r[767:512] <=
                        {ndy_r[519:512], ndy_r[767:520]};
                    add_carry_b_r <= byte_sum[8];
                    ge_b_r <= ge_lane_next;
                    lane_b_r <= 1'b0;

                    if (core_byte_seq_r == C_BYTE_LAST) begin
                        need_sub_m_r <= add_carry_m_r | ge_m_r;
                        need_sub_b_r <= byte_sum[8] | ge_lane_next;
                        core_byte_seq_r <= 6'h01;

                        if ((add_carry_m_r | ge_m_r) ||
                            (byte_sum[8] | ge_lane_next)) begin
                            sub_carry_m_r <= 1'b1;
                            sub_carry_b_r <= 1'b1;
                            core_state_r <= C_MUL_SUB;
                        end else if (mul_seq_r == C_BIT_LAST) begin
                            core_state_r <= C_MUL_DONE;
                        end else begin
                            a_r <= {1'b0, a_r[255:1]};
                            mul_seq_r <= core_lfsr9_next(mul_seq_r);
                            add_carry_m_r <= 1'b0;
                            add_carry_b_r <= 1'b0;
                            ge_m_r <= 1'b1;
                            ge_b_r <= 1'b1;
                        end
                    end else begin
                        core_byte_seq_r <=
                            core_lfsr6_next(core_byte_seq_r);
                    end
                end
            end

            C_MUL_SUB: begin
                // The same adder performs m-N and B-N in two phases.
                if (!lane_b_r) begin
                    m <= need_sub_m_r ?
                         {byte_sum[7:0], m[255:8]} :
                         {m[7:0], m[255:8]};
                    if (need_sub_m_r)
                        sub_carry_m_r <= byte_sum[8];
                    lane_b_r <= 1'b1;
                end else begin
                    b_r <= need_sub_b_r ?
                           {byte_sum[7:0], b_r[255:8]} :
                           {b_r[7:0], b_r[255:8]};
                    ndy_r[767:512] <=
                        {ndy_r[519:512], ndy_r[767:520]};
                    if (need_sub_b_r)
                        sub_carry_b_r <= byte_sum[8];
                    lane_b_r <= 1'b0;

                    if (core_byte_seq_r == C_BYTE_LAST) begin
                        core_byte_seq_r <= 6'h01;
                        if (mul_seq_r == C_BIT_LAST) begin
                            core_state_r <= C_MUL_DONE;
                        end else begin
                            a_r <= {1'b0, a_r[255:1]};
                            mul_seq_r <= core_lfsr9_next(mul_seq_r);
                            add_carry_m_r <= 1'b0;
                            add_carry_b_r <= 1'b0;
                            ge_m_r <= 1'b1;
                            ge_b_r <= 1'b1;
                            need_sub_m_r <= 1'b0;
                            need_sub_b_r <= 1'b0;
                            core_state_r <= C_MUL_ADD;
                        end
                    end else begin
                        core_byte_seq_r <=
                            core_lfsr6_next(core_byte_seq_r);
                    end
                end
            end

            C_MUL_DONE: begin
                if (!mul_by_y_r && ndy_r[511]) begin
                    // Square finished and current exponent bit is one.
                    core_state_r <= C_START_MUL;
                end else if (round_seq_r == C_BIT_LAST) begin
                    core_state_r <= C_DONE;
                end else begin
                    // Consume the current MSB of d in the existing buffer.
                    ndy_r[511:256] <= {ndy_r[510:256], 1'b0};
                    round_seq_r <= core_lfsr9_next(round_seq_r);
                    core_state_r <= C_ROUND;
                end
            end

            C_DONE: begin
                core_state_r <= C_HOLD;
            end

            C_HOLD: begin
                if (!finish_r)
                    core_state_r <= C_IDLE;
            end

            default: core_state_r <= C_IDLE;
        endcase
    end
end

endmodule
