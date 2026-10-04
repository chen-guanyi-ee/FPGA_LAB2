`timescale 1ns/1ps

module Rsa256Wrapper (
    input  logic i_clk,
    input  logic i_rst,
    input  logic rx,
    output logic tx
);

localparam logic [3:0] RX_BASE     = 4'h0;
localparam logic [3:0] TX_BASE     = 4'h4;
localparam logic [3:0] STATUS_BASE = 4'h8;

localparam int RX_OK_BIT   = 0;
localparam int TX_FULL_BIT = 3;

localparam logic [2:0] S_STATUS_ADDR = 3'd0;
localparam logic [2:0] S_STATUS_DATA = 3'd1;
localparam logic [2:0] S_RX_ADDR     = 3'd2;
localparam logic [2:0] S_RX_DATA_HOLD= 3'd3;
localparam logic [2:0] S_MUL_LOAD    = 3'd4;
localparam logic [2:0] S_MUL_ADD     = 3'd5;
localparam logic [2:0] S_MUL_SUB     = 3'd6;
localparam logic [2:0] S_TX_DATA     = 3'd7;

/********************************************************
    Byte memories

    All values use little-endian byte addresses:
      address 0  : least-significant byte
      address 31 : most-significant byte

    These memories intentionally have no reset. RX or
    S_MUL_LOAD completely overwrites every location before
    it is consumed. Asynchronous reads allow the original
    one-byte-per-lane datapath to infer small LUT RAMs.
**********************************************************/
(* ram_style = "distributed" *) logic [7:0] n_mem [0:31];
(* ram_style = "distributed" *) logic [7:0] d_mem [0:31];
(* ram_style = "distributed" *) logic [7:0] y_mem [0:31];
(* ram_style = "distributed" *) logic [7:0] a_mem [0:31];
(* ram_style = "distributed" *) logic [7:0] b_mem [0:31];
(* ram_style = "distributed" *) logic [7:0] m_mem [0:31];

logic n_we, d_we, y_we, a_we, b_we, m_we;
logic [4:0] mem_waddr;
logic [7:0] n_wdata, d_wdata, y_wdata;
logic [7:0] a_wdata, b_wdata, m_wdata;

/********************************************************
    Shared control

    byte_count_r is reused for 96-byte RX, 32-byte core
    passes, operand loading and 31-byte TX. The two 8-bit
    counters identify the current multiplier and exponent
    bits. Binary counters also directly address RAM.
**********************************************************/
(* fsm_encoding = "sequential" *) logic [2:0] state_r;
logic [6:0] byte_count_r;
logic [7:0] mul_bit_r, round_bit_r;
logic mul_by_y_r;
logic load_one_r;
logic carry_m_r, carry_b_r;
logic ge_m_r, ge_b_r;
logic lane_b_r;
logic a_bit_r;
logic aw_done_r, w_done_r;

wire [4:0] core_addr = byte_count_r[4:0];
wire [7:0] n_byte = n_mem[core_addr];
wire [7:0] b_byte = b_mem[core_addr];
wire [7:0] m_byte = m_mem[core_addr];
wire [7:0] y_byte = y_mem[core_addr];
wire [7:0] mul_bit_next = mul_bit_r + 1'b1;
wire [7:0] next_a_byte = a_mem[mul_bit_next[7:3]];
wire       next_a_bit  = next_a_byte[mul_bit_next[2:0]];

wire [4:0] exp_byte_addr = 5'd31 - round_bit_r[7:3];
wire [2:0] exp_bit_addr  = 3'd7  - round_bit_r[2:0];
wire       exp_bit       = d_mem[exp_byte_addr][exp_bit_addr];

wire [4:0] tx_addr = 5'd30 - byte_count_r[4:0];
wire [7:0] tx_byte = m_mem[tx_addr];

// One byte-wide carry chain and comparator are time-shared by M and B.
wire [8:0] byte_sum =
    (lane_b_r ? {1'b0, b_byte} : {1'b0, m_byte}) +
    ((state_r == S_MUL_SUB) ? {1'b0, ~n_byte} :
     lane_b_r ? {1'b0, b_byte} :
     a_bit_r ? {1'b0, b_byte} : 9'd0) +
    (lane_b_r ? carry_b_r : carry_m_r);

wire ge_lane_next = (byte_sum[7:0] > n_byte) ? 1'b1 :
                    (byte_sum[7:0] < n_byte) ? 1'b0 :
                    (lane_b_r ? ge_b_r : ge_m_r);

/********************************************************
    AXI UART Lite
**********************************************************/
logic [3:0]  s_axi_awaddr;
logic        s_axi_awvalid;
logic        s_axi_awready;
logic [31:0] s_axi_wdata;
logic        s_axi_wvalid;
logic        s_axi_wready;
logic [3:0]  s_axi_araddr;
logic        s_axi_arvalid;
logic        s_axi_arready;
logic [31:0] s_axi_rdata;
logic        s_axi_rvalid;
logic        s_axi_rready;

axi_uartlite_0 uart (
    .s_axi_aclk(i_clk),
    .s_axi_aresetn(!i_rst),
    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_wstrb(4'b1111),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bresp(),
    .s_axi_bvalid(),
    .s_axi_bready(1'b1),
    .s_axi_araddr(s_axi_araddr),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready),
    .rx(rx),
    .tx(tx)
);

assign s_axi_awaddr  = TX_BASE;
assign s_axi_awvalid = (state_r == S_TX_DATA) && !aw_done_r;
assign s_axi_wdata   = {24'b0, tx_byte};
assign s_axi_wvalid  = (state_r == S_TX_DATA) && !w_done_r;

assign s_axi_araddr  = (state_r == S_RX_ADDR) ? RX_BASE : STATUS_BASE;
assign s_axi_arvalid = (state_r == S_STATUS_ADDR) ||
                       (state_r == S_RX_ADDR);
assign s_axi_rready  = (state_r == S_STATUS_DATA) ||
                       ((state_r == S_RX_DATA_HOLD) && !mul_by_y_r);

/********************************************************
    Single write port for each byte memory, with no reset.
**********************************************************/
always_comb begin
    n_we = 1'b0;
    d_we = 1'b0;
    y_we = 1'b0;
    a_we = 1'b0;
    b_we = 1'b0;
    m_we = 1'b0;

    mem_waddr = core_addr;
    n_wdata = 8'b0;
    d_wdata = 8'b0;
    y_wdata = 8'b0;
    a_wdata = 8'b0;
    b_wdata = 8'b0;
    m_wdata = 8'b0;

    if (!i_rst) begin
        case (state_r)
            S_RX_DATA_HOLD: begin
                if (!mul_by_y_r && s_axi_rvalid) begin
                    mem_waddr = 5'd31 - byte_count_r[4:0];
                    case (byte_count_r[6:5])
                        2'b00: begin
                            n_we = 1'b1;
                            n_wdata = s_axi_rdata[7:0];
                        end
                        2'b01: begin
                            d_we = 1'b1;
                            d_wdata = s_axi_rdata[7:0];
                        end
                        default: begin
                            y_we = 1'b1;
                            y_wdata = s_axi_rdata[7:0];
                        end
                    endcase
                end
            end

            S_MUL_LOAD: begin
                a_we = 1'b1;
                b_we = 1'b1;
                m_we = 1'b1;

                if (load_one_r) begin
                    a_wdata = (core_addr == 5'd0) ? 8'h01 : 8'h00;
                    b_wdata = (core_addr == 5'd0) ? 8'h01 : 8'h00;
                end else begin
                    a_wdata = m_byte;
                    b_wdata = mul_by_y_r ? y_byte : m_byte;
                end
                m_wdata = 8'h00;
            end

            S_MUL_ADD: begin
                if (!lane_b_r) begin
                    m_we = 1'b1;
                    m_wdata = byte_sum[7:0];
                end else begin
                    b_we = 1'b1;
                    b_wdata = byte_sum[7:0];
                end
            end

            S_MUL_SUB: begin
                if (!lane_b_r && ge_m_r) begin
                    m_we = 1'b1;
                    m_wdata = byte_sum[7:0];
                end else if (lane_b_r && ge_b_r) begin
                    b_we = 1'b1;
                    b_wdata = byte_sum[7:0];
                end
            end

            default: begin
            end
        endcase
    end
end

always_ff @(posedge i_clk) begin
    if (n_we) n_mem[mem_waddr] <= n_wdata;
    if (d_we) d_mem[mem_waddr] <= d_wdata;
    if (y_we) y_mem[mem_waddr] <= y_wdata;
    if (a_we) a_mem[mem_waddr] <= a_wdata;
    if (b_we) b_mem[mem_waddr] <= b_wdata;
    if (m_we) m_mem[mem_waddr] <= m_wdata;
end

/********************************************************
    Resettable control only. Datapath RAM is deliberately
    excluded from this asynchronous-reset process.
**********************************************************/
always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        state_r       <= S_STATUS_ADDR;
        byte_count_r  <= 7'd0;
        mul_bit_r     <= 8'd0;
        round_bit_r   <= 8'd0;
        mul_by_y_r    <= 1'b0;
        load_one_r    <= 1'b0;
        carry_m_r     <= 1'b0;
        carry_b_r     <= 1'b0;
        ge_m_r        <= 1'b1;
        ge_b_r        <= 1'b1;
        lane_b_r      <= 1'b0;
        a_bit_r       <= 1'b0;
        aw_done_r     <= 1'b0;
        w_done_r      <= 1'b0;
    end else begin
        case (state_r)
            S_STATUS_ADDR: begin
                if (s_axi_arready)
                    state_r <= S_STATUS_DATA;
            end

            S_STATUS_DATA: begin
                if (s_axi_rvalid) begin
                    if (!mul_by_y_r) begin
                        state_r <= s_axi_rdata[RX_OK_BIT] ?
                                   S_RX_ADDR : S_STATUS_ADDR;
                    end else if (s_axi_rdata[TX_FULL_BIT]) begin
                        state_r <= S_STATUS_ADDR;
                    end else begin
                        aw_done_r <= 1'b0;
                        w_done_r  <= 1'b0;
                        state_r   <= S_TX_DATA;
                    end
                end
            end

            S_RX_ADDR: begin
                if (s_axi_arready)
                    state_r <= S_RX_DATA_HOLD;
            end

            S_RX_DATA_HOLD: begin
                if (!mul_by_y_r && s_axi_rvalid) begin
                    if (byte_count_r == 7'd95) begin
                        byte_count_r <= 7'd0;
                        round_bit_r  <= 8'd0;
                        mul_bit_r    <= 8'd0;
                        mul_by_y_r   <= 1'b0;
                        load_one_r   <= 1'b1;
                        state_r      <= S_MUL_LOAD;
                    end else begin
                        byte_count_r <= byte_count_r + 1'b1;
                        state_r <= S_STATUS_ADDR;
                    end
                end
            end

            S_MUL_LOAD: begin
                if (byte_count_r == 7'd0)
                    a_bit_r <= a_wdata[0];

                if (byte_count_r == 7'd31) begin
                    byte_count_r <= 7'd0;
                    mul_bit_r    <= 8'd0;
                    load_one_r   <= 1'b0;
                    carry_m_r    <= 1'b0;
                    carry_b_r    <= 1'b0;
                    ge_m_r       <= 1'b1;
                    ge_b_r       <= 1'b1;
                    lane_b_r     <= 1'b0;
                    state_r      <= S_MUL_ADD;
                end else begin
                    byte_count_r <= byte_count_r + 1'b1;
                end
            end

            S_MUL_ADD: begin
                if (!lane_b_r) begin
                    carry_m_r <= byte_sum[8];
                    ge_m_r    <= ge_lane_next;
                    lane_b_r  <= 1'b1;
                end else begin
                    carry_b_r <= byte_sum[8];
                    ge_b_r    <= ge_lane_next;
                    lane_b_r  <= 1'b0;

                    if (byte_count_r != 7'd31) begin
                        byte_count_r <= byte_count_r + 1'b1;
                    end else begin
                        byte_count_r <= 7'd0;

                        if ((carry_m_r | ge_m_r) ||
                            (byte_sum[8] | ge_lane_next)) begin
                            ge_m_r    <= carry_m_r | ge_m_r;
                            ge_b_r    <= byte_sum[8] | ge_lane_next;
                            carry_m_r <= 1'b1;
                            carry_b_r <= 1'b1;
                            state_r   <= S_MUL_SUB;
                        end else if (mul_bit_r != 8'hff) begin
                            mul_bit_r  <= mul_bit_r + 1'b1;
                            a_bit_r    <= next_a_bit;
                            carry_m_r  <= 1'b0;
                            carry_b_r  <= 1'b0;
                            ge_m_r     <= 1'b1;
                            ge_b_r     <= 1'b1;
                        end else if (!mul_by_y_r && exp_bit) begin
                            mul_by_y_r <= 1'b1;
                            load_one_r <= 1'b0;
                            state_r    <= S_MUL_LOAD;
                        end else if (round_bit_r == 8'hff) begin
                            mul_by_y_r <= 1'b1;
                            state_r    <= S_STATUS_ADDR;
                        end else begin
                            round_bit_r <= round_bit_r + 1'b1;
                            mul_by_y_r  <= 1'b0;
                            load_one_r  <= 1'b0;
                            state_r     <= S_MUL_LOAD;
                        end
                    end
                end
            end

            S_MUL_SUB: begin
                if (!lane_b_r) begin
                    if (ge_m_r)
                        carry_m_r <= byte_sum[8];
                    lane_b_r <= 1'b1;
                end else begin
                    if (ge_b_r)
                        carry_b_r <= byte_sum[8];
                    lane_b_r <= 1'b0;

                    if (byte_count_r != 7'd31) begin
                        byte_count_r <= byte_count_r + 1'b1;
                    end else begin
                        byte_count_r <= 7'd0;

                        if (mul_bit_r != 8'hff) begin
                            mul_bit_r <= mul_bit_r + 1'b1;
                            a_bit_r   <= next_a_bit;
                            carry_m_r <= 1'b0;
                            carry_b_r <= 1'b0;
                            ge_m_r    <= 1'b1;
                            ge_b_r    <= 1'b1;
                            state_r   <= S_MUL_ADD;
                        end else if (!mul_by_y_r && exp_bit) begin
                            mul_by_y_r <= 1'b1;
                            load_one_r <= 1'b0;
                            state_r    <= S_MUL_LOAD;
                        end else if (round_bit_r == 8'hff) begin
                            mul_by_y_r <= 1'b1;
                            state_r    <= S_STATUS_ADDR;
                        end else begin
                            round_bit_r <= round_bit_r + 1'b1;
                            mul_by_y_r  <= 1'b0;
                            load_one_r  <= 1'b0;
                            state_r     <= S_MUL_LOAD;
                        end
                    end
                end
            end

            S_TX_DATA: begin
                if (s_axi_awready)
                    aw_done_r <= 1'b1;
                if (s_axi_wready)
                    w_done_r <= 1'b1;

                if ((aw_done_r || s_axi_awready) &&
                    (w_done_r || s_axi_wready)) begin
                    if (byte_count_r == 7'd30) begin
                        state_r <= S_RX_DATA_HOLD;
                    end else begin
                        byte_count_r <= byte_count_r + 1'b1;
                        state_r <= S_STATUS_ADDR;
                    end
                end
            end

            default: state_r <= S_STATUS_ADDR;
        endcase
    end
end

endmodule


