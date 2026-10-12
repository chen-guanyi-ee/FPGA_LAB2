`timescale 1ns/1ps
/********************************************************
    VgaFlowViz : live data-flow display for Rsa256Wrapper

    640x480 @ 60 Hz, pixel clock = 100 MHz / 4 = 25 MHz
    (25.175 MHz nominal; every common monitor accepts 25.0)

    Picture = 320x240 pre-rendered ROM (viz_rom.mem), shown 2x.
    Each ROM byte: [7:3] object ID, [2:0] shade / phase / state.
    The ROM only stores *what* is where; *which colour* every
    object gets is decided here, every frame, from the live
    FSM / RAM-write / UART signals of the RSA engine.

    Short events (one RAM write, one UART byte) are stretched
    with a per-signal hold counter so the eye can see them.
**********************************************************/
module VgaFlowViz (
    input  logic       i_clk,          // 100 MHz
    input  logic       i_rst,

    // live probes from Rsa256Wrapper
    input  logic [2:0] i_state,
    input  logic [6:0] i_byte_count,
    input  logic [7:0] i_mul_bit,
    input  logic [7:0] i_round_bit,
    input  logic       i_mul_by_y,
    input  logic       i_rx_busy,
    input  logic       i_rx_valid,
    input  logic       i_tx_busy,
    input  logic       i_tx_start,
    input  logic       i_n_we,
    input  logic       i_d_we,
    input  logic       i_y_we,

    output logic [3:0] o_vga_r,
    output logic [3:0] o_vga_g,
    output logic [3:0] o_vga_b,
    output logic       o_vga_hs,
    output logic       o_vga_vs
);

// ---------------------------------------------------- VGA timing
localparam int H_VIS = 640, H_FP = 16, H_SY = 96, H_BP = 48, H_TOT = 800;
localparam int V_VIS = 480, V_FP = 10, V_SY = 2,  V_BP = 33, V_TOT = 525;

// must match gen_viz_rom.py
localparam int ROM_W  = 320;
localparam int BAR_X0 = 190, BAR_W = 128, BAR_H = 5;
localparam int BAR_Y0 = 186, BAR_Y1 = 204, BAR_Y2 = 222;

localparam logic [2:0] S_MUL_LOAD = 3'd4;
localparam logic [2:0] S_MUL_ADD  = 3'd5;
localparam logic [2:0] S_MUL_SUB  = 3'd6;

logic [1:0] pix_div_r;
logic [9:0] h_r, v_r;
wire        pix_ce     = (pix_div_r == 2'd3);
wire        frame_tick = pix_ce && (h_r == H_TOT-1) && (v_r == V_TOT-1);

always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        pix_div_r <= 2'd0;
        h_r       <= 10'd0;
        v_r       <= 10'd0;
    end else begin
        pix_div_r <= pix_div_r + 1'b1;
        if (pix_ce) begin
            if (h_r == H_TOT-1) begin
                h_r <= 10'd0;
                v_r <= (v_r == V_TOT-1) ? 10'd0 : v_r + 1'b1;
            end else begin
                h_r <= h_r + 1'b1;
            end
        end
    end
end

wire       visible = (h_r < H_VIS) && (v_r < V_VIS);
wire [8:0] rx_px   = h_r[9:1];               // 0..319 while visible
wire [7:0] ry_px   = v_r[8:1];               // 0..239 while visible

// ---------------------------------------------------- picture ROM (BRAM)
(* rom_style = "block" *) logic [7:0] rom [0:76799];
initial $readmemh("viz_rom.mem", rom);

wire [16:0] rom_addr = visible ? ({1'b0, ry_px, 8'd0} + {3'd0, ry_px, 6'd0} + {8'd0, rx_px})
                               : 17'd0;
// Two-stage read (BRAM + its output register): data for the current
// h/v is ready 2 clocks after they change and is used on the 4th clock.
logic [7:0] rom_q0, rom_q;
always_ff @(posedge i_clk) begin
    rom_q0 <= rom[rom_addr];
    rom_q  <= rom_q0;
end

// ---------------------------------------------------- activity tracking
localparam int A_RX    = 0;
localparam int A_WN    = 1;
localparam int A_WD    = 2;
localparam int A_WY    = 3;
localparam int A_LOAD  = 4;
localparam int A_LOADY = 5;
localparam int A_ALU   = 6;
localparam int A_TXW   = 7;
localparam int A_TX    = 8;
localparam int A_ST0   = 9;                  // 9..16 : FSM state k was visited
localparam int NACT    = 17;
localparam logic [2:0] HOLD_FRAMES = 3'd5;   // ~83 ms afterglow

logic [NACT-1:0] act_now, seen_r, act;
logic [2:0]      hold_r [0:NACT-1];

wire in_core = (i_state == S_MUL_LOAD) || (i_state == S_MUL_ADD) || (i_state == S_MUL_SUB);

always_comb begin
    act_now          = '0;
    act_now[A_RX]    = i_rx_busy | i_rx_valid;
    act_now[A_WN]    = i_n_we;
    act_now[A_WD]    = i_d_we;
    act_now[A_WY]    = i_y_we;
    act_now[A_LOAD]  = (i_state == S_MUL_LOAD);
    act_now[A_LOADY] = (i_state == S_MUL_LOAD) && i_mul_by_y;
    act_now[A_ALU]   = (i_state == S_MUL_ADD) || (i_state == S_MUL_SUB);
    act_now[A_TXW]   = i_tx_start;
    act_now[A_TX]    = i_tx_busy;
    for (int k = 0; k < 8; k++)
        act_now[A_ST0 + k] = (i_state == 3'(k));
end

always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        seen_r <= '0;
        for (int k = 0; k < NACT; k++) hold_r[k] <= 3'd0;
    end else if (frame_tick) begin
        seen_r <= '0;
        for (int k = 0; k < NACT; k++) begin
            if (seen_r[k] | act_now[k])  hold_r[k] <= HOLD_FRAMES;
            else if (hold_r[k] != 3'd0)  hold_r[k] <= hold_r[k] - 1'b1;
        end
    end else begin
        seen_r <= seen_r | act_now;
    end
end

always_comb
    for (int k = 0; k < NACT; k++) act[k] = (hold_r[k] != 3'd0);

wire [7:0] st_act = act[A_ST0 +: 8];

// object-ID -> "is active" (IDs defined in gen_viz_rom.py)
logic [31:0] obj_act;
always_comb begin
    obj_act      = '0;
    obj_act[3]   = act[A_RX];                                // UART RX
    obj_act[4]   = act[A_TX];                                // UART TX
    obj_act[5]   = act[A_WN]  | act[A_ALU];                  // N RAM
    obj_act[6]   = act[A_WD]  | act[A_LOAD];                 // d RAM (exp bit read)
    obj_act[7]   = act[A_WY]  | act[A_LOADY];                // y RAM
    obj_act[8]   = act[A_LOAD] | act[A_ALU];                 // a RAM
    obj_act[9]   = act[A_LOAD] | act[A_ALU];                 // b RAM
    obj_act[10]  = act[A_LOAD] | act[A_ALU] | act[A_TXW];    // m RAM
    obj_act[11]  = act[A_ALU];                               // ALU
    obj_act[12]  = 1'b1;                                     // FSM always running
    obj_act[13]  = act[A_RX];                                // PC  -> RX
    obj_act[14]  = act[A_WN];                                // RX  -> N
    obj_act[15]  = act[A_WD];                                // RX  -> d
    obj_act[16]  = act[A_WY];                                // RX  -> y
    obj_act[17]  = act[A_LOAD];                              // m/y -> a,b
    obj_act[18]  = act[A_ALU];                               // operands -> ALU
    obj_act[19]  = act[A_ALU];                               // ALU -> m,b
    obj_act[20]  = act[A_TXW];                               // m   -> TX
    obj_act[21]  = act[A_TX];                                // TX  -> PC
    obj_act[22]  = act[A_LOAD];                              // d[i] -> FSM
end

// ---------------------------------------------------- frame counter / progress values
logic [5:0] frame_cnt_r;
logic [6:0] io_fill_r;

always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        frame_cnt_r <= 6'd0;
        io_fill_r   <= 7'd0;
    end else begin
        if (frame_tick) frame_cnt_r <= frame_cnt_r + 1'b1;
        if (!in_core) begin
            if (i_mul_by_y)   // TX: 0..30 bytes
                io_fill_r <= {i_byte_count[4:0], 2'b00};
            else              // RX: 0..95 bytes  (x1.31 ~ 128/96)
                io_fill_r <= i_byte_count + {2'b00, i_byte_count[6:2]} + {4'b0000, i_byte_count[6:4]};
        end
    end
end

wire [2:0] anim = frame_cnt_r[3:1];         // marching-dot phase, 30 steps/s

// ---------------------------------------------------- colour functions
function automatic logic [11:0] shade_c(input logic [11:0] c, input logic [2:0] s);
    logic [7:0] r, g, b;
    begin
        r = c[11:8] * ({1'b0, s} + 4'd1);
        g = c[7:4]  * ({1'b0, s} + 4'd1);
        b = c[3:0]  * ({1'b0, s} + 4'd1);
        shade_c = {r[6:3], g[6:3], b[6:3]};
    end
endfunction

function automatic logic [11:0] solid_color(input logic [4:0] id, input logic on);
    case (id)
        5'd0:                solid_color = 12'h359;              // background
        5'd1:                solid_color = 12'hFFF;              // text
        5'd2:                solid_color = 12'h366;              // board / frames
        5'd3, 5'd4:          solid_color = on ? 12'h3F6 : 12'h253;   // UART
        5'd5, 5'd6, 5'd7:    solid_color = on ? 12'h68F : 12'h337;   // N, d, y RAM
        5'd8, 5'd9, 5'd10:   solid_color = on ? 12'h3DF : 12'h256;   // a, b, m RAM
        5'd11:               solid_color = on ? 12'hF82 : 12'h631;   // ALU
        5'd12:               solid_color = on ? 12'hC5F : 12'h526;   // FSM
        default:             solid_color = 12'h000;
    endcase
endfunction

function automatic logic [11:0] node_color(input logic [2:0] st, input logic on);
    if (!on) node_color = 12'h335;
    else case (st)
        3'd0, 3'd1: node_color = 12'h6AF;    // status polling
        3'd2, 3'd3: node_color = 12'h3F6;    // RX
        3'd4:       node_color = 12'h3DF;    // load
        3'd5:       node_color = 12'hF82;    // add
        3'd6:       node_color = 12'hF44;    // subtract
        default:    node_color = 12'h3F6;    // TX
    endcase
endfunction

// ---------------------------------------------------- pixel colour
wire [4:0] obj = rom_q[7:3];
wire [2:0] low = rom_q[2:0];
wire [2:0] dot = low - anim;

wire in_bar_x = (rx_px >= BAR_X0) && (rx_px < BAR_X0 + BAR_W);
wire in_bar0  = in_bar_x && (ry_px >= BAR_Y0) && (ry_px < BAR_Y0 + BAR_H);
wire in_bar1  = in_bar_x && (ry_px >= BAR_Y1) && (ry_px < BAR_Y1 + BAR_H);
wire in_bar2  = in_bar_x && (ry_px >= BAR_Y2) && (ry_px < BAR_Y2 + BAR_H);
wire [8:0] bar_off = rx_px - BAR_X0;

logic [11:0] pix_color;
always_comb begin
    if (obj <= 5'd12)
        pix_color = shade_c(solid_color(obj, obj_act[obj]), low);
    else if (obj <= 5'd22)                                   // pipes: marching dots
        pix_color = !obj_act[obj]  ? 12'h334 :
                    (dot < 3'd3)   ? 12'hFF8 : 12'h651;
    else if (obj == 5'd23)                                   // flowchart arrows
        pix_color = st_act[low] ? 12'hFD4 : 12'h334;
    else                                                     // flowchart nodes / legend
        pix_color = shade_c(node_color(obj[2:0], st_act[obj[2:0]]), low);

    if (in_bar0 && (bar_off < {2'b00, io_fill_r}))             pix_color = 12'h3F6;
    if (in_bar1 && (bar_off < {2'b00, i_mul_bit[7:1]}))        pix_color = 12'hF82;
    if (in_bar2 && (bar_off < {2'b00, i_round_bit[7:1]}))      pix_color = 12'hC5F;
end

always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        {o_vga_r, o_vga_g, o_vga_b} <= 12'h000;
        o_vga_hs <= 1'b1;
        o_vga_vs <= 1'b1;
    end else if (pix_ce) begin
        {o_vga_r, o_vga_g, o_vga_b} <= visible ? pix_color : 12'h000;
        o_vga_hs <= !((h_r >= H_VIS + H_FP) && (h_r < H_VIS + H_FP + H_SY));
        o_vga_vs <= !((v_r >= V_VIS + V_FP) && (v_r < V_VIS + V_FP + V_SY));
    end
end

endmodule
