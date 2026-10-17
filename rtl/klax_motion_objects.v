// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.
//
//============================================================================
// Klax Motion Object Line-Buffer Renderer
//
// Full Atari motion-object renderer for Klax. Per scanline, walks the
// SLIP-linked descriptor list and renders every covering object's tile row
// into a double-buffered 512-pixel line buffer. Display side reads the
// front buffer at native X and erases behind the read pointer so the buffer
// is clean when it becomes the back buffer two lines later.
//
// Behavior is implemented from verified MAME 0.288 sources:
//   mame/atarimo.cpp  build_active_list()  — SLIP[line>>3] start link,
//                      linked walk on word0[7:0], visited-array stop,
//                      MAX_PER_BANK=1024 iteration cap (maxperline=0)
//   mame/atarimo.cpp  render_object()      — field extraction, Y negation
//                      and height adjust, mod-512 wrap with visible-area
//                      threshold, hflip tile order, code wrap mod 4096,
//                      transparent pen 0, list-order overwrite
//   mame/atarimo.cpp  draw()               — per-8-line SLIP bands,
//                      non-reversed render order
//   mame/sprite.h     draw_async()         — sprite bitmap sized to the
//                      visible area (336x240), giving the wrap thresholds
//   mame/klax.cpp     s_mob_config         — all field masks; gfx layout
//                      shared with tiles (gfx_8x8x4_packed_msb)
//
// Coordinate model (equivalent to MAME's clipping; proved in simulation
// against the atarimo.cpp reference quoted above):
//   top9   = (0 - (yfield + height*8)) & 511
//   delta9 = (line - top9) & 511,  covers iff delta9 < height*8
//   X writes wrap mod 512; only x = 0..335 is ever displayed.
//
// Sprite ROM row port: one 8x8 tile row = 32 bits, packed MSB
// (pixel 0 in bits [31:28] ... pixel 7 in bits [3:0]), matching byte order
// rom[code*32 + row*4 .. +3] with even pixels in the upper nibble.
// Request/ack handshake so the backend can be a simple hex ROM in
// simulation or a DDRAM fetch engine on MiSTer.
//
// Render budget: one scanline in this clock domain. If the walk plus
// rendering exceeds the line, rendering is aborted deterministically at the
// next line start. Measured over the attract-mode gameplay demo (MAME,
// frames 2400-3600): Klax peaks at only 4 motion objects on any
// scanline (<=14 distinct/frame), vs the 1024-link iteration cap (MAME's
// MAX_PER_BANK, implemented below) and the time budget (peak 550/3648 clk =
// 15%, dbg_abort_seen=0). The line abort is therefore unreachable by Klax
// content (~250x margin); MAME models no time-based dropout (maxperline=0 ->
// the 1024 cap only), so the RTL matches MAME in every case the game produces.
// A true per-line sprite-dropout threshold would need PCB measurement, but it
// is unobservable for this game.
//============================================================================

module klax_motion_objects (
    input               clk,
    input               reset,

    input       [8:0]   scanline,     // current displayed line (0..261)
    input               vblank,

    // Display-side line buffer read, 1-cycle latency.
    // vid_x is the native source X (0..335) of the pixel being displayed.
    // Repeated reads of the same vid_x are allowed (MiSTer upscaling);
    // erase happens only when vid_x advances.
    input       [8:0]   vid_x,
    input               vid_active,
    output reg  [8:0]   mo_pix,       // {valid, color[3:0], pen[3:0]}

    // SLIP RAM video-side read port (klax_mob_ram, addr reg + 1-cycle data)
    output reg  [5:0]   slip_rd_addr,
    input       [15:0]  slip_rd_data,

    // Motion object RAM video-side read port (same timing)
    output reg  [9:0]   mob_rd_addr,
    input       [15:0]  mob_rd_data,

    // Sprite ROM tile-row fetch port
    output reg          sprow_req,
    output reg  [11:0]  sprow_code,   // tile code 0..4095
    output reg  [2:0]   sprow_row,    // row within tile 0..7
    input       [31:0]  sprow_data,   // packed MSB row, pixel 0 = [31:28]
    input               sprow_ack,    // pulses with valid data while req=1

    output      [15:0]  debug_flags
);

    // ====== Line buffers ======
    // Two 512 x 9 buffers. Entry = {valid, color, pen}. Buffer parity:
    // buffer[L & 1] holds line L. While line L is displayed from buffer
    // L[0], the renderer writes line L+1 into buffer ~L[0].
    reg [8:0] linebuf0 [0:511];
    reg [8:0] linebuf1 [0:511];

    // ====== Display + erase side ======
    reg  [8:0] vid_x_d = 9'd0;
    reg        vid_active_d = 1'b0;
    wire       disp_buf = scanline[0];

    // Erase the previous read address once the read pointer moves past it
    // (or when the active region ends). Erasing only displayed addresses
    // (0..335) is sufficient: 336..511 is never read back.
    wire erase_strobe = vid_active_d &&
                        ((vid_x_d != vid_x) || !vid_active);

    // ====== Renderer state ======
    localparam S_CLEAR      = 4'd0;   // post-reset sweep of both buffers
    localparam S_IDLE       = 4'd1;
    localparam S_SLIP_WAIT1 = 4'd2;
    localparam S_SLIP_WAIT2 = 4'd3;
    localparam S_WALK_CHECK = 4'd4;
    localparam S_DESC_RD    = 4'd5;
    localparam S_DESC_WAIT  = 4'd6;
    localparam S_EVAL       = 4'd7;
    localparam S_OBJ_SETUP  = 4'd8;
    localparam S_ROW_WAIT   = 4'd9;
    localparam S_PIX        = 4'd10;
    localparam S_LINE_DONE  = 4'd11;

    reg [3:0]   state = S_CLEAR;
    reg [9:0]   clear_addr = 10'd0;

    reg [8:0]   scanline_d = 9'd0;
    reg [8:0]   target_line = 9'd0;
    reg         render_busy = 1'b0;   // renderer owns the back buffer
    wire        render_buf = target_line[0];

    wire [8:0]  next_scanline = (scanline >= 9'd261) ? 9'd0
                                                     : (scanline + 9'd1);
    wire        line_advance = (scanline != scanline_d);
    wire        target_visible = (next_scanline < 9'd240);

    // Linked-list walk
    reg [255:0] visited = 256'd0;
    reg [7:0]   cur_link = 8'd0;
    reg [10:0]  walk_count = 11'd0;
    reg [1:0]   desc_word = 2'd0;
    reg [1:0]   desc_lat0 = 2'd0;     // RAM latency pipeline tags
    reg         desc_v0 = 1'b0;
    reg [1:0]   desc_lat1 = 2'd0;
    reg         desc_v1 = 1'b0;
    reg [15:0]  desc_w0 = 16'd0;
    reg [15:0]  desc_w1 = 16'd0;
    reg [15:0]  desc_w2 = 16'd0;
    reg [15:0]  desc_w3 = 16'd0;

    // Decoded object (atarimo.cpp render_object field extraction)
    wire [7:0]  obj_link_next = desc_w0[7:0];
    wire [11:0] obj_code      = desc_w1[11:0];
    wire [3:0]  obj_color     = desc_w2[3:0];
    wire [8:0]  obj_x         = desc_w2[15:7];
    wire [8:0]  obj_yfield    = desc_w3[15:7];
    wire [3:0]  obj_w_tiles   = {1'b0, desc_w3[6:4]} + 4'd1;  // 1..8
    wire [3:0]  obj_h_tiles   = {1'b0, desc_w3[2:0]} + 4'd1;  // 1..8
    wire        obj_hflip     = desc_w3[3];

    // Y coverage (verified atarimo.cpp math, 9-bit domain)
    wire [6:0]  obj_h_pix = {obj_h_tiles, 3'b000};            // 8..64
    wire [8:0]  obj_top9  = 9'd0 - (obj_yfield + {2'b00, obj_h_pix});
    wire [8:0]  obj_delta = target_line - obj_top9;           // mod 512
    wire        obj_covers = ({2'b00, obj_delta} < {4'b0000, obj_h_pix});

    // row*width product: 3-bit row (0..7) x 4-bit width (1..8) <= 56
    wire [6:0]  row_w_prod = obj_delta[5:3] * obj_w_tiles;

    // Object render registers
    reg [11:0]  r_code_base = 12'd0;  // code + row*width (wraps mod 4096)
    reg [3:0]   r_width = 4'd1;
    reg [3:0]   r_col = 4'd0;         // column being rendered
    reg [3:0]   r_fetch_col = 4'd0;   // last column issued to fetch
    reg [8:0]   r_xpos = 9'd0;
    reg [3:0]   r_color = 4'd0;
    reg         r_hflip = 1'b0;
    reg [2:0]   r_fine = 3'd0;
    reg [2:0]   r_px = 3'd0;
    reg [31:0]  r_row_cur = 32'd0;    // row being written out
    reg [31:0]  r_row_next = 32'd0;   // prefetched next column row
    reg         r_row_next_ok = 1'b0;
    reg         r_fetch_busy = 1'b0;
    reg [8:0]   r_sx_cur = 9'd0;      // screen x of column being written

    // Tile column draw position: with hflip the first code is drawn at
    // the rightmost column (atarimo.cpp:479-484)
    wire [3:0]  fetch_col_draw = r_hflip ? (r_width - 4'd1 - r_fetch_col)
                                         : r_fetch_col;
    wire [8:0]  fetch_sx = r_xpos + {2'b00, fetch_col_draw, 3'b000};

    // Draw position of the next column after r_col
    wire [3:0]  next_col_draw = r_hflip ? (r_width - 4'd2 - r_col)
                                        : (r_col + 4'd1);
    wire [8:0]  next_col_sx = r_xpos + {2'b00, next_col_draw, 3'b000};

    // Source pixel column within the tile (hflip mirrors pixels)
    wire [2:0]  pix_srccol = r_hflip ? (3'd7 - r_px) : r_px;
    // Packed MSB: pixel 0 in bits [31:28]
    wire [3:0]  pix_pen =
        (pix_srccol == 3'd0) ? r_row_cur[31:28] :
        (pix_srccol == 3'd1) ? r_row_cur[27:24] :
        (pix_srccol == 3'd2) ? r_row_cur[23:20] :
        (pix_srccol == 3'd3) ? r_row_cur[19:16] :
        (pix_srccol == 3'd4) ? r_row_cur[15:12] :
        (pix_srccol == 3'd5) ? r_row_cur[11:8]  :
        (pix_srccol == 3'd6) ? r_row_cur[7:4]   :
                               r_row_cur[3:0];
    wire [8:0]  pix_addr = r_sx_cur + {6'd0, r_px};

    // ====== Line buffer write muxing ======
    // Render writes target the back buffer (render_buf) while the renderer
    // is busy; erase writes target the displayed buffer (disp_buf). The
    // two can only select the same buffer when the renderer is idle with a
    // stale target parity, in which case no render writes occur.
    wire        rend_wr = (state == S_PIX) && (pix_pen != 4'd0);
    wire [8:0]  rend_data = {1'b1, r_color, pix_pen};

    wire        buf0_rend = (render_buf == 1'b0) && rend_wr;
    wire        buf1_rend = (render_buf == 1'b1) && rend_wr;
    wire        buf0_erase = (disp_buf == 1'b0) && erase_strobe;
    wire        buf1_erase = (disp_buf == 1'b1) && erase_strobe;

    wire        buf0_wr   = (state == S_CLEAR) || buf0_rend || buf0_erase;
    wire [8:0]  buf0_addr = (state == S_CLEAR) ? clear_addr[8:0]
                          : buf0_rend          ? pix_addr
                          : vid_x_d;
    wire [8:0]  buf0_data = buf0_rend ? rend_data : 9'd0;

    wire        buf1_wr   = (state == S_CLEAR) || buf1_rend || buf1_erase;
    wire [8:0]  buf1_addr = (state == S_CLEAR) ? clear_addr[8:0]
                          : buf1_rend          ? pix_addr
                          : vid_x_d;
    wire [8:0]  buf1_data = buf1_rend ? rend_data : 9'd0;

    // ====== Debug flags ======
    reg dbg_clear_done = 1'b0;
    reg dbg_scan_started = 1'b0;
    reg dbg_scan_done = 1'b0;
    reg dbg_slip_link_nonzero = 1'b0;
    reg dbg_desc_seen = 1'b0;
    reg dbg_cover_seen = 1'b0;
    reg dbg_row_req_seen = 1'b0;
    reg dbg_row_ack_seen = 1'b0;
    reg dbg_pix_opaque_seen = 1'b0;
    reg dbg_walk_cycle_stop = 1'b0;
    reg dbg_walk_cap_stop = 1'b0;
    reg dbg_abort_seen = 1'b0;
    reg dbg_erase_seen = 1'b0;
    reg dbg_disp_valid_seen = 1'b0;

    assign debug_flags = {
        1'b1,
        dbg_disp_valid_seen,
        dbg_erase_seen,
        dbg_abort_seen,
        dbg_walk_cap_stop,
        dbg_walk_cycle_stop,
        dbg_pix_opaque_seen,
        dbg_row_ack_seen,
        dbg_row_req_seen,
        dbg_cover_seen,
        dbg_desc_seen,
        dbg_slip_link_nonzero,
        dbg_scan_done,
        dbg_scan_started,
        dbg_clear_done,
        (state != S_IDLE)
    };

`ifdef SIMULATION
    integer init_i;
    initial begin
        for (init_i = 0; init_i < 512; init_i = init_i + 1) begin
            linebuf0[init_i] = 9'd0;
            linebuf1[init_i] = 9'd0;
        end
        mo_pix = 9'd0;
    end
`endif

    // ====== Line buffer ports ======
    always @(posedge clk) begin
        if (buf0_wr)
            linebuf0[buf0_addr] <= buf0_data;
        if (buf1_wr)
            linebuf1[buf1_addr] <= buf1_data;
        mo_pix <= disp_buf ? linebuf1[vid_x] : linebuf0[vid_x];
    end

    // ====== Display-side tracking ======
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            vid_x_d <= 9'd0;
            vid_active_d <= 1'b0;
        end else begin
            vid_x_d <= vid_x;
            vid_active_d <= vid_active;
        end
    end

    // ====== Renderer FSM ======
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= S_CLEAR;
            clear_addr <= 10'd0;
            scanline_d <= 9'd0;
            target_line <= 9'd0;
            render_busy <= 1'b0;
            visited <= 256'd0;
            cur_link <= 8'd0;
            walk_count <= 11'd0;
            desc_word <= 2'd0;
            desc_v0 <= 1'b0;
            desc_v1 <= 1'b0;
            desc_lat0 <= 2'd0;
            desc_lat1 <= 2'd0;
            desc_w0 <= 16'd0;
            desc_w1 <= 16'd0;
            desc_w2 <= 16'd0;
            desc_w3 <= 16'd0;
            slip_rd_addr <= 6'd0;
            mob_rd_addr <= 10'd0;
            sprow_req <= 1'b0;
            sprow_code <= 12'd0;
            sprow_row <= 3'd0;
            r_code_base <= 12'd0;
            r_width <= 4'd1;
            r_col <= 4'd0;
            r_fetch_col <= 4'd0;
            r_xpos <= 9'd0;
            r_color <= 4'd0;
            r_hflip <= 1'b0;
            r_fine <= 3'd0;
            r_px <= 3'd0;
            r_row_cur <= 32'd0;
            r_row_next <= 32'd0;
            r_row_next_ok <= 1'b0;
            r_fetch_busy <= 1'b0;
            r_sx_cur <= 9'd0;
            dbg_clear_done <= 1'b0;
            dbg_scan_started <= 1'b0;
            dbg_scan_done <= 1'b0;
            dbg_slip_link_nonzero <= 1'b0;
            dbg_desc_seen <= 1'b0;
            dbg_cover_seen <= 1'b0;
            dbg_row_req_seen <= 1'b0;
            dbg_row_ack_seen <= 1'b0;
            dbg_pix_opaque_seen <= 1'b0;
            dbg_walk_cycle_stop <= 1'b0;
            dbg_walk_cap_stop <= 1'b0;
            dbg_abort_seen <= 1'b0;
            dbg_erase_seen <= 1'b0;
            dbg_disp_valid_seen <= 1'b0;
        end else begin
            scanline_d <= scanline;

            // Descriptor read latency pipeline (2 cycles addr -> data)
            desc_v1 <= desc_v0;
            desc_lat1 <= desc_lat0;
            desc_v0 <= 1'b0;

            if (desc_v1) begin
                case (desc_lat1)
                    2'd0: desc_w0 <= mob_rd_data;
                    2'd1: desc_w1 <= mob_rd_data;
                    2'd2: desc_w2 <= mob_rd_data;
                    default: desc_w3 <= mob_rd_data;
                endcase
                if (mob_rd_data != 16'd0)
                    dbg_desc_seen <= 1'b1;
            end

            if (erase_strobe)
                dbg_erase_seen <= 1'b1;
            if (vid_active && mo_pix[8])
                dbg_disp_valid_seen <= 1'b1;
            if (sprow_req)
                dbg_row_req_seen <= 1'b1;
            if (sprow_req && sprow_ack)
                dbg_row_ack_seen <= 1'b1;

            if (state == S_CLEAR) begin
                // Sweep both buffers once after reset for determinism
                clear_addr <= clear_addr + 10'd1;
                if (clear_addr == 10'd511) begin
                    state <= S_IDLE;
                    dbg_clear_done <= 1'b1;
                end
            end else if (line_advance) begin
                // New displayed line: start rendering the next line.
                // Aborts any in-progress walk deterministically.
                if ((state != S_IDLE) && (state != S_LINE_DONE))
                    dbg_abort_seen <= 1'b1;
                sprow_req <= 1'b0;
                r_fetch_busy <= 1'b0;
                r_row_next_ok <= 1'b0;
                desc_v0 <= 1'b0;
                desc_v1 <= 1'b0;
                if (target_visible) begin
                    target_line <= next_scanline;
                    render_busy <= 1'b1;
                    slip_rd_addr <= next_scanline[8:3];
                    visited <= 256'd0;
                    walk_count <= 11'd0;
                    state <= S_SLIP_WAIT1;
                    dbg_scan_started <= 1'b1;
                end else begin
                    render_busy <= 1'b0;
                    state <= S_IDLE;
                end
            end else begin
                case (state)
                    S_IDLE: ;

                    S_SLIP_WAIT1: state <= S_SLIP_WAIT2;

                    S_SLIP_WAIT2: begin
                        // slip_rd_data is valid one RAM cycle after the
                        // address was presented at line start
                        cur_link <= slip_rd_data[7:0];
                        if (slip_rd_data[7:0] != 8'd0)
                            dbg_slip_link_nonzero <= 1'b1;
                        state <= S_WALK_CHECK;
                    end

                    S_WALK_CHECK: begin
                        if (walk_count == 11'd1024) begin
                            dbg_walk_cap_stop <= 1'b1;
                            state <= S_LINE_DONE;
                        end else if (visited[cur_link]) begin
                            if (walk_count != 11'd0)
                                dbg_walk_cycle_stop <= 1'b1;
                            state <= S_LINE_DONE;
                        end else begin
                            desc_word <= 2'd0;
                            state <= S_DESC_RD;
                        end
                    end

                    S_DESC_RD: begin
                        mob_rd_addr <= {cur_link, desc_word};
                        desc_v0 <= 1'b1;
                        desc_lat0 <= desc_word;
                        if (desc_word == 2'd3)
                            state <= S_DESC_WAIT;
                        else
                            desc_word <= desc_word + 2'd1;
                    end

                    S_DESC_WAIT: begin
                        // Wait for word 3 to land in desc_w3
                        if (desc_v1 && (desc_lat1 == 2'd3))
                            state <= S_EVAL;
                    end

                    S_EVAL: begin
                        visited[cur_link] <= 1'b1;
                        walk_count <= walk_count + 11'd1;
                        cur_link <= obj_link_next;
                        if (obj_covers) begin
                            dbg_cover_seen <= 1'b1;
                            // code + row*width (wraps mod 4096 at use,
                            // matching gfx element code wrapping)
                            r_code_base <= obj_code + {5'd0, row_w_prod};
                            r_width <= obj_w_tiles;
                            r_xpos <= obj_x;
                            r_color <= obj_color;
                            r_hflip <= obj_hflip;
                            r_fine <= obj_delta[2:0];
                            r_col <= 4'd0;
                            r_fetch_col <= 4'd0;
                            r_row_next_ok <= 1'b0;
                            r_fetch_busy <= 1'b0;
                            state <= S_OBJ_SETUP;
                        end else begin
                            state <= S_WALK_CHECK;
                        end
                    end

                    S_OBJ_SETUP: begin
                        // Issue fetch for the first column
                        sprow_code <= r_code_base;
                        sprow_row <= r_fine;
                        sprow_req <= 1'b1;
                        r_fetch_busy <= 1'b1;
                        state <= S_ROW_WAIT;
                    end

                    S_ROW_WAIT: begin
                        if (sprow_req && sprow_ack) begin
                            sprow_req <= 1'b0;
                            r_fetch_busy <= 1'b0;
                            r_row_cur <= sprow_data;
                            r_sx_cur <= fetch_sx;
                            r_px <= 3'd0;
                            state <= S_PIX;
                        end
                    end

                    S_PIX: begin
                        // One pixel per cycle; opaque writes only
                        if (pix_pen != 4'd0)
                            dbg_pix_opaque_seen <= 1'b1;

                        // Prefetch the next column while writing pixels.
                        // Exactly one prefetch per column: busy blocks
                        // re-issue, next_ok blocks issue after completion.
                        if (!r_fetch_busy && !r_row_next_ok &&
                            ((r_col + 4'd1) < r_width)) begin
                            sprow_code <= r_code_base
                                          + {8'd0, r_fetch_col} + 12'd1;
                            sprow_row <= r_fine;
                            sprow_req <= 1'b1;
                            r_fetch_busy <= 1'b1;
                            r_fetch_col <= r_fetch_col + 4'd1;
                        end else if (sprow_req && sprow_ack) begin
                            sprow_req <= 1'b0;
                            r_fetch_busy <= 1'b0;
                            r_row_next <= sprow_data;
                            r_row_next_ok <= 1'b1;
                        end

                        if (r_px == 3'd7) begin
                            if ((r_col + 4'd1) >= r_width) begin
                                // Object done; continue the walk
                                state <= S_WALK_CHECK;
                            end else if (r_row_next_ok) begin
                                r_row_cur <= r_row_next;
                                r_row_next_ok <= 1'b0;
                                r_col <= r_col + 4'd1;
                                r_sx_cur <= next_col_sx;
                                r_px <= 3'd0;
                            end
                            // else: hold at px 7 until the prefetched row
                            // arrives (rewrites of the same pixel with the
                            // same data are harmless)
                        end else begin
                            r_px <= r_px + 3'd1;
                        end
                    end

                    S_LINE_DONE: begin
                        render_busy <= 1'b0;
                        dbg_scan_done <= 1'b1;
                        state <= S_IDLE;
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
