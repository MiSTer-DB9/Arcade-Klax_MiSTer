// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.
//
//============================================================================
// Klax Playfield Tilemap Renderer
//
// Hardware-accurate playfield renderer for the Klax arcade hardware.
// Reads tile codes, colors, and flip attributes from tilemap RAM,
// fetches tile pixel data from ROM, and looks up final colors in
// palette RAM.
//
// Tilemap: 60 columns × 32 rows, TILEMAP_SCAN_COLS (column-major)
//   tile_index = column * 32 + row
//
// Tile entry format (from MAME klax.cpp:125-132):
//   data1 = pf_ram[tile_index]:
//     [12:0]  = tile code (0..8191, indexes into tile ROM)
//     [15]    = flip_x (horizontal flip)
//   data2 = pfx_ram[tile_index]:
//     [11:8]  = color bank (0..15)
//
// Palette addressing:
//   palette_index = 256 + color * 16 + pixel_value
//   Tiles use palette entries 256-511 (gfx index 0, base 256)
//
// Pipeline (7 stages, 6 clock cycles of latency):
//   Stage 0: Compute tile coordinates, present tilemap address
//   Stage 1: Wait for tilemap RAM read
//   Stage 2: Tilemap data available → extract code/flip/color, present ROM addr
//   Stage 3: Wait for tile ROM read
//   Stage 4: ROM data available → extract pixel, present palette address
//   Stage 5: Wait for palette RAM read
//   Stage 6: Palette data available → IRGB_1555 → RGB888, output
//
// MAME references:
//   klax.cpp:125-132   get_playfield_tile_info
//   klax.cpp:405       TILEMAP config: TILEMAP_SCAN_COLS, 60, 32
//   klax.cpp:403       PALETTE: IRGB_1555, 512 entries, 8-bit membits
//   klax.cpp:366       GFXDECODE: tiles at palette base 256
//============================================================================

module klax_playfield #(
    parameter ROM_AW = 18,
    parameter H_VIS  = 336,    // visible H pixels (native timing: always 336 — see src_x note)
    parameter V_VIS  = 240,    // visible V lines
    // Tile-graphics prefetch. 1 = normal. 0 exists ONLY so a regression
    // testbench can disable the prefetch and prove it is what removes the
    // demand misses; never ship a build with it off.
    parameter PREFETCH_EN = 1
) (
    input               clk,
    input               reset,
    input       [10:0]  hc,        // horizontal counter
    input       [9:0]   vc,        // vertical counter
    input               hblank,
    input               vblank,

    // Scroll offsets. Klax has no playfield scroll; every instance ties
    // these to 0 (see the note at pf_x below).
    input       [8:0]   scroll_x,
    input       [7:0]   scroll_y,

    // Tilemap RAM read (shared address for pf + pfx, 1-cycle latency)
    output reg  [10:0]  tilemap_addr,
    input       [15:0]  pf_data,       // playfield data (code, flip_x)
    input       [15:0]  pfx_data,      // extended data (color)

    // Tile ROM read (1-cycle latency)
    output reg [ROM_AW-1:0] rom_addr,
    input       [7:0]   rom_data,

    // Tile ROM prefetch (warming only - no data comes back).
    //
    // This path exists because `rom_data` above has NO handshake and this
    // pipeline cannot stall: if the DDRAM tile cache misses, the read returns
    // 0 and the renderer paints pen 0 of the colour bank. Real silicon cannot
    // miss - the 9W slot machine schedules two playfield fetch slots per tile
    // AHEAD of display. Asking for tile column N+PREFETCH_TILES here restores
    // that "already resident when the raster arrives" property.
    // Bus priority for the graphics ROM is demand > prefetch > sprite.
    output reg  [10:0]  pre_tilemap_addr,
    input       [15:0]  pf_pre_data,
    output reg [ROM_AW-1:0] pre_rom_addr,
    output reg          pre_rom_en,

    // Palette RAM read (1-cycle latency)
    output reg  [8:0]   pal_addr,
    input       [15:0]  pal_data,      // IRGB_1555

    // Video output
    output reg  [7:0]   VGA_R,
    output reg  [7:0]   VGA_G,
    output reg  [7:0]   VGA_B,
    output reg          pf_priority_block,

    output      [15:0]  debug_flags
);

    // ====== Constants ======
    localparam SRC_H_VIS  = 336;
    localparam SRC_V_VIS  = 240;
    localparam PF_COLS    = 60;
    localparam PF_ROWS    = 32;

    // ====== Coordinate mapping ======

    // View visibility
    wire in_view = (hc < H_VIS) && (vc < V_VIS) && ~hblank && ~vblank;

    // hc IS the native Klax pixel X (every instance passes H_VIS ==
    // SRC_H_VIS == 336). Do not generalise this to src_x = hc * SRC_H_VIS /
    // H_VIS: Quartus cannot fold the (hc*336)/336 identity back to hc
    // (truncating division isn't provably invertible), so it builds a DSP
    // multiply + a combinational LPM divider — a ~31 ns path that breaks
    // setup on the 57.27 MHz clk_sys. hc[8:0] is bit-identical for every hc
    // in a 456-wide line.
    wire [8:0]  src_x     = hc[8:0];
    wire [8:0]  src_y     = vc[8:0];

    // Apply scroll offsets.
    //
    // Klax hardware has NO playfield scroll registers: the tilemap is drawn
    // at a fixed origin (MAME agrees - klax.cpp instantiates
    // TILEMAP_SCAN_COLS 60x32 and never calls set_scrollx/set_scrolly), and
    // all three klax_playfield instantiations tie scroll_x/scroll_y to 0.
    // The ports are unused generality, not a missing feature - there is
    // nothing here to hook up. Because scroll_x is always 0, pf_x never
    // exceeds the 480-pixel (PF_COLS*8) map width, so no modular wrap is
    // needed; a nonzero scroll_x would require one.
    wire [8:0] pf_x = src_x + scroll_x;
    wire [7:0] pf_y = src_y[7:0] + scroll_y;  // wraps naturally at 256 (PF_ROWS * 8)

    // Tile coordinates
    wire [5:0] tile_col = pf_x[8:3];   // 0..59
    wire [4:0] tile_row = pf_y[7:3];   // 0..31
    wire [2:0] tx       = pf_x[2:0];   // pixel within tile, horizontal
    wire [2:0] ty       = pf_y[2:0];   // pixel within tile, vertical

    // Column-major tile index: col * 32 + row = {col, row}
    wire [10:0] tile_index = {tile_col, tile_row};

    // ====== Prefetch address generation ======
    // Two tiles of lookahead = 16 pixels = 128 clks of slack against a fill,
    // while sampling the tilemap only ~16 px early (the real 9W fetch slots
    // run 1-2 tiles ahead too, so this stays sub-scanline and faithful).
    //
    // During active display we chase tile_col + PREFETCH_TILES. That leaves
    // the first PREFETCH_TILES columns of each line uncovered, so during
    // hblank we walk the NEXT line's leading columns instead - hblank is 120
    // px, far more than enough. Only the first row of each row-pair can miss
    // (a 64-bit cache word spans two tile rows), but prefetching on every
    // line is harmless and keeps this logic simple.
    // Sized deliberately: an unsized localparam makes `tile_col +
    // PREFETCH_TILES` a 32-bit add that Quartus then truncates to 6 bits
    // (Warning 10230). Harmless in practice - tile_col is at most 41 during
    // active display - but sizing it keeps the arithmetic 6-bit throughout
    // and the wrap explicit, guarded by pre_col_ok below.
    localparam [5:0] PREFETCH_TILES = 6'd2;

    wire        pre_in_hblank = (hc >= H_VIS);
    wire [8:0]  pre_hb_off    = hc[8:0] - H_VIS[8:0];
    wire [7:0]  pre_y         = pre_in_hblank ? (src_y[7:0] + 8'd1) : pf_y;
    wire [5:0]  pre_col       = pre_in_hblank ? pre_hb_off[8:3]
                                              : (tile_col + PREFETCH_TILES);
    wire [4:0]  pre_row       = pre_y[7:3];
    wire [2:0]  pre_ty        = pre_y[2:0];
    wire [10:0] pre_index     = {pre_col, pre_row};
    wire        pre_col_ok    = (pre_col < PF_COLS);

    reg [2:0] pre_ty_d;
    reg       pre_valid_d;

    // ====== Stage 2 combinatorial decode (tilemap data) ======
    wire [12:0] pf_code  = pf_data[12:0];
    wire        pf_flipx = pf_data[15];
    wire [3:0]  pf_color = pfx_data[11:8];

    // Flip-X: mirror pixel x within tile (7-tx = ~tx for 3-bit values)
    wire [2:0] tx_flip_comb = pf_flipx ? ~tx_p1 : tx_p1;

    // ====== Stage 4 combinatorial decode (ROM data → pixel) ======
    wire [3:0] tile_pixel = tx_flip0_p3 ? rom_data[3:0] : rom_data[7:4];

    // ====== Stage 6 combinatorial decode (palette data → RGB) ======
    wire        pal_i  = pal_data[15];
    wire [4:0]  pal_r5 = pal_data[14:10];
    wire [4:0]  pal_g5 = pal_data[9:5];
    wire [4:0]  pal_b5 = pal_data[4:0];

    // Silicon DAC truth (schematic Sheet 9): each channel is a 6-bit R/2R
    // ladder whose LSB is the shared intensity bit — channel = {color5, I}.
    // MAME is identical: IRGB_1555 -> pal6bit((c << 1) | i). 6-to-8-bit
    // expansion replicates the top bits: {v6, v6[5:4]} (= pal6bit).
    wire [5:0] r6 = {pal_r5, pal_i};
    wire [5:0] g6 = {pal_g5, pal_i};
    wire [5:0] b6 = {pal_b5, pal_i};

    wire [7:0] pal_r8 = {r6, r6[5:4]};
    wire [7:0] pal_g8 = {g6, g6[5:4]};
    wire [7:0] pal_b8 = {b6, b6[5:4]};

    // ====== Pipeline registers ======
    reg [6:0]  en;               // enable pipeline (7 stages)
    reg [2:0]  tx_p0, ty_p0;    // stage 0 → stage 1
    reg [2:0]  tx_p1, ty_p1;    // stage 1 → stage 2
    reg [2:0]  tx_flip_p2;      // stage 2 → stage 3
    reg        tx_flip0_p3;     // stage 3 → stage 4 (nibble select bit)
    reg [3:0]  color_p2;        // stage 2 → stage 3
    reg [3:0]  color_p3;        // stage 3 → stage 4
    reg [3:0]  color_p4;        // stage 4 → stage 5
    reg [3:0]  color_p5;        // stage 5 → stage 6

    reg dbg_in_view_seen = 1'b0;
    reg dbg_tilemap_addr_seen = 1'b0;
    reg dbg_pf_data_nonzero_seen = 1'b0;
    reg dbg_pfx_data_nonzero_seen = 1'b0;
    reg dbg_pf_code_nonzero_seen = 1'b0;
    reg dbg_color_nonzero_seen = 1'b0;
    reg dbg_rom_addr_nonzero_seen = 1'b0;
    reg dbg_rom_data_nonzero_seen = 1'b0;
    reg dbg_tile_pixel_nonzero_seen = 1'b0;
    reg dbg_pal_addr_tile_seen = 1'b0;
    reg dbg_pal_data_nonzero_seen = 1'b0;
    reg dbg_rgb_nonzero_seen = 1'b0;

    assign debug_flags = {
        1'b1,
        dbg_rgb_nonzero_seen,
        dbg_pal_data_nonzero_seen,
        dbg_pal_addr_tile_seen,
        dbg_tile_pixel_nonzero_seen,
        dbg_rom_data_nonzero_seen,
        dbg_rom_addr_nonzero_seen,
        dbg_color_nonzero_seen,
        dbg_pf_code_nonzero_seen,
        dbg_pfx_data_nonzero_seen,
        dbg_pf_data_nonzero_seen,
        dbg_tilemap_addr_seen,
        dbg_in_view_seen,
        |en,
        en[5],
        en[6]
    };

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            en           <= 7'b0;
            tilemap_addr <= 11'b0;
            rom_addr     <= {ROM_AW{1'b0}};
            pre_tilemap_addr <= 11'b0;
            pre_rom_addr <= {ROM_AW{1'b0}};
            pre_rom_en   <= 1'b0;
            pre_ty_d     <= 3'd0;
            pre_valid_d  <= 1'b0;
            pal_addr     <= 9'b0;
            tx_p0        <= 3'b0;
            ty_p0        <= 3'b0;
            tx_p1        <= 3'b0;
            ty_p1        <= 3'b0;
            tx_flip_p2   <= 3'b0;
            tx_flip0_p3  <= 1'b0;
            color_p2     <= 4'b0;
            color_p3     <= 4'b0;
            color_p4     <= 4'b0;
            color_p5     <= 4'b0;
            VGA_R        <= 8'b0;
            VGA_G        <= 8'b0;
            VGA_B        <= 8'b0;
            pf_priority_block <= 1'b0;
            dbg_in_view_seen <= 1'b0;
            dbg_tilemap_addr_seen <= 1'b0;
            dbg_pf_data_nonzero_seen <= 1'b0;
            dbg_pfx_data_nonzero_seen <= 1'b0;
            dbg_pf_code_nonzero_seen <= 1'b0;
            dbg_color_nonzero_seen <= 1'b0;
            dbg_rom_addr_nonzero_seen <= 1'b0;
            dbg_rom_data_nonzero_seen <= 1'b0;
            dbg_tile_pixel_nonzero_seen <= 1'b0;
            dbg_pal_addr_tile_seen <= 1'b0;
            dbg_pal_data_nonzero_seen <= 1'b0;
            dbg_rgb_nonzero_seen <= 1'b0;
        end else begin
            // === Prefetch: present tilemap address, then the ROM word ===
            // Runs free alongside the display pipeline. tx is forced to 0
            // because a 64-bit cache line covers the whole tile row pair, so
            // any byte of it warms the line the raster will need.
            pre_tilemap_addr <= pre_index;
            pre_ty_d         <= pre_ty;
            pre_valid_d      <= pre_col_ok;
            pre_rom_addr     <= {pf_pre_data[12:0], pre_ty_d, 2'b00};
            pre_rom_en       <= pre_valid_d && (PREFETCH_EN != 0);

            // === Stage 0: Present tilemap address ===
            en[0] <= in_view;
            tilemap_addr <= tile_index;
            tx_p0 <= tx;
            ty_p0 <= ty;
            if (in_view)
                dbg_in_view_seen <= 1'b1;
            if (in_view && (tile_index != 11'd0))
                dbg_tilemap_addr_seen <= 1'b1;

            // === Stage 1: Wait for tilemap RAM read ===
            en[1] <= en[0];
            tx_p1 <= tx_p0;
            ty_p1 <= ty_p0;

            // === Stage 2: Tilemap data available → present ROM address ===
            en[2] <= en[1];
            if (en[1]) begin
                rom_addr   <= {pf_code, ty_p1, tx_flip_comb[2:1]};
                tx_flip_p2 <= tx_flip_comb;
                color_p2   <= pf_color;
                if (pf_data != 16'd0)
                    dbg_pf_data_nonzero_seen <= 1'b1;
                if (pfx_data != 16'd0)
                    dbg_pfx_data_nonzero_seen <= 1'b1;
                if (pf_code != 13'd0)
                    dbg_pf_code_nonzero_seen <= 1'b1;
                if (pf_color != 4'd0)
                    dbg_color_nonzero_seen <= 1'b1;
                if ({pf_code, ty_p1, tx_flip_comb[2:1]} != {ROM_AW{1'b0}})
                    dbg_rom_addr_nonzero_seen <= 1'b1;
            end

            // === Stage 3: Wait for tile ROM read ===
            en[3] <= en[2];
            tx_flip0_p3 <= tx_flip_p2[0];
            color_p3    <= color_p2;

            // === Stage 4: ROM data available → present palette address ===
            en[4] <= en[3];
            color_p4 <= color_p3;
            if (en[3]) begin
                // palette_index = 256 + color * 16 + pixel
                // = {1'b1, color[3:0], pixel[3:0]}
                pal_addr <= {1'b1, color_p3, tile_pixel};
                if (rom_data != 8'd0)
                    dbg_rom_data_nonzero_seen <= 1'b1;
                if (tile_pixel != 4'd0)
                    dbg_tile_pixel_nonzero_seen <= 1'b1;
            end

            // === Stage 5: Wait for palette RAM read ===
            en[5] <= en[4];
            color_p5 <= color_p4;
            if (en[4] && pal_addr[8])
                dbg_pal_addr_tile_seen <= 1'b1;

            // === Stage 6: Palette data available → RGB output ===
            en[6] <= en[5];
            pf_priority_block <= en[5] && (color_p5 == 4'hF);
            if (en[5]) begin
                VGA_R <= pal_r8;
                VGA_G <= pal_g8;
                VGA_B <= pal_b8;
                if (pal_data != 16'd0)
                    dbg_pal_data_nonzero_seen <= 1'b1;
                if ((pal_r8 != 8'd0) || (pal_g8 != 8'd0) || (pal_b8 != 8'd0))
                    dbg_rgb_nonzero_seen <= 1'b1;
            end else begin
                VGA_R <= 8'h00;
                VGA_G <= 8'h00;
                VGA_B <= 8'h00;
            end
        end
    end

endmodule
