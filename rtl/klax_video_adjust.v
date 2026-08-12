//============================================================================
// Klax Video Output Adjust
//
// OSD-driven CRT alignment for the analog VGA path: H-Shift, V-Shift, H-Size
// (horizontal stretch) and H-Position. Ported from rmonic79's
// Arcade-Raiden_MiSTer, which drives the same controls through the same
// analog_hsize linebuffer.
//
// Everything here acts on the OUTPUT side only. The core's own counters, the
// native 7.15909 MHz dot clock, the 456x262 raster and the 59.92 Hz refresh
// are untouched, and every control is a
// true bypass at its zero setting: with a default OSD the outputs of this
// module are bit-identical to the core's own video, so the shipping default
// stays the hardware-validated one.
//
// Klax raster, referenced to the HSync rise that analog_hsize keys off
// (hc = 362): 94 pixels of back porch, 336 pixels of
// active video (ending at index 430 of 456), 26 pixels of front porch.
//============================================================================

module klax_video_adjust #(
    parameter H_TOTAL = 456,     // klax_top_mister.v H_TOTAL
    parameter V_TOTAL = 262      // klax_top_mister.v V_TOTAL
) (
    input             clk_sys,
    input             ce_pix,        // core pixel enable (/8 of clk_sys)

    // Native video from the core
    input       [7:0] core_r,
    input       [7:0] core_g,
    input       [7:0] core_b,
    input             HSync,
    input             VSync,
    input             HBlank,
    input             VBlank,

    // OSD selections, raw menu indices straight off status[]
    input       [4:0] hsize_sel,     // 5-bit two's complement, 0 = bypass
    input       [6:0] hpos_sel,      // 0..48 = +0..+48, 49..96 = -48..-1
    input       [5:0] hshift_sel,    // 0..48 = +0..+48, 49..63 = -15..-1
    input       [5:0] vshift_sel,    // 6-bit two's complement, +-32 lines

    // Adjusted video out
    output      [7:0] vga_r,
    output      [7:0] vga_g,
    output      [7:0] vga_b,
    output            vga_hs,
    output            vga_vs,
    output            ce_pixel,      // read-rate CE, = ce_pix when not stretching
    output            de_in          // VGA_DE_IN for video_freak
);

    // The tap arithmetic is 9-bit throughout: the negative-shift taps rely on
    // the two's-complement wrap of (TOTAL + negative) inside 9 bits. Quartus
    // warns 10230 on both of these (32-bit parameter narrowed to 9); that is
    // the intent - both totals fit in 9 bits.
    localparam [8:0] H_TOTAL_RD = H_TOTAL;
    localparam [8:0] V_TOTAL_RD = V_TOTAL;

    // One pulse per line, at the line boundary. klax_top_mister keeps hc
    // internal, so key off the falling edge of HBlank (hc = 0) rather than a
    // counter compare; HBlank only changes on ce_pix boundaries, so this is a
    // single ce_pix-wide pulse one pixel into the line.
    reg hblank_line_d;
    always @(posedge clk_sys) if (ce_pix) hblank_line_d <= HBlank;
    wire line_tick = ce_pix && ~HBlank && hblank_line_d;

    //------------------------------------------------------------------------
    // H-Size: one bidirectional control (5-bit two's complement)
    //   0        = native (analog_hsize bypassed entirely)
    //   +1..+15  = enlarge (wider image)    -> slower read
    //   -1..-16  = shrink  (narrower image) -> faster read
    // Step = 1/64 of a pixel period = 1.5625% (see the rd_acc accumulator).
    //
    // Shrink has the whole line to play with. Enlarge does not: the read
    // pointer is reset by HSync and the active window already ends at index
    // 430 of the 456-pixel line, so a stretched line can run out of room and
    // clip its right edge. Measured in simulation:
    //   H-Position 0    -> lossless to +3, clipping from +4
    //   H-Position -48  -> lossless to +12, clipping from +13
    // The full +15 is kept so the step size stays uniform, but past +12 the
    // right edge is lost whatever H-Position is set to.
    //------------------------------------------------------------------------
    reg signed [4:0] hsize_s;
    always @(posedge clk_sys) if (ce_pix) hsize_s <= $signed(hsize_sel);
    wire hsize_active = (hsize_s != 5'sd0);

    //------------------------------------------------------------------------
    // H-Position: moves the content, not the sync, so it cannot desync a CRT.
    // hpos_sel is a menu index over the 97 listed entries: 0..48 = +0..+48,
    // 49..96 = -48..-1. (Upstream decodes the negative half as index-128,
    // which is off by 31 against its own 97-entry list; -97 is the encoding
    // the list actually describes.) Indices above 96 are unreachable from the
    // OSD; they decode to a small positive offset rather than misbehaving.
    //------------------------------------------------------------------------
    reg [6:0] hpos_d;
    always @(posedge clk_sys) if (ce_pix) hpos_d <= hpos_sel;
    wire signed [8:0] hoffset = (hpos_d <= 7'd48)
        ? $signed({2'b0, hpos_d})
        : $signed({2'b0, hpos_d}) - 9'sd97;

    //------------------------------------------------------------------------
    // H-Shift: delays HSync against unchanged content, so the image walks
    // left. 0..48 = +0..+48, 49..63 = -15..-1 (a "negative" delay is just a
    // delay of nearly a whole line, which wraps round to an advance).
    //
    // At zero the tapped register is bypassed rather than passed through
    // hsync_shreg's one-ce_pix pipeline stage, so HSync stays bit-identical to
    // the core's. Same for V-Shift below.
    //
    // NOTE: H-Shift only bites while H-Size is 0. With the stretch active the
    // linebuffer read window is referenced to this same shifted HSync, so sync
    // and content move together and nothing changes on screen - H-Position is
    // the control to use there.
    //------------------------------------------------------------------------
    reg [5:0] hshift_d;
    always @(posedge clk_sys) if (ce_pix) hshift_d <= hshift_sel;
    wire hshift_active = (hshift_d != 6'd0);
    // tap is 1-based; forced to 1 when inactive purely to keep the bit select
    // in range (the result is discarded by the hshift_active mux).
    wire [8:0] hshift_tap = !hshift_active  ? 9'd1
        : (hshift_d <= 6'd48)               ? {3'd0, hshift_d}
        : (H_TOTAL_RD - (9'd64 - {3'd0, hshift_d}));
    reg [H_TOTAL-1:0] hsync_shreg;
    always @(posedge clk_sys) if (ce_pix) hsync_shreg <= {hsync_shreg[H_TOTAL-2:0], HSync};
    // hs_tapped is itself a ce_pix stage, so it contributes the first pixel of
    // delay and the shift register supplies the rest: tap N reads stage N-2.
    // (Upstream reads N-1 here, which makes its whole H-Shift scale one pixel
    // longer than the menu says; harmless there because it never bypasses, but
    // it would show up against the true bypass at 0 below.)
    reg hs_tapped;
    always @(posedge clk_sys) if (ce_pix)
        hs_tapped <= (hshift_tap <= 9'd1) ? HSync : hsync_shreg[hshift_tap - 9'd2];
    wire hs_shifted = hshift_active ? hs_tapped : HSync;

    //------------------------------------------------------------------------
    // V-Shift: the same idea a line at a time, signed +-32 lines. Positive
    // delays VSync, which moves the picture UP (content arrives earlier
    // relative to sync), mirroring H-Shift moving it left.
    //------------------------------------------------------------------------
    reg signed [5:0] vshift_d;
    always @(posedge clk_sys) if (line_tick) vshift_d <= $signed(vshift_sel);
    wire vshift_active = (vshift_d != 6'sd0);
    wire [8:0] vshift_tap = !vshift_active ? 9'd1
        : vshift_d[5] ? (V_TOTAL_RD + {{3{vshift_d[5]}}, vshift_d})
                      : {3'd0, vshift_d};
    reg [V_TOTAL-1:0] vsync_line_shreg;
    always @(posedge clk_sys) if (line_tick) vsync_line_shreg <= {vsync_line_shreg[V_TOTAL-2:0], VSync};
    // Reads stage N-1, not N-2 as the H path does, and that asymmetry is
    // correct: line_tick fires one pixel INTO the line, by which point VSync
    // already holds this line's value, whereas HSync updates on the very
    // ce_pix edge that samples it and so is one step older. Both paths end up
    // delaying by exactly N. Both are measured in simulation.
    reg vs_tapped;
    always @(posedge clk_sys) if (line_tick) vs_tapped <= vsync_line_shreg[vshift_tap - 9'd1];
    wire vs_shifted = vshift_active ? vs_tapped : VSync;

    //------------------------------------------------------------------------
    // Read rate, in EIGHTHS of a clk_sys cycle. A Klax pixel is 8 clk_sys
    // cycles (57.27272 / 7.15909), so a native period is 64 eighths and the
    // accumulator adds 8 per clock; period = 64 + hsize gives the 1.5625%
    // step. Resetting on the HSync rise makes the tick pattern identical on
    // every line of every frame, so the sub-pixel irregularity is static and
    // invisible - no shimmer on moving content.
    //------------------------------------------------------------------------
    reg hs_shifted_d;
    always @(posedge clk_sys) hs_shifted_d <= hs_shifted;
    wire shifted_hs_rise = hs_shifted & ~hs_shifted_d;
    wire [7:0] rd_period = 8'd64 + {{3{hsize_s[4]}}, hsize_s};  // eighths: 48..79
    reg  [7:0] rd_acc;
    wire rd_tick = ({1'b0, rd_acc} + 9'd8) >= {1'b0, rd_period};
    always @(posedge clk_sys) begin
        if (shifted_hs_rise)  rd_acc <= 8'd0;
        // rd_tick guarantees rd_acc + 8 >= rd_period, and the previous value
        // was below rd_period, so the remainder is always 0..7 - never a
        // borrow, always 8 bits.
        else if (rd_tick)     rd_acc <= rd_acc + 8'd8 - rd_period;
        else                  rd_acc <= rd_acc + 8'd8;
    end
    wire rd_ce = hsize_active ? rd_tick : ce_pix;

    wire [7:0] str_r, str_g, str_b;
    wire       str_hs, str_vs, str_hb, str_vb;
    analog_hsize u_analog_hsize (
        .clk      (clk_sys),
        .pxl_cen  (ce_pix),
        .pxl2_cen (rd_ce),
        .hsize    (hsize_s),
        .hoffset  (hoffset),
        .r_in     (core_r), .g_in (core_g), .b_in (core_b),
        .hs_in    (hs_shifted),
        .vs_in    (vs_shifted),
        .hb_in    (HBlank | VBlank),
        .vb_in    (VBlank),
        .r_out    (str_r), .g_out (str_g), .b_out (str_b),
        .hs_out   (str_hs), .vs_out (str_vs),
        .hb_out   (str_hb), .vb_out (str_vb)
    );

    // DE window for the OSD while stretching: opens on the native active edge
    // (delayed one line, matching the linebuffer's one-line read lag) and
    // closes at the full stretched width.
    reg vblank_1l;
    always @(posedge clk_sys) if (line_tick) vblank_1l <= VBlank;
    wire native_active = ~(HBlank | vblank_1l);
    reg  native_active_d;
    always @(posedge clk_sys) if (ce_pix) native_active_d <= native_active;
    wire native_rise = native_active & ~native_active_d;
    wire str_active = ~str_hb;
    reg  str_active_d;
    always @(posedge clk_sys) if (rd_ce) str_active_d <= str_active;
    wire str_fall = str_active_d & ~str_active;
    reg de_osd;
    always @(posedge clk_sys) begin
        if      (native_rise) de_osd <= 1'b1;
        else if (str_fall)    de_osd <= 1'b0;
    end

    // H-Size active -> take the stretched stream (it already carries the
    // shifts); bypass -> the raw core pixels with the shifted syncs.
    assign vga_r    = hsize_active ? str_r  : core_r;
    assign vga_g    = hsize_active ? str_g  : core_g;
    assign vga_b    = hsize_active ? str_b  : core_b;
    assign vga_hs   = hsize_active ? str_hs : hs_shifted;
    assign vga_vs   = hsize_active ? str_vs : vs_shifted;
    assign ce_pixel = rd_ce;
    assign de_in    = hsize_active ? de_osd : ~(HBlank | VBlank);

endmodule
