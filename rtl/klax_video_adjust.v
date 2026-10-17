// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v3 or later.
//
//============================================================================
// Klax CRT Adjust
//
// The OSD decode, the read-rate generator and the blanking/DE glue around
// rmonic79's `crt_adjust` engine (rtl/crt_adjust.sv, vendored from
// MiSTer-CRT-Adjust c682de9). Written from that project's
// examples/core_side_snippet.v (GPL-3.0, Umberto Parisi), re-derived for
// Klax's 8-clock pixel and for driving the VGA_* pins and video_freak's
// VGA_DE_IN directly (this core has no arcade_video stage).
//
// The OSD carries an On/Off plus three amounts: CRT H-Size, CRT H-Position
// and CRT V-Shift. All three are plain two's complement.
//
// Everything here acts on the OUTPUT side only. The core's own counters, the
// native 7.15909 MHz dot clock, the 456x262 raster and the 59.92 Hz refresh
// are untouched. Off is a true bypass: the core's own pixels, syncs, pixel
// enable and DE go straight to the outputs, bit-identical to the core, so
// the shipping default stays the hardware-validated video path. (crt_adjust's
// own bypass would delay everything by two input registers; this wrapper
// does not use it.)
//
// ---------------- HPOS_MODE for Klax: CONTENTSHIFT --------------------------
// Upstream's rule is "CONTENTSHIFT for wide/centred games, SYNCSHIFT for
// narrow side-anchored ones, because CONTENTSHIFT runs the picture out of the
// buffer window on the short-margin side and shows a black block there".
// Klax's raster, in pixels:
//
//     line total        456 px
//     active            hc 0 .. 335           (336 px)
//     HSync high        hc 362 .. 389
//
// The line buffer's write pointer resets on the HSync rise, so in BUFFER
// coordinates the active region is 94 .. 429: a 94-pixel lead-in and a
// 26-pixel run-out. The short margin is the RIGHT one. The menu offers
// H-Position +-64, but the output pipeline eats a couple of those 26 pixels,
// so measured in simulation, with "lossless" meaning all 336 source pixels
// land outside the HSync pulse:
//     H-Size 0                 lossless to H-Position +24 (+26 loses a pixel)
//     H-Position 0             lossless to H-Size +3, clips from +4
//     H-Position -48           lossless to H-Size +12, clips from +13
//     H-Position -64           the full H-Size +15 fits
// The H-Size limits are the same as the previous engine's. Shrinking never
// clips.
//
// CONTENTSHIFT is still the more USEFUL mode here. Enlarging (H-Size > 0)
// slows the read, so the 94-pixel lead-in is stretched too and the right of
// the picture runs off the end of the line; a NEGATIVE H-Position pulls the
// content earlier and buys that room back. SYNCSHIFT cannot do that -- it
// moves the sync and the content together, so it re-centres the picture on
// the tube but never changes what fits in a line.
//
// If a real CRT ever shows a black block at the right edge, set HPOS_MODE to
// 0 at the instantiation and nothing else changes.
//
// ---------------- the read rate, and why 64/CLK_PER_PIX --------------------
// Upstream steps the read period in quarter clocks because its reference
// core's pixel is 16 clk_sys cycles: 64 quarters = one pixel, so one H-Size
// click is 1/64 = 1.5625%. A Klax pixel is 8 clk_sys cycles (57.27272 /
// 7.15909), and a fixed quarter-clock step would make one click 3.125%. The
// accumulator below counts 1/CLK_PER_PIX-th of a clock instead, which keeps
// the 1.5625% step at any pixel period:
//
//     period = 64 + hsize      units, where 64 units == CLK_PER_PIX clocks
//     acc   += 64/CLK_PER_PIX  per clock
//
// so hsize spans 48..79 units = 6.00..9.88 clocks per pixel at CLK_PER_PIX
// = 8. ACC_STEP must stay an exact integer -- 64/CLK_PER_PIX with
// CLK_PER_PIX a power of two -- or the OSD step silently drifts away from
// the menu text. The accumulator is reset on the rise of crt_adjust's
// `hs_ref_out`, never on the raw HSync, so the write side, the engine's read
// counter and this read rate all restart on the same edge; the tick pattern
// is then identical on every line, so the sub-pixel irregularity is static
// and invisible (no shimmer on moving content).
//
// ---------------- the vertical blank, and the DE handed on ------------------
// On the Klax raster HSync follows the active pixels and the line counter
// advances at the wrap, so the bank that closes at line L's HSync rise
// already holds line L and content L is emitted during native line L+1.
// crt_adjust is built with VB_LATE = 0 so its read-side gate (`vb_active`)
// samples VBlank once, on that same rise; `vb_read` below is a one-flop
// replica of it, sampled on the same `hs_ref` edge. The two must match: the
// upstream two-flop chain (VB_LATE = 1) blanks source line 0 and shows a
// stale line under the picture.
//
// DE follows the picture: de_in = ~(str_hb | vb_read), where str_hb is the
// engine's own registered ~pass_q, so DE opens and closes exactly on the
// emitted content at any H-Size / H-Position. Upstream's core-side snippet
// instead builds a `de_osd` window anchored to the NATIVE active start, so
// the MiSTer OSD (which centres itself on the rise of VGA_DE) does not slide
// with H-Position. It does not survive this raster: with a 94-pixel lead-in
// the stretched content starts BEFORE the native active edge whenever
// H-Position is negative or H-Size shrinks, so the anchored window cuts the
// first source pixels of every line, and since it opens on the native rise
// of the line before the first visible one it adds an all-black DE line
// above the picture. Both were measured on the previous Klax glue. The cost
// of following the picture is the one upstream set out to remove: the OSD
// slides with H-Position. Anyone re-attempting the anchored window has to
// make its rising edge the EARLIER of the native and the stretched active,
// and suppress the native rise on a line whose stretched active has already
// ended.
//============================================================================

module klax_video_adjust #(
    parameter H_TOTAL     = 456,   // klax_top_mister.v H_TOTAL
    parameter V_TOTAL     = 262,   // klax_top_mister.v V_TOTAL
    parameter CLK_PER_PIX = 8,     // clk_sys cycles per ce_pix
    parameter HPOS_MODE   = 1,     // 1 = CONTENTSHIFT, 0 = SYNCSHIFT (see above)
    parameter AW          = 10,    // line-buffer address bits: 512 samples per bank
    // Test control only -- always 0 in the build. 1 restores upstream's
    // two-line vblank gate AND a matching two-flop vb_read, i.e. the defect
    // described above; the line-identity bench requires that build to FAIL so
    // its check is known to bite.
    parameter VB_LATE     = 0
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

    // OSD
    input             osd_on,        // CRT Adjust Off/On; Off is a bit-exact bypass
    input       [4:0] osd_hsize,     // CRT H-Size        +15 / -16   two's complement
    input       [6:0] osd_hpos,      // CRT H-Position    +63 / -64   two's complement
    input       [5:0] osd_vshift,    // CRT V-Shift       +31 / -32   two's complement

    // Adjusted video out
    output      [7:0] vga_r,
    output      [7:0] vga_g,
    output      [7:0] vga_b,
    output            vga_hs,
    output            vga_vs,
    output            ce_pixel,      // read-rate CE while On, = ce_pix when Off
    output            de_in          // VGA_DE_IN for video_freak
);

    localparam [7:0] ACC_STEP = 64 / CLK_PER_PIX;

    //------------------------------------------------------------------------
    // OSD decode. Everything is sampled at ce_pix so a mid-frame menu change
    // lands on a pixel boundary rather than anywhere inside one.
    //------------------------------------------------------------------------
    reg              active;
    reg signed [4:0] hsize_s;
    reg signed [8:0] hpos_s;
    reg signed [5:0] vshift_s;

    always @(posedge clk_sys) if (ce_pix) begin
        active   <= osd_on;
        hsize_s  <= $signed(osd_hsize);
        hpos_s   <= $signed({{2{osd_hpos[6]}}, osd_hpos});   // sign-extend 7 -> 9
        vshift_s <= $signed(osd_vshift);
    end

    //------------------------------------------------------------------------
    // Read-rate generator, reset on the rise of the engine's hs_ref_out.
    //------------------------------------------------------------------------
    wire hs_ref;
    reg  hs_ref_d;
    always @(posedge clk_sys) hs_ref_d <= hs_ref;
    wire hs_ref_rise = hs_ref & ~hs_ref_d;

    wire [7:0] rd_period = 8'd64 + {{3{hsize_s[4]}}, hsize_s};   // 48..79
    reg  [7:0] rd_acc;
    // rd_acc < rd_period <= 79 always, so rd_acc + ACC_STEP fits 8 bits.
    wire       rd_tick = (rd_acc + ACC_STEP) >= rd_period;

    always @(posedge clk_sys) begin
        if      (hs_ref_rise) rd_acc <= 8'd0;
        else if (rd_tick)     rd_acc <= rd_acc + ACC_STEP - rd_period;
        else                  rd_acc <= rd_acc + ACC_STEP;
    end

    // When On the line buffer is always live -- even at H-Size 0, where
    // rd_tick runs at the same average rate as ce_pix but re-phased onto
    // hs_ref.
    wire rd_ce = active ? rd_tick : ce_pix;

    //------------------------------------------------------------------------
    // The engine
    //------------------------------------------------------------------------
    wire [7:0] str_r, str_g, str_b;
    wire       str_hs, str_vs, str_hb, str_vb;

    crt_adjust #(
        .VTOTAL    (V_TOTAL),
        .HTOTAL    (H_TOTAL),
        .HPOS_MODE (HPOS_MODE),
        .AW        (AW),
        .VB_LATE   (VB_LATE)
    ) u_crt_adjust (
        .clk        (clk_sys),
        .pxl_cen    (ce_pix),
        .pxl2_cen   (rd_ce),
        .active     (active),
        .hsize      (hsize_s),       // kept for upstream parity; the rate above is H-Size
        .hoffset    (hpos_s),
        .voffset    (vshift_s),
        .r_in       (core_r), .g_in (core_g), .b_in (core_b),
        .hs_in      (HSync),         // NATIVE sync in -- the engine derives hs_ref itself
        .vs_in      (VSync),
        .hb_in      (HBlank | VBlank),
        .vb_in      (VBlank),        // TRUE vertical blank, not the combined blank
        .r_out      (str_r), .g_out (str_g), .b_out (str_b),
        .hs_out     (str_hs), .vs_out (str_vs),
        .hb_out     (str_hb), .vb_out (str_vb),
        .hs_ref_out (hs_ref)
    );

    // str_vb is the engine's native, write-rate blank; DE uses vb_read.
    wire unused_str_vb = str_vb;

    //------------------------------------------------------------------------
    // Read-side vertical blank: one flop on the hs_ref line edge, matching
    // VB_LATE = 0 (see the header). The second flop exists only for the
    // VB_LATE = 1 test control.
    //------------------------------------------------------------------------
    reg hs_ref_q;
    always @(posedge clk_sys) if (ce_pix) hs_ref_q <= hs_ref;
    wire ref_line_tick = ce_pix & hs_ref & ~hs_ref_q;

    reg vb_line, vb_read;
    always @(posedge clk_sys) if (ref_line_tick) begin
        vb_line <= VBlank;
        vb_read <= VB_LATE ? vb_line : VBlank;
    end

    //------------------------------------------------------------------------
    // Outputs. Off: the core's own video, untouched. On: the engine's stream,
    // which carries the native HSync (CONTENTSHIFT) and the V-Shifted VSync.
    //------------------------------------------------------------------------
    assign vga_r    = active ? str_r  : core_r;
    assign vga_g    = active ? str_g  : core_g;
    assign vga_b    = active ? str_b  : core_b;
    assign vga_hs   = active ? str_hs : HSync;
    assign vga_vs   = active ? str_vs : VSync;
    assign ce_pixel = rd_ce;
    assign de_in    = active ? ~(str_hb | vb_read) : ~(HBlank | VBlank);

endmodule
