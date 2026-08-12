// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
//============================================================================
//  analog_hsize.sv
//
//  Horizontal pixel-stretch module for the ANALOG VGA output path of a
//  MiSTer FPGA arcade core.
//
//  Ported into Klax from the Arcade-Raiden_MiSTer core
//  (https://github.com/rmonic79/Arcade-Raiden_MiSTer, rtl/Raiden/analog_hsize.sv).
//  The logic is byte-for-byte the upstream logic; only the comments have been
//  translated from Italian and the clock-ratio examples retargeted to Klax.
//  NOTE: this file is GPL-3.0-or-later while the rest of the core is
//  GPL-2.0-or-later, so the combined work ships under GPL v3.
//
//  --- What it does ---------------------------------------------------------
//  Each source pixel is emitted to the DAC for a longer, integer-uniform
//  number of pixel-clock periods. Every pixel of every line is stretched by
//  the same exact factor (no fractional ratio, no nearest-neighbour decisions
//  per pixel), so there is:
//      - NO shimmering on moving content
//      - NO blending / blur (output = source pixel, byte-exact)
//      - NO line buffer mismatch (1-line ping-pong, deterministic phase)
//
//  The horizontal sync rate seen by the CRT is unchanged: the extra time is
//  absorbed by the porches, so the line stays inside the tolerance of vintage
//  15 kHz CRT and PVM monitors.
//
//  The module is inserted EMU-SIDE: instantiated in Klax.sv at the video
//  output boundary, with zero sys_top.v changes. On this path the stretch
//  reaches the analog DAC, and (unlike the sys-side variant) HDMI follows the
//  stretch too.
//
//  --- Resource cost --------------------------------------------------------
//  ~1 M10K (24-bit linebuffer with ping-pong banks), ~50 ALM, 0 DSP.
//
//  --- Required external signals --------------------------------------------
//  pxl_cen   : the core's pixel clock enable (write rate). For Klax that is
//              the /8 ce_pix pulse on the 57.27272 MHz clk_sys = 7.15909 MHz.
//  pxl2_cen  : the DAC read clock enable, generated externally from a
//              fractional accumulator so it is phase-aligned to HSync
//              (see the rd_acc / rd_tick block in Klax.sv).
//  hsize     : signed 5-bit, OSD-controlled stretch factor.
//              hsize = 0 -> bypass (passthrough at pxl_cen rate)
//              hsize > 0 -> slower read = wider image
//              hsize < 0 -> faster read = narrower image
//============================================================================

module analog_hsize
(
    input              clk,
    input              pxl_cen,      // write clock enable (core pixel rate)
    input              pxl2_cen,     // read clock enable  (DAC pixel rate)

    input  signed [4:0] hsize,       // 0 = bypass, !=0 = stretch active
    input  signed [8:0] hoffset,     // moves the CONTENT (not the sync):
                                     // >0 = RIGHT, <0 = LEFT.
                                     // hs_out is untouched -> no CRT desync.

    input        [7:0] r_in,
    input        [7:0] g_in,
    input        [7:0] b_in,
    input              hs_in,
    input              vs_in,
    input              hb_in,
    input              vb_in,

    output reg   [7:0] r_out,
    output reg   [7:0] g_out,
    output reg   [7:0] b_out,
    output reg         hs_out,
    output reg         vs_out,
    output reg         hb_out,
    output reg         vb_out
);

    localparam integer AW = 10;  // 1024 samples per line (ping-pong banks)

    // ------------------------------------------------------------------
    //  Input pipeline @ pxl_cen (for latency matching in bypass mode)
    // ------------------------------------------------------------------
    reg [7:0] r_in_q, g_in_q, b_in_q;
    reg       hs_in_q, hb_in_q, vs_in_q, vb_in_q;
    reg       hs_in_d;
    initial begin
        r_in_q = 0; g_in_q = 0; b_in_q = 0;
        hs_in_q = 0; hb_in_q = 1; vs_in_q = 0; vb_in_q = 0;
        hs_in_d = 0;
    end

    always @(posedge clk) if (pxl_cen) begin
        r_in_q   <= r_in;
        g_in_q   <= g_in;
        b_in_q   <= b_in;
        hs_in_q  <= hs_in;
        hb_in_q  <= hb_in;
        vs_in_q  <= vs_in;
        vb_in_q  <= vb_in;
        hs_in_d  <= hs_in;
    end

    wire hs_rise_in = pxl_cen && (hs_in & ~hs_in_d);

    // ------------------------------------------------------------------
    //  Linebuffer ping-pong (24-bit RGB, single M10K, two banks).
    //  Written @ pxl_cen by the core, read @ pxl2_cen by the DAC.
    //  Two banks (selected by `bank` flipped on each HSync rise) avoid
    //  read/write collisions: write current line into `bank`, read the
    //  previous line (`~bank`) which is already complete.
    // ------------------------------------------------------------------
    (* ramstyle = "no_rw_check, M10K" *) reg [23:0] mem [0:(1<<AW)-1];
    integer ii;
    initial for (ii = 0; ii < (1<<AW); ii = ii + 1) mem[ii] = 24'd0;

    // ------------------------------------------------------------------
    //  WRITE side @ pxl_cen
    // ------------------------------------------------------------------
    reg [AW-1:0] wrp;
    reg [AW-1:0] hmax;
    reg [AW-1:0] hb0, hb1;
    reg          lhb_l;
    reg          bank;
    initial begin
        wrp = 0; hmax = 0;
        hb0 = 0; hb1 = 0;
        lhb_l = 0;
        bank = 0;
    end

    wire lhb = ~hb_in;

    always @(posedge clk) if (pxl_cen) begin
        lhb_l <= lhb;
        mem[{bank, wrp[AW-2:0]}] <= {r_in, g_in, b_in};
        if (hs_rise_in) begin
            wrp  <= {AW{1'b0}};
            hmax <= wrp;
            bank <= ~bank;
        end else begin
            wrp <= wrp + 1'b1;
        end
        if (lhb   & ~lhb_l) hb1 <= wrp;  // start of active region (wrp value)
        if (~lhb  &  lhb_l) hb0 <= wrp;  // end of active region   (wrp value)
    end

    // ------------------------------------------------------------------
    //  READ side @ pxl2_cen.
    //  rdcnt increments by 1 at each pxl2_cen pulse -> exactly one source
    //  pixel is emitted to the DAC per read tick. Reset is triggered by
    //  the rising edge of HSync, detected at FULL clk rate to avoid
    //  missing edges when pxl2_cen is slow.
    // ------------------------------------------------------------------
    reg [AW-1:0] rdcnt;
    reg          hs_in_d2;
    reg          hs_rise_pending;
    initial begin
        rdcnt = 0;
        hs_in_d2 = 0;
        hs_rise_pending = 0;
    end

    always @(posedge clk) begin
        hs_in_d2 <= hs_in;
        if (hs_in & ~hs_in_d2)        hs_rise_pending <= 1'b1;
        else if (pxl2_cen)            hs_rise_pending <= 1'b0;
    end

    always @(posedge clk) if (pxl2_cen) begin
        if (hs_rise_pending) begin
            rdcnt <= {AW{1'b0}};
        end else begin
            rdcnt <= rdcnt + 1'b1;
        end
    end

    // ------------------------------------------------------------------
    //  Read from the linebuffer @ pxl2_cen, on the OPPOSITE bank to the
    //  one currently being written (so the previous fully-written line).
    //  pass_q gates active video against blanking, in linebuffer units.
    // ------------------------------------------------------------------
    reg [23:0] rd_data;
    reg        pass_q;
    initial begin
        rd_data = 0;
        pass_q = 0;
    end

    // vb_active: true vertical VBlank, aligned to the READ side. It drops
    // pass_q during VBlank lines so VGA_DE goes low there, giving the OSD a
    // vertical frame boundary to latch onto. It does NOT touch hb0/hb1 (the
    // horizontal edges), so the active window is never re-clamped.
    //
    // Bottom-row fix: the read side emits the PREVIOUS line (~bank), so it
    // lags the write side / native vb_in by one line. If vb_active tracked
    // vb_in at the write rate it would clear pass_q while the read side was
    // still emitting the last active line, eating that line. Sample vb_in
    // once per line (hs_rise) and delay it one further line so vb_active
    // lines up with what the read side is actually emitting.
    reg vb_line, vb_active;
    initial begin vb_line = 0; vb_active = 0; end
    always @(posedge clk) if (hs_rise_in) begin
        vb_line   <= vb_in;
        vb_active <= vb_line;
    end

    // hoffset (signed) slides the active window: >0 = content RIGHT, <0 =
    // LEFT. rd_addr compensates so the correct source pixel is fetched.
    // hs_out is NOT touched -> HSync stays intact -> no desync at any shift.
    wire signed [AW+1:0] rdcnt_s = $signed({2'b0, rdcnt});
    wire signed [AW+1:0] hb1_s   = $signed({2'b0, hb1});
    wire signed [AW+1:0] hb0_s   = $signed({2'b0, hb0});
    wire signed [AW+1:0] hoff_s  = $signed(hoffset);
    wire [AW-1:0] rd_addr = (rdcnt_s - hoff_s);
    always @(posedge clk) if (pxl2_cen) begin
        rd_data <= mem[{~bank, rd_addr[AW-2:0]}];
        pass_q  <= (rdcnt_s >= (hb1_s + hoff_s)) && (rdcnt_s < (hb0_s + hoff_s)) && ~vb_active;
    end

    // ------------------------------------------------------------------
    //  Output mux. CRITICAL: when stretch is active, outputs MUST be
    //  registered @ pxl2_cen (the DAC rate), NOT @ pxl_cen (the write
    //  rate). Otherwise the fast write clock would re-sample the slow
    //  read data at write rate, breaking the "every pixel lasts exactly
    //  one read period" property and re-introducing shimmering.
    //  In bypass mode, registers run at pxl_cen for full passthrough.
    // ------------------------------------------------------------------
    wire bypass = (hsize == 5'sd0);

    initial begin
        r_out = 0; g_out = 0; b_out = 0;
        hs_out = 0; vs_out = 0; hb_out = 1; vb_out = 0;
    end

    always @(posedge clk) begin
        if (bypass) begin
            if (pxl_cen) begin
                r_out  <= r_in_q;
                g_out  <= g_in_q;
                b_out  <= b_in_q;
                hb_out <= hb_in_q;
                hs_out <= hs_in_q;
                vs_out <= vs_in_q;
                vb_out <= vb_in_q;
            end
        end else begin
            if (pxl2_cen) begin
                if (pass_q) begin
                    r_out <= rd_data[23:16];
                    g_out <= rd_data[15:8];
                    b_out <= rd_data[7:0];
                end else begin
                    r_out <= 8'd0;
                    g_out <= 8'd0;
                    b_out <= 8'd0;
                end
                hb_out <= ~pass_q;
                hs_out <= hs_in_q;
                vs_out <= vs_in_q;
                vb_out <= vb_in_q;
            end
        end
    end

endmodule
