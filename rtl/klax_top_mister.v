// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.
//
//============================================================================
// Klax Top-Level — MiSTer Integration
//
// Integrates the Klax core modules with the MiSTer framework:
//   - Native video timing (456 × 262 @ 7.15909 MHz pixel rate)
//   - klax_sys — 68000 CPU + full memory map (tilemap, palette, work and
//     MOB/SLIP RAM, EEPROM, OKI sound, IRQ, watchdog) plus the motion-object
//     line-buffer renderer; exposes tilemap/palette read ports for video
//   - Playfield renderer (klax_playfield — 7-stage pipeline), driven by the
//     tilemap/palette read ports klax_sys exposes
//   - Graphics ROM: klax_ddram_tile_rom (DDRAM tile + sprite cache) or
//     klax_rom_bank (on-chip M10K), selected by build parameter
//   - Sprite ROM fetch adapter (klax_sprow_adapter)
//   - MiSTer ioctl download/upload (EEPROM NVRAM save) and DDRAM glue
//
// klax_sys exports a set of dbg_* taps (CPU bus state and per-subsystem
// activity flags). Nothing here reads them, so synthesis prunes the logic that
// feeds them; they are left in place as observation points for simulation.
//
// Clock: clk_sys at 57.27272 MHz PLL (8 × pixel clock, 4 × board master).
//   ce_pix = /8 pixel enable; the 68000 ce_cpu shares the same pulse
//   (CPU clock = pixel clock = 14.31818 MHz / 2 on the real board).
//   Frame rate = 57.27272e6 / (8 × 456 × 262) ≈ 59.92 Hz
//============================================================================

module klax_top_mister #(
    // Graphics ROM storage: 1 = DDRAM-backed cache (the shipping choice),
    // 0 = on-chip M10K bank, which needs block RAM the design no longer has.
    parameter ENABLE_DDRAM_TILE_ROM = 1,
    parameter DDRAM_TILE_CACHE_AW = 12,  // 3-way set-assoc: 2048 sets x 3.
    // EEPROM save request fires after 2^N clk_sys of write quiet
    // (N=27 → ~2.3 s at 57.27 MHz; testbenches override with a small N).
    parameter EEPROM_SAVE_QUIET_BITS = 27
) (
    input               clk_sys,        // 57.27272 MHz system clock
    input               reset,

    // MiSTer ioctl download interface
    input               ioctl_download,
    input       [15:0]  ioctl_index,
    input               ioctl_wr,
    input       [26:0]  ioctl_addr,
    input       [7:0]   ioctl_dout,
    output              ioctl_wait,

    // MiSTer ioctl upload interface (EEPROM NVRAM save).
    // During an upload at the EEPROM index the framework samples ioctl_din
    // for the byte at ioctl_addr; ioctl_upload_req asks the HPS to start a
    // save (polled via command 0x3C, written to the MRA <nvram> file).
    input               ioctl_upload,
    output      [7:0]   ioctl_din,
    output              ioctl_upload_req,
    output      [7:0]   ioctl_upload_index,

    // MiSTer DDRAM interface for large graphics ROM storage
    output              DDRAM_CLK,
    input               DDRAM_BUSY,
    output      [7:0]   DDRAM_BURSTCNT,
    output      [28:0]  DDRAM_ADDR,
    input       [63:0]  DDRAM_DOUT,
    input               DDRAM_DOUT_READY,
    output              DDRAM_RD,
    output      [63:0]  DDRAM_DIN,
    output      [7:0]   DDRAM_BE,
    output              DDRAM_WE,

    // User controls
    input               p1_coin1,
    input               p1_coin2,
    input               p1_button1,
    input               p1_right,
    input               p1_left,
    input               p1_down,
    input               p1_up,
    input               p2_button1,
    input               p2_right,
    input               p2_left,
    input               p2_down,
    input               p2_up,
    input               service,

    // Video outputs
    output              ce_pix,         // pixel clock enable
    output              HBlank,
    output              HSync,
    output              VBlank,
    output              VSync,
    output      [7:0]   VGA_R,
    output      [7:0]   VGA_G,
    output      [7:0]   VGA_B,

    // Signed mono audio from the OKI MSM6295 model.
    output      [15:0]  audio_l,
    output      [15:0]  audio_r
);

    // === Clock: 57.27272 MHz system clock, native 7.15909 MHz pixel rate ===
    // clk_sys = 8 × 7.15909 MHz (4 × the 14.31818 MHz board master), so one
    // native Klax pixel is exactly 8 clk_sys cycles. The horizontal counter
    // advances on ce_pix and IS the native source coordinate.
    // The 68000 clock equals the pixel clock on the real board
    // (14.31818 MHz / 2), so ce_cpu shares the same /8 pulse.
    // Frame rate = 57.27272e6 / (8 × 456 × 262) ≈ 59.92 Hz
    // Line rate  = 57.27272e6 / (8 × 456) ≈ 15.70 kHz
    //
    // ce_pix marks the pixel boundary (counters advance, MiSTer CE_PIXEL).
    // ce_cap fires one clock later: every render path — playfield 7-stage
    // pipeline (including the DDRAM tile cache's 1-clock-late hit path),
    // line-buffer read + MO palette + merge decision — has settled by then,
    // so the output register stage captures a coherent pixel.
    // Nominal clk_sys frequency: 4 × 14.318181 MHz NTSC master = 630/11 MHz.
    // Used for OKI sample-tick generation (long-term-exact accumulator).
    localparam CLK_SYS_HZ = 57272727;

    reg [2:0] ce_cnt = 3'd0;
    reg       ce_pix_r = 1'b0;
    reg       ce_cap_r = 1'b0;
    always @(posedge clk_sys or posedge reset) begin
        if (reset) begin
            ce_cnt   <= 3'd0;
            ce_pix_r <= 1'b0;
            ce_cap_r <= 1'b0;
        end else begin
            ce_cnt   <= ce_cnt + 3'd1;
            ce_pix_r <= (ce_cnt == 3'd6);
            ce_cap_r <= (ce_cnt == 3'd7);
        end
    end
    assign ce_pix = ce_pix_r;

    // Native Klax video parameters.
    // Totals/visible: verified from MAME klax.cpp set_raw.
    // H SYNC: GROUND TRUTH reverse-engineered from the dumped sync PAL16R6 at
    //   6W (Atari 136075-1004; 136075-1004.6w.bin) via jedutil. The SOS-2
    //   custom chip (9Y) emits the 1H..256H counter taps; the 6W PAL decodes
    //   /PHSYNC = hc 352..379 (28 px) and HBLANK (336 visible). Porch split
    //   front/sync/back = 16/28/76.
    // V SYNC: still ASSUMED. VSYNC/VBLANK are generated inside the SOS-2 custom
    //   chip (not in any dumped PAL), so exact placement needs a scope capture.
    localparam H_TOTAL      = 456;
    localparam H_VIS_END    = 336;
    // /PHSYNC from the dumped 6W PAL = hc 352..379. The PCB delays it through
    // the LS174 @17K (2H clock, R102 tap = stage 3 stuffed, Sheet 8) before
    // the monitor: +10 px. Monitor-faithful /HSYNC = hc 362..389 (28 px).
    localparam H_SYNC_START = 362;   // 6W /PHSYNC (352) + 17K delay (+10)
    localparam H_SYNC_END   = 390;   // sync = hc 362..389 (28 px)
    localparam V_TOTAL      = 262;
    localparam V_VIS_END    = 240;
    localparam V_SYNC_START = 248;   // ASSUMED (SOS-2 internal; needs capture)
    localparam V_SYNC_END   = 251;   // ASSUMED
    reg [10:0] hc;
    reg [9:0]  vc;
    reg        HBlank_r;
    reg       HSync_r;
    reg       VBlank_r;
    reg       VSync_r;

    always @(posedge clk_sys or posedge reset) begin
        if (reset) begin
            hc <= 0;
            vc <= 0;
        end else if (ce_pix) begin
            if (hc == H_TOTAL - 1) begin
                hc <= 0;
                if (vc == V_TOTAL - 1)
                    vc <= 0;
                else
                    vc <= vc + 1'd1;
            end else begin
                hc <= hc + 1'd1;
            end
        end
    end

    always @(posedge clk_sys or posedge reset) begin
        if (reset) begin
            HBlank_r <= 0;
            HSync_r  <= 0;
            VBlank_r <= 0;
            VSync_r  <= 0;
        end else if (ce_pix) begin
            HBlank_r <= (hc >= H_VIS_END);
            HSync_r  <= (hc >= H_SYNC_START) && (hc < H_SYNC_END);
            VBlank_r <= (vc >= V_VIS_END);
            VSync_r  <= (vc >= V_SYNC_START) && (vc < V_SYNC_END);
        end
    end

    // Current-pixel blanking (combinational). The registered HBlank_r /
    // VBlank_r lag hc by one ce_pix, which would blank the first visible
    // column inside the renderers, so renderer view gating uses these
    // instead.
    wire hblank_now = (hc >= H_VIS_END);
    wire vblank_now = (vc >= V_VIS_END);

    localparam USE_DDRAM_TILE_ROM = (ENABLE_DDRAM_TILE_ROM != 0);
    localparam USE_INTERNAL_TILE_ROM = (USE_DDRAM_TILE_ROM == 0);
    localparam [28:0] DDRAM_TILE_BASE_ADDR = 29'h04800000;
    localparam [28:0] DDRAM_SPRITE_BASE_ADDR = 29'h04808000;

    // === ROM Bank (tile ROM loaded via ioctl) ===
    wire [7:0]  rom_data;
    wire [31:0] sprite_rom_row;
    wire        sprite_rom_row_ready;
    wire [17:0] rom_addr;
    wire [16:0] sprite_rom_addr;
    wire        sprite_rom_rd_en;
    wire [17:0] pf_rom_addr;
    // Playfield tile-graphics prefetch (warms the DDRAM tile cache ahead of
    // the raster so the demand read can never miss; see klax_playfield.v).
    wire [10:0] tilemap_pre_addr;
    wire [15:0] pf_pre_data;
    wire [17:0] pf_pre_rom_addr;
    wire        pf_pre_rom_en;
    assign rom_addr = pf_rom_addr;

    wire [15:0] tile_rom_debug_flags;
    wire [15:0] sprite_rom_debug_flags;
    wire [15:0] playfield_debug_flags;
    wire [15:0] vram_debug_flags;
    wire [15:0] mob_debug_flags;
    wire [15:0] mo_debug_flags;
    wire        pf_priority_block;
    wire [8:0]  mob_pal_addr;
    wire [15:0] mob_pal_data;

    // Motion object line-buffer interface (from klax_sys)
    wire [8:0]  mo_pix;               // {valid, color[3:0], pen[3:0]}
    wire        mo_sprow_req;
    wire [11:0] mo_sprow_code;
    wire [2:0]  mo_sprow_row;
    wire [31:0] mo_sprow_data;
    wire        mo_sprow_ack;

    generate
        if (USE_DDRAM_TILE_ROM != 0) begin : gen_ddram_tile_rom
            klax_ddram_tile_rom #(
                .BASE_ADDR(DDRAM_TILE_BASE_ADDR),
                .SPRITE_BASE_ADDR(DDRAM_SPRITE_BASE_ADDR),
                .CACHE_AW(DDRAM_TILE_CACHE_AW)
            ) u_rom (
                .clk                (clk_sys),
                .reset              (reset),
                .ioctl_download     (ioctl_download),
                .ioctl_index        (ioctl_index),
                .ioctl_wr           (ioctl_wr),
                .ioctl_addr         (ioctl_addr),
                .ioctl_dout         (ioctl_dout),
                .ioctl_wait         (ioctl_wait),
                .rd_addr            (rom_addr),
                .rd_data            (rom_data),
                .debug_flags        (tile_rom_debug_flags),
                .pre_en             (pf_pre_rom_en),
                .pre_addr           (pf_pre_rom_addr),
                .sprite_rd_en       (sprite_rom_rd_en),
                .sprite_rd_addr     (sprite_rom_addr),
                .sprite_rd_row      (sprite_rom_row),
                .sprite_rd_ready    (sprite_rom_row_ready),
                .sprite_debug_flags (sprite_rom_debug_flags),
                .DDRAM_CLK          (DDRAM_CLK),
                .DDRAM_BUSY         (DDRAM_BUSY),
                .DDRAM_BURSTCNT     (DDRAM_BURSTCNT),
                .DDRAM_ADDR         (DDRAM_ADDR),
                .DDRAM_DOUT         (DDRAM_DOUT),
                .DDRAM_DOUT_READY   (DDRAM_DOUT_READY),
                .DDRAM_RD           (DDRAM_RD),
                .DDRAM_DIN          (DDRAM_DIN),
                .DDRAM_BE           (DDRAM_BE),
                .DDRAM_WE           (DDRAM_WE)
            );
        end else if (USE_INTERNAL_TILE_ROM != 0) begin : gen_tile_rom
            klax_rom_bank #(
                .ROM_AW(18),
                .ROM_DW(8)
            ) u_rom (
                .clk            (clk_sys),
                .reset          (reset),
                .ioctl_download (ioctl_download),
                .ioctl_wr       (ioctl_wr && (ioctl_index == 1)),
                .ioctl_addr     (ioctl_addr),
                .ioctl_dout     (ioctl_dout),
                .rd_addr        (rom_addr),
                .rd_data        (rom_data)
            );
            assign tile_rom_debug_flags = 16'd0;
            assign sprite_rom_debug_flags = 16'd0;
            assign sprite_rom_row = 32'd0;
            assign sprite_rom_row_ready = 1'b0;
        end else begin : gen_no_tile_rom
            assign rom_data = 8'd0;
            assign sprite_rom_row = 32'd0;
            assign sprite_rom_row_ready = 1'b0;
            assign tile_rom_debug_flags = 16'd0;
            assign sprite_rom_debug_flags = 16'd0;
            assign ioctl_wait = 1'b0;
            assign DDRAM_CLK = 1'b0;
            assign DDRAM_BURSTCNT = 8'd0;
            assign DDRAM_ADDR = 29'd0;
            assign DDRAM_RD = 1'b0;
            assign DDRAM_DIN = 64'd0;
            assign DDRAM_BE = 8'd0;
            assign DDRAM_WE = 1'b0;
        end

        if ((USE_DDRAM_TILE_ROM == 0) && (USE_INTERNAL_TILE_ROM != 0)) begin : gen_no_ddram_tile_rom
            assign ioctl_wait = 1'b0;
            assign DDRAM_CLK = 1'b0;
            assign DDRAM_BURSTCNT = 8'd0;
            assign DDRAM_ADDR = 29'd0;
            assign DDRAM_RD = 1'b0;
            assign DDRAM_DIN = 64'd0;
            assign DDRAM_BE = 8'd0;
            assign DDRAM_WE = 1'b0;
        end
    endgenerate

    // === ROM load tracking ===
    // Per-region byte counters and done flags. The exact-size checks below
    // hold the CPU system in reset until the program ROM and EEPROM images
    // have loaded completely, and keep the screen black until the tile ROM
    // download has finished, so the 68000 never executes from a half-written
    // ROM after a mid-download abort.
    reg        ioctl_prev = 0;
    reg        rom_load_done = 0;
    reg        tile_download_active = 0;
    reg [18:0] cpu_bytes_loaded = 19'd0;
    reg        cpu_download_active = 1'b0;
    reg        cpu_rom_load_done = 1'b0;
    reg [11:0] eeprom_bytes_loaded = 12'd0;
    reg        eeprom_download_active = 1'b0;
    reg        eeprom_load_done = 1'b0;
    reg [17:0] sprite_bytes_loaded = 18'd0;
    reg        sprite_download_active = 1'b0;
    reg        sprite_rom_load_done = 1'b0;

    always @(posedge clk_sys) begin
        ioctl_prev <= ioctl_download;

        if (~ioctl_prev && ioctl_download) begin
            tile_download_active <= 1'b0;
            cpu_download_active <= 1'b0;
            eeprom_download_active <= 1'b0;
            sprite_download_active <= 1'b0;
        end else if (ioctl_download) begin
            if (ioctl_wr) begin
                if (ioctl_index == 16'd1) begin
                    if (!tile_download_active)
                        rom_load_done <= 1'b0;
                    tile_download_active <= 1'b1;
                end
                if (ioctl_index == 16'd0) begin
                    if (!cpu_download_active) begin
                        cpu_bytes_loaded <= 19'd1;
                        cpu_rom_load_done <= 1'b0;
                    end else begin
                        cpu_bytes_loaded <= cpu_bytes_loaded + 19'd1;
                    end
                    cpu_download_active <= 1'b1;
                end
                if (ioctl_index == 16'd2) begin
                    if (!eeprom_download_active) begin
                        eeprom_bytes_loaded <= 12'd1;
                        eeprom_load_done <= 1'b0;
                    end else begin
                        eeprom_bytes_loaded <= eeprom_bytes_loaded + 12'd1;
                    end
                    eeprom_download_active <= 1'b1;
                end
                if (ioctl_index == 16'd4) begin
                    if (!sprite_download_active) begin
                        sprite_bytes_loaded <= 18'd1;
                        sprite_rom_load_done <= 1'b0;
                    end else begin
                        sprite_bytes_loaded <= sprite_bytes_loaded + 18'd1;
                    end
                    sprite_download_active <= 1'b1;
                end
            end
        end else if (ioctl_prev && ~ioctl_download) begin
            if (tile_download_active)
                rom_load_done <= 1'b1;
            if (cpu_download_active) begin
                cpu_download_active <= 1'b0;
                cpu_rom_load_done <= 1'b1;
            end
            if (eeprom_download_active) begin
                eeprom_download_active <= 1'b0;
                eeprom_load_done <= 1'b1;
            end
            if (sprite_download_active) begin
                sprite_download_active <= 1'b0;
                sprite_rom_load_done <= 1'b1;
            end
        end
    end

    wire cpu_bytes_ok = cpu_bytes_loaded[18] &&
                        (cpu_bytes_loaded[17:9] == 9'd0) &&
                        (cpu_bytes_loaded[8:0] == 9'd0);
    wire cpu_rom_ready = cpu_rom_load_done && cpu_bytes_ok;
    wire eeprom_bytes_ok = eeprom_bytes_loaded[11] &&
                           (eeprom_bytes_loaded[10:0] == 11'd0);
    wire eeprom_ready = eeprom_load_done && eeprom_bytes_ok;
    wire cpu_system_ready = cpu_rom_ready && eeprom_ready;
    wire sprite_bytes_ok = sprite_bytes_loaded[17] &&
                           (sprite_bytes_loaded[16:0] == 17'd0);
    wire sprite_rom_ready = sprite_rom_load_done && sprite_bytes_ok;

    // === EEPROM NVRAM save ===
    // Upload serving: during an index-2 upload the framework samples
    // ioctl_din for the byte at ioctl_addr (which auto-increments after
    // each beat), so the EEPROM's dedicated upload read port just follows
    // ioctl_addr. Inter-beat spacing is many clk_sys, far beyond the
    // 1-clock registered read.
    wire [10:0] eeprom_up_addr = ioctl_addr[10:0];
    wire [7:0]  eeprom_up_data;   // driven by the CPU generate branches
    wire        eeprom_cpu_wr;    // pulse per committed CPU EEPROM write

    assign ioctl_din = (ioctl_upload && (ioctl_index == 16'd2)) ?
                       eeprom_up_data : 8'hFF;

    // Save request: once the game commits EEPROM writes (high scores or
    // settings) and the contents stay quiet for 2^EEPROM_SAVE_QUIET_BITS
    // clk_sys (~2.3 s on hardware), pulse one upload request at the EEPROM
    // ioctl index; the HPS polls it (command 0x3C) and stores the upload
    // in the MRA <nvram> save file. New writes during the countdown
    // restart it; a served index-2 upload clears the pending state, and
    // any active transfer holds the countdown.
    reg                            eeprom_save_pending = 1'b0;
    reg [EEPROM_SAVE_QUIET_BITS:0] eeprom_quiet_cnt = 0;
    reg                            eeprom_save_req_r = 1'b0;

    always @(posedge clk_sys or posedge reset) begin
        if (reset) begin
            eeprom_save_pending <= 1'b0;
            eeprom_quiet_cnt    <= 0;
            eeprom_save_req_r   <= 1'b0;
        end else begin
            eeprom_save_req_r <= 1'b0;
            if (eeprom_cpu_wr) begin
                eeprom_save_pending <= 1'b1;
                eeprom_quiet_cnt    <= 0;
            end else if (ioctl_upload || ioctl_download) begin
                if (ioctl_upload && (ioctl_index == 16'd2))
                    eeprom_save_pending <= 1'b0;
                eeprom_quiet_cnt <= 0;
            end else if (eeprom_save_pending) begin
                if (eeprom_quiet_cnt[EEPROM_SAVE_QUIET_BITS]) begin
                    eeprom_save_req_r   <= 1'b1;
                    eeprom_save_pending <= 1'b0;
                    eeprom_quiet_cnt    <= 0;
                end else begin
                    eeprom_quiet_cnt <= eeprom_quiet_cnt + 1'd1;
                end
            end
        end
    end

    assign ioctl_upload_req   = eeprom_save_req_r;
    assign ioctl_upload_index = 8'd2;

    // === Sprite Row Adapter ===
    // Bridges klax_motion_objects sprow_req/ack to the DDRAM row port.
    generate
        if (USE_DDRAM_TILE_ROM != 0) begin : gen_sprow_adapter
            klax_sprow_adapter u_sprow_adapter (
                .clk             (clk_sys),
                .reset           (reset),
                .sprow_req       (mo_sprow_req),
                .sprow_code      (mo_sprow_code),
                .sprow_row       (mo_sprow_row),
                .sprow_data      (mo_sprow_data),
                .sprow_ack       (mo_sprow_ack),
                .sprite_rd_en    (sprite_rom_rd_en),
                .sprite_rd_addr  (sprite_rom_addr),
                .sprite_rd_row   (sprite_rom_row),
                .sprite_rd_ready (sprite_rom_row_ready)
            );
        end else begin : gen_no_sprow_adapter
            assign mo_sprow_data = 32'd0;
            assign mo_sprow_ack  = 1'b0;
            assign sprite_rom_rd_en = 1'b0;
            assign sprite_rom_addr = 17'd0;
        end
    endgenerate

    // Native source X coordinate for motion-object line-buffer readback:
    // the horizontal counter IS the source coordinate (hc advances once per
    // ce_pix). The line-buffer read + palette + merge path settles within
    // the first few clocks of the 8-clock pixel window and is captured
    // together with the playfield by the ce_cap output register stage, so
    // no read-coordinate skew is needed.
    wire [8:0]  mo_vid_x = hc[8:0];
    wire        mo_vid_active = ~HBlank_r && ~VBlank_r;

    wire [7:0] pf_r, pf_g, pf_b;
    wire [23:0] dbg_cpu_addr;
    wire [15:0] dbg_cpu_data_in;
    wire [15:0] dbg_cpu_data_out;
    wire [15:0] cpu_audio_sample;
    wire [2:0]  dbg_cpu_fc;
    wire        dbg_cpu_as_n;
    wire        dbg_cpu_uds_n;
    wire        dbg_cpu_lds_n;
    wire        dbg_cpu_rw_n;
    wire        dbg_cpu_dtack_n;
    wire        dbg_cpu_reset;
    wire [2:0]  dbg_cpu_ipl_n;

    // === CPU + Playfield + Motion Objects ===

    // === CPU Clock Enable (exact 7.15909 MHz = clk_sys / 8) ===
    // The 68000 clock equals the pixel clock on the real board, so
    // the CPU ticks once per native pixel, phase-locked to ce_pix.
    wire ce_cpu = ce_pix_r;

    // === CPU System ===
    wire [10:0] tilemap_addr;
    wire [15:0] pf_data;
    wire [15:0] pfx_data;
    wire [8:0]  pal_addr;
    wire [15:0] pal_data;

    klax_sys #(
        .ENABLE_RAM_BLOCKS(1),
        .CLK_HZ(CLK_SYS_HZ)
    ) u_sys (
        .clk              (clk_sys),
        .reset            (reset || !cpu_system_ready),
        .ce_cpu           (ce_cpu),
        .vblank           (VBlank_r),
        .scanline         (vc[8:0]),
        .hc               (hc[8:0]),
        .tilemap_rd_addr  (tilemap_addr),
        .tilemap_pre_addr (tilemap_pre_addr),
        .pf_pre_rd_data   (pf_pre_data),
        .pf_rd_data       (pf_data),
        .pfx_rd_data      (pfx_data),
        .pal_rd_addr      (pal_addr),
        .pal_rd_data      (pal_data),
        .mob_pal_rd_addr  (mob_pal_addr),
        .mob_pal_rd_data  (mob_pal_data),
        .vid_x            (mo_vid_x),
        .vid_active       (mo_vid_active),
        .mo_pix           (mo_pix),
        .sprow_req        (mo_sprow_req),
        .sprow_code       (mo_sprow_code),
        .sprow_row        (mo_sprow_row),
        .sprow_data       (mo_sprow_data),
        .sprow_ack        (mo_sprow_ack),
        .ioctl_download   (ioctl_download),
        .ioctl_index      (ioctl_index),
        .ioctl_wr         (ioctl_wr),
        .ioctl_addr       (ioctl_addr),
        .ioctl_dout       (ioctl_dout),
        .eeprom_up_addr   (eeprom_up_addr),
        .eeprom_up_data   (eeprom_up_data),
        .eeprom_cpu_wr    (eeprom_cpu_wr),
        .p1_coin1         (p1_coin1),
        .p1_coin2         (p1_coin2),
        .p1_button1       (p1_button1),
        .p1_right         (p1_right),
        .p1_left          (p1_left),
        .p1_down          (p1_down),
        .p1_up            (p1_up),
        .p2_button1       (p2_button1),
        .p2_right         (p2_right),
        .p2_left          (p2_left),
        .p2_down          (p2_down),
        .p2_up            (p2_up),
        .service          (service),
        .audio_sample     (cpu_audio_sample),
        .dbg_cpu_addr     (dbg_cpu_addr),
        .dbg_cpu_data_in  (dbg_cpu_data_in),
        .dbg_cpu_data_out (dbg_cpu_data_out),
        .dbg_cpu_fc       (dbg_cpu_fc),
        .dbg_cpu_as_n     (dbg_cpu_as_n),
        .dbg_cpu_uds_n    (dbg_cpu_uds_n),
        .dbg_cpu_lds_n    (dbg_cpu_lds_n),
        .dbg_cpu_rw_n     (dbg_cpu_rw_n),
        .dbg_cpu_dtack_n  (dbg_cpu_dtack_n),
        .dbg_cpu_reset    (dbg_cpu_reset),
        .dbg_cpu_ipl_n    (dbg_cpu_ipl_n),
        .dbg_vram_flags   (vram_debug_flags),
        .dbg_mob_flags    (mob_debug_flags),
        .dbg_mo_flags     (mo_debug_flags)
    );

    klax_playfield #(
        .ROM_AW(18),
        .H_VIS(336),
        .V_VIS(240)
    ) u_pf (
        .clk          (clk_sys),
        .reset        (reset),
        .hc           (hc),
        .vc           (vc),
        .hblank       (hblank_now),
        .vblank       (vblank_now),
        .scroll_x     (9'd0),
        .scroll_y     (8'd0),
        .tilemap_addr (tilemap_addr),
        .pf_data      (pf_data),
        .pfx_data     (pfx_data),
        .pre_tilemap_addr (tilemap_pre_addr),
        .pf_pre_data  (pf_pre_data),
        .pre_rom_addr (pf_pre_rom_addr),
        .pre_rom_en   (pf_pre_rom_en),
        .rom_addr     (pf_rom_addr),
        .rom_data     (rom_data),
        .pal_addr     (pal_addr),
        .pal_data     (pal_data),
        .VGA_R        (pf_r),
        .VGA_G        (pf_g),
        .VGA_B        (pf_b),
        .pf_priority_block(pf_priority_block),
        .debug_flags  (playfield_debug_flags)
    );

    assign audio_l = cpu_audio_sample;
    assign audio_r = cpu_audio_sample;

    // === Playfield output ===
    // The playfield renderer is the only source of base video. Nothing is
    // displayed until the graphics ROMs have loaded and the CPU system is out
    // of reset; the output capture stage below blanks the screen until then.
    wire [7:0] active_r = pf_r;
    wire [7:0] active_g = pf_g;
    wire [7:0] active_b = pf_b;
    wire graphics_display_ready = cpu_system_ready && rom_load_done;

    // === Motion Object Pixel Merge ===
    //
    // mo_pix = {valid, color[3:0], pen[3:0]} from the line-buffer renderer.
    // When valid and pen != 0, look up the palette color via mob_pal_rd_addr
    // and merge over the playfield unless pf_priority_block is set.

    wire        mo_valid = mo_pix[8];
    wire [3:0]  mo_color = mo_pix[7:4];
    wire [3:0]  mo_pen   = mo_pix[3:0];
    wire        mo_opaque = mo_valid && (mo_pen != 4'd0);

    // Palette lookup: {1'b0, color, pen} = 9-bit address into palette
    assign mob_pal_addr = mo_opaque ? {1'b0, mo_color, mo_pen} : 9'd0;

    // Decode IRGB_1555 palette entry
    wire        mob_pal_i  = mob_pal_data[15];
    wire [4:0]  mob_pal_r5 = mob_pal_data[14:10];
    wire [4:0]  mob_pal_g5 = mob_pal_data[9:5];
    wire [4:0]  mob_pal_b5 = mob_pal_data[4:0];
    // Silicon DAC truth (schematic Sheet 9): 6-bit R/2R ladder per channel,
    // LSB = the shared intensity bit ({color5, I}); MAME's IRGB_1555 decoder
    // is identical (pal6bit((c << 1) | i)). 8-bit expansion = {v6, v6[5:4]}.
    wire [5:0]  mob_r6 = {mob_pal_r5, mob_pal_i};
    wire [5:0]  mob_g6 = {mob_pal_g5, mob_pal_i};
    wire [5:0]  mob_b6 = {mob_pal_b5, mob_pal_i};
    wire [7:0]  mob_pal_r8 = {mob_r6, mob_r6[5:4]};
    wire [7:0]  mob_pal_g8 = {mob_g6, mob_g6[5:4]};
    wire [7:0]  mob_pal_b8 = {mob_b6, mob_b6[5:4]};

    // Pipeline the merge decision by one cycle to match palette RAM latency.
    // The active-area gate uses the current-pixel window (hc/vc directly):
    // the registered HBlank_r/VBlank_r lag by one pixel, which would drop
    // motion-object pixels in the first visible column. Registered at the
    // pixel-boundary edge this picks up the settled pf_priority_block and
    // line-buffer read of the pixel just finished, and the ce_cap output
    // stage captures it one clock later.
    reg        mo_merge_d = 1'b0;
    always @(posedge clk_sys or posedge reset) begin
        if (reset)
            mo_merge_d <= 1'b0;
        else
            mo_merge_d <= (USE_DDRAM_TILE_ROM != 0) &&
                          mo_opaque && !pf_priority_block &&
                          !hblank_now && !vblank_now;
    end

    // Base that the motion objects merge over.
    wire [7:0] merge_base_r = active_r;
    wire [7:0] merge_base_g = active_g;
    wire [7:0] merge_base_b = active_b;

    wire [7:0] active_mo_r = mo_merge_d ? mob_pal_r8 : merge_base_r;
    wire [7:0] active_mo_g = mo_merge_d ? mob_pal_g8 : merge_base_g;
    wire [7:0] active_mo_b = mo_merge_d ? mob_pal_b8 : merge_base_b;

    // === VGA Output ===
    wire       display_ready = graphics_display_ready;

    // === Output capture stage (ce_cap) ===
    // One clock after each pixel boundary every render path has settled —
    // playfield stage-6 RGB and priority, the line-buffer read + MO palette
    // + merge decision, and the registered blanking all describe the pixel
    // that just finished. Capture RGB and blanking/sync together so the
    // MiSTer framework sees glitch-free, pixel-aligned outputs with a
    // uniform one-pixel latency.
    reg [7:0] vga_r_q = 8'd0;
    reg [7:0] vga_g_q = 8'd0;
    reg [7:0] vga_b_q = 8'd0;
    reg       hblank_q = 1'b1;
    reg       hsync_q  = 1'b0;
    reg       vblank_q = 1'b1;
    reg       vsync_q  = 1'b0;

    always @(posedge clk_sys or posedge reset) begin
        if (reset) begin
            vga_r_q  <= 8'd0;
            vga_g_q  <= 8'd0;
            vga_b_q  <= 8'd0;
            hblank_q <= 1'b1;
            hsync_q  <= 1'b0;
            vblank_q <= 1'b1;
            vsync_q  <= 1'b0;
        end else if (ce_cap_r) begin
            // Selected renderer once ready; black screen while ROMs load.
            vga_r_q  <= display_ready ? active_mo_r : 8'h00;
            vga_g_q  <= display_ready ? active_mo_g : 8'h00;
            vga_b_q  <= display_ready ? active_mo_b : 8'h00;
            hblank_q <= HBlank_r;
            hsync_q  <= HSync_r;
            vblank_q <= VBlank_r;
            vsync_q  <= VSync_r;
        end
    end

    assign HBlank = hblank_q;
    assign HSync  = hsync_q;
    assign VBlank = vblank_q;
    assign VSync  = vsync_q;
    assign VGA_R  = vga_r_q;
    assign VGA_G  = vga_g_q;
    assign VGA_B  = vga_b_q;

endmodule
