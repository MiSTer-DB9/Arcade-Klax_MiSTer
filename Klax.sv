// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.
//
//============================================================================
//
//  Klax — MiSTer FPGA Hardware Recreation
//  Atari Games. Debuted 1989; MAME dates every set 1990 from the ROMs' own
//  internal build stamps (parent set: 13FEB1990 11:10:09), and the MRAs follow.
//
//  Full-game MiSTer integration: TG68K 68000 core, playfield + motion-object
//  rendering, OKI MSM6295 audio, and EEPROM NVRAM save, with the graphics ROM
//  served from DDRAM.
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_OSD + per-pin push-pull mask, USER_IO widened to 8 bits
	output        USER_OSD,
	output  [7:0] USER_PP,
	input   [7:0] USER_IN,
	output  [7:0] USER_OUT,
	// [MiSTer-DB9 END]

	input         OSD_STATUS
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_PP driven by wrapper; USER_OUT driven by joydb (USER_OUT_DRIVE) below
assign USER_PP = USER_PP_DRIVE;
// [MiSTer-DB9 END]
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

// Aspect ratio request. VIDEO_ARX/ARY are driven by video_freak down in the
// VIDEO OUT section so the Scale option can override them with an integer-
// scaled pixel size instead of a plain ratio.
wire [1:0] ar = status[122:121];

wire [11:0] arx = (!ar) ? 12'd4 : (ar - 1'd1);
wire [11:0] ary = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v"
// Graphics ROM storage: 1 = DDRAM-backed cache (the shipping choice), 0 = the
// on-chip M10K bank, which needs block RAM the design no longer has spare.
localparam KLAX_ENABLE_DDRAM_TILE_ROM = 1'b1;

// The menu is the Video and CRT Adjust pages, service, autosave, controls and
// reset.
//
// Video page: Aspect ratio, and Scale = the integer-scaling mode handed to
// video_freak.
//
// CRT Adjust page: rmonic79's MiSTer-CRT-Adjust, core-side variant. An
// On/Off plus three amounts, all plain two's complement (the list is
// 0,+1..+max then -min..-1 and the decode is a $signed()):
//   CRT H-Size     - analog pixel stretch, steps of 1/64 pixel period
//                    (1.5625%).
//   CRT H-Position - slides the content inside the line buffer without
//                    moving HSync (no desync).
//   CRT V-Shift    - delays VSync by N lines.
// None of these touch the core's timing. Off is a bit-exact bypass, so the
// analog output and HDMI are untouched unless the user turns it On. H1 hides
// the three amounts while the stage is inactive, which includes the case
// where the framework has forced the scandoubler and the stage gates itself
// off (see `crt_enable`). See rtl/klax_video_adjust.v.
//
// Status bits: [13] On, [18:14] H-Size, [25:19] H-Position, [31:26] V-Shift.
// The previous engine's bits ([61:56] V-Shift, [66:62] H-Size, [97:92]
// H-Shift, [104:98] H-Position, index-encoded) are retired, not reused, so
// an old saved config cannot leak into the new controls.
`define KLAX_VIDEO_MENU \
	"P1,Video;", \
	"P1O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];", \
	"P1O[2:1],Scale,Normal,V-Integer,HV-Integer,Narrower HV-Integer;", \
	"P2,CRT Adjust;", \
	"P2O[13],CRT Adjust,Off,On;", \
	"H1P2O[18:14],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;", \
	"H1P2O[25:19],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,+49,+50,+51,+52,+53,+54,+55,+56,+57,+58,+59,+60,+61,+62,+63,-64,-63,-62,-61,-60,-59,-58,-57,-56,-55,-54,-53,-52,-51,-50,-49,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;", \
	"H1P2O[31:26],CRT V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;"

localparam CONF_STR = {
	"Klax;;",
	"-;",
	`KLAX_VIDEO_MENU,
	"-;",
	"O[11],Service,Off,On;",
	"O[12],Autosave,On,Off;",
	"-;",
	// [MiSTer-DB9-Pro BEGIN] - Saturn-first joy_type (canonical bit notation)
	"O[127:126],UserIO Joystick,Off,Saturn,DB9MD,DB15;",
	"O[125],UserIO Players, 1 Player,2 Players;",
	// [MiSTer-DB9-Pro END]
	"T[0],Reset;",
	"-;",
	"J1,Flip/Start,Flip (2nd),Coin 1,Coin 2,Service;",
	"jn,A,B,R,L,Select;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [10:0] ps2_key;
wire  [31:0] joystick_0;
wire  [31:0] joystick_1;

// CRT Adjust master enable. Declared here because status_menumask below reads
// it; what it means is documented at the stage itself (VIDEO OUT section). The
// forced-scandoubler term is upstream's own integration rule: the read-rate
// base assumes the native 15 kHz pixel-CE ratio.
wire crt_enable = status[13] & ~forced_scandoubler;

// ioctl signals for ROM download
wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_wait;

// ioctl upload signals for EEPROM NVRAM save
wire        ioctl_upload;
wire  [7:0] ioctl_din;
wire        ioctl_upload_req;
wire  [7:0] ioctl_upload_index;

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper
wire         CLK_JOY = CLK_50M;                 // Assign clock between 40-50Mhz
wire   [1:0] joy_type_raw    = status[127:126]; // 0=Off, 1=Saturn, 2=DB9MD, 3=DB15
wire         joy_2p          = status[125];
// SNAC cores: replace 1'b0 with the core's SNAC enable expression so SNAC
// preempts the joydb wrapper on shared USER_IO pins. Default 1'b0 is no-op.
wire         snac_active     = 1'b0;
// MT32-pi cores on primary USER_IO: replace 1'b0 with the core's MT32-active
// expression (e.g. `mt32_use` under `ifndef SECOND_MT32`, `~mt32_disable` for
// TRS-80's inverted polarity). Suppresses the OSD-open autodetect probe so it
// doesn't read the RPi's I2C master traffic as a ghost Saturn signature.
wire         mt32_primary_active = 1'b0;
wire   [1:0] joy_type        = snac_active ? 2'd0 : joy_type_raw;
wire         joy_db9md_en    = (joy_type == 2'd2);
wire         joy_db15_en     = (joy_type == 2'd3);
wire         joy_any_en      = |joy_type;
// [MiSTer-DB9 END]

// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
wire         saturn_unlocked;                   // driven by hps_io UIO_DB9_KEY (0xFE)
// [MiSTer-DB9-Pro END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper wires + instance
wire   [7:0] USER_OUT_DRIVE;
wire   [7:0] USER_PP_DRIVE;
wire  [15:0] joydb_1, joydb_2;
wire         joydb_1ena, joydb_2ena;
wire  [15:0] joy_raw_payload;
// Programmable-remap matrix (joydb_remap inside joydb): clk_sys carries the
// 0xFD selector load (HPS-bus domain). joydb_*_mapped are the MiSTer-standard
// joystick words the core consumes at its joy0_USB merge point. db9_remap_*
// are driven by the hps_io instance (see the .db9_remap_* bindings there).
// Until a core consumes joydb_*_mapped (Layer B), the matrix stays at identity
// and synthesis prunes it -- binding these is dormant + fleet-safe.
wire  [15:0] joydb_1_mapped, joydb_2_mapped;
wire         db9_remap_cmd;
wire   [5:0] db9_remap_byte_cnt;
wire  [15:0] db9_remap_din;

// [MiSTer-DB9 BEGIN] - DB9 remap factory default (used until Main_MiSTer streams UIO 0xFD)
// Derived from CONF_STR J1, same rule as db9_map.cpp; lets the core work on a stock MiSTer binary.
// DB15:  Flip/Start=A, Flip=B, Coin 1=SELECT, Coin 2=D, Service=E
// DB9MD: Flip/Start=B, Flip=C, Coin 1=MODE, Coin 2=X, Service=START
wire  [35:0] db9_remap_default_db15  = 36'hFFFF87B54;
wire  [35:0] db9_remap_default_db9md = 36'hFFFFA7B65;
// [MiSTer-DB9 END]
joydb joydb (
  .clk             ( CLK_JOY         ),
  .clk_sys         ( clk_sys            ),
  .USER_IN         ( USER_IN         ),
  .OSD_STATUS          ( OSD_STATUS          ),
  .snac_active         ( snac_active         ),
  .mt32_primary_active ( mt32_primary_active ),
  .joy_type        ( joy_type        ),
  .joy_2p          ( joy_2p          ),
  .saturn_unlocked ( saturn_unlocked ),
  .USER_OUT_DRIVE  ( USER_OUT_DRIVE  ),
  .USER_PP_DRIVE   ( USER_PP_DRIVE   ),
  .USER_OSD        ( USER_OSD        ),
  .joydb_1         ( joydb_1         ),
  .joydb_2         ( joydb_2         ),
  .joydb_1ena      ( joydb_1ena      ),
  .joydb_2ena      ( joydb_2ena      ),
  .remap_cmd       ( db9_remap_cmd      ),
  .remap_byte_cnt  ( db9_remap_byte_cnt ),
  .remap_din       ( db9_remap_din      ),
  .remap_default_db15  ( db9_remap_default_db15  ),
  .remap_default_db9md ( db9_remap_default_db9md ),
  .joydb_1_mapped  ( joydb_1_mapped     ),
  .joydb_2_mapped  ( joydb_2_mapped     ),
  .joy_raw         ( joy_raw_payload )
);

assign USER_OUT = USER_OUT_DRIVE;
// [MiSTer-DB9 END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: rename USB joystick wires
wire [31:0] joystick_0_USB, joystick_1_USB;
// [MiSTer-DB9 END]

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.joystick_0(joystick_0_USB),
	.joystick_1(joystick_1_USB),
	.status(status),
	// H1 = "hide while set": hide the three CRT Adjust amounts whenever the
	// stage is not actually doing anything.
	.status_menumask({14'd0, ~crt_enable, 1'b0}),

	.ps2_key(ps2_key),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index),
	.ioctl_din(ioctl_din),
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joy_raw
	.joy_raw(joy_raw_payload),
	// programmable remap matrix selector load (UIO_DB9_MAP 0xFD)
	.db9_remap_cmd(db9_remap_cmd),
	.db9_remap_byte_cnt(db9_remap_byte_cnt),
	.db9_remap_din(db9_remap_din),
	// [MiSTer-DB9 END]
	// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
	.saturn_unlocked(saturn_unlocked)
	// [MiSTer-DB9-Pro END]
);

// [MiSTer-DB9-Pro BEGIN] - DB controllers muted while OSD is open
// joydb_*_mapped is already in MiSTer-standard J1 order, identical to the USB
// joystick word, so it replaces the USB input as-is.
assign joystick_0 = joydb_1ena ? (OSD_STATUS ? 16'b0 : joydb_1_mapped) : joystick_0_USB;
assign joystick_1 = joydb_2ena ? (OSD_STATUS ? 16'b0 : joydb_2_mapped) : joydb_1ena ? joystick_0_USB : joystick_1_USB;
// [MiSTer-DB9-Pro END]

///////////////////////   CLOCKS   ///////////////////////////////

// PLL: 50 MHz input → 57.27272 MHz clk_sys (8 × the native 7.15909 MHz
// Klax pixel clock, 4 × the 14.31818 MHz board master). The core divides
// by 8 for ce_pix/ce_cpu, so video timing and the 68000 run at the exact
// native rates.
wire clk_sys;
pll pll
(
	.refclk(CLK_50M),
	.rst(1'b0),
	.outclk_0(clk_sys)
);

wire reset = RESET | status[0] | buttons[1];

reg [7:0] kbd1 = 8'h00;
reg [7:0] kbd2 = 8'h00;
reg       kbd_coin1 = 1'b0;
reg       kbd_coin2 = 1'b0;
reg       kbd_service = 1'b0;

wire ps2_pressed = ps2_key[9];

always @(posedge clk_sys) begin
	reg old_ps2_state;

	old_ps2_state <= ps2_key[10];
	if (old_ps2_state != ps2_key[10]) begin
		case (ps2_key[8:0])
			9'h175: kbd1[7] <= ps2_pressed; // P1 up
			9'h172: kbd1[6] <= ps2_pressed; // P1 down
			9'h16B: kbd1[5] <= ps2_pressed; // P1 left
			9'h174: kbd1[4] <= ps2_pressed; // P1 right
			9'h014: kbd1[0] <= ps2_pressed; // P1 button 1

			9'h02D: kbd2[7] <= ps2_pressed; // P2 up
			9'h02B: kbd2[6] <= ps2_pressed; // P2 down
			9'h023: kbd2[5] <= ps2_pressed; // P2 left
			9'h034: kbd2[4] <= ps2_pressed; // P2 right
			9'h01C: kbd2[0] <= ps2_pressed; // P2 button 1

			9'h02E: kbd_coin1   <= ps2_pressed; // 5
			9'h036: kbd_coin2   <= ps2_pressed; // 6
			9'h006: kbd_service <= ps2_pressed; // F2
			default: begin end
		endcase
	end
end

// Control panel, per the Klax Universal Kit manual (Table 1-3, JAMMA Wire
// Connections, p1-5):
//
//   U  RT BUTTON 1  "Connect this wire OR the one at pin Z to the N.O.
//                    terminals of the right button switchES."
//   Z  RT BUTTON 2  "Connect this wire OR the one at pin U to the N.O.
//                    terminals of the right button switchES."
//
// So the cabinet really does have TWO Flip buttons per player - one either
// side of the joystick - but they are wired in PARALLEL onto a single game
// input. JAMMA "Button 2" is an alternative pin, not a second function, which
// is why MAME declares only BUTTON1 per player. Both are exposed here and
// OR'd together so a two-button pad reproduces the panel exactly.
//
// There is NO start input: the manual's right-player rows are COIN, BUTTON 1,
// four joystick directions and BUTTON 2, and the neighbouring pins are "Not
// used". The button collars read "start/flip" (manual p1-3, "Four start/flip
// button collars") because the one button both starts the game and flips
// tiles - "Add Coins to Start", then Flip begins play.
wire       p1_coin1 = joystick_0[6] | kbd_coin1;
wire       p1_coin2 = joystick_0[7] | joystick_1[6] | kbd_coin2;
wire       p1_button1 = joystick_0[4] | joystick_0[5] | kbd1[0];
wire       p1_right = joystick_0[0] | kbd1[4];
wire       p1_left = joystick_0[1] | kbd1[5];
wire       p1_down = joystick_0[2] | kbd1[6];
wire       p1_up = joystick_0[3] | kbd1[7];
wire       p2_button1 = joystick_1[4] | joystick_1[5] | kbd2[0];
wire       p2_right = joystick_1[0] | kbd2[4];
wire       p2_left = joystick_1[1] | kbd2[5];
wire       p2_down = joystick_1[2] | kbd2[6];
wire       p2_up = joystick_1[3] | kbd2[7];
// Self-test control. On the real cabinet this is a TOGGLE switch on the
// utility panel, not a pushbutton - you flip it, the board enters self-test
// and stays there, and you flip it back. A momentary gamepad button is the
// wrong model for that, so the mapped button (and F2) TOGGLE a latch instead
// of asserting only while held. The OSD item stays a plain override.
//
// Leaving self-test resetting the game is authentic, not a defect: the test
// code stops petting the watchdog, the LS197 @5B bites and reboots the board.
// MAME cannot reproduce that at all - its Klax watchdog is instantiated but
// never enabled, so MAME gets stuck in self-test forever (verified: frames
// 699-1900 byte-identical).
wire       service_btn = joystick_0[8] | joystick_1[8] | kbd_service;
reg        service_btn_d = 1'b0;
reg        service_toggle = 1'b0;
always @(posedge clk_sys) begin
	service_btn_d <= service_btn;
	if (service_btn && !service_btn_d) service_toggle <= ~service_toggle;
end
wire       service = status[11] | service_toggle;

///////////////////////   CORE   ///////////////////////////////

wire ce_pix;
wire HBlank;
wire HSync;
wire VBlank;
wire VSync;
wire [7:0] core_r, core_g, core_b;
wire [15:0] core_audio_l;
wire [15:0] core_audio_r;

klax_top_mister #(
	.ENABLE_DDRAM_TILE_ROM(KLAX_ENABLE_DDRAM_TILE_ROM)
) u_core (
	.clk_sys        (clk_sys),
	.reset          (reset),

	// ROM download interface
	.ioctl_download (ioctl_download),
	.ioctl_index    (ioctl_index),
	.ioctl_wr       (ioctl_wr),
	.ioctl_addr     (ioctl_addr),
	.ioctl_dout     (ioctl_dout),
	.ioctl_wait     (ioctl_wait),

	// EEPROM NVRAM save interface
	.ioctl_upload       (ioctl_upload),
	.ioctl_din          (ioctl_din),
	.ioctl_upload_req   (ioctl_upload_req),
	.ioctl_upload_index (ioctl_upload_index),

	// DDRAM tile ROM storage
	.DDRAM_CLK          (DDRAM_CLK),
	.DDRAM_BUSY         (DDRAM_BUSY),
	.DDRAM_BURSTCNT     (DDRAM_BURSTCNT),
	.DDRAM_ADDR         (DDRAM_ADDR),
	.DDRAM_DOUT         (DDRAM_DOUT),
	.DDRAM_DOUT_READY   (DDRAM_DOUT_READY),
	.DDRAM_RD           (DDRAM_RD),
	.DDRAM_DIN          (DDRAM_DIN),
	.DDRAM_BE           (DDRAM_BE),
	.DDRAM_WE           (DDRAM_WE),

	// User controls
	.p1_coin1       (p1_coin1),
	.p1_coin2       (p1_coin2),
	.p1_button1     (p1_button1),
	.p1_right       (p1_right),
	.p1_left        (p1_left),
	.p1_down        (p1_down),
	.p1_up          (p1_up),
	.p2_button1     (p2_button1),
	.p2_right       (p2_right),
	.p2_left        (p2_left),
	.p2_down        (p2_down),
	.p2_up          (p2_up),
	.service        (service),

	// Video outputs
	.ce_pix         (ce_pix),
	.HBlank         (HBlank),
	.HSync          (HSync),
	.VBlank         (VBlank),
	.VSync          (VSync),
	.VGA_R          (core_r),
	.VGA_G          (core_g),
	.VGA_B          (core_b),
	.audio_l        (core_audio_l),
	.audio_r        (core_audio_r)
);

///////////////////////   VIDEO OUT   ///////////////////////////

assign CLK_VIDEO = clk_sys;

assign AUDIO_L = core_audio_l;
assign AUDIO_R = core_audio_r;

// CRT Adjust (H-Size / H-Position / V-Shift): rmonic79's crt_adjust engine
// with this core's glue. Output-side only: Off is a true bypass, so a default
// OSD leaves the video bit-identical to the hardware-validated build. See
// rtl/klax_video_adjust.v for what each control does and where it bites.
wire adj_de_in;
klax_video_adjust u_video_adjust (
	.clk_sys    (clk_sys),
	.ce_pix     (ce_pix),

	.core_r     (core_r),
	.core_g     (core_g),
	.core_b     (core_b),
	.HSync      (HSync),
	.VSync      (VSync),
	.HBlank     (HBlank),
	.VBlank     (VBlank),

	.osd_on     (crt_enable),
	.osd_hsize  (status[18:14]),
	.osd_hpos   (status[25:19]),
	.osd_vshift (status[31:26]),

	.vga_r      (VGA_R),
	.vga_g      (VGA_G),
	.vga_b      (VGA_B),
	.vga_hs     (VGA_HS),
	.vga_vs     (VGA_VS),
	.ce_pixel   (CE_PIXEL),
	.de_in      (adj_de_in)
);

// Scale (status[2:1]) picks the video_freak integer-scaling mode:
//   0 Normal, 1 V-Integer, 4 HV-Integer, 2 HV-Integer- (narrower).
// VGA_VS feeds the UNSHIFTED VSync so the line/frame measurement behind the
// aspect maths is unaffected by CRT V-Shift. Cropping is unused.
video_freak video_freak
(
	.CLK_VIDEO(clk_sys),
	.CE_PIXEL(CE_PIXEL),
	.VGA_VS(VSync),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),
	.VGA_DE_IN(adj_de_in),
	.ARX(arx),
	.ARY(ary),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE((status[2:1] == 2'd0) ? 3'd0 :   // Normal
	       (status[2:1] == 2'd1) ? 3'd1 :   // V-Integer
	       (status[2:1] == 2'd2) ? 3'd4 :   // HV-Integer
	                               3'd2)    // Narrower HV-Integer
);

///////////////////////   STATUS LED   //////////////////////////

reg  [26:0] act_cnt;
always @(posedge clk_sys) act_cnt <= act_cnt + 1'd1;
assign LED_USER = act_cnt[26] ? act_cnt[25:18] > act_cnt[7:0] : act_cnt[25:18] <= act_cnt[7:0];

endmodule
