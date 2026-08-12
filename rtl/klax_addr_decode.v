//============================================================================
// Klax Address Decode — silicon ground truth
//
// Transcribed from the DUMPED production decode PAL: GAL16V8 136075-1000 at
// 11C (136075-1000.11c.bin, jedutil) feeding the 74F138 at 11B
// (schematic Sheet 3). The schematic draws a 24-pin PAL20L10 at 11C, but
// Sheet 3 notes "USE 16L8 IF SLAPSTIK IS NOT USED" — production Klax has no
// slapstic, and the 16L8-class GAL at 11C is exactly what MAME dumped. The
// pin mapping was recovered by constraint-solving the dumped equations
// against the known memory map.
//
// Headline hardware facts modeled here (all differ from MAME's klax_map):
//   - MA22/MA23 reach no decoder: the whole map MIRRORS x4 across the 24-bit
//     space (only cpu_addr[21:0] participates below).
//   - The F138 (G2A=/AS) always resolves one Y per bus cycle. Y7 is the
//     "nothing selected" slot; Y4=/STIK is the slapstic select (unstuffed on
//     production boards, 0x058000-0x05FFFF inside the empty ROM2 range).
//     "/PAR" (Y5) is the 0x260000 parallel I/O port (inputs + output latch).
//   - Selects are PAGE-wide, not word-wide: /PAR and /PCM cover their whole
//     64KB page; /WDOG, /VACK (irq ack), /EEPROM and /UNLOCK cover 16KB.
//   - /VRAM (Y3) covers the FULL 128KB 0x3E0000-0x3FFFFF. MA16 sub-selects
//     color RAM (0x3E pages, /CTIME path, F257 @5N) vs the single 8Kx16
//     video RAM (2X/2Y). VRA14=MA14 is the video-RAM chip select and MA15 is
//     unused, so the 16KB RAM appears at xx0000-3FFF and mirrors at
//     xx8000-BFFF; xx4000-7FFF and xxC000-FFFF are open (no chip).
//     The pf/slip/pf_ext/mob/work "regions" below are the RTL's BRAM split of
//     that one linear RAM; their union is exactly the 16KB image.
//   - The color RAM CPU image (1KB used) mirrors every 2KB across the whole
//     0x3E page (only MA10..MA1 reach it).
//   - The 2KB EEPROM (A10..A0 = MA11..MA1) mirrors x4 inside its 16KB window.
//
// Everything not selected is open bus: the DTACK generator still completes
// the cycle (see klax_cpu_bus) and reads return 0xFFFF.
//
// Residual ambiguity (does not affect any game-exercised address): the GAL
// input pins carrying {MA15,MA14,MA13} and the {MA18,MA17} order are known
// only up to the 8 consistent permutations; the natural 20L10-order solution
// is used.
//============================================================================

module klax_addr_decode (
    input  [23:0] cpu_addr,     // 68000 address bus (A23..A0)

    // Region select outputs (active high)
    output        cs_rom,       // 0x000000-0x03FFFF  Program ROM (/ROM0+/ROM1)
    output        cs_eeprom,    // 0x0E0000-0x0E3FFF  EEPROM (2KB, mirrors x4)
    output        cs_eeprom_unlock, // 0x1F0000-0x1F3FFF  EEPROM unlock
    output        cs_input_p1,  // /PAR page, even word: P1 inputs / output latch
    output        cs_input_p2,  // /PAR page, odd word: P2 inputs
    output        cs_oki,       // 0x270000-0x27FFFF  OKI MSM6295 (/PCM page)
    output        cs_watchdog,  // 0x2E0000-0x2E3FFF  Watchdog reset (/WDOG)
    output        cs_irq_ack,   // 0x360000-0x363FFF  Interrupt acknowledge (/VACK)
    output        cs_palette,   // /VRAM window, MA16=0: color RAM (CTIME path)
    output        cs_playfield, // main VRAM image: playfield tilemap words
    output        cs_slip,      // main VRAM image: SLIP words
    output        cs_pf_ext,    // main VRAM image: playfield attribute words
    output        cs_mob,       // main VRAM image: motion object words
    output        cs_work_ram,  // main VRAM image: work RAM words
    output        cs_vram_window // full /VRAM select (Y3) — DTACK wait class
);

    // MA22/MA23 are not wired to the decode PAL: x4 mirror.
    wire [21:0] a = cpu_addr[21:0];

    // --- 11C GAL dedicated outputs (single product terms) ---
    wire rom0 = (a[21:17] == 5'b00000);          // /ROM0: pair 3N/1N
    wire rom1 = (a[21:17] == 5'b00001);          // /ROM1: pair 3K/1K
    // /ROM2 (a[21:17]==5'b00010) selects the UNSTUFFED 3E/1E sockets and
    // /AUX (a[21:14]==8'b10100000) the unstuffed aux connector: open bus,
    // so neither needs an output — the data mux default covers them.
    assign cs_irq_ack = (a[21:14] == 8'b11011000); // /VACK 0x360000-0x363FFF

    // --- 11C GAL F138 select encoder (o12=SELA, o13=SELB, o14=SELC) ---
    // Active-low sums straight from the dump; pin low when a term matches.
    wire sela_lo = (a[21:16] == 6'b100111)       // PCM page
                 | (a[21:15] == 7'b0001011)      // slapstic window (with SELB)
                 | (a[20:14] == 7'b0111000);     // EEPROM + WDOG windows
    wire selb_lo = (a[21:16] == 6'b100110)       // PAR page
                 | (a[21:15] == 7'b0001011)      // slapstic window
                 | (a[21:14] == 8'b01111100)     // UNLOCK window
                 | (a[21:14] == 8'b00111000);    // EEPROM window
    wire selc_lo = (a[21:17] == 5'b11111)        // VRAM 128KB window
                 | (a[20:14] == 7'b1111100)      // UNLOCK + VRAM (redundant)
                 | (a[20:14] == 7'b0111000);     // EEPROM + WDOG (redundant)

    // 74F138 @11B: Y = {SELC,SELB,SELA} pin levels.
    // Y0=/EEPROM Y1=/UNLOCK Y2=/WDOG Y3=/VRAM Y4=/STIK Y5=/PAR Y6=/PCM Y7=nc
    wire [2:0] sel_y = {~selc_lo, ~selb_lo, ~sela_lo};

    assign cs_eeprom        = (sel_y == 3'd0);
    assign cs_eeprom_unlock = (sel_y == 3'd1);
    assign cs_watchdog      = (sel_y == 3'd2);
    assign cs_vram_window   = (sel_y == 3'd3);
    wire   cs_par           = (sel_y == 3'd5);   // parallel I/O port page
    assign cs_oki           = (sel_y == 3'd6);
    // sel_y == 3'd4 (/STIK, slapstic) is unstuffed: open bus.

    assign cs_rom = rom0 | rom1;

    // /PAR sub-decode: only MA1 distinguishes the two switch buffers, so
    // P1/P2 repeat across the whole page (0x260000+4n / 0x260002+4n).
    assign cs_input_p1 = cs_par && !cpu_addr[1];
    assign cs_input_p2 = cs_par &&  cpu_addr[1];

    // /VRAM sub-decode (video side, Sheets 4/5/8):
    //   MA16=0 -> color RAM (/CTIME = MA16 during the CPU slot, F257 @5N)
    //   MA16=1 -> 8Kx16 video RAM; chip select VRA14=MA14, MA15 unused.
    assign cs_palette = cs_vram_window && !cpu_addr[16];
    wire   vram_main  = cs_vram_window && cpu_addr[16] && !cpu_addr[14];

    // RTL BRAM split of the single linear 8Kx16 video RAM (MA13..MA1).
    // The union of these five terms is exactly vram_main.
    assign cs_playfield = vram_main && (cpu_addr[13:12] == 2'b00) &&
                          (cpu_addr[11:7] != 5'h1F);
    assign cs_slip      = vram_main && (cpu_addr[13:12] == 2'b00) &&
                          (cpu_addr[11:7] == 5'h1F);
    assign cs_pf_ext    = vram_main && (cpu_addr[13:12] == 2'b01);
    assign cs_mob       = vram_main && (cpu_addr[13:11] == 3'b100);
    assign cs_work_ram  = vram_main && cpu_addr[13] &&
                          (cpu_addr[12] || cpu_addr[11]);

endmodule
