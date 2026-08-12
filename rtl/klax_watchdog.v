//============================================================================
// Klax Watchdog Timer — silicon ground truth
//
// Schematic Sheet 2: an LS197 4-bit counter at 5B clocked by VBLANK
// (QA ripples into CK2), with data inputs {D,C,B,A} = {+5V,GND,GND,GND},
// count/load pin driven by /WDOG (the 0x2E0000 write, gated by the WDIS
// service jumper), async /CLR = POWER-ON, and QD wired directly out as
// /SYSRES:
//   - power-on clears the counter to 0: QD=0 -> /SYSRES asserted, and the
//     board is HELD IN RESET for the first 8 VBLANKs (~133 ms),
//   - a watchdog pet (0x2E0000 write) async-loads 8: QD=1, reset released,
//   - unpetted, the counter runs 8..15 and wraps to 0: reset asserts 8
//     frames after the last pet and self-releases 8 frames later as the
//     count passes 8 again.
//
// MAME note: MAME 0.288 instantiates WATCHDOG_TIMER for klax but never calls
// set_time()/set_vblank_count()/watchdog_enable(), so its watchdog stays
// DISABLED and never fires — this module tracks real silicon, not MAME. It
// only ever bites on a genuine CPU hang (the game pets every frame; the 5bl
// bootleg omits the device entirely).
//
// KLAX_WDOG_FAST_BOOT (simulation builds only): starts the counter at 8
// (reset released immediately), skipping the 8-frame power-on hold so the
// existing reset-relative testbench timelines stay short. Post-pet behavior
// is identical either way. The FPGA build never defines it.
//============================================================================

module klax_watchdog (
    input  clk,         // clk_sys
    input  reset,       // external/system reset (active high) = POWER-ON clear
    input  vblank,      // active-high VBLANK from video timing
    input  pet,         // watchdog write strobe (held across the bus cycle)
    output wdog_reset   // active-high watchdog reset request (= NOT QD)
);

`ifdef KLAX_WDOG_FAST_BOOT
    localparam [3:0] POWER_ON_COUNT = 4'd8;  // sim: skip the power-on hold
`else
    localparam [3:0] POWER_ON_COUNT = 4'd0;  // silicon: LS197 cleared
`endif

    reg [3:0] ctr;
    reg       vblank_d;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ctr      <= POWER_ON_COUNT;
            vblank_d <= 1'b0;
        end else begin
            vblank_d <= vblank;
            if (pet)
                ctr <= 4'd8;                  // LS197 async load of {1,0,0,0}
            else if (vblank && !vblank_d)     // one tick per frame
                ctr <= ctr + 4'd1;            // free-running; wraps to bite
        end
    end

    // /SYSRES = QD: reset asserted while the count is 0..7.
    assign wdog_reset = ~ctr[3];

endmodule
