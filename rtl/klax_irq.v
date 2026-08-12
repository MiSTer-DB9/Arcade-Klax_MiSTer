//============================================================================
// Klax IRQ Controller — silicon ground truth
//
// The 68000 receives only IRQ4. Schematic Sheets 5 and 2: an LS74
// at 5K with D = P = +5V, clocked by CK = VBLANK | ~32V (LS32 @10N + LS04
// @9N), cleared by /VACK (the 0x360000 write). /Q = /VINT drives IPL2
// through the LS08 @7B (the other input is the pulled-up /SINT from the
// unstuffed AUX connector); IPL1/IPL0 are tied high, so /VINT is IRQ level 4.
//
// With D tied high, every RISING edge of (VBLANK | ~32V) sets the latch:
//   - 32V (V-counter bit 5) falling edges outside VBLANK: scanlines 64, 128
//     and 192 (the 32V fall at 256 is masked because VBLANK already holds CK
//     high),
//   - the VBLANK rising edge at scanline 240 (32V is high there, so CK was
//     low and rises with VBLANK).
// There is NO edge at scanline 0: CK stays high from the VBLANK rise at 240
// through scanline 31 (~32V is already high when VBLANK falls).
//
// MAME divergence (intentional, silicon over MAME): MAME's scanline timer
// (klax.cpp scanline_update, every 32 lines, "(scanline & 32) == 0 &&
// !vblank") fires an EXTRA IRQ4 at scanline 0 that the real circuit cannot
// generate. Silicon IRQs per frame: 64, 128, 192, 240 (vblank).
//
// The /VACK clear is level-acting while the write is in progress, exactly
// like the LS74's async /CLR held low.
//============================================================================

module klax_irq (
    input               clk,
    input               reset,
    input               vblank,
    input       [8:0]   scanline,
    input               irq_ack_wr,

    output      [2:0]   ipl_n
);

    // CK = VBLANK | ~32V  (32V = V-counter bit 5)
    wire irq_ck = vblank | ~scanline[5];

    reg irq_ck_d;
    reg irq4_pending;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            irq_ck_d <= 1'b1;      // matches CK at scanline 0 (~32V = 1)
            irq4_pending <= 1'b0;
        end else begin
            irq_ck_d <= irq_ck;

            if (irq_ack_wr)
                irq4_pending <= 1'b0;              // /VACK async clear
            else if (irq_ck && !irq_ck_d)
                irq4_pending <= 1'b1;              // D=1 clocked in
        end
    end

    // 68000 IPL lines are active low. IRQ4 is IPL2:0 = 011.
    assign ipl_n = irq4_pending ? 3'b011 : 3'b111;

endmodule
