//============================================================================
// Klax Input Ports
//
// Idle input values for the original Klax memory map. Most controls are
// active-low and therefore read as 1 when unpressed. P1 bit 11 is the
// active-high screen vblank status line from MAME's P1 port definition.
//
// MAME reference: klax.cpp input_ports, P1/P2 definitions.
//============================================================================

module klax_inputs (
    input         vblank,
    input         p1_coin1,
    input         p1_coin2,
    input         p1_button1,
    input         p1_right,
    input         p1_left,
    input         p1_down,
    input         p1_up,
    input         p2_button1,
    input         p2_right,
    input         p2_left,
    input         p2_down,
    input         p2_up,
    input         service,
    output [15:0] p1_data,
    output [15:0] p2_data
);

    wire [15:0] p1_idle = 16'hF7FF | (vblank ? 16'h0800 : 16'h0000);
    wire [15:0] p2_idle = 16'hFFFF;

    assign p1_data = p1_idle &
                     (p1_coin1   ? ~16'h0001 : 16'hFFFF) &
                     (p1_coin2   ? ~16'h0002 : 16'hFFFF) &
                     (p1_button1 ? ~16'h0100 : 16'hFFFF) &
                     (p1_right   ? ~16'h1000 : 16'hFFFF) &
                     (p1_left    ? ~16'h2000 : 16'hFFFF) &
                     (p1_down    ? ~16'h4000 : 16'hFFFF) &
                     (p1_up      ? ~16'h8000 : 16'hFFFF);

    assign p2_data = p2_idle &
                     (p2_button1 ? ~16'h0100 : 16'hFFFF) &
                     (service    ? ~16'h0800 : 16'hFFFF) &
                     (p2_right   ? ~16'h1000 : 16'hFFFF) &
                     (p2_left    ? ~16'h2000 : 16'hFFFF) &
                     (p2_down    ? ~16'h4000 : 16'hFFFF) &
                     (p2_up      ? ~16'h8000 : 16'hFFFF);

endmodule
