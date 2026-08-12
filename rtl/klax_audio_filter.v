//============================================================================
// Klax Analog Audio Output Filter (board-level, Sheet 4)
//
// The MSM6295 @14D drives a 6779.44 Hz zero-order-hold DAC. Between that pin
// and the speaker the PCB runs a 5-pole active low-pass built from two
// LM324 sections (18C) plus a passive interstage network. This module is the
// digital equivalent of that analog chain, so the core reproduces the
// board's actual audio bandwidth and its anti-imaging behaviour instead of
// emitting the raw DAC staircase.
//
// SILICON REFERENCE - Klax schematic package, Figure 5-1 Sheet 4 of 9
// (the sheet spans two pages: MSM6295 + first stage, then the rest):
//
//   DAO(36) -> C49 .1uF -> R73 1M --+-- [R72 1M || C48 180pF] --+
//                                   |                            |
//                                   +--(-)6 \                    |
//                            AUDMP --(+)5   >18C LM324 --7-------+--> stage A
//                                          /
//   stage A -> R56 20K --+-- R55 20K --+-- R71 20K --+--(+)3 \
//                        |             |             |        >18C --1--+
//                     C43 .0068uF   C51 .033uF    C44 270pF  (-)2 ------+
//                        |             |             |        (unity buffer)
//                      AUDMP           +--> buffer out       AUDMP
//
//   buffer out -> R62 20K --+-- [R63 47K || C46 270pF] --+
//                 AUX1 R60 -|                            |
//                 AUX2 R61 -+--(-)13 \                   |
//                     AUDMP ---(+)12  >18C LM324 --14-----+--> R28 volume
//                                    /                         -> TDA2030
//
// AUDMP is the mid-supply analog reference (R57/R59 1.5K divider, C42/C45
// bypassed) = AC ground. AUX1/AUX2 are the unstuffed AUX audio-board inputs.
//
// The response that produces (DC-normalised):
//     stage A pole      884.2 Hz   (R72*C48), gain -1
//     Sallen-Key        f0 2665.9 Hz, but R56/C43 damps the bare Q of 5.53
//                       away entirely - the composite is monotonic
//     stage C pole    12541.8 Hz   (R63*C46), gain -2.35
//   => -3 dB at 790 Hz, -22.5 dB at the 3389.7 Hz Nyquist point,
//      -49.6 dB at the 6779.4 Hz first image. In-band DC gain normalised
//      to exactly 1.0 here; the board's +7.42 dB is absorbed by the volume
//      pot R28 and the TDA2030, which are flat in band.
//
// Deliberately NOT modelled (all sub-audible or user-set):
//   - the C49/R73 1.59 Hz input high-pass, the TDA2030 C31 7.2 Hz gain
//     corner, and the C34/8ohm 42 Hz speaker coupling pole
//   - R28 volume pot position and the TDA2030's flat 21x power gain
//
// IMPLEMENTATION
//   3 cascaded Direct-Form-I biquads (bilinear transform of the 5th-order
//   analog transfer function, prewarp-free, no frequency correction needed
//   below 5 kHz).
//   Each section is normalised to unity DC gain so no intermediate node
//   exceeds full scale (verified: cumulative peak is 0.00 dB).
//
//   Sample rate = 12x the OKI sample rate = 81353.3 Hz, derived by dividing
//   the chip's own 894886.3 Hz clock enable by 11. Running well above the
//   6779.4 Hz DAC rate is what lets the model reproduce image rejection:
//   the OKI holds its sample between ticks, so this filter sees the same
//   staircase the real op-amps see.
//
//   Arithmetic is a sequential single-multiplier engine: 3 phases per tap,
//   5 taps + 1 finalise per section = 48 clks per output sample out of the
//   ~704 available at clk_sys. Coefficients are Q2.22, state is Q16.10.
//   Verified bit-exact against a golden model that re-derives itself from
//   the component values above and asserts that the coefficient literals
//   below match that derivation.
//============================================================================

`default_nettype none

module klax_audio_filter #(
    // oki_clk_en (894886.3 Hz) / OKI_CE_DIV = filter rate.
    // 132 (the chip's sample divisor) / 11 = 12x oversampling.
    parameter OKI_CE_DIV = 11
) (
    input  wire                clk,
    input  wire                reset,
    input  wire                oki_clk_en,   // 894886.3 Hz from klax_oki6295
    input  wire signed [15:0]  sample_in,
    output reg  signed [15:0]  sample_out
);

    localparam COEF_W = 25;   // Q2.22 signed
    localparam COEF_F = 22;
    localparam DATA_W = 26;   // Q16.10 signed
    localparam DATA_F = 10;
    localparam ACC_W  = 56;

    // Explicitly-signed constants. Verilog makes a whole expression unsigned
    // if any operand is unsigned, which has bitten this project twice, so
    // these are written out rather than built with shifts at the use site.
    localparam signed [ACC_W-1:0]  RND_COEF = 56'sd2097152;    // 1 << (COEF_F-1)
    localparam signed [ACC_W-1:0]  SAT_HI   = 56'sd33554431;   // (1 << (DATA_W-1)) - 1
    localparam signed [ACC_W-1:0]  SAT_LO   = -56'sd33554432;  // -(1 << (DATA_W-1))
    localparam signed [DATA_W:0]   RND_DATA_EXT = 27'sd512;    // 1 << (DATA_F-1)
    localparam signed [DATA_W:0]   OUT_HI_EXT   = 27'sd32767;
    localparam signed [DATA_W:0]   OUT_LO_EXT   = -27'sd32768;
    // Written as hex, not -16'sd32768: decimal 32768 does not fit a 16-bit
    // SIGNED literal, so `-16'sd32768` wraps to 0x8000 and the unary minus
    // overflows it back again. That happens to land on the intended value,
    // but it is accidental and Quartus flags it (Warning 10259).
    localparam signed [15:0]       OUT_SAT_HI   = 16'sh7FFF;   // +32767
    localparam signed [15:0]       OUT_SAT_LO   = 16'sh8000;   // -32768

    // ---- Coefficients: [b0, b1, b2, -a1, -a2] per section, Q2.22 ----
    // Machine-generated from the component values in the header (bilinear
    // transform, each section normalised to unity DC gain). Do not hand-edit.
    function signed [COEF_W-1:0] coef;
        input [1:0] sec;
        input [2:0] tap;
        begin
            case ({sec, tap})
                5'd0:  coef = 25'sd684282;    // sec0 b0
                5'd1:  coef = 25'sd1368565;   // sec0 b1
                5'd2:  coef = 25'sd684282;    // sec0 b2
                5'd3:  coef = 25'sd1457175;   // sec0 -a1
                5'd4:  coef = 25'sd0;         // sec0 -a2
                5'd8:  coef = 25'sd7079;      // sec1 b0
                5'd9:  coef = 25'sd14158;     // sec1 b1
                5'd10: coef = 25'sd7079;      // sec1 b2
                5'd11: coef = 25'sd7682831;   // sec1 -a1
                5'd12: coef = -25'sd3516843;  // sec1 -a2
                5'd16: coef = 25'sd70095;     // sec2 b0
                5'd17: coef = 25'sd70095;     // sec2 b1
                5'd18: coef = 25'sd0;         // sec2 b2
                5'd19: coef = 25'sd7813740;   // sec2 -a1
                5'd20: coef = -25'sd3759625;  // sec2 -a2
                default: coef = 25'sd0;
            endcase
        end
    endfunction

    // ---- Section state ----
    reg signed [DATA_W-1:0] x1 [0:2];
    reg signed [DATA_W-1:0] x2 [0:2];
    reg signed [DATA_W-1:0] y1 [0:2];
    reg signed [DATA_W-1:0] y2 [0:2];
    reg signed [DATA_W-1:0] v;          // running section input / output

    // ---- Rate divider ----
    reg [3:0] ce_ctr;
    wire      filt_ce = oki_clk_en && (ce_ctr == OKI_CE_DIV - 1);

    // ---- Sequencer ----
    reg        busy;
    reg [1:0]  sec;
    reg [2:0]  tap;     // 0..4 = MAC taps, 5 = finalise
    reg [1:0]  phase;   // 0 = latch operands, 1 = multiply, 2 = accumulate

    reg signed [ACC_W-1:0]  acc;
    reg signed [COEF_W-1:0] mul_a;
    reg signed [DATA_W-1:0] mul_b;
    reg signed [COEF_W+DATA_W-1:0] prod;

    // Operand select for the current tap.
    reg signed [DATA_W-1:0] operand;
    always @(*) begin
        case (tap)
            3'd0:    operand = v;
            3'd1:    operand = x1[sec];
            3'd2:    operand = x2[sec];
            3'd3:    operand = y1[sec];
            default: operand = y2[sec];
        endcase
    end

    // Round-to-nearest then saturate to DATA_W (matches the golden model).
    wire signed [ACC_W-1:0]  acc_rnd = (acc + RND_COEF) >>> COEF_F;
    wire signed [DATA_W-1:0] sec_out = (acc_rnd > SAT_HI) ? SAT_HI[DATA_W-1:0] :
                                       (acc_rnd < SAT_LO) ? SAT_LO[DATA_W-1:0] :
                                                            acc_rnd[DATA_W-1:0];

    // Final 16-bit conversion takes the LAST section's result (`sec_out`),
    // not `v` - `v` is only updated by the same finalise cycle that latches
    // sample_out, so reading it here would emit the section-2 input.
    //
    // Done one bit wide so the rounding add cannot wrap: a saturated sec_out
    // is 2**(DATA_W-1)-1 and adding RND_DATA_EXT overflows DATA_W bits.
    // The golden model uses arbitrary precision, so wrapping here would be a
    // silent divergence at exactly the inputs least likely to be tested.
    wire signed [DATA_W:0] sec_out_ext = {sec_out[DATA_W-1], sec_out};
    wire signed [DATA_W:0] out_rnd     = (sec_out_ext + RND_DATA_EXT) >>> DATA_F;
    wire signed [15:0]     out_sat     = (out_rnd > OUT_HI_EXT) ? OUT_SAT_HI :
                                         (out_rnd < OUT_LO_EXT) ? OUT_SAT_LO :
                                                                  out_rnd[15:0];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ce_ctr     <= 4'd0;
            busy       <= 1'b0;
            sec        <= 2'd0;
            tap        <= 3'd0;
            phase      <= 2'd0;
            acc        <= {ACC_W{1'b0}};
            v          <= {DATA_W{1'b0}};
            prod       <= {(COEF_W + DATA_W){1'b0}};
            mul_a      <= {COEF_W{1'b0}};
            mul_b      <= {DATA_W{1'b0}};
            sample_out <= 16'sd0;
            // Unrolled rather than a for-loop: a module-level `integer` loop
            // variable assigned inside a clocked block makes Quartus report a
            // latch inference for it (Warning 10240). Only 3 sections.
            x1[0] <= {DATA_W{1'b0}};  x2[0] <= {DATA_W{1'b0}};
            y1[0] <= {DATA_W{1'b0}};  y2[0] <= {DATA_W{1'b0}};
            x1[1] <= {DATA_W{1'b0}};  x2[1] <= {DATA_W{1'b0}};
            y1[1] <= {DATA_W{1'b0}};  y2[1] <= {DATA_W{1'b0}};
            x1[2] <= {DATA_W{1'b0}};  x2[2] <= {DATA_W{1'b0}};
            y1[2] <= {DATA_W{1'b0}};  y2[2] <= {DATA_W{1'b0}};
        end else begin
            if (oki_clk_en)
                ce_ctr <= (ce_ctr == OKI_CE_DIV - 1) ? 4'd0 : ce_ctr + 4'd1;

            if (filt_ce && !busy) begin
                // Latch the held DAC sample and start the cascade.
                busy  <= 1'b1;
                sec   <= 2'd0;
                tap   <= 3'd0;
                phase <= 2'd0;
                acc   <= {ACC_W{1'b0}};
                v     <= $signed({sample_in, {DATA_F{1'b0}}});
            end else if (busy) begin
                if (tap == 3'd5) begin
                    // Finalise this section: shift its history, pass y on.
                    x2[sec] <= x1[sec];
                    x1[sec] <= v;
                    y2[sec] <= y1[sec];
                    y1[sec] <= sec_out;
                    v       <= sec_out;
                    acc     <= {ACC_W{1'b0}};
                    tap     <= 3'd0;
                    phase   <= 2'd0;
                    if (sec == 2'd2) begin
                        busy       <= 1'b0;
                        sample_out <= out_sat;
                    end else begin
                        sec <= sec + 2'd1;
                    end
                end else begin
                    case (phase)
                        2'd0: begin
                            mul_a <= coef(sec, tap);
                            mul_b <= operand;
                            phase <= 2'd1;
                        end
                        2'd1: begin
                            prod  <= mul_a * mul_b;
                            phase <= 2'd2;
                        end
                        default: begin
                            acc   <= acc + $signed(prod);
                            phase <= 2'd0;
                            tap   <= tap + 3'd1;
                        end
                    endcase
                end
            end
        end
    end

endmodule

`default_nettype wire
