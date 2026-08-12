//============================================================================
// Klax Sprite Row Fetch Adapter
//
// Bridges the 32-bit tile-row fetch interface of klax_motion_objects
// (sprow_req/ack handshake) to the row-wide sprite ROM port on
// klax_ddram_tile_rom (sprite_rd_en/addr -> sprite_rd_row + ready pulse).
//
// Handshake contract (both sides verified against klax_motion_objects):
//   - The requester holds sprow_req high, with stable code/row, until it
//     observes sprow_ack, then drops sprow_req for at least one cycle.
//   - Exactly one sprow_ack pulse is produced per request. After delivering
//     the ack the adapter waits for sprow_req to deassert before accepting
//     a new request, so a still-high "old" req is never mistaken for a new
//     one (re-latching in the ack cycle would skew every subsequent fetch
//     one request behind).
//   - If the requester aborts (sprow_req drops mid-fetch on line advance),
//     the in-flight ROM read is drained and discarded without an ack; a
//     re-raised request is then served fresh from S_IDLE.
//
// Row byte address: code * 32 + row * 4 = {code, row, 2'b00}. The ROM
// returns the full 4-byte row in one transaction, already packed MSB
// (pixel 0 in bits [31:28]), so no byte assembly happens here.
//============================================================================

module klax_sprow_adapter (
    input               clk,
    input               reset,

    // klax_motion_objects tile-row port
    input               sprow_req,
    input       [11:0]  sprow_code,
    input       [2:0]   sprow_row,
    output reg  [31:0]  sprow_data,
    output reg          sprow_ack,

    // klax_ddram_tile_rom sprite row port
    output reg          sprite_rd_en,     // one-cycle request pulse (latched by the ROM)
    output reg  [16:0]  sprite_rd_addr,   // byte address of the row, low 2 bits zero
    input       [31:0]  sprite_rd_row,    // packed-MSB row, pixel 0 = [31:28]
    input               sprite_rd_ready   // one-cycle pulse when sprite_rd_row is valid
);

    localparam S_IDLE  = 2'd0;
    localparam S_WAIT  = 2'd1;
    localparam S_DRAIN = 2'd2;

    reg [1:0] state = S_IDLE;
    reg       cancelled = 1'b0;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= S_IDLE;
            cancelled <= 1'b0;
            sprow_data <= 32'd0;
            sprow_ack <= 1'b0;
            sprite_rd_en <= 1'b0;
            sprite_rd_addr <= 17'd0;
        end else begin
            sprow_ack <= 1'b0;
            sprite_rd_en <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (sprow_req) begin
                        sprite_rd_addr <= {sprow_code, sprow_row, 2'b00};
                        sprite_rd_en <= 1'b1;
                        cancelled <= 1'b0;
                        state <= S_WAIT;
                    end
                end

                S_WAIT: begin
                    // A dropped request means the renderer aborted this
                    // line; the ROM read still completes but is discarded.
                    if (!sprow_req)
                        cancelled <= 1'b1;

                    if (sprite_rd_ready) begin
                        if (sprow_req && !cancelled) begin
                            sprow_data <= sprite_rd_row;
                            sprow_ack <= 1'b1;
                            state <= S_DRAIN;
                        end else begin
                            state <= S_IDLE;
                        end
                    end
                end

                S_DRAIN: begin
                    // Wait for the requester to consume the ack and drop
                    // sprow_req. No new request is accepted here, so the
                    // single ack above can never be paired with stale data.
                    if (!sprow_req)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
