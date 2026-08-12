//============================================================================
// Klax DDRAM Graphics ROM Cache
//
// Stores large graphics ROM regions in MiSTer DDRAM instead of M10K:
//   ioctl index 1: 256 KB playfield tile ROM
//   ioctl index 4: 128 KB motion-object sprite ROM
//
// The playfield port preserves the original 1-cycle cached byte interface.
//
// The sprite port serves whole 8-pixel tile rows for the motion-object
// renderer: sprite_rd_en pulses with a row byte address (low 2 bits zero),
// the request is latched until the DDRAM bus is free (playfield tile fills
// keep priority), and one 64-bit DDRAM read returns the full 4-byte row on
// sprite_rd_row with a sprite_rd_ready pulse. Latching the request is
// required for correctness: the requester pulses sprite_rd_en exactly once
// and then waits, so an unlatched request that lost arbitration would be
// dropped and the requester would wait forever.
//============================================================================

module klax_ddram_tile_rom #(
    // MiSTer DDR3 word address. 0x04800000 corresponds to byte address
    // 0x24000000, matching the high DDR region used by MiSTer helpers.
    parameter BASE_ADDR        = 29'h04800000,
    parameter SPRITE_BASE_ADDR = 29'h04808000,
    parameter CACHE_AW         = 12
) (
    input               clk,
    input               reset,

    input               ioctl_download,
    input       [15:0]  ioctl_index,
    input               ioctl_wr,
    input       [26:0]  ioctl_addr,
    input       [7:0]   ioctl_dout,
    output              ioctl_wait,

    input       [17:0]  rd_addr,
    output reg  [7:0]   rd_data,
    output      [15:0]  debug_flags,

    // Playfield prefetch port. `pre_addr` is the same kind of address as
    // rd_addr but for a tile a couple of columns AHEAD of the raster; asking
    // for it here warms the cache line so the demand read above always hits.
    //
    // This exists because the demand path has no handshake and the playfield
    // pipeline cannot stall: on a miss the read below returns 0 and the
    // renderer paints pen 0 of the colour bank. Real silicon cannot do that -
    // the 9W slot machine SCHEDULES two playfield fetch slots per tile ahead
    // of display, so the data is always there. This port restores that
    // property. Purely a warming hint: it returns no data and never blocks.
    input               pre_en,
    input       [17:0]  pre_addr,

    input               sprite_rd_en,     // one-cycle row request pulse
    input       [16:0]  sprite_rd_addr,   // row byte address, low 2 bits zero
    output reg  [31:0]  sprite_rd_row,    // packed-MSB row, pixel 0 = [31:28]
    output reg          sprite_rd_ready,  // one-cycle pulse when row valid
    output      [15:0]  sprite_debug_flags,

    output              DDRAM_CLK,
    input               DDRAM_BUSY,
    output reg  [7:0]   DDRAM_BURSTCNT,
    output reg  [28:0]  DDRAM_ADDR,
    input       [63:0]  DDRAM_DOUT,
    input               DDRAM_DOUT_READY,
    output reg          DDRAM_RD,
    output reg  [63:0]  DDRAM_DIN,
    output reg  [7:0]   DDRAM_BE,
    output reg          DDRAM_WE
);

    // 3-way set-associative: NUM_SETS sets x 3 ways (SET_AW = CACHE_AW - 1, so
    // CACHE_AW=12 -> 2048 sets). Associativity matters because a conflict miss
    // returns rd_data=0 to a demand port that cannot stall, painting wrong
    // pixels. The measured working set maps at most 3 words to any set, so 3
    // ways hold all of them and round-robin fill suffices (once a set's <=3
    // words are cached, no eviction occurs during display). 3 ways at 2048
    // sets = 48 M10K blocks vs 64 for an over-provisioned 2-way @ 4096.
    localparam SET_AW   = CACHE_AW - 1;
    localparam NUM_SETS = 32'd1 << SET_AW;
    localparam TAG_W    = 15 - SET_AW;
    localparam REGION_TILE   = 1'b0;
    localparam REGION_SPRITE = 1'b1;

    assign DDRAM_CLK = clk;

    wire tile_download = ioctl_download && (ioctl_index == 16'd1);
    wire sprite_download = ioctl_download && (ioctl_index == 16'd4);
    wire gfx_download = tile_download || sprite_download;
    wire tile_wr = tile_download && ioctl_wr;
    wire sprite_wr = sprite_download && ioctl_wr;
    wire gfx_wr = tile_wr || sprite_wr;

    wire [17:0] wr_byte_addr;
    klax_flat_rom_addr u_flat_addr (
        .flat_addr (ioctl_addr),
        .byte_addr (wr_byte_addr)
    );

    wire [14:0] wr_word_addr = wr_byte_addr[17:3];
    wire [2:0]  wr_byte_sel  = wr_byte_addr[2:0];

    // Cache metadata is {valid, tag} packed into ONE M10K-backed word per way.
    //
    // The valid bit used to be its own `reg way0_valid [0:NUM_SETS-1]` array,
    // which forced it into flip-flops: a reset that clears all NUM_SETS
    // entries in a single cycle cannot be done to block RAM. Registers made
    // every lookup a NUM_SETS:1 multiplexer, and adding the prefetch port's
    // second lookup duplicated all of them - that alone cost ~2000 ALMs and
    // took the design from 92% to 97% utilisation.
    //
    // Packing valid into the tag word puts it in M10K (Quartus replicates for
    // the second read port, as it already did for the tags), and the one-cycle
    // reset is replaced by the sequential invalidation walk below. `repl`
    // moves to M10K for the same reason.
    localparam META_W = TAG_W + 1;         // {valid, tag}

    (* ramstyle = "M10K" *) reg [63:0]       way0_data [0:NUM_SETS-1];
    (* ramstyle = "M10K" *) reg [63:0]       way1_data [0:NUM_SETS-1];
    (* ramstyle = "M10K" *) reg [63:0]       way2_data [0:NUM_SETS-1];
    (* ramstyle = "M10K" *) reg [META_W-1:0] way0_meta [0:NUM_SETS-1];
    (* ramstyle = "M10K" *) reg [META_W-1:0] way1_meta [0:NUM_SETS-1];
    (* ramstyle = "M10K" *) reg [META_W-1:0] way2_meta [0:NUM_SETS-1];
    (* ramstyle = "M10K" *) reg [1:0]        repl      [0:NUM_SETS-1];

    // Post-reset invalidation: walks every set clearing valid (and seeding
    // repl) before the cache may be used. NUM_SETS clks = ~36 us at 57.27 MHz,
    // inside the watchdog's 8-frame power-on reset hold, so nothing is on
    // screen while it runs.
    reg              cache_ready = 1'b0;
    reg [SET_AW-1:0] inv_addr = {SET_AW{1'b0}};

    reg rd_pending = 1'b0;
    reg rd_cmd_pending = 1'b0;
    reg wr_cmd_pending = 1'b0;
    reg rd_region = REGION_TILE;
    reg wr_region = REGION_TILE;

    assign ioctl_wait = gfx_download &&
                        (DDRAM_BUSY || rd_pending ||
                         rd_cmd_pending || wr_cmd_pending);

    wire [14:0] tile_rd_word_addr = rd_addr[17:3];
    wire [SET_AW-1:0] tile_rd_set = tile_rd_word_addr[SET_AW-1:0];
    wire [TAG_W-1:0]  tile_rd_tag = tile_rd_word_addr[14:SET_AW];

    reg [2:0]        tile_rd_byte_sel_d = 3'd0;
    reg [14:0]       tile_rd_word_addr_d = 15'd0;
    reg [SET_AW-1:0] tile_rd_set_d = {SET_AW{1'b0}};
    reg [TAG_W-1:0]  tile_rd_tag_d = {TAG_W{1'b0}};
    reg [63:0]       w0_data_q = 64'd0, w1_data_q = 64'd0, w2_data_q = 64'd0;
    reg [META_W-1:0] w0_meta_q = {META_W{1'b0}}, w1_meta_q = {META_W{1'b0}},
                     w2_meta_q = {META_W{1'b0}};
    reg [1:0]        repl_q = 2'd0;

    // --- Prefetch lookup ---------------------------------------------------
    // Only the TAG and VALID arrays get a second read port; the prefetch just
    // decides whether a line needs filling and never returns data, so the
    // 64-bit way*_data arrays (which dominate the RAM cost) are untouched.
    wire [14:0]       pre_word_addr = pre_addr[17:3];
    wire [SET_AW-1:0] pre_set = pre_word_addr[SET_AW-1:0];
    wire [TAG_W-1:0]  pre_tag = pre_word_addr[14:SET_AW];

    reg               pre_en_d = 1'b0;
    reg [14:0]        pre_word_addr_d = 15'd0;
    reg [SET_AW-1:0]  pre_set_d = {SET_AW{1'b0}};
    reg [TAG_W-1:0]   pre_tag_d = {TAG_W{1'b0}};
    reg [META_W-1:0]  p0_meta_q = {META_W{1'b0}}, p1_meta_q = {META_W{1'b0}},
                      p2_meta_q = {META_W{1'b0}};
    reg [1:0]         pre_repl_q = 2'd0;

    // Suppress re-issue: pre_addr is stable for the whole 8-clk pixel group,
    // and a fill takes longer than that, so without this the same line would
    // be requested several times and round-robin would burn every way on
    // duplicates of one tag.
    reg               pre_inflight = 1'b0;
    reg [14:0]        pre_inflight_addr = 15'd0;

    // Latched sprite row request: held until the DDRAM bus grants it.
    reg                sprite_req_pending = 1'b0;
    reg [16:0]         sprite_req_addr = 17'd0;
    reg                sprite_fill_half = 1'b0;   // addr[2]: which 32-bit half
    reg                sprite_read_busy = 1'b0;

    reg [SET_AW-1:0]   fill_set = {SET_AW{1'b0}};
    reg [TAG_W-1:0]    fill_tag = {TAG_W{1'b0}};
    reg [1:0]          fill_way = 2'd0;
    reg                fill_region = REGION_TILE;

    reg tile_download_d = 1'b0;
    reg sprite_download_d = 1'b0;

    reg dbg_tile_download_seen = 1'b0;
    reg dbg_tile_wr_seen = 1'b0;
    reg dbg_tile_ddram_we_seen = 1'b0;
    reg dbg_tile_ddram_rd_seen = 1'b0;
    reg dbg_tile_ddram_ready_seen = 1'b0;
    reg dbg_tile_cache_fill_seen = 1'b0;
    reg dbg_tile_cache_hit_seen = 1'b0;
    reg dbg_tile_cache_miss_seen = 1'b0;
    reg dbg_tile_rom_data_nonzero_seen = 1'b0;
    reg dbg_tile_ddram_dout_nonzero_seen = 1'b0;
    reg dbg_tile_ddram_busy_seen = 1'b0;
    reg dbg_tile_dout_nonzero_seen = 1'b0;

    reg dbg_sprite_download_seen = 1'b0;
    reg dbg_sprite_wr_seen = 1'b0;
    reg dbg_sprite_ddram_we_seen = 1'b0;
    reg dbg_sprite_ddram_rd_seen = 1'b0;
    reg dbg_sprite_ddram_ready_seen = 1'b0;
    reg dbg_sprite_cache_fill_seen = 1'b0;
    reg dbg_sprite_cache_hit_seen = 1'b0;
    reg dbg_sprite_cache_miss_seen = 1'b0;
    reg dbg_sprite_rom_data_nonzero_seen = 1'b0;
    reg dbg_sprite_ddram_dout_nonzero_seen = 1'b0;
    reg dbg_sprite_ddram_busy_seen = 1'b0;
    reg dbg_sprite_dout_nonzero_seen = 1'b0;

    wire w0_hit = cache_ready && w0_meta_q[META_W-1] &&
                  (w0_meta_q[TAG_W-1:0] == tile_rd_tag_d);
    wire w1_hit = cache_ready && w1_meta_q[META_W-1] &&
                  (w1_meta_q[TAG_W-1:0] == tile_rd_tag_d);
    wire w2_hit = cache_ready && w2_meta_q[META_W-1] &&
                  (w2_meta_q[TAG_W-1:0] == tile_rd_tag_d);
    wire current_tile_cache_hit = w0_hit || w1_hit || w2_hit;
    wire [63:0] hit_data = w0_hit ? w0_data_q : (w1_hit ? w1_data_q : w2_data_q);

    wire pre_cache_hit =
        (p0_meta_q[META_W-1] && (p0_meta_q[TAG_W-1:0] == pre_tag_d)) ||
        (p1_meta_q[META_W-1] && (p1_meta_q[TAG_W-1:0] == pre_tag_d)) ||
        (p2_meta_q[META_W-1] && (p2_meta_q[TAG_W-1:0] == pre_tag_d));
    wire pre_miss_request = cache_ready && !gfx_download && pre_en_d &&
                            !pre_cache_hit &&
                            !(pre_inflight && (pre_inflight_addr == pre_word_addr_d));

    assign debug_flags = {
        1'b1,
        current_tile_cache_hit,
        rd_pending || rd_cmd_pending,
        dbg_tile_dout_nonzero_seen,
        dbg_tile_ddram_busy_seen,
        dbg_tile_ddram_dout_nonzero_seen,
        dbg_tile_rom_data_nonzero_seen,
        dbg_tile_cache_miss_seen,
        dbg_tile_cache_hit_seen,
        dbg_tile_cache_fill_seen,
        dbg_tile_ddram_ready_seen,
        dbg_tile_ddram_rd_seen,
        dbg_tile_ddram_we_seen,
        dbg_tile_wr_seen,
        dbg_tile_download_seen,
        tile_download
    };

    assign sprite_debug_flags = {
        1'b1,
        sprite_rd_ready,
        sprite_rd_en,
        dbg_sprite_dout_nonzero_seen,
        dbg_sprite_ddram_busy_seen,
        dbg_sprite_ddram_dout_nonzero_seen,
        dbg_sprite_rom_data_nonzero_seen,
        dbg_sprite_cache_miss_seen,
        dbg_sprite_cache_hit_seen,
        dbg_sprite_cache_fill_seen,
        dbg_sprite_ddram_ready_seen,
        dbg_sprite_ddram_rd_seen,
        dbg_sprite_ddram_we_seen,
        dbg_sprite_wr_seen,
        dbg_sprite_download_seen,
        sprite_download
    };

    wire bus_idle = !rd_pending && !rd_cmd_pending && !wr_cmd_pending;
    wire tile_miss_request = cache_ready && !gfx_download &&
                             !current_tile_cache_hit;
    wire sprite_row_request = !gfx_download && sprite_req_pending &&
                              !sprite_read_busy;

    integer i;
    always @(posedge clk) begin
        DDRAM_BURSTCNT <= 8'd1;
        tile_download_d <= tile_download;
        sprite_download_d <= sprite_download;
        sprite_rd_ready <= 1'b0;

        if (tile_download && !tile_download_d) begin
            dbg_tile_download_seen <= 1'b0;
            dbg_tile_wr_seen <= 1'b0;
            dbg_tile_ddram_we_seen <= 1'b0;
            dbg_tile_ddram_rd_seen <= 1'b0;
            dbg_tile_ddram_ready_seen <= 1'b0;
            dbg_tile_cache_fill_seen <= 1'b0;
            dbg_tile_cache_hit_seen <= 1'b0;
            dbg_tile_cache_miss_seen <= 1'b0;
            dbg_tile_rom_data_nonzero_seen <= 1'b0;
            dbg_tile_ddram_dout_nonzero_seen <= 1'b0;
            dbg_tile_ddram_busy_seen <= 1'b0;
            dbg_tile_dout_nonzero_seen <= 1'b0;
        end

        if (sprite_download && !sprite_download_d) begin
            dbg_sprite_download_seen <= 1'b0;
            dbg_sprite_wr_seen <= 1'b0;
            dbg_sprite_ddram_we_seen <= 1'b0;
            dbg_sprite_ddram_rd_seen <= 1'b0;
            dbg_sprite_ddram_ready_seen <= 1'b0;
            dbg_sprite_cache_fill_seen <= 1'b0;
            dbg_sprite_cache_hit_seen <= 1'b0;
            dbg_sprite_cache_miss_seen <= 1'b0;
            dbg_sprite_rom_data_nonzero_seen <= 1'b0;
            dbg_sprite_ddram_dout_nonzero_seen <= 1'b0;
            dbg_sprite_ddram_busy_seen <= 1'b0;
            dbg_sprite_dout_nonzero_seen <= 1'b0;
        end

        if (tile_download)
            dbg_tile_download_seen <= 1'b1;
        if (sprite_download)
            dbg_sprite_download_seen <= 1'b1;
        if (tile_wr)
            dbg_tile_wr_seen <= 1'b1;
        if (sprite_wr)
            dbg_sprite_wr_seen <= 1'b1;
        if (tile_wr && (ioctl_dout != 8'd0))
            dbg_tile_dout_nonzero_seen <= 1'b1;
        if (sprite_wr && (ioctl_dout != 8'd0))
            dbg_sprite_dout_nonzero_seen <= 1'b1;
        if (DDRAM_BUSY) begin
            if (wr_region == REGION_TILE || rd_region == REGION_TILE)
                dbg_tile_ddram_busy_seen <= 1'b1;
            if (wr_region == REGION_SPRITE || rd_region == REGION_SPRITE)
                dbg_sprite_ddram_busy_seen <= 1'b1;
        end

        if (reset) begin
            rd_data <= 8'd0;
            sprite_rd_row <= 32'd0;
            rd_pending <= 1'b0;
            rd_cmd_pending <= 1'b0;
            wr_cmd_pending <= 1'b0;
            rd_region <= REGION_TILE;
            wr_region <= REGION_TILE;
            DDRAM_RD <= 1'b0;
            DDRAM_WE <= 1'b0;
            sprite_req_pending <= 1'b0;
            sprite_req_addr <= 17'd0;
            sprite_fill_half <= 1'b0;
            sprite_read_busy <= 1'b0;
            sprite_rd_ready <= 1'b0;
            DDRAM_DIN <= 64'd0;
            DDRAM_BE <= 8'd0;
            // No NUM_SETS-wide clear here any more: that is what forced the
            // valid bits and repl into flip-flops. The sequential walk in the
            // else-branch invalidates the cache instead.
            cache_ready <= 1'b0;
            inv_addr <= {SET_AW{1'b0}};
            pre_en_d <= 1'b0;
            pre_inflight <= 1'b0;
            pre_inflight_addr <= 15'd0;
            p0_meta_q <= {META_W{1'b0}};
            p1_meta_q <= {META_W{1'b0}};
            p2_meta_q <= {META_W{1'b0}};
            w0_meta_q <= {META_W{1'b0}};
            w1_meta_q <= {META_W{1'b0}};
            w2_meta_q <= {META_W{1'b0}};
        end else if (!cache_ready) begin
            // Post-reset invalidation walk. The array writes themselves are
            // done by the arbitrated write port further down; this only drives
            // the address and the completion flag. Both miss requests are
            // gated on cache_ready, so no fill can collide, and no lookup can
            // hit until the walk finishes.
            //
            // A concurrent download-time invalidate takes priority over the
            // walk's write, so hold the address in that case and retry it -
            // otherwise one set would be skipped and could later false-hit.
            if (!tile_gfx_inval) begin
                if (inv_addr == NUM_SETS[SET_AW-1:0] - 1'b1)
                    cache_ready <= 1'b1;
                inv_addr <= inv_addr + 1'b1;
            end
        end else begin
            tile_rd_byte_sel_d <= rd_addr[2:0];
            tile_rd_word_addr_d <= tile_rd_word_addr;
            tile_rd_set_d <= tile_rd_set;
            tile_rd_tag_d <= tile_rd_tag;
            w0_data_q  <= way0_data[tile_rd_set];
            w0_meta_q  <= way0_meta[tile_rd_set];
            w1_data_q  <= way1_data[tile_rd_set];
            w1_meta_q  <= way1_meta[tile_rd_set];
            w2_data_q  <= way2_data[tile_rd_set];
            w2_meta_q  <= way2_meta[tile_rd_set];
            repl_q     <= repl[tile_rd_set];

            // Second (prefetch) tag lookup - tags and valids only.
            pre_en_d        <= pre_en;
            pre_word_addr_d <= pre_word_addr;
            pre_set_d       <= pre_set;
            pre_tag_d       <= pre_tag;
            p0_meta_q       <= way0_meta[pre_set];
            p1_meta_q       <= way1_meta[pre_set];
            p2_meta_q       <= way2_meta[pre_set];
            pre_repl_q      <= repl[pre_set];

            // Latch the row request so it survives arbitration delays
            // (tile fills have priority and the bus may be mid-transaction).
            if (sprite_rd_en) begin
                sprite_req_pending <= 1'b1;
                sprite_req_addr <= sprite_rd_addr;
            end

            if (wr_cmd_pending) begin
                DDRAM_WE <= 1'b1;
                DDRAM_RD <= 1'b0;
                if (!DDRAM_BUSY) begin
                    wr_cmd_pending <= 1'b0;
                    DDRAM_WE <= 1'b0;
                    if (wr_region == REGION_TILE)
                        dbg_tile_ddram_we_seen <= 1'b1;
                    else
                        dbg_sprite_ddram_we_seen <= 1'b1;
                end
            end else if (rd_cmd_pending) begin
                DDRAM_RD <= 1'b1;
                DDRAM_WE <= 1'b0;
                if (!DDRAM_BUSY) begin
                    rd_cmd_pending <= 1'b0;
                    rd_pending <= 1'b1;
                    DDRAM_RD <= 1'b0;
                    if (rd_region == REGION_TILE)
                        dbg_tile_ddram_rd_seen <= 1'b1;
                    else
                        dbg_sprite_ddram_rd_seen <= 1'b1;
                end
            end else begin
                DDRAM_RD <= 1'b0;
                DDRAM_WE <= 1'b0;
            end

            if (DDRAM_DOUT_READY && rd_pending) begin
                rd_pending <= 1'b0;
                // Any completed tile fill retires an outstanding prefetch:
                // the line is now resident (or was filled by the demand path
                // for the same word), so re-issue suppression can lift.
                if (fill_region == REGION_TILE)
                    pre_inflight <= 1'b0;
                if (fill_region == REGION_TILE) begin
                    case (fill_way)
                        // meta and repl are written by the arbitrated port
                        // below; only the data array is written here.
                        2'd0: way0_data[fill_set] <= DDRAM_DOUT;
                        2'd1: way1_data[fill_set] <= DDRAM_DOUT;
                        default: way2_data[fill_set] <= DDRAM_DOUT;
                    endcase
                    dbg_tile_ddram_ready_seen <= 1'b1;
                    dbg_tile_cache_fill_seen <= 1'b1;
                    if (DDRAM_DOUT != 64'd0)
                        dbg_tile_ddram_dout_nonzero_seen <= 1'b1;
                end else begin
                    // Row bytes 0..3 live in ascending little-endian lanes
                    // of the selected 32-bit half; packed-MSB order wants
                    // byte 0 (pixels 0-1) in bits [31:24].
                    sprite_read_busy <= 1'b0;
                    sprite_rd_ready <= 1'b1;
                    sprite_rd_row <= sprite_fill_half
                        ? {DDRAM_DOUT[39:32], DDRAM_DOUT[47:40],
                           DDRAM_DOUT[55:48], DDRAM_DOUT[63:56]}
                        : {DDRAM_DOUT[7:0],   DDRAM_DOUT[15:8],
                           DDRAM_DOUT[23:16], DDRAM_DOUT[31:24]};
                    dbg_sprite_ddram_ready_seen <= 1'b1;
                    dbg_sprite_cache_fill_seen <= 1'b1;
                    dbg_sprite_cache_hit_seen <= 1'b1;
                    if (DDRAM_DOUT != 64'd0)
                        dbg_sprite_ddram_dout_nonzero_seen <= 1'b1;
                    if ((sprite_fill_half ? DDRAM_DOUT[63:32]
                                          : DDRAM_DOUT[31:0]) != 32'd0)
                        dbg_sprite_rom_data_nonzero_seen <= 1'b1;
                end
            end

            if (bus_idle) begin
                if (tile_miss_request) begin
                    DDRAM_ADDR <= BASE_ADDR + {14'd0, tile_rd_word_addr_d};
                    fill_set <= tile_rd_set_d;
                    fill_tag <= tile_rd_tag_d;
                    fill_way <= repl_q;
                    fill_region <= REGION_TILE;
                    rd_region <= REGION_TILE;
                    rd_cmd_pending <= 1'b1;
                    DDRAM_RD <= 1'b1;
                    dbg_tile_cache_miss_seen <= 1'b1;
                end else if (pre_miss_request) begin
                    // Ranked above sprites: this is what keeps the display
                    // path from ever painting pen 0, and it is only ~42
                    // fetches per line, so the motion-object reads (which
                    // have a real handshake and can wait) still get the bus.
                    DDRAM_ADDR <= BASE_ADDR + {14'd0, pre_word_addr_d};
                    fill_set <= pre_set_d;
                    fill_tag <= pre_tag_d;
                    fill_way <= pre_repl_q;
                    fill_region <= REGION_TILE;
                    rd_region <= REGION_TILE;
                    rd_cmd_pending <= 1'b1;
                    DDRAM_RD <= 1'b1;
                    pre_inflight <= 1'b1;
                    pre_inflight_addr <= pre_word_addr_d;
                    dbg_tile_cache_fill_seen <= 1'b1;
                end else if (sprite_row_request) begin
                    DDRAM_ADDR <= SPRITE_BASE_ADDR +
                                  {15'd0, sprite_req_addr[16:3]};
                    fill_region <= REGION_SPRITE;
                    rd_region <= REGION_SPRITE;
                    rd_cmd_pending <= 1'b1;
                    sprite_fill_half <= sprite_req_addr[2];
                    sprite_req_pending <= 1'b0;
                    sprite_read_busy <= 1'b1;
                    DDRAM_RD <= 1'b1;
                    dbg_sprite_cache_miss_seen <= 1'b1;
                end
            end

            if (current_tile_cache_hit) begin
                rd_data <= select_byte(hit_data, tile_rd_byte_sel_d);
                dbg_tile_cache_hit_seen <= 1'b1;
                if (select_byte(hit_data, tile_rd_byte_sel_d) != 8'd0)
                    dbg_tile_rom_data_nonzero_seen <= 1'b1;
            end else begin
                rd_data <= 8'd0;
            end

        end

        if (gfx_wr &&
            !wr_cmd_pending && !rd_cmd_pending && !rd_pending) begin
            DDRAM_ADDR <= (sprite_wr ? SPRITE_BASE_ADDR : BASE_ADDR) +
                          {14'd0, wr_word_addr};
            DDRAM_DIN <= {8{ioctl_dout}};
            DDRAM_BE <= byte_enable(wr_byte_sel);
            wr_region <= sprite_wr ? REGION_SPRITE : REGION_TILE;
            wr_cmd_pending <= 1'b1;
            DDRAM_WE <= 1'b1;
            if (sprite_wr) begin
                dbg_sprite_ddram_we_seen <= 1'b1;
            end else begin
                dbg_tile_ddram_we_seen <= 1'b1;
            end
        end
    end

    // ---- Single arbitrated write port per way's meta array ---------------
    // M10K inference requires ONE write port. Three things want to write:
    // a completed fill, the post-reset invalidation walk, and the
    // download-time invalidate of the set a ROM word lands in. They are
    // merged here with a fixed priority instead of being written from three
    // places in the main block, which would have forced the arrays back into
    // logic and undone the reason for packing valid into the tag word.
    wire tile_gfx_inval = gfx_wr && !sprite_wr &&
                          !wr_cmd_pending && !rd_cmd_pending && !rd_pending;
    wire tile_fill_done = DDRAM_DOUT_READY && rd_pending &&
                          (fill_region == REGION_TILE);
    wire meta_clear = tile_gfx_inval || !cache_ready;

    wire [SET_AW-1:0] meta_wr_addr = tile_gfx_inval ? wr_word_addr[SET_AW-1:0]
                                   : (!cache_ready) ? inv_addr
                                                    : fill_set;
    wire [META_W-1:0] meta_wr_data = meta_clear ? {META_W{1'b0}}
                                                : {1'b1, fill_tag};

    wire w0_meta_we = meta_clear || (tile_fill_done && (fill_way == 2'd0));
    wire w1_meta_we = meta_clear || (tile_fill_done && (fill_way == 2'd1));
    wire w2_meta_we = meta_clear || (tile_fill_done && (fill_way == 2'd2));

    // repl shares the same arbitration: seeded by the walk, advanced by fills.
    wire             repl_we   = meta_clear || tile_fill_done;
    wire [SET_AW-1:0] repl_addr = meta_clear ? meta_wr_addr : fill_set;
    wire [1:0]       repl_data  = meta_clear ? 2'd0
                                : ((fill_way == 2'd2) ? 2'd0 : (fill_way + 2'd1));

    always @(posedge clk) begin
        if (w0_meta_we) way0_meta[meta_wr_addr] <= meta_wr_data;
        if (w1_meta_we) way1_meta[meta_wr_addr] <= meta_wr_data;
        if (w2_meta_we) way2_meta[meta_wr_addr] <= meta_wr_data;
        if (repl_we)    repl[repl_addr]         <= repl_data;
    end

    function [7:0] byte_enable;
        input [2:0] sel;
        begin
            case (sel)
                3'd0: byte_enable = 8'b0000_0001;
                3'd1: byte_enable = 8'b0000_0010;
                3'd2: byte_enable = 8'b0000_0100;
                3'd3: byte_enable = 8'b0000_1000;
                3'd4: byte_enable = 8'b0001_0000;
                3'd5: byte_enable = 8'b0010_0000;
                3'd6: byte_enable = 8'b0100_0000;
                default: byte_enable = 8'b1000_0000;
            endcase
        end
    endfunction

    function [7:0] select_byte;
        input [63:0] word;
        input [2:0]  sel;
        begin
            case (sel)
                3'd0: select_byte = word[7:0];
                3'd1: select_byte = word[15:8];
                3'd2: select_byte = word[23:16];
                3'd3: select_byte = word[31:24];
                3'd4: select_byte = word[39:32];
                3'd5: select_byte = word[47:40];
                3'd6: select_byte = word[55:48];
                default: select_byte = word[63:56];
            endcase
        end
    endfunction

endmodule
