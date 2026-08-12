//============================================================================
// Klax Motion Object RAM
//
// CPU-visible storage from the MAME Klax map:
//   0x3F0F80-0x3F0FFF: Motion object SLIP RAM, 128 bytes / 64 words
//   0x3F2000-0x3F27FF: Motion object RAM,      2 KB / 1024 words
//
// The renderer read ports feed klax_motion_objects (the line-buffer renderer),
// which reads this storage on its own ports, independent of the CPU ports.
//============================================================================

module klax_mob_ram (
    input               clk,
    input               reset,

    // CPU SLIP port
    input       [5:0]   slip_cpu_addr,
    output reg  [15:0]  slip_cpu_rd_data,
    input       [15:0]  slip_wr_data,
    input               slip_wr_en,
    input               slip_uds_n,
    input               slip_lds_n,
    input               slip_cpu_rd_en,

    // CPU motion-object port
    input       [9:0]   mob_cpu_addr,
    output reg  [15:0]  mob_cpu_rd_data,
    input       [15:0]  mob_wr_data,
    input               mob_wr_en,
    input               mob_uds_n,
    input               mob_lds_n,
    input               mob_cpu_rd_en,

    // Renderer SLIP read port (1-cycle latency)
    input       [5:0]   slip_rd_addr,
    output reg  [15:0]  slip_rd_data,

    // Renderer motion-object read port (1-cycle latency)
    input       [9:0]   mob_rd_addr,
    output reg  [15:0]  mob_rd_data,

    output      [15:0]  debug_flags
);

    (* ramstyle = "logic" *) reg [7:0] slip_hi [0:63];
    (* ramstyle = "logic" *) reg [7:0] slip_lo [0:63];
    (* ramstyle = "M10K"  *) reg [7:0] mob_hi  [0:1023];
    (* ramstyle = "M10K"  *) reg [7:0] mob_lo  [0:1023];

    reg dbg_slip_wr_seen = 1'b0;
    reg dbg_mob_wr_seen = 1'b0;
    reg dbg_slip_wr_nonzero_seen = 1'b0;
    reg dbg_mob_wr_nonzero_seen = 1'b0;
    reg dbg_slip_hi_wr_seen = 1'b0;
    reg dbg_slip_lo_wr_seen = 1'b0;
    reg dbg_mob_hi_wr_seen = 1'b0;
    reg dbg_mob_lo_wr_seen = 1'b0;
    reg dbg_slip_cpu_rd_seen = 1'b0;
    reg dbg_mob_cpu_rd_seen = 1'b0;
    reg dbg_slip_cpu_rd_nonzero_seen = 1'b0;
    reg dbg_mob_cpu_rd_nonzero_seen = 1'b0;
    reg dbg_slip_video_addr_seen = 1'b0;
    reg dbg_mob_video_addr_seen = 1'b0;
    reg dbg_slip_video_nonzero_seen = 1'b0;

    wire slip_write_nonzero =
        (!slip_uds_n && (slip_wr_data[15:8] != 8'd0)) ||
        (!slip_lds_n && (slip_wr_data[7:0] != 8'd0));
    wire mob_write_nonzero =
        (!mob_uds_n && (mob_wr_data[15:8] != 8'd0)) ||
        (!mob_lds_n && (mob_wr_data[7:0] != 8'd0));

    assign debug_flags = {
        1'b1,
        dbg_slip_video_nonzero_seen,
        dbg_mob_video_addr_seen,
        dbg_slip_video_addr_seen,
        dbg_mob_cpu_rd_nonzero_seen,
        dbg_slip_cpu_rd_nonzero_seen,
        dbg_mob_cpu_rd_seen,
        dbg_slip_cpu_rd_seen,
        dbg_mob_lo_wr_seen,
        dbg_mob_hi_wr_seen,
        dbg_slip_lo_wr_seen,
        dbg_slip_hi_wr_seen,
        dbg_mob_wr_nonzero_seen,
        dbg_slip_wr_nonzero_seen,
        dbg_mob_wr_seen,
        dbg_slip_wr_seen
    };

`ifdef SIMULATION
    integer init_i;
    initial begin
        for (init_i = 0; init_i < 64; init_i = init_i + 1) begin
            slip_hi[init_i] = 8'd0;
            slip_lo[init_i] = 8'd0;
        end
        for (init_i = 0; init_i < 1024; init_i = init_i + 1) begin
            mob_hi[init_i] = 8'd0;
            mob_lo[init_i] = 8'd0;
        end
        slip_cpu_rd_data = 16'd0;
        mob_cpu_rd_data = 16'd0;
        slip_rd_data = 16'd0;
        mob_rd_data = 16'd0;
    end
`endif

    always @(posedge clk) begin
        slip_cpu_rd_data <= {slip_hi[slip_cpu_addr], slip_lo[slip_cpu_addr]};
        mob_cpu_rd_data  <= {mob_hi[mob_cpu_addr], mob_lo[mob_cpu_addr]};
        slip_rd_data     <= {slip_hi[slip_rd_addr], slip_lo[slip_rd_addr]};
        mob_rd_data      <= {mob_hi[mob_rd_addr], mob_lo[mob_rd_addr]};

        if (slip_wr_en) begin
            if (!slip_uds_n)
                slip_hi[slip_cpu_addr] <= slip_wr_data[15:8];
            if (!slip_lds_n)
                slip_lo[slip_cpu_addr] <= slip_wr_data[7:0];
        end

        if (mob_wr_en) begin
            if (!mob_uds_n)
                mob_hi[mob_cpu_addr] <= mob_wr_data[15:8];
            if (!mob_lds_n)
                mob_lo[mob_cpu_addr] <= mob_wr_data[7:0];
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            dbg_slip_wr_seen <= 1'b0;
            dbg_mob_wr_seen <= 1'b0;
            dbg_slip_wr_nonzero_seen <= 1'b0;
            dbg_mob_wr_nonzero_seen <= 1'b0;
            dbg_slip_hi_wr_seen <= 1'b0;
            dbg_slip_lo_wr_seen <= 1'b0;
            dbg_mob_hi_wr_seen <= 1'b0;
            dbg_mob_lo_wr_seen <= 1'b0;
            dbg_slip_cpu_rd_seen <= 1'b0;
            dbg_mob_cpu_rd_seen <= 1'b0;
            dbg_slip_cpu_rd_nonzero_seen <= 1'b0;
            dbg_mob_cpu_rd_nonzero_seen <= 1'b0;
            dbg_slip_video_addr_seen <= 1'b0;
            dbg_mob_video_addr_seen <= 1'b0;
            dbg_slip_video_nonzero_seen <= 1'b0;
        end else begin
            if (slip_wr_en) begin
                dbg_slip_wr_seen <= 1'b1;
                if (!slip_uds_n)
                    dbg_slip_hi_wr_seen <= 1'b1;
                if (!slip_lds_n)
                    dbg_slip_lo_wr_seen <= 1'b1;
                if (slip_write_nonzero)
                    dbg_slip_wr_nonzero_seen <= 1'b1;
            end

            if (mob_wr_en) begin
                dbg_mob_wr_seen <= 1'b1;
                if (!mob_uds_n)
                    dbg_mob_hi_wr_seen <= 1'b1;
                if (!mob_lds_n)
                    dbg_mob_lo_wr_seen <= 1'b1;
                if (mob_write_nonzero)
                    dbg_mob_wr_nonzero_seen <= 1'b1;
            end

            if (slip_cpu_rd_en) begin
                dbg_slip_cpu_rd_seen <= 1'b1;
                if (slip_cpu_rd_data != 16'd0)
                    dbg_slip_cpu_rd_nonzero_seen <= 1'b1;
            end

            if (mob_cpu_rd_en) begin
                dbg_mob_cpu_rd_seen <= 1'b1;
                if (mob_cpu_rd_data != 16'd0)
                    dbg_mob_cpu_rd_nonzero_seen <= 1'b1;
            end

            if (slip_rd_addr != 6'd0)
                dbg_slip_video_addr_seen <= 1'b1;
            if (mob_rd_addr != 10'd0)
                dbg_mob_video_addr_seen <= 1'b1;
            if (slip_rd_data != 16'd0)
                dbg_slip_video_nonzero_seen <= 1'b1;
        end
    end

endmodule
