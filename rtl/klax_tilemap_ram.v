//============================================================================
// Klax Tilemap RAM
//
// Two independent 2K-word (4KB) tilemap regions on the 68000 CPU bus, each
// with its own video read port:
//   0x3F0000 - 0x3F0F7F: Playfield          (pf_ram)
//   0x3F1000 - 0x3F1FFF: Extended playfield (pfx_ram)
// Byte-addressable via the 68000 UDS/LDS strobes.
//============================================================================

module klax_tilemap_ram (
    input               clk,

    // Video read port (1-cycle latency)
    input       [10:0]  rd_addr,
    output reg  [15:0]  pf_rd_data,
    output reg  [15:0]  pfx_rd_data,

    // Playfield prefetch read port (1-cycle latency).
    //
    // The renderer fetches tile graphics from DDRAM a couple of tiles ahead
    // of the raster so the data is resident before the display path needs it
    // (that path cannot stall, and a tile-cache miss would otherwise paint
    // pen 0). Working out the address for
    // tile column N+2 needs that column's code, hence a second pf read port.
    // Only pf is duplicated: the prefetch needs the code, not the pfx colour.
    input       [10:0]  pre_addr,
    output reg  [15:0]  pf_pre_rd_data,

    // CPU port (read/write)
    input       [10:0]  cpu_addr,
    
    output reg  [15:0]  pf_cpu_rd_data,
    input       [15:0]  pf_wr_data,
    input               pf_wr_en,
    input               pf_uds_n,
    input               pf_lds_n,

    output reg  [15:0]  pfx_cpu_rd_data,
    input       [15:0]  pfx_wr_data,
    input               pfx_wr_en,
    input               pfx_uds_n,
    input               pfx_lds_n
);

    (* ramstyle = "M10K" *) reg [7:0] pf_ram_hi  [0:2047];
    (* ramstyle = "M10K" *) reg [7:0] pf_ram_lo  [0:2047];
    (* ramstyle = "M10K" *) reg [7:0] pfx_ram_hi [0:2047];
    (* ramstyle = "M10K" *) reg [7:0] pfx_ram_lo [0:2047];

    // Explicit mirror of pf for the prefetch port. M10K is dual-port and the
    // two ports are already taken (video read + CPU read/write), so a third
    // reader needs its own copy. Written in lockstep below; duplicating here
    // rather than letting inference replicate keeps the cost visible.
    (* ramstyle = "M10K" *) reg [7:0] pf_pre_hi  [0:2047];
    (* ramstyle = "M10K" *) reg [7:0] pf_pre_lo  [0:2047];

`ifdef SIMULATION
    integer init_i;
    initial begin
        for (init_i = 0; init_i < 2048; init_i = init_i + 1) begin
            pf_ram_hi[init_i] = 8'd0;
            pf_ram_lo[init_i] = 8'd0;
            pfx_ram_hi[init_i] = 8'd0;
            pfx_ram_lo[init_i] = 8'd0;
            pf_pre_hi[init_i] = 8'd0;
            pf_pre_lo[init_i] = 8'd0;
        end
        pf_rd_data = 16'd0;
        pfx_rd_data = 16'd0;
        pf_cpu_rd_data = 16'd0;
        pfx_cpu_rd_data = 16'd0;
        pf_pre_rd_data = 16'd0;
    end
`endif

    always @(posedge clk) begin
        pf_rd_data      <= {pf_ram_hi[rd_addr], pf_ram_lo[rd_addr]};
        pfx_rd_data     <= {pfx_ram_hi[rd_addr], pfx_ram_lo[rd_addr]};
        pf_cpu_rd_data  <= {pf_ram_hi[cpu_addr], pf_ram_lo[cpu_addr]};
        pfx_cpu_rd_data <= {pfx_ram_hi[cpu_addr], pfx_ram_lo[cpu_addr]};
        pf_pre_rd_data  <= {pf_pre_hi[pre_addr], pf_pre_lo[pre_addr]};

        if (pf_wr_en) begin
            if (!pf_uds_n) begin
                pf_ram_hi[cpu_addr] <= pf_wr_data[15:8];
                pf_pre_hi[cpu_addr] <= pf_wr_data[15:8];
            end
            if (!pf_lds_n) begin
                pf_ram_lo[cpu_addr] <= pf_wr_data[7:0];
                pf_pre_lo[cpu_addr] <= pf_wr_data[7:0];
            end
        end

        if (pfx_wr_en) begin
            if (!pfx_uds_n)
                pfx_ram_hi[cpu_addr] <= pfx_wr_data[15:8];
            if (!pfx_lds_n)
                pfx_ram_lo[cpu_addr] <= pfx_wr_data[7:0];
        end
    end

endmodule
