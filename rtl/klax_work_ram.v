// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.
//
//============================================================================
// Klax Work RAM
//
// General 68000 work RAM at 0x3F2800 - 0x3F3FFF.
//
// A plain 4096 x 16-bit (8KB) block indexed by A12..A1 across the
// 0x3F2000-0x3F3FFF span; it holds no address decode of its own, so the
// work-RAM chip select in klax_sys gates the 0x3F2800-0x3F3FFF window into it.
// The neighboring Motion Object / SLIP RAM (0x3F2000-0x3F27FF and
// 0x3F0F80-0x3F0FFF) is a separate module, klax_mob_ram.
//
// Supports 68000 upper/lower data strobes (UDS/LDS) via byte-enables.
//============================================================================

module klax_work_ram (
    input               clk,
    input       [11:0]  addr,      // 4096 words
    output reg  [15:0]  rd_data,

    input       [15:0]  wr_data,
    input               wr_en,
    input               uds_n,     // upper data strobe (active low, byte [15:8])
    input               lds_n      // lower data strobe (active low, byte [7:0])
);

    (* ramstyle = "M10K" *) reg [7:0] ram_hi [0:4095];
    (* ramstyle = "M10K" *) reg [7:0] ram_lo [0:4095];

`ifdef SIMULATION
    integer init_i;
    initial begin
        for (init_i = 0; init_i < 4096; init_i = init_i + 1) begin
            ram_hi[init_i] = 8'd0;
            ram_lo[init_i] = 8'd0;
        end
        rd_data = 16'd0;
    end
`endif

    always @(posedge clk) begin
        rd_data <= {ram_hi[addr], ram_lo[addr]};

        if (wr_en) begin
            if (!uds_n)
                ram_hi[addr] <= wr_data[15:8];
            if (!lds_n)
                ram_lo[addr] <= wr_data[7:0];
        end
    end

endmodule
