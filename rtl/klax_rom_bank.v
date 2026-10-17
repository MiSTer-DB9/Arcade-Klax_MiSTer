// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.
//
//============================================================================
// Klax ROM Bank — Dual-Port Tile ROM
//
// Write port: MiSTer ioctl download interface (loads ROMs from SD card)
// Read port:  standard synchronous read (1 cycle latency)
//
// Stores 256 KB of tile data (18-bit address, 8-bit data).
// Uses block RAM inference for Cyclone V synthesis (M10K blocks).
//
// The parent wrapper gates this module to the tile download index.
// The MRA sends four flat 64 KB parts; this module interleaves them into the
// byte layout consumed by the tile decoder.
//============================================================================

module klax_rom_bank #(
    parameter ROM_AW = 18,      // 256 KB addressable as bytes
    parameter ROM_DW = 8
) (
    input                       clk,
    input                       reset,

    // --- Write port (MiSTer ioctl download) ---
    input                       ioctl_download,
    input                       ioctl_wr,
    input       [26:0]          ioctl_addr,
    input       [7:0]           ioctl_dout,

    // --- Read port (display / rendering) ---
    input       [ROM_AW-1:0]    rd_addr,
    output reg  [ROM_DW-1:0]    rd_data
);

    localparam ROM_DEPTH = 1 << ROM_AW;  // 262144 bytes

    // Dual-port RAM (inferred as block RAM by Quartus)
    (* ramstyle = "M10K" *) reg [ROM_DW-1:0] ram [0:ROM_DEPTH-1];

    wire [17:0] wr_addr;

    klax_flat_rom_addr u_flat_addr (
        .flat_addr (ioctl_addr),
        .byte_addr (wr_addr)
    );

    // Inferred block RAM write
    always @(posedge clk) begin
        if (ioctl_download && ioctl_wr)
            ram[wr_addr[ROM_AW-1:0]] <= ioctl_dout;
    end

    // Read port (1 cycle latency)
    always @(posedge clk) begin
        rd_data <= ram[rd_addr];
    end

endmodule
