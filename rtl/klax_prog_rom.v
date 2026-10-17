// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.
//
//============================================================================
// Klax Program ROM
//
// 256KB Program ROM (128K x 16-bit).
// Mapped to 68000 CPU at 0x000000 - 0x03FFFF.
//
// For simulation, this is initialized from klax_cpu.hex.
// Synthesized as Block RAM (M10K) in Quartus.
//============================================================================

module klax_prog_rom (
    input               clk,

    // CPU Read Port
    input       [16:0]  addr,    // 128K words (256KB)
    output reg  [15:0]  data,

    // MiSTer IOCTL Write Port
    input               ioctl_download,
    input               ioctl_wr,
    input       [26:0]  ioctl_addr,
    input       [7:0]   ioctl_dout
);

    // Store the 68000 high/low byte lanes as separate byte-wide RAMs. Quartus
    // 17 infers this much more reliably than a 16-bit RAM with partial writes.
    (* ramstyle = "M10K" *) reg [7:0] rom_hi [0:131071];
    (* ramstyle = "M10K" *) reg [7:0] rom_lo [0:131071];

`ifdef SIMULATION
    reg [15:0] init_rom [0:131071];
    integer init_i;

    initial begin
        $readmemh("klax_cpu.hex", init_rom);
        for (init_i = 0; init_i < 131072; init_i = init_i + 1) begin
            rom_hi[init_i] = init_rom[init_i][15:8];
            rom_lo[init_i] = init_rom[init_i][7:0];
        end
    end
`endif

    // Convert the flat MRA stream to the MAME/68000 byte layout in hardware.
    wire [17:0] wr_byte_addr;

    klax_flat_rom_addr u_flat_addr (
        .flat_addr (ioctl_addr),
        .byte_addr (wr_byte_addr)
    );

    wire [16:0] wr_word_addr = wr_byte_addr[17:1];

    // Inferred Simple Dual-Port Block RAM - Port A (Write)
    always @(posedge clk) begin
        if (ioctl_download && ioctl_wr) begin
            // 68000 is big-endian: even byte lane is D15:D8, odd byte lane is D7:D0.
            if (wr_byte_addr[0] == 1'b0)
                rom_hi[wr_word_addr] <= ioctl_dout;
            else
                rom_lo[wr_word_addr] <= ioctl_dout;
        end
    end

    // Inferred Simple Dual-Port Block RAM - Port B (Read)
    always @(posedge clk) begin
        data <= {rom_hi[addr], rom_lo[addr]};
    end

endmodule
