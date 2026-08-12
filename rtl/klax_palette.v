//============================================================================
// Klax Palette (Color) RAM
//
// Silicon (schematic Sheet 8): one 2Kx8/35 SRAM at 11J holds TWO banks of
// 512 IRGB_1555 entries. Address bits: A10 = byte half (video: FCLOCK phase;
// CPU: MA1), A9 = CBANK (the 0x260000 output latch's D13 — a plain address
// line, so it switches BOTH display fetches and CPU access), A8..A0 = the
// 9-bit entry index (video: pixel color index; CPU: MA10..MA2).
//   Entries   0-255: sprite palette (gfx index 1, base 0)
//   Entries 256-511: tile palette   (gfx index 0, base 256)
//
// The game never sets CBANK, so bank 0 is the only one it ever uses — MAME
// models exactly that slice (1KB byte palette at 0x3E0000-0x3E07FF on the
// upper data lane, klax.cpp .umask16(0xff00)). Bank 1 is modeled for
// silicon faithfulness; on the PCB it holds SRAM power-up garbage until
// written.
//
// CPU address word N accesses palette byte N of the CBANK-selected bank via
// D15..D8; two successive bytes form one IRGB_1555 entry. Only MA10..MA1
// reach the chip, so the CPU image repeats every 2KB across the 0x3E page
// (the page-wide /VRAM+CTIME select is handled in klax_addr_decode).
//============================================================================

module klax_palette (
    input               clk,

    // Color-RAM bank select (output latch D13 = CBANK = color RAM A9)
    input               cbank,

    // Video read port (1-cycle latency)
    input       [8:0]   rd_addr,
    output reg  [15:0]  rd_data,

    // Motion-object video read port (1-cycle latency)
    input       [8:0]   mob_rd_addr,
    output reg  [15:0]  mob_rd_data,

    // CPU port
    input       [9:0]   cpu_addr,
    output      [15:0]  cpu_rd_data,
    input       [15:0]  wr_data,
    input               wr_en,
    input               uds_n,
    input               lds_n
);

    (* ramstyle = "M10K" *) reg [7:0] pal_hi [0:1023];
    (* ramstyle = "M10K" *) reg [7:0] pal_lo [0:1023];

    wire [9:0] vid_rd_bank_addr  = {cbank, rd_addr};
    wire [9:0] mob_rd_bank_addr  = {cbank, mob_rd_addr};
    wire [9:0] cpu_word_addr = {cbank, cpu_addr[9:1]};
    wire       cpu_byte_lo   = cpu_addr[0];
    reg        cpu_byte_lo_d = 1'b0;
    reg [7:0]  cpu_hi_rd_data = 8'd0;
    reg [7:0]  cpu_lo_rd_data = 8'd0;

    assign cpu_rd_data = {cpu_byte_lo_d ? cpu_lo_rd_data : cpu_hi_rd_data, 8'hFF};

`ifdef SIMULATION
    integer init_i;
    initial begin
        for (init_i = 0; init_i < 1024; init_i = init_i + 1) begin
            pal_hi[init_i] = 8'd0;
            pal_lo[init_i] = 8'd0;
        end
        rd_data = 16'd0;
        mob_rd_data = 16'd0;
        cpu_hi_rd_data = 8'd0;
        cpu_lo_rd_data = 8'd0;
        cpu_byte_lo_d = 1'b0;
    end
`endif

    always @(posedge clk) begin
        rd_data     <= {pal_hi[vid_rd_bank_addr], pal_lo[vid_rd_bank_addr]};
        mob_rd_data <= {pal_hi[mob_rd_bank_addr], pal_lo[mob_rd_bank_addr]};
        cpu_hi_rd_data <= pal_hi[cpu_word_addr];
        cpu_lo_rd_data <= pal_lo[cpu_word_addr];
        cpu_byte_lo_d  <= cpu_byte_lo;

        if (wr_en && !uds_n) begin
            if (cpu_byte_lo)
                pal_lo[cpu_word_addr] <= wr_data[15:8];
            else
                pal_hi[cpu_word_addr] <= wr_data[15:8];
        end
    end

endmodule
