//============================================================================
// Klax EEPROM
//
// 2816-style 2KB x 8 parallel EEPROM on the 68000 low byte lane.
// MAME map: 0x0E0000-0x0E0FFF, .umask16(0x00ff)
//           0x1F0000-0x1FFFFF write unlock
//
// This models the decoded bus behavior needed by the CPU:
//   - reads return the EEPROM byte on D7:D0 with D15:D8 pulled high
//   - writes honor the lower data strobe
//   - a write to the unlock range enables one following EEPROM byte write
//============================================================================

module klax_eeprom (
    input               clk,
    input               reset,

    input       [10:0]  addr,
    output reg  [15:0]  rd_data,

    input       [15:0]  wr_data,
    input               wr_en,
    input               lds_n,
    input               unlock_wr,

    input               ioctl_wr,
    input       [10:0]  ioctl_addr,
    input       [7:0]   ioctl_dout,

    // NVRAM save support: independent read port for the
    // MiSTer ioctl upload path, and a pulse on each committed CPU byte
    // write so the top level can schedule a save request.
    input       [10:0]  up_addr,
    output reg  [7:0]   up_data,
    output reg          cpu_wr_done
);

    (* ramstyle = "M10K" *) reg [7:0] eeprom [0:2047];

    reg unlocked;
`ifdef SIMULATION
    integer init_i;

    initial begin
        for (init_i = 0; init_i < 2048; init_i = init_i + 1)
            eeprom[init_i] = 8'hFF;

        rd_data = 16'hFFFF;
        unlocked = 1'b0;

        $readmemh("klax_eeprom.hex", eeprom);
    end
`endif

    always @(posedge clk) begin
        rd_data <= {8'hFF, eeprom[addr]};

        if (ioctl_wr) begin
            eeprom[ioctl_addr] <= ioctl_dout;
        end else if (wr_en && !lds_n && unlocked) begin
            eeprom[addr] <= wr_data[7:0];
        end
    end

    // Upload read port (its own port so a save never disturbs CPU reads).
    always @(posedge clk) begin
        up_data <= eeprom[up_addr];
    end

    // One pulse per committed CPU byte write (excludes ioctl downloads,
    // which take priority in the write mux above and must not mark the
    // contents dirty).
    always @(posedge clk or posedge reset) begin
        if (reset)
            cpu_wr_done <= 1'b0;
        else
            cpu_wr_done <= !ioctl_wr && wr_en && !lds_n && unlocked;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            unlocked <= 1'b0;
        end else begin
            if (unlock_wr) begin
                unlocked <= 1'b1;
            end else if (wr_en && !lds_n) begin
                unlocked <= 1'b0;
            end
        end
    end

endmodule
