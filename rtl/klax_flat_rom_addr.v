//============================================================================
// Klax Flat ROM Address Mapper
//
// MiSTer MRA downloads paired graphics/program regions as flat 64 KB parts:
//   part 0: even bytes, low half
//   part 1: odd  bytes, low half
//   part 2: even bytes, high half
//   part 3: odd  bytes, high half
// Sprite graphics only use the first two parts, so flat_addr[17] remains 0.
//
// This maps the flat download address to the byte-interleaved region layout:
//   0x00000, 0x10000, 0x00001, 0x10001... becomes 0x00000, 0x00001, 0x00002...
// for each 128 KB half.
//============================================================================

module klax_flat_rom_addr (
    input  [26:0] flat_addr,
    output [17:0] byte_addr
);

    assign byte_addr = {flat_addr[17], flat_addr[15:0], flat_addr[16]};

endmodule
