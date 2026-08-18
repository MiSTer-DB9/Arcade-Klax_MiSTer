# Klax for MiSTer FPGA

FPGA hardware recreation of the **Atari Games Klax** arcade PCB — a single-board
68000 design running at 7.15909 MHz off a 14.318181 MHz master crystal, with
Atari's SOS-2 custom driving video and an OKI MSM6295 doing sound directly, with
no secondary sound CPU.

The core is a hardware recreation, not a re-implementation of the game. It was
built from the PCB schematics and the dumped on-board PALs, with MAME as a
cross-reference rather than as the specification. Where the two disagree, the
silicon wins — so a few behaviours here deliberately differ from MAME. Those
divergences are called out below, and in the RTL comments at the point where
they matter.

<img src="https://img.shields.io/badge/Quartus-17.0.2-blue" alt="Quartus 17.0.2"> <img src="https://img.shields.io/badge/license-GPL--3.0--or--later-blue" alt="GPL-3.0-or-later">

---

## Supported games

| MRA | Set | Test-menu date |
|-----|-----|----------------|
| `Klax.mra` | `klax` | `13FEB1990 11:10:09` |
| `_alternatives/_Klax/Klax (version 5).mra` | `klax5` | `20JAN1990 14:47:06` |
| `_alternatives/_Klax/Klax (version 4).mra` | `klax4` | `20JAN1990 14:47:06` |
| `_alternatives/_Klax/Klax (Germany, version 2).mra` | `klaxd2` | `13FEB1990 11:10:09` |
| `_alternatives/_Klax/Klax (Japan, version 3).mra` | `klaxj3` | `20JAN1990 14:47:06` |
| `_alternatives/_Klax/Klax (Japan, version 4).mra` | `klaxj4` | `13FEB1990 11:10:09` |

All six sets run on the same RTL. Only the four `maincpu` ROMs differ — tile,
sprite, OKI sample and PAL data are byte-identical across every set — which is
why each alternate MRA is the parent with four parts swapped. Each set reports
its own build date in the service-mode test menu, so that string doubles as an
acceptance check: a mismatch means the wrong ROM set, not a core fault.

Every rom index in the alternates lists `zip="<clone>.zip|klax.zip"`, because a
MAME clone zip contains only the ROMs that differ; without the fallback the
shared graphics and sample ROMs would not be found.

Klax has **no DIP switches**. Cabinet configuration (coinage, difficulty,
attract sound) lives in the on-board EEPROM and is edited through the game's own
service menu.

Bootlegs (`klax5bl`, `klax5bl2`, `klax5bl3`) are deliberately not supported —
they are different hardware, with a Z80 and two MSM5205s in place of the OKI, a
different memory map, and no watchdog. `klaxp1` / `klaxp2` are also out of
scope: despite being filed as Klax clones they are the prototypes, and they run
on *Escape from the Planet of the Robot Monsters* hardware, which pairs a full
Atari JSA II sound board with a completely different video path.

---

## Installing

1. Copy `releases/Klax_<date>.rbf` to `/media/fat/_Arcade/cores/`.
2. Copy `releases/Klax.mra` to `/media/fat/_Arcade/`, and any alternates you
   want from `releases/_alternatives/`.
3. Put the MAME ROM set (`klax.zip`, matching the CRCs in the MRA) in
   `/media/fat/games/mame/`.

**No ROM data is included in this repository.** The MRA files reference ROM
parts by name and CRC only.

The MRA also carries `<nvram index="2" size="2048"/>`: a couple of seconds after
the game commits EEPROM writes, the core asks MiSTer to save the 2 KB image into
its saves directory, and reloads it on the next boot. Leave the OSD `Autosave`
option on for that to happen. MiSTer only collects the save request when the OSD
is opened, so settings land on disk at menu-open time.

---

## Controls

Klax is a one-button game: the button flips the paddle. On a real cabinet each
player has two physical Flip buttons wired in parallel to the same input, which
is why two Flip bits are exposed here; both are OR'd together. There is no
separate start input — the same button starts the game and flips tiles.

| Function | Joystick | Keyboard |
|----------|----------|----------|
| Move paddle | D-pad / stick | Arrow keys (P1); `R` `F` `D` `G` (P2) |
| Flip | Button 1 | Left Ctrl (P1); `A` (P2) |
| Flip (2nd input) | Button 2 | — |
| Coin 1 / Coin 2 | Buttons 3 / 4 | `5` / `6` |
| Service | Button 5 | `F2` |

Service is a **toggle** on the real utility panel, so the mapped button and `F2`
flip a latch rather than acting as a momentary press. The OSD `Service` option
overrides it.

Two authentic behaviours that look like bugs:

- Switching service **on** briefly paints white blocks before the menu draws.
  That is Klax's own self-test painting RAM/video test patterns.
- **Leaving** self-test reboots the board. The test code stops petting the
  watchdog, so the LS197 bites. MAME cannot reproduce this, because its Klax
  watchdog is never enabled.

### OSD options

| Option | Effect |
|--------|--------|
| Aspect ratio | Original / Full Screen / custom |
| Scale | Integer-scaling mode passed to `video_freak` |
| CRT H-Size | Analog pixel stretch, ±16 steps of 1/64 pixel period. `0` bypasses the stretch logic entirely |
| CRT H-Position | Slides stretched content inside the line without moving HSync. Only meaningful when CRT H-Size ≠ 0 |
| Analog VGA H-Shift / V-Shift | Classic centring nudge — delays HSync/VSync against unchanged content. H-Shift is inert while CRT H-Size ≠ 0, because the read window is referenced to the same shifted HSync; use CRT H-Position there |
| Service | Routes the Service input to the board's service line |
| Autosave | EEPROM write-back to SD (on by default) |

Every alignment control is a true bypass at zero, not merely "set to 0", so a
default OSD leaves the output bit-identical to the unmodified video path.

---

## Accuracy notes

What makes this core worth having is how much of it came off the schematics and
the dumped PALs rather than out of an emulator:

- **Address decode from silicon.** The production decoder is the dumped 11C
  GAL16V8; the PAL20L10 that appears in some documentation is the
  slapstic-variant part, which production Klax does not use. The GAL equations
  are transcribed verbatim, giving the real behaviour: the whole map mirrors ×4
  (MA22/MA23 undecoded), selects are page- or window-wide rather than
  byte-exact, `/VRAM` is the full 128 KB window sub-split by MA16, main video
  RAM is one 8K×16 SRAM pair (playfield / SLIP / extended / MOB / work are
  software layout, not separate devices), and unmapped reads return open-bus
  `0xFFFF`.
- **Every bus cycle completes.** DTACK comes from the LS163A at 10B, loaded
  from the decode: 0 wait states by default, +1 for EEPROM, +2 for the OKI, and
  0-or-4 for video RAM via the CPU-slot arbitration in the dumped 9W GAL. MAME
  models no wait states at all. IACK is handled too — `VPA` from the F20 at 9B
  always beats DTACK.
- **IRQ from the LS74 at 5K**, clocked by `VBLANK | ~32V` and cleared by
  `/VACK`, giving IRQ4 at scanlines 64, 128, 192 and 240 only. **MAME fires an
  extra IRQ4 at scanline 0 that the silicon cannot produce**; this core follows
  the silicon.
- **LS197 watchdog at 5B**, fully modelled including its power-on state: it
  clears to 0, which holds the board in reset for 8 frames before the game
  starts. MAME leaves its Klax watchdog disabled, so it never fires at all.
- **Native timing.** 456 × 262 pixels at 7.15909 MHz, with `clk_sys` at
  57.27272 MHz — 4 × the board's master crystal — every clock enable an exact
  integer division, and the 68000 clock enable equal to the pixel clock as on
  the PCB.
- **DAC-exact colour.** Sheet 9 is a 6-bit R/2R ladder per channel whose LSB is
  a shared intensity bit, so the output follows the ladder rather than a naive
  bit expansion. The CBANK bit (D13 of the output latch, which is colour RAM
  A9) is modelled as a second 512-entry palette bank, switching CPU and display
  views together.
- **JAMMA-accurate HSync.** `/PHSYNC` from the dumped 6W PAL is delayed through
  the LS174 at 17K — only the stage-3 tap is stuffed — for +10 pixels, putting
  monitor HSync at hc 362..389.
- **OKI MSM6295 ADPCM** decoding that is bit-exact against a MAME-0.288 golden
  model, plus — unlike MAME, which models none — the PCB's **analog audio output
  filter**: the 5-pole LM324 low-pass on schematic sheet 4, −3 dB at 790 Hz and
  −49.6 dB at the 6779.4 Hz first zero-order-hold image. The naive reading of
  that circuit is wrong in an interesting way: the bare Sallen-Key section
  computes to Q ≈ 5.5, a +15 dB peak, but the preceding R56/C43 network damps it
  away completely, so the composite response is monotonic. Klax genuinely sounds
  dark and lo-fi; bright, gritty emulator audio is the artifact.
- **Motion objects** through the real Atari line-buffer pipeline: SLIP-linked
  descriptor walk with a visited-array stop and 1024-link cap, one 64-bit
  graphics ROM read per 8-pixel row, a double-buffered 512-pixel line buffer
  with mod-512 X wrap and pen-0 transparency, and erase-behind-the-read-pointer
  on the display side. Merged motion-object pixels override the playfield unless
  the playfield colour nibble is `0xF`.
- **Playfield tile prefetch.** The tile graphics port has no handshake and the
  playfield pipeline runs at pixel cadence, so it cannot stall on a cache
  miss — a miss paints pen 0. Real silicon cannot miss, because the 9W slot
  machine schedules two playfield fetch slots per tile *ahead* of display. The
  renderer reproduces that by chasing tile column N+2 and warming the cache
  line, restoring the "already resident when the raster arrives" property.

### Known deviations and open questions

These cannot be settled from dumps, schematics or MAME, and would need
measurement on a real PCB. All are cosmetic or archival:

- Exact VSync placement inside the SOS-2 custom. It appears in no dumped PAL, so
  it needs a scope capture on the 9Y VSync pin. Nothing on the board reads VSync
  back, and VBLANK/IRQ placement is independently grounded.
- The 9W VRAM slot machine's phase, to within ±1 pixel.
- The slapstic PAL20L10 used by the slapstic board variant. Production Klax
  leaves that socket unstuffed, so it is not implemented.
- The analog video alignment page is simulation-verified and Quartus-clean but
  has not yet been confirmed on hardware. Leaving those options at their
  defaults reproduces the hardware-validated video path bit-for-bit.

---

## Verification

This core was developed simulation-first, with AI assistance, under human review
and hardware validation. AI-assisted RTL is easy to get *plausibly* wrong, so the
working rule throughout was that no subsystem counts as done until it has been
checked against an independent reference rather than against expectation:

| Subsystem | Method | Result |
|-----------|--------|--------|
| Address decode, DTACK, IACK | Dumped GAL equations transcribed and replicated, then exercised by the real TG68K in a mixed-language simulation | Reset-vector fetch through EEPROM service reads and on into the main loop |
| Playfield renderer | MAME-dumped tilemap and palette RAM rendered through the RTL pipeline, compared against MAME's own frame | Pixel-exact |
| Motion objects | MAME-dumped MOB, SLIP, tilemap and palette RAM rendered through the full MiSTer top, compared against MAME at 336 × 240 | 98.50 % match at offset 0; residual is dump-vs-render animation skew of moving content |
| Whole video path | Same comparison, after the DAC-exact colour work | 80,640 px, **0 mismatches, worst channel difference 0** at tolerance 0 |
| Tile pixel decode | Compared pixel-by-pixel against the MAME tile formula | Exact |
| OKI MSM6295 | Sample-for-sample against a MAME-0.288-equivalent golden model, driven by the real sample ROMs | Bit-exact in audio and status, zero mismatches |
| Analog audio filter | Against a golden model re-derived from the Sheet 4 component values. The generator also asserts that the coefficient literals in the RTL match its own derivation, so model and RTL cannot drift | 4,502 / 4,502 samples bit-exact, tail decays to exactly 0 |
| Watchdog | Unit simulation of the 8-frame power-on hold, the self-releasing reset pulse and the pet path | Pass |
| EEPROM persistence | Two-pass save / reload round trip in simulation | Pass |
| Playfield prefetch | Multi-frame test with a changing tilemap, counting demand misses in the visible window, with prefetch disabled as a control | 0 misses with prefetch; 62,528 miss-cycles per animated frame in the control |
| Whole core | Compiled in Quartus 17.0.2 and run on real MiSTer hardware | Validated 2026-07-27 |

Two lessons from that work are worth passing to anyone extending this core:

- **Byte-exact single-frame parity proves nothing about state-carrying bugs.**
  A tile-cache cold-miss defect that caused visible sparkle on animated tiles
  survived a "100 % byte-identical" parity result, because parity preloads a
  static RAM snapshot and renders one frame — so the cache is always warm by the
  time the compared frame is drawn. Catching it needed consecutive frames with a
  changing tilemap. Any future change to caching, prefetch, or other carried
  state needs a multi-frame test with changing input.
- **A `reg` array cleared wholesale on reset can never become block RAM**, and
  every indexed read of it becomes an N:1 mux. Packing the graphics cache's
  valid bits into the RAM word and invalidating sequentially after reset instead
  took that module from 14,817 ALMs to 233, and the whole design from 97 % to
  59 % utilisation.

Resource use of the shipped build (Cyclone V 5CSEBA6U23I7): 24,916 / 41,910 ALMs
(59 %), 524 / 553 RAM blocks (95 %), 42 / 112 DSP blocks, 3 / 6 PLLs. Timing
closes with TNS 0 on every clock: worst setup +0.271 ns, worst hold +0.200 ns,
and +1.626 ns of setup slack on the core's own 57.27272 MHz domain.
**Block RAM is the binding resource** — budget future work against it, not
against logic.

One build note worth knowing before you change fitter settings: the worst setup
path in this design is not in the core at all, it is in the framework's HDMI
pipeline (ascal's polyphase luma, and shadowmask feeding the HDMI OSD counters),
constrained at the 148.5 MHz 1080p60 pixel clock. It is tight enough that
placement decides whether it closes, which is why `Klax.qsf` pins a specific
fitter seed. Re-check timing if you change it.

---

## Building

Quartus Prime **17.0.2** (Lite or Standard), as required for MiSTer cores.

1. Open `Klax.qpf`.
2. Compile. The output is `output_files/Klax.rbf`.
3. For a release, copy it to `releases/Klax_YYYYMMDD.rbf`.

Sources are listed in `files.qip` — add files there by hand, not through the
Quartus IDE, which writes them into the `.qsf` instead and lets the two lists
drift apart. If the `.qsf` ever grows a pile of pin and instance assignments,
Quartus has regurgitated what `sys/sys.tcl` already provides; delete everything
after `source files.qip` except the `PARTITION_HIERARCHY` line.

There is one build configuration. `klax_top_mister` takes a single
`ENABLE_DDRAM_TILE_ROM` parameter, which selects between the DDRAM-backed
graphics ROM cache (the shipping choice) and an on-chip M10K bank; the latter
needs block RAM this design no longer has spare, so leave it at 1.

---

## Repository layout

```text
Klax.qpf / .qsf / .sdc / .srf   Quartus project
Klax.sv                         Glue between the MiSTer framework and the core
files.qip                       Source file list
rtl/                            The core
  TG68K/                        68000 CPU core
  pll/                          PLL megafunction
sys/                            MiSTer framework — unmodified
releases/                       Dated RBF + MRA files
```

`sys/` is a verbatim copy of [Template_MiSTer](https://github.com/MiSTer-devel/Template_MiSTer)
at commit `b4726d2` (2026-08-04), byte-identical, with no core-specific edits. The
`emu` port list comes from the framework's own `sys/emu_ports.vh` rather than
being copied into `Klax.sv`, so a framework update cannot leave the two
out of step.

---

## Licensing

The combined work is **GPL-3.0-or-later** (see `LICENSE`). The core's own RTL is
GPL-2.0-or-later, but it links components that require v3:

| Component | License |
|-----------|---------|
| `rtl/klax_*.v` (this core) | GPL-2.0-or-later |
| `Klax.sv` — MiSTer framework glue layer | GPL-2.0-or-later |
| `rtl/TG68K/` — TG68K 68000 core, © Tobias Gubener et al. | LGPL-3.0-or-later |
| `rtl/analog_hsize.sv` — © Umberto Parisi (rmonic79) | GPL-3.0-or-later |
| `sys/` — MiSTer framework, © Alexey Melnikov and contributors | GPL-2.0-or-later / GPL-3.0-or-later |

---

## Credits

- **Atari Games** — the original hardware and game. Klax debuted in 1989; the
  ROMs' own internal build stamps read 1990, which is why MAME dates every set
  1990 and the MRAs follow.
- **Tobias Gubener** and contributors — the TG68K 68000 core.
- **Umberto Parisi (rmonic79)** — `analog_hsize.sv`, ported from
  [Arcade-Raiden_MiSTer](https://github.com/rmonic79/Arcade-Raiden_MiSTer).
- **Alexey Melnikov (Sorgelig)** and the MiSTer-devel community — the MiSTer
  framework and the arcade infrastructure this core plugs into.
- **The MAME team** — `klax.cpp` (Aaron Giles), `atarimo.cpp`, `okim6295.cpp`
  and `okiadpcm.cpp` were the cross-reference for everything the schematics do
  not settle. References in the RTL comments of the form `mame/klax.cpp` point
  at MAME's own sources.
