# Where the 21 corpus boards come from

The harness list (`boards21.txt`) uses absolute paths, including temporary
checkouts that no longer exist. This file records the real origins so the set
can be rebuilt from scratch.

## KiCad 9 demos (8 boards, always available locally)

Installed with KiCad at `C:\Program Files\KiCad\9.0\share\kicad\demos\`:

| board | relative path |
|---|---|
| `custom_pads_test` | `custom_pads_test/custom_pads_test.kicad_pcb` |
| `ecc83-pp` | `ecc83/ecc83-pp.kicad_pcb` |
| `flat_hierarchy` | `flat_hierarchy/flat_hierarchy.kicad_pcb` |
| `interf_u` | `interf_u/interf_u.kicad_pcb` |
| `pic_programmer` | `pic_programmer/pic_programmer.kicad_pcb` |
| `sonde xilinx` | `sonde xilinx/sonde xilinx.kicad_pcb` |
| `test_pads_inside_pads` | `test_pads_inside_pads/test_pads_inside_pads.kicad_pcb` |
| `tinytapeout-demo` | `tiny_tapeout/tinytapeout-demo.kicad_pcb` |

## GitHub repositories (13 boards, frozen as pads-only JSON in `boards/`)

| repo | boards | in-repo path |
|---|---|---|
| https://github.com/bt-soft/pico-radio-9 | `pico-radio-9` (b009), `pico-radio-9-kotoldalas` (b008) | `kicad/pico-radio-9/pico-radio-9.kicad_pcb`, `kicad/pico-radio-9-k<...>/pico-radio-9-k<...>.kicad_pcb` |
| https://github.com/sabogalc/project-piCo | `Pico 2 C` (b010), `RPI-PICO-R3a-PUBLIC` (b011), `Pico WH` (b012) | `KiCad Projects/Pico 2 C/Pico 2 C.kicad_pcb`, `KiCad Projects/Pico C/RPI-PICO-R3a-PUBLIC.kicad_pcb`, `KiCad Projects/Pico WH - No License/Pico WH.kicad_pcb` |
| https://github.com/raspberrypipress/raspberry-pi-pico-with-kicad | `working_with_schematics` (b013), `laying_out_a_pcb` (b014), `stamp_and_module` (b015), `h_bridge` (b016), `KiCAD_Board_3_Minimal_Full_RP2040` (b017), `rp2040_nema` (b018), `rp2040_game_con` (b019) | `eg/ch01/...`, `eg/ch02/...`, `eg/ch03/...`, `eg/ch04/ch05`/`ch10`/`ch11/...` |
| https://github.com/OpenJBOD/rp2040 | `OpenJBOD-RP2040` (b020) | `OpenJBOD-RP2040.kicad_pcb` |

(The `k<...>` and chapter paths are as they appear on disk; the non-ASCII
directory name of the second `pico-radio-9` board is the Hungarian
"kotoldalas" spelling.)

Because these four checkouts lived only in the temporary workspace, the
pads-only inputs are committed next to this file as
`boards/bNNN_stripped.json`. The index in the filename is the row index in
`corpus.json`, whose `board` field names the board. They can be routed
directly:

```
build/router.exe route docs/validation/corpus/2026-10-29/boards/b010_stripped.json \
  --json --no-board-echo --seed 42 --threads 4 --memory-budget-mb 512 \
  --timeout 180 --report /tmp/b010.rep.json
```

## Filtering

The corpus was built with `tools/build_corpus.py`: signature dedupe (board
size, layer count, net count, pad count), at least 4 pads, and only boards
whose pads-only input verifies clean. The wide 118-board sweep behind that
selection (`corpus/wide.json`) is kept too.
