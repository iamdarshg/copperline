# DipTrace `.dip` native binary format — clean-room observations

Companion to `docs/reverse/diptrace/README.md`. These are **byte-level
observations** from the extracted installer's example boards
(`example_unrouted.dip` vs `example_routed.dip`, `Astable_Flip_Flop.dip`,
`BGA_autorouter.dip`) plus semantic labels from Novarm's **public** XML
format specifications (the XML dialect is the ASCII twin of this binary
record stream). No decompiled code; no proprietary implementation text.

Status: **PARTIAL decode.** The record framing, string encoding and the
component/pin record layout are confirmed with byte evidence. The
net/trace/via binary payloads are located and bounded but not fully
field-decoded (the DSN/SES path remains the production import route —
see `router_engine.md` §5).

---

## 1. Container (confirmed)

| Field | Value |
|---|---|
| Magic | `07 44 54 42 4F 41 52 44` = `\x07DTBOARD` (board), `\x07DTSCHEM` (schematic) |
| Library magics | `\x06DTCLIB` (pattern lib), `\x06DTELIB` (component lib) |
| After magic | binary record stream; `0F 42 xx` bytes are the Delphi "single+shortstring" serializer signature |
| Encoding | **u16 length-prefixed UTF-16LE strings** interleaved with LE 4-byte IEEE-754 floats (`00 00 0F 42` = 35.75) and small ints |

String framing observed (offset `0x3EE` in `example_unrouted.dip`):

```
3e9: 42 40 00 03 00 43 00 41 00 50 00   02 00 43 00 31 00 00 00
     |─── float …──| len=3  C  A  P       len=2  C  1        (len=0)
```

i.e. `u16 len` then `len` UTF-16LE code units; `len=0` = empty string.
Component records emit three consecutive strings:
`pattern-style` → `refdes` → `value` (value may be empty).

## 2. Top-level layout (observed)

```
0x0000  magic + version floats
0x000c  u16-len UTF-16LE: source schematic path (e.g. "…\Schematic_6.dch")
        + float run (view settings)
…       component records (confirmed layout below)
…       [routed files only] trace/via/net record block (located, partially decoded)
tail    shared board-footer record block (identical bytes in routed & unrouted
        files at the very end)
```

Routed-vs-unrouted evidence (`example_routed.dip` 538,117 B vs
`example_unrouted.dip` 202,409 B):

* first divergent byte: `0x11` — only the embedded source-path string
  differs (routed was saved from `Schematic_6.dch`, unrouted from a
  different schematic)
* component/placement region is **byte-identical** across the pair
* the routed file is +335,708 B, all inserted before the shared tail:
  the unrouted file's final 128-byte tail was found at `0x801D3` in the
  routed file ⇒ trace/via records are appended between components and
  the footer.

## 3. Component record (confirmed)

A component is a contiguous run of:

```
u16-len UTF-16LE: pattern style (e.g. "CAP", "RES", "IDC2X17M", "DIMM72")
u16-len UTF-16LE: refdes (e.g. "C1", "R9", "U1")
u16-len UTF-16LE: value ("22uF", "47k", may be empty)
binary: float X, float Y, float rotation, size/flags run
u16-len UTF-16LE pin-name strings (e.g. U1 = "26 25 10 12 14 … 34":
       one length-prefixed string per pin, each followed by binary floats
       = pin offset/size data)
u16-len UTF-16LE: 3D model path ("Files\DipTrace\models3d\…")
```

Evidence (pin strings): `Astable_Flip_Flop.dip` U1 (IDC2X17M) emits 26
pin strings `26 25 10 12 14 16 18 20 19 17 15 13 11 21 22 23 24 27 28
29 30 31 32 33 34`; `example_unrouted.dip` U3 (IDC40) emits 35–40, U4
(IDC44) 41–44, U5 (DIMM72) 45–66 — pin numbering strings, one per pad.

## 4. Net records (partially decoded)

Net names appear as u16-len UTF-16LE strings preceded by a binary float
run. Observed at `0x2DA38` in `example_unrouted.dip`:

```
00 03 00 33 00 56 00 33   = len 3, "3V3"
00 02 00 35 00 56         = len 2, "5V"
00 02 00 41 00 30         = len 2, "A0"
```

Each is preceded by `… 00 0F 42 40 0F 42 40 0F 42 40 01 00`-style float
runs (position/size) and followed by `3B 9B 3F 30 3B 9A E7 4C 01 00 00 …`
(pad membership? cost values?). Only a handful of nets are stored as
named strings (power nets); signal nets appear to be identified by
binary indices, not names.

## 5. Trace/via block (located, not field-decoded)

The routed file's extra ~335KB contains the copper. Observable pattern
in the tail region: repeating `3B 9A CA 00` (LE float ≈ 0.00472 =
1/100000-inch-scale? — also read as `0x00CA9A3B` = 13273659 when u32)
and `00 00 0F 42 3F`/`0F 42 40` (floats 0.558/2.234/8.937/35.75 —
consistent with coordinate scales ×4), plus `0F 42 50 0F 42 71 …`
runs. Field positions (x1,y1,x2,y2,width,layer,vianet) are *probable*
but not proven; a full decoder needs a sample set of known traces.

Honest status: **do not use for production import yet.** The proven
production path is DSN/SES (`router_engine.md` §5) or the XML plugin
exchange (`dip_binary_format`'s ASCII twin).

## 6. Unit evidence

`unitscore.py` brute force over `Astable_Flip_Flop.dip`:
`be-u32 × 2.54` scored 169 (coordinates cluster on the 100-mil/2.54-mm
through-hole grid) vs `le-u32 × 2.54` = 151, `le-f32` = 41. This is
consistent with parts placed on a 2.54 mm pitch (RES400 pads = 10.16 mm
= 400 mil), but it is a *clustering* signal, not a proven unit; the
native serializer stores positions as LE floats with the X4 scale hints
above.

## 7. Reproducible probes

See the scratch analysis scripts (offsets above are byte-exact):
`%TEMP%\opencode\re-dt\probe1..5.py`, `unitscore.py`, `scale_hunt.py`,
`diff_big.py`, `period_test.py`. The small already-copied fixtures in
`fixtures/reverse/diptrace/` reproduce all confirmed observations.
