#!/usr/bin/env python3
"""Clean-room DipTrace `.dip`/`.dch` native-binary reader → copperline JSON.

Based on byte-level observations documented in
`docs/reverse/diptrace/dip_binary_format.md`. This is a PARTIAL decoder:
it extracts the confirmed record framing (u16-length-prefixed UTF-16LE
strings interleaved with LE floats) and the component/pin records, and
locates net-name strings. The trace/via binary payloads are located but
not field-decoded — boards imported this way are placed-components +
nets only, intended as a starting skeleton for the DSN/SES flow.

No vendor code is used or reproduced. See the format notes for the
evidence behind every field.

Usage:
    python tools/diptrace_import.py <in.dip> [--out board.json]

Output: copperline native board JSON, schema `copperline/board/1`, with
an `import_warnings` list stating exactly what is decoded vs guessed.
"""

from __future__ import annotations

import argparse
import json
import re
import struct
import sys


def get_string(data: bytes, off: int):
    """Read a u16-length-prefixed UTF-16LE string at off.

    Returns (next_off, string) on success, (off, None) if the bytes at
    off do not look like a valid framed string.
    """
    if off + 2 > len(data):
        return off, None
    ln = struct.unpack_from('<H', data, off)[0]
    if not (0 < ln < 512):
        return off, None
    end = off + 2 + 2 * ln
    if end > len(data):
        return off, None
    raw = data[off + 2:end]
    # UTF-16LE check: every other byte must be 0 for ASCII-range strings
    if any(raw[i] != 0 for i in range(1, 2 * ln, 2)):
        return off, None
    try:
        s = raw.decode('utf-16-le')
    except UnicodeDecodeError:
        return off, None
    if not all(32 <= ord(c) < 127 for c in s):
        return off, None
    return end, s


def walk_strings(data: bytes, start: int = 8, end: int | None = None):
    """Iterate (offset, string) for every confirmable framed string."""
    end = end if end is not None else len(data)
    off = start
    while off + 2 <= end:
        no, s = get_string(data, off)
        if s is not None:
            yield off, s
            off = no
        else:
            off += 1


def parse_board(data: bytes, name: str = "imported") -> dict:
    """Parse the confirmed portions of a .dip file.

    Returns copperline board JSON (schema copperline/board/1).  nets and
    terminals are populated from component/pin strings and net-name
    strings; geometry is NOT decoded (see import_warnings).
    """
    if len(data) < 16 or not data.startswith(b'\x07DTBOARD'):
        raise ValueError('not a DipTrace board file (magic \\x07DTBOARD missing)')

    warnings = [
        'PARTIAL import: record framing, components and net names only.',
        'Trace/via coordinates are not decoded (binary payloads need a',
        'full decoder). Use the DSN/SES flow for routed copper.',
    ]
    strings = list(walk_strings(data))

    # --- components: pattern/refdes/value triples -------------------------
    # Confirmed record framing (see dip_binary_format.md §3):
    #   pattern-str, refdes-str, value-str, [binary], pin-names..., 3D path
    # Component records are separated by binary runs; a record ends when
    # we reach a 3D path string (model path) or the next pattern+refdes.
    components = []   # {"ref": .., "pattern": .., "value": .., "pins": [..]}
    i = 0
    n = len(strings)
    S = lambda idx: strings[idx][1] if 0 <= idx < n else None
    seen_refs = set()

    def is_path(s):
        return (s.endswith(('.wrl', '.png', '.dip', '.dch'))
                or s.startswith(('Files\\', '\\Program', '\\\\Program',
                                 'D:\\', 'C:\\')))

    def looks_value(s):
        # short printable string that is not a refdes/path/net frame word
        if not s or len(s) > 32 or is_path(s):
            return False
        return not re.fullmatch(r'[A-Z]\d+.*', s) or len(s) > 1 and not s.isdigit()

    while i < n - 2:
        s1 = S(i)
        s2 = S(i + 1)
        s3 = S(i + 2)
        # refdes-like: letter(s) + digits, e.g. C1, R9, U1, LED1, IC2
        if (s2 and re.fullmatch(r'[A-Z][A-Za-z0-9_]*\d+', s2)
                and s2 not in seen_refs
                and s1 and not re.fullmatch(r'\d+', s1)
                and not is_path(s1)):
            # value is the string right after the refdes (may be empty
            # string len=0 which the walker skips -> use s3 when it is
            # there and value-like, else '')
            value = ''
            j = i + 2
            if s3 is not None and looks_value(s3) and len(s3) <= 16:
                value = s3
                j = i + 3
            # pin strings follow: short alphanumeric pin names (e.g.
            # PLUS, MINUS, 1, 2, A, K, E, B, C) until a 3D path or the
            # next component triple (pattern+refdes) is reached.
            pins = []
            while j < n:
                sj = S(j)
                if sj is None:
                    break
                if is_path(sj):
                    j += 1
                    break  # 3D model path ends the record
                if sj in seen_refs:
                    break
                if (re.fullmatch(r'[A-Za-z0-9_\-]{1,8}', sj)
                        and not re.fullmatch(r'[A-Z][A-Za-z0-9_]*\d{2,}', sj)
                        and len(sj) <= 8):
                    # numeric or latin pin label
                    if sj.isdigit() or re.fullmatch(r'[A-Za-z]{1,3}', sj) or (
                            sj in ('PLUS', 'MINUS', 'A', 'K', 'B', 'C', 'E')):
                        pins.append(sj)
                        j += 1
                        continue
                # if we hit a probable next-component pattern, stop
                nxt = S(j + 1)
                if (nxt and re.fullmatch(r'[A-Z][A-Za-z0-9_]*\d+', nxt)
                        and not is_path(sj)):
                    break
                break
            components.append({
                'ref': s2,
                'pattern': s1,
                'value': value,
                'pins': pins,
            })
            seen_refs.add(s2)
            i = j
            continue
        i += 1

    # --- net names ----------------------------------------------------------
    # Net-name strings are short alnum with a digit/letter mix that are
    # NOT pins and NOT parts. Prefer strings that look like power/signal
    # net names: <= 12 chars, contain a letter and a digit or VCC/GND/…
    pin_sets = set(p for c in components for p in c['pins'])
    net_names = []
    frame_words = {'Arial', 'SIZE', 'REV', 'SCALE', 'REVISIONS', 'ZONE',
                   'DESCRIPTION', 'DATE', 'APPROVED', 'TITLE', 'FILE',
                   'SHEET', 'OF', 'DRAWN', 'CHECKED', 'ENGINEER'}
    for off, s in strings:
        if s in pin_sets or s in seen_refs:
            continue
        if s in frame_words:
            continue
        if not re.fullmatch(r'[A-Za-z0-9_.\-]{1,12}', s):
            continue
        if re.fullmatch(r'\d+', s):
            continue
        if is_path(s):
            continue
        if s not in net_names:
            net_names.append(s)

    # --- build copperline board JSON --------------------------------------
    board = {
        'schema': 'copperline/board/1',
        'board': {'width_mm': 100.0, 'height_mm': 100.0},  # unknown: default
        'layers': [
            {'id': 0, 'name': 'Top'},
            {'id': 1, 'name': 'Bottom'},
        ],
        'defaults': {
            'trace_width_mm': 0.2,
            'clearance_mm': 0.15,
            'via_outer_mm': 0.6,
            'via_hole_mm': 0.3,
            'default_current_a': 0.5,
        },
        'nets': [],
        'planes': [],
        'traces': [],
        'vias': [],
        'import_warnings': warnings,
    }

    # one net per identified net name; terminals attached as vias come
    # from component pins when a net name matches a pin's owning net is
    # NOT decodable, so terminals are placed at component centroids as
    # unconnected stubs (honest skeleton).
    net_ids = {}
    for k, name in enumerate(net_names):
        net_ids[name] = k
        board['nets'].append({
            'id': k,
            'name': name,
            'terminals': [],
        })

    # place every component as a terminal on its own net so the board is
    # routable as a netlist skeleton (one terminal per pin, coordinates
    # 0 = unknown, reported as skeleton terminals). Terminal ids are
    # globally unique across nets (copperline board model).
    next_net = len(board['nets'])
    next_term = 0
    for c in components:
        if not c['pins']:
            continue
        # group all pins of this component under one skeleton net with
        # terminals at synthetic positions
        nid = next_net
        next_net += 1
        board['nets'].append({
            'id': nid,
            'name': f'SKEL_{c["ref"]}',
            'terminals': [],
        })
        for p in c['pins']:
            board['nets'][-1]['terminals'].append({
                'id': next_term,
                'x_mm': 0.0, 'y_mm': 0.0, 'layer': 0,
            })
            next_term += 1

    board['_components'] = components
    return board


def main() -> int:
    ap = argparse.ArgumentParser(description='DipTrace .dip partial importer')
    ap.add_argument('input', help='.dip board file')
    ap.add_argument('--out', help='output JSON (default: stdout)')
    ap.add_argument('--summary', action='store_true',
                    help='print a human summary instead of full JSON')
    args = ap.parse_args()

    with open(args.input, 'rb') as fh:
        data = fh.read()

    try:
        board = parse_board(data, name=args.input)
    except ValueError as exc:
        print(f'diptrace_import: {exc}', file=sys.stderr)
        return 2

    comps = board.get('_components', [])
    if args.summary:
        print(f'components: {len(comps)}')
        for c in comps[:15]:
            print(f'  {c["ref"]:8s} {c["pattern"]:16s} '
                  f'value={c["value"]!r:20s} pins={len(c["pins"])}')
        print(f'nets (named): {len(board["nets"])}')
        for n in board['nets']:
            print(f'  {n["id"]}: {n["name"]} terminals={len(n["terminals"])}')
        print('WARNINGS:')
        for w in board['import_warnings']:
            print(f'  ! {w}')
        return 0

    out = {k: v for k, v in board.items() if not k.startswith('_')}
    out['component_count'] = len(comps)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, indent=1)
        print(f'wrote {args.out}')
    else:
        print(json.dumps(out, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())