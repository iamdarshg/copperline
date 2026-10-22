#!/usr/bin/env python3
"""QSetup SFX payload extractor (clean-room).

Reverse-engineered from the DipTrace trial installer
(diptrace_trial_setup.exe, QSetup Installation Suite 12.0.0.5) by observing
the overlay structure. No vendor code used. Works on any QSetup-built SFX
that uses the same container format:

    PE32 stub ... [config/strings block] payload records:

    loop:
      u32 csize (little-endian)          ; compressed size
      zlib(csize)                        ; DEFLATE stream
        -> "|NNNNN#filename|unixtime|\\x00" + file bytes     (stream 9+)
        -> "|filename|unixtime|\\x00"   + file bytes          (stream 0-8)

The payload begins at the offset where the PE overlay starts
(= end of the last section / first file-mapped byte past the PE). For
streams 9+ the 5-digit NNNNN is the sequence number referenced by the
embedded build script (Setup.txt, stream 1) whose SET_SUB_DIR /
SET_COPY_FILES pairs map seq -> install path.

Usage:
    python qsetup_extract.py <installer.exe> <outdir>

Output:
    outdir/<NNNN>_<filename>   (name sanitized; NNNN = stream index)
    outdir/manifest.json       list of {stream, seq, name, mtime, offset,
                                        csize, size}
"""

import json
import os
import re
import struct
import sys
import zlib


MAGIC_RE = re.compile(rb'\|(\d*)([#*])([^|]*)\|(\d+)\|\x00')


def find_overlay(path: str) -> int:
    """Return the offset of the first byte after the PE image (the overlay)."""
    with open(path, 'rb') as f:
        data = f.read(0x1000)
    if data[:2] != b'MZ':
        raise SystemExit(f'not a PE file: {path}')
    e_lfanew = struct.unpack_from('<I', data, 0x3C)[0]
    if data[e_lfanew:e_lfanew + 4] != b'PE\x00\x00':
        raise SystemExit(f'no PE signature at {e_lfanew:#x}')
    nsec = struct.unpack_from('<H', data, e_lfanew + 6)[0]
    optsz = struct.unpack_from('<H', data, e_lfanew + 20)[0]
    secoff = e_lfanew + 24 + optsz
    # overlay = end of last section with nonzero raw size
    with open(path, 'rb') as f:
        f.seek(secoff + (nsec - 1) * 40)
        hdr = f.read(40)   # section header fields: VirtualSize@0 VirtualAddr@4
                           # SizeOfRawData@8 PointerToRawData@12 (rel to hdr, +8 overall)
    raw_size = struct.unpack_from('<I', hdr, 16)[0]
    raw_ptr = struct.unpack_from('<I', hdr, 20)[0]
    return raw_ptr + raw_size


def extract(path: str, outdir: str):
    os.makedirs(outdir, exist_ok=True)
    overlay = find_overlay(path)
    size = os.path.getsize(path)
    manifest = []

    with open(path, 'rb') as f:
        f.seek(overlay)
        data = f.read()          # overlay (payload + authenticode cert)

    pos = 0
    idx = 0

    def zlib_header_ok(b0, b1):
        # valid zlib header: deflate method, no preset dict, 31-divisible check
        return (b0 & 0x0F) == 0x08 and (b0 >> 4) <= 7 and not (b1 & 0x20) and \
               ((b0 << 8) + b1) % 31 == 0

    # Skip the plaintext SFX config block: it ends at the first payload record.
    # Payload records begin at the first position where a valid zlib stream
    # follows a small LE length; the config block has no such pattern before it.
    # Scan only the first 64 KiB (config blocks are tiny) for the anchor, and
    # confirm with a full decompression of the claimed size (rare attempt).
    while pos + 5 < len(data) and pos < 64 * 1024:
        (csize,) = struct.unpack_from('<I', data, pos)
        if 0 < csize < 64 * 1024 * 1024 and pos + 4 + csize <= len(data) and \
                zlib_header_ok(data[pos + 4], data[pos + 5]):
            try:
                zlib.decompress(data[pos + 4: pos + 4 + csize])
                break
            except zlib.error:
                pass
        pos += 1
    if pos >= 64 * 1024:
        raise SystemExit('no payload records found (unexpected format)')

    while pos + 5 <= len(data):
        (csize,) = struct.unpack_from('<I', data, pos)
        if csize <= 0 or pos + 4 + csize > len(data):
            break
        blob = data[pos + 4: pos + 4 + csize]
        try:
            out = zlib.decompress(blob)
        except zlib.error:
            break                     # end of records (certificate follows)
        head = out[:512]
        m = MAGIC_RE.match(head)
        if m:
            seq = m.group(1).decode('latin1', 'replace')
            sep = m.group(2).decode()
            name = m.group(3).decode('latin1', 'replace')
            mtime = int(m.group(4))
            body = out[m.end():]
        else:
            # simple "|name|ts|"? (streams 0-8)
            m2 = re.match(rb'\|([^|]*)\|(\d+)\|\x00', head)
            if m2:
                seq, sep, name, mtime = '', '', m2.group(1).decode('latin1', 'replace'), int(m2.group(2))
                body = out[m2.end():]
            else:
                # no header: keep whole content, synthetic name
                seq, sep, name, mtime = '', '', f'blob_{idx}', 0
                body = out
        safe = re.sub(r'[^A-Za-z0-9._-]+', '_', name) or f'file_{idx}'
        target = os.path.join(outdir, f'{idx:04d}_{safe}')
        with open(target, 'wb') as g:
            g.write(body)
        manifest.append({
            'stream': idx, 'seq': int(seq) if seq.isdigit() else None,
            'sep': sep, 'name': name, 'mtime': mtime,
            'offset': overlay + pos, 'csize': csize, 'size': len(body),
        })
        pos += 4 + csize
        idx += 1

    with open(os.path.join(outdir, 'manifest.json'), 'w') as g:
        json.dump(manifest, g, indent=1)
    total = sum(m['size'] for m in manifest)
    print(f'extracted {idx} files, {total} bytes uncompressed')
    print(f'last record ended at file offset {overlay + pos:#x}')
    return manifest


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    extract(sys.argv[1], sys.argv[2])