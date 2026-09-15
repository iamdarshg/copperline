// Copperline: Specctra DSN import + SES export (Prompt 5 adapters).
//
// One real PCB workflow, kept separate from the routing core:
//   DSN file -> Board (import) -> RouterEngine -> Board -> SES file (export)
//
// Documented DSN subset (writer + reader agree; anything else warns):
//   (pcb <name>
//     (unit mm|mil|um|inch)            ; default mm
//     (structure
//       (layer <name> (type signal|plane))
//       (boundary (rect x1 y1 x2 y2))  ; mm (in unit)
//       (via <name> <outer> <hole>)    ; via geometries, same unit
//       (rule (width w) (clearance c)) ; board defaults
//       (plane <net> (polygon ...) (layer <name>))
//       (copper_pour [<net>|(net <net>)] [(polygon ...)|(rect ...)|
//                    (circle ...) -octagon approx-|(path ...) -closed-]
//                    [(layer <name>)] [(window ...) -cutouts ignored-])
//       ((wire|via|place|bend|elongate)_keepout [(polygon ...)|(rect ...)|
//                    (circle ...)] [(layer <name>)] -polygons by bbox-))
//     (placement (component <ref> (place <part> x y <front|back> <rot>))
//                ...)
//     (library (image <part> (pin <pad> x y) ... (outline ...) ) ...)
//     (network
//       (net <name> (pins <ref-pad> ...) (class <class>))
//       (class <name> (rule (width w) (clearance c))
//              (circuit (use_via <via> ...)))))
//   (plane ...) and (copper_pour ...) map to PlaneZone entries when they
//   name a known net and layer (each pour keeps its own island id: no
//   silent stitching across pours; overlaps still bridge geometrically).
//   Unresolvable copper is SKIPPED with an explicit warning (never
//   silently dropped). Polygon vertex counts are bounded (16k); larger
//   shapes warn and skip.
//
// SES subset (electrical EDA session):
//   (session <file> (base_design <name>)
//     (route (library (padstack <via> <outer> <hole>) ...)
//            (network (net <name> (wire (path <layer> x1 y1 x2 y2 ... <w>)
//                                        (via <via> x y) ...)) ...)))
// Coordinates are written in mm with 6 decimals and re-quantized to integer
// nm on import (the same mm boundary quantization as native JSON).
#pragma once

#include <string>
#include <vector>

#include "router/board.h"

namespace copperline {

// Specctra DSN importer (registered in import_board_auto for *.dsn).
class DsnImporter : public BoardImporter {
  public:
    std::string format_name() const override { return "dsn"; }
    bool claims(const std::string& path, const std::string& head_bytes) const override;
    ImportResult import_file(const std::string& path) const override;
    ImportResult import_text(const std::string& text, const std::string& path) const;
};

// Serialize a (possibly routed) board to a Specctra SES session string.
// Only committed copper (traces/vias) is emitted; metadata (net names,
// layers, via geometries) round-trips through the library/network headers.
std::string board_to_ses(const Board& board, const std::string& base_design = "");

// Parse an SES session and merge its copper into `board` (nets matched by
// name; unknown nets/layers are errors). Returns warnings for ignored lines.
// Throws BoardError on malformed input.
std::vector<std::string> ses_merge_into(Board& board, const std::string& text,
                                        const std::string& path);

// Merge committed copper from another board file (native JSON or SES) into
// `board`, remapping net ids by net name. Used by `router verify --routes`.
std::vector<std::string> merge_routes_file(Board& board, const std::string& path);

// Sidecar extension (Prompt 5): apply a "nets" object from --config onto an
// already-imported board, so DSN/KiCad boards without current/voltage intent
// gain it without replacing the existing ampacity/voltage_table system:
//
//   {"nets": {"VBAT": {"current_a": 8.0, "voltage_v": 16.8,
//                       "trace_width_min_mm": 1.2, "class": "HIGH_CURRENT",
//                       "clearance_mm": 0.5, "via_class": "POWER",
//                       "voltage_class": "HV", "width_class": "PWR"}}}
// Throws BoardError(kRule) on bad values; unknown nets are errors (typos
// must not pass silently).
void apply_sidecar_nets(Board& board, const JsonValue& config);

// Registry singleton (defined in dsn.cpp; referenced by board.cpp).
const BoardImporter& dsn_importer_singleton();

}  // namespace copperline
