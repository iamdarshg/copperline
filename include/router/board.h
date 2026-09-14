// Copperline: board data model and format importers.
//
// A Board is the single in-memory representation every importer targets:
// native JSON (authoritative schema), KiCad .kicad_pcb (phase 1 ingest),
// with Specctra DSN / SES / IPC-2581 reserved for Prompt 5 behind the same
// Importer interface. All coordinates are integer nanometres.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "router/geometry.h"
#include "router/json.h"

namespace copperline {

struct Layer {
    LayerId id = 0;
    std::string name;
    // Preferred routing direction / cost bias (phase 1: informational).
    bool preferred_horizontal = false;
    double cost_multiplier = 1.0;
    // Copper weight in oz (< 0 = inherit board default). 1 oz ~ 35 um.
    // Used by the ampacity width model (issue #7).
    double copper_weight_oz = -1.0;
    // Stackup position for the ampacity model: inner layers use the
    // internal (derated) IPC-2221 constant. Explicit override wins;
    // otherwise outer layers of the stackup count as external.
    bool has_internal_flag = false;
    bool is_internal = false;
};

struct Terminal {
    TermId id = 0;
    NetId net = -1;
    Point pos{};
    LayerId layer = 0;
    Coord pad_w_nm = 0;  // copper extent of the pad
    Coord pad_h_nm = 0;
    std::string component;  // e.g. "U1"
    std::string pin;        // e.g. "A3"
    Rect pad_rect() const { return Rect::from_center_size(pos, pad_w_nm, pad_h_nm); }
};

struct TraceSeg {
    NetId net = -1;
    LayerId layer = 0;
    Point a{};
    Point b{};
    Coord width_nm = 0;
    Segment segment() const { return {a, b}; }
};

struct Via {
    NetId net = -1;
    Point pos{};
    LayerId top_layer = 0;
    LayerId bottom_layer = 0;
    Coord outer_d_nm = 0;
    Coord hole_d_nm = 0;
    std::string via_class;  // style name; empty = board default
};

struct Keepout {
    Rect rect{};
    LayerId layer = kAllLayers;  // kAllLayers = applies on every layer
    std::string reason;
};

struct NetInfo {
    NetId id = -1;
    std::string name;

    // --- current / amperage intent (all optional) ---
    bool has_current = false;
    double current_a = 0.0;  // continuous
    bool has_peak = false;
    double peak_a = 0.0;
    bool has_min_width = false;
    Coord min_width_nm = 0;
    bool has_pref_width = false;
    Coord pref_width_nm = 0;
    std::string width_class;  // user-defined width/current class
    std::string via_class;    // via-class preference
    // KiCad-style per-net clearance floor: the enforced clearance between two
    // nets is max(...) of applicable floors (matches KiCam net-class DRC).
    bool has_min_clearance = false;
    Coord min_clearance_nm = 0;
    // Neckdown: narrower short section (e.g. BGA escape). Legal only when
    // explicitly allowed here or by board rule; the router never invents it.
    bool allow_neckdown = false;
    Coord neck_width_nm = 0;
    Coord neck_max_len_nm = 0;

    // --- voltage intent (optional) ---
    bool has_voltage = false;
    double voltage_v = 0.0;
    std::string voltage_class;

    std::vector<TermId> terminals;
};

struct BoardDefaults {
    Coord trace_width_nm = mm_to_nm(0.2);
    Coord clearance_nm = mm_to_nm(0.15);
    Coord via_outer_nm = mm_to_nm(0.6);
    Coord via_hole_nm = mm_to_nm(0.3);
    double default_current_a = 0.5;  // used only when a net has no current data
    double default_voltage_v = 0.0;
    // Ampacity metadata (issue #7): board-level copper weight and allowed
    // temperature rise used when no layer/sidecar/ctx override applies.
    double copper_weight_oz = 1.0;
    double temp_rise_c = 20.0;
};

struct Board {
    Coord width_nm = 0;
    Coord height_nm = 0;
    std::vector<Layer> layers;
    std::vector<NetInfo> nets;
    std::vector<Terminal> terminals;
    std::vector<TraceSeg> traces;  // committed copper (pre-routed + routed)
    std::vector<Via> vias;
    std::vector<Keepout> keepouts;
    BoardDefaults defaults;
    std::string source_format;  // "json" | "kicad_pcb" | ...
    std::string source_file;

    Rect bounds() const { return {0, 0, width_nm, height_nm}; }
    const NetInfo* find_net(NetId id) const;
    NetInfo* find_net(NetId id);
    const NetInfo* find_net_by_name(const std::string& name) const;
    const Terminal* find_terminal(TermId id) const;
    bool valid_layer(LayerId id) const;
};

// ---- Importer framework ----

struct ImportResult {
    Board board;
    std::vector<std::string> warnings;  // non-fatal approximations, all reported
};

class BoardImporter {
  public:
    virtual ~BoardImporter() = default;
    virtual std::string format_name() const = 0;
    // Returns true when this importer claims the file (by extension/content).
    virtual bool claims(const std::string& path, const std::string& head_bytes) const = 0;
    // Throws std::runtime_error with a human reason on invalid input.
    virtual ImportResult import_file(const std::string& path) const = 0;
};

// Native Copperline JSON schema (authoritative, fully specified).
class JsonBoardImporter : public BoardImporter {
  public:
    std::string format_name() const override { return "json"; }
    bool claims(const std::string& path, const std::string& head_bytes) const override;
    ImportResult import_file(const std::string& path) const override;
    // Shared with tests: import from an already-parsed value.
    ImportResult import_value(const JsonValue& root, const std::string& path) const;
};

// KiCad .kicad_pcb s-expression ingest (phase 1 subset; see README).
class KicadPcbImporter : public BoardImporter {
  public:
    std::string format_name() const override { return "kicad_pcb"; }
    bool claims(const std::string& path, const std::string& head_bytes) const override;
    ImportResult import_file(const std::string& path) const override;
    ImportResult import_text(const std::string& text, const std::string& path) const;
};

// Registry: tries importers in order; throws listing supported formats.
ImportResult import_board_auto(const std::string& path);
std::vector<std::string> supported_formats();

// Serialize a (possibly routed) board back to native JSON.
JsonValue board_to_json(const Board& board);

// Typed input failure: lets the CLI map invalid files vs malformed rules
// to distinct process exit codes without parsing prose.
enum class InputKind { kInvalid, kRule };

class BoardError : public std::runtime_error {
  public:
    InputKind kind;
    BoardError(InputKind k, const std::string& msg) : std::runtime_error(msg), kind(k) {}
};

}  // namespace copperline
