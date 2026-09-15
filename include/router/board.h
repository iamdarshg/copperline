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
    // ---- Issue #11: stackup metadata for impedance-aware routing ----
    // layer_type is "signal" (routable) or "plane" (reference copper).
    std::string layer_type = "signal";
    // Height to the reference plane (dielectric thickness). has_* false =
    // unspecified, in which case the layer is ineligible for impedance
    // solving (the router falls back to current-aware widths).
    bool has_dielectric_thickness = false;
    Coord dielectric_thickness_nm = 0;
    bool has_dielectric_er = false;
    double dielectric_er = 0.0;  // relative permittivity (e.g. FR-4 ~= 4.4)
    // Reference-plane identity: which layer is the impedance reference.
    // kAllLayers (-1) = unspecified (any plane; no preference penalty).
    bool has_ref_plane = false;
    LayerId ref_plane_layer = kAllLayers;
    // Explicit foil thickness override (integer nm). When unset the foil is
    // derived from copper_weight_oz (1 oz ~= 34.8 um).
    bool has_copper_thickness = false;
    Coord copper_thickness_nm = 0;
    // Per-layer estimator override: "" = auto (outer -> microstrip, inner ->
    // stripline), else "microstrip" | "stripline".
    std::string impedance_model;
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

// Issue #16: declared power-plane / copper-pour zone.
//
// A plane is fixed copper owned by one net on one layer. Terminals of that
// net may terminate directly into the plane (short low-impedance access)
// instead of running point-to-point traces. `island` groups planes of one
// net into electrically connected sets: planes sharing (net, island) are
// declared connected (e.g. stitched pours); different islands are isolated
// unless routed copper visibly bridges them. `routable` marks whether the
// zone may be used as a routing target (false = reference copper only).
struct PlaneZone {
    int id = -1;
    NetId net = -1;
    LayerId layer = 0;
    std::vector<Point> poly;  // integer-nm polygon, >= 3 points
    int island = 0;
    bool routable = true;
    Rect bounds() const;
};

// Issue #12: differential-pair declaration.
//
// A pair couples two 2-terminal nets (P/N) that must be planned as one
// corridor resource and materialized as two coupled traces post-route.
// gap_nm is the edge-to-edge target spacing between P and N copper;
// gap_tol_nm is the symmetric tolerance. width_nm (when has_width) is the
// explicit trace width for BOTH members (still floored by the ampacity
// minimum, like any explicit width). preferred_layers restricts corridor
// layer choice (empty = any signal layer). target impedance is
// informational for #12 (single-ended #11 hooks size the width; odd-mode
// solving is out of scope). max_skew_nm caps |len(P)-len(N)| (0 with
// has_max_skew=false = unchecked). via_policy is "paired" (default:
// symmetric paired vias) or "independent" (still committed atomically,
// but each member picks its own transition site when the corridor allows;
// v1 materializes both atomically either way).
struct DiffPair {
    int id = -1;
    std::string name;
    NetId net_p = -1;
    NetId net_n = -1;
    Coord gap_nm = 0;
    Coord gap_tol_nm = 0;
    bool has_width = false;
    Coord width_nm = 0;
    std::vector<LayerId> preferred_layers;
    bool has_impedance = false;
    double target_impedance_ohms = 0.0;
    bool has_max_skew = false;
    Coord max_skew_nm = 0;
    std::string via_policy = "paired";
    // Issue #15: pair-aware length-tuning mode. False (default) tunes the
    // shorter member only (minimum added length to satisfy skew/targets).
    // True tunes both members symmetrically to a common target length
    // (fixes skew to ~0 while meeting single-ended targets on both).
    bool symmetric_tuning = false;
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

    // --- impedance intent (issue #11, optional) ---
    // Single-ended target impedance. has_impedance false = no controlled
    // impedance (router behavior for the net is unchanged).
    bool has_impedance = false;
    double target_impedance_ohms = 0.0;
    // Fractional tolerance, e.g. 0.10 = +/-10%. Default when the net names
    // a target but no tolerance.
    double impedance_tolerance_frac = 0.10;
    bool has_impedance_tolerance = false;
    // Eligible signal layers for this net. Empty = every eligible signal
    // layer is a candidate. Unknown layer ids are rejected at import.
    std::vector<LayerId> impedance_layers;
    // Preferred reference plane identity. Layers whose ref_plane_layer
    // matches are preferred (soft cost); others stay feasible.
    bool has_impedance_ref_plane = false;
    LayerId impedance_ref_plane = kAllLayers;

    // ---- Issue #15: single-ended length-tuning intent (optional) ----
    // target_length_nm is the desired total committed copper length for
    // this net; length_tol_nm is the symmetric acceptance window. The
    // post-route LengthTuner adds deterministic trombone meanders to reach
    // [target-tol, target+tol]. has_target_length false = no tuning.
    bool has_target_length = false;
    Coord target_length_nm = 0;
    Coord length_tol_nm = 0;

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
    std::vector<PlaneZone> planes;  // issue #16: declared plane/zone copper
    std::vector<DiffPair> diffpairs;  // issue #12: differential-pair declarations
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

// ---- Issue #16: integer-nm polygon helpers for plane geometry ----

// Exact point-in-polygon (boundary counts as inside). Deterministic.
bool plane_poly_contains(const std::vector<Point>& poly, Point p);
// Nearest point on the polygon boundary (or the point itself when inside).
Point plane_poly_nearest(const std::vector<Point>& poly, Point p);
// True when the segment touches or crosses the polygon.
bool plane_seg_hits_poly(const Segment& s, const std::vector<Point>& poly);
// True when the rect touches or overlaps the polygon.
bool plane_rect_hits_poly(const Rect& r, const std::vector<Point>& poly);
// Exact squared centerline distance from a segment to a polygon (0 inside).
__int128 plane_seg_poly_dist2(const Segment& s, const std::vector<Point>& poly);
// Exact squared edge-to-edge distance from a rect to a polygon (0 on hit).
__int128 plane_rect_poly_dist2(const Rect& r, const std::vector<Point>& poly);

// Typed input failure: lets the CLI map invalid files vs malformed rules
// to distinct process exit codes without parsing prose.
enum class InputKind { kInvalid, kRule };

class BoardError : public std::runtime_error {
  public:
    InputKind kind;
    BoardError(InputKind k, const std::string& msg) : std::runtime_error(msg), kind(k) {}
};

}  // namespace copperline
