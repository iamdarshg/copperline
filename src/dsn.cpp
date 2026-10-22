// Copperline: Specctra DSN import + SES export (Prompt 5 adapters).
//
// Adapters live here, separate from the routing core: they translate between
// EDA file formats and the in-memory Board every engine/verifier caller
// shares. No routing decisions are made in this file.
#include "router/dsn.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <map>
#include <sstream>
#include <stdexcept>

#include "router/sexpr.h"

namespace copperline {
namespace {

// Unit scale to mm.
double unit_to_mm(const std::string& unit) {
    if (unit == "mm") return 1.0;
    if (unit == "um") return 0.001;
    if (unit == "mil") return 0.0254;
    if (unit == "inch" || unit == "in") return 25.4;
    if (unit == "cm") return 10.0;
    return -1.0;
}

bool ends_with(const std::string& s, const std::string& suf) {
    return s.size() >= suf.size() &&
           s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

// Strict numeric atom: integers, floats and the trailing-dot forms Altium
// emits ("10." / "0."). Anything else (layer names, reserved tokens) is not a
// number, which is how the parsers below tell `(rect pcb 0 0 20 20)` from
// `(rect 0 0 20 20)` without a grammar table.
bool parse_number(const std::string& s, double& out) {
    if (s.empty()) return false;
    const char* begin = s.c_str();
    char* end = nullptr;
    double v = std::strtod(begin, &end);
    if (end == begin || (end && *end != '\0')) return false;
    out = v;
    return true;
}

bool is_number_atom(const SexprNode* n) {
    double v = 0.0;
    return n && n->is_atom && parse_number(n->atom, v);
}

// Cross-dialect identifier matching: Altium/FreeRouting quote human names
// ("Top Layer") while canonical Specctra uses reserved upper-case tokens
// (TOP). Separators (space/tab/underscore/dash) are dropped and the result is
// upper-cased, so `"Top Layer"`, `TopLayer` and `TOP` are one layer.
// Applied to LAYER and PADSTACK identifiers only: net names stay exact so a
// board never silently merges `VCC-1` with `VCC1`.
std::string norm_name(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == ' ' || c == '\t' || c == '_' || c == '-') continue;
        out += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return out;
}

// First atom that parses as a number. Specctra list shapes carry an optional
// leading layer token (`(rect pcb 0 0 20 20)`, `(polygon Bottom 1 8 ...)`);
// older writers omit it. Detect it per shape instead of assuming either form.
std::size_t first_numeric_atom(const SexprNode* n, std::size_t start = 1) {
    if (!n) return start;
    for (std::size_t i = start; i < n->children.size(); ++i)
        if (is_number_atom(n->children[i].get())) return i;
    return start;
}

double atom_num(const SexprNode* n, std::size_t idx, double fallback = 0.0) {
    if (!n || idx >= n->children.size() || !n->children[idx]->is_atom) return fallback;
    try {
        return std::stod(n->children[idx]->atom);
    } catch (...) {
        return fallback;
    }
}

std::string atom_str(const SexprNode* n, std::size_t idx, const std::string& fallback = "") {
    if (!n || idx >= n->children.size() || !n->children[idx]->is_atom) return fallback;
    return n->children[idx]->atom;
}

std::string read_file_text(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw BoardError(InputKind::kInvalid, "cannot open file: " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

void write_mm(std::ostringstream& os, Coord nm) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6f", nm_to_mm(nm));
    // Writer contract: SES coordinates are always mm (the session declares
    // (unit mm)); the reader quantizes back to integer nm.
    os << buf;
}

// Parse a (width ...) rule the way the Specctra router reads it (M9):
// the preferred value drives routing, the minimum is the floor, and the
// maximum is out of scope (recorded, never enforced). Three spellings are
// accepted because exporters disagree:
//   (width 0.2)                    - single value: min = pref = 0.2
//   (width 0.15 0.2 0.3)           - positional: min pref max
//   (width (min 0.15) (pref 0.2))   - labelled fragment
// `scale_mm` is the file's coordinate quantum (mm per count).
struct WidthTriplet {
    Coord min_nm = 0;
    Coord pref_nm = 0;
    bool has_pref = false;
    std::string source;
};

struct DsnClassRule {
    std::string name;
    bool has_width = false;
    Coord width_nm = 0;
    bool has_pref_width = false;
    Coord pref_width_nm = 0;
    std::string width_source;  // "single" | "pref" | "min" (M9: preferred else min)
    bool has_clearance = false;
    Coord clearance_nm = 0;
    std::vector<std::string> use_via;
    // Item 9 (M9 scope flattening): per-layer width/clearance variants.
    // Specctra scopes are (netclass on layer); the router resolves the
    // most specific scope: layer-scoped > class-wide. Maps keyed by the
    // DSN layer name as written ("Top Layer"/"TOP"/"TopLayer" unify via
    // normalize_layer_name during class resolution).
    std::map<std::string, WidthTriplet> layer_width;    // layer -> (min/pref)
    std::map<std::string, Coord> layer_clearance;       // layer -> nm
    // Item 12 (Electra via policy knobs, parsed + carried as soft inputs;
    // enforcement lives in via-bundle cost, never hard DRC).
    bool via_share = false;          // (via_share on)
    bool via_at_smd = false;         // (via_at_smd on)
    bool via_at_smd_grid = false;    // (via_at_smd (grid on))
    Coord smd_to_via_gap_nm = 0;     // (smd_to_via_gap X)
    Coord via_to_via_gap_nm = 0;     // (via_to_via_gap X)
    Coord buried_via_gap_nm = 0;     // (buried_via_gap X)
    // Item 13 (Altium PolygonConnect, carried on the class; enforcement is
    // the EM/thermal tier's job, recorded for rule round-trips).
    std::string poly_connect_style;  // "relief" | "solid" | "none" | ""
    Coord relief_airgap_nm = 0;      // AIRGAPWIDTH
    Coord relief_spoke_w_nm = 0;     // RELIEFCONDUCTORWIDTH
    int relief_entries = 0;          // RELIEFENTRIES (spoke count)
};

// Parse a (width ...) rule the way the Specctra router reads it (M9):
// the preferred value drives routing, the minimum is the floor, and the
// maximum is out of scope (recorded, never enforced). Three spellings are
// accepted because exporters disagree:
//   (width 0.2)                    - single value: min = pref = 0.2
//   (width 0.15 0.2 0.3)           - positional: min pref max
//   (width (min 0.15) (pref 0.2))   - labelled fragment
// `scale_mm` is the file's coordinate quantum (mm per count).
WidthTriplet parse_width_rule(const SexprNode* w, double scale_mm) {
    WidthTriplet out;
    if (!w) return out;
    auto u2mm_local = [&](double v) { return v * scale_mm; };
    // Labelled form: (width (min x) (pref y) (max z)).
    const SexprNode* mn = w->find_child("min");
    const SexprNode* pf = w->find_child("pref");
    if (mn || pf) {
        double minv = mn ? atom_num(mn, 1, 0.0) : 0.0;
        double prefv = pf ? atom_num(pf, 1, 0.0) : 0.0;
        if (pf && prefv > 0) {
            out.min_nm = mm_to_nm(u2mm_local(minv));
            out.pref_nm = mm_to_nm(u2mm_local(prefv));
            out.has_pref = true;
            out.source = "pref";
        } else {
            out.min_nm = mm_to_nm(u2mm_local(minv));
            out.source = "min";
        }
        return out;
    }
    // Positional form: collect leading numeric atoms.
    std::vector<double> nums;
    for (std::size_t i = 1; i < w->children.size(); ++i) {
        if (!is_number_atom(w->children[i].get())) break;
        nums.push_back(atom_num(w, i));
    }
    if (nums.empty()) return out;  // caller reports a default
    out.min_nm = mm_to_nm(u2mm_local(nums[0]));
    if (nums.size() >= 2) {
        out.pref_nm = mm_to_nm(u2mm_local(nums[1]));
        out.has_pref = true;
        out.source = "pref";
    } else {
        out.source = "single";
    }
    return out;
}

// Memory bound (router <= 2048MB total): no pour/keepout polygon may carry
// more vertices than this; larger shapes are skipped with an explicit
// warning instead of buffering unbounded copper.
constexpr std::size_t kMaxPourVertices = 16384;

}  // namespace

bool DsnImporter::claims(const std::string& path, const std::string& head_bytes) const {
    if (ends_with(path, ".dsn")) return true;
    return head_bytes.find("(pcb") != std::string::npos;
}

ImportResult DsnImporter::import_file(const std::string& path) const {
    return import_text(read_file_text(path), path);
}

ImportResult DsnImporter::import_text(const std::string& text, const std::string& path) const {
    std::unique_ptr<SexprNode> root;
    try {
        root = parse_sexpr(text);
    } catch (const std::exception& e) {
        throw BoardError(InputKind::kInvalid, std::string("bad DSN: ") + e.what());
    }
    // parse_sexpr returns a wrapper root holding all top-level forms.
    const SexprNode* pcb = root->find_child("pcb");
    if (!pcb)
        throw BoardError(InputKind::kInvalid, "DSN: missing (pcb ...) root");
    std::string design = atom_str(pcb, 1, "dsn_design");

    // ---- unit / resolution -------------------------------------------------
    // Specctra declares the coordinate quantum twice in the wild:
    //   (resolution <unit> [<digits>])   ; one file count = unit * 10^-digits
    //   (unit <unit> [<digits>])         ; older/alternate writer form
    // When both appear the resolution wins (docs/reverse/circuitmaker/
    // autorouter_modules.md M1). Every numeric literal below flows through
    // u2mm, so the two declarations are the only place a scale can enter.
    struct UnitDecl {
        bool present = false;
        std::string unit;
        bool has_digits = false;
        double digits = 0.0;
    };
    auto read_unit_decl = [&](const char* head) {
        UnitDecl d;
        const SexprNode* n = pcb->find_child(head);
        if (!n) return d;
        d.present = true;
        d.unit = atom_str(n, 1, "");
        double dv = 0.0;
        if (n->children.size() > 2 && parse_number(atom_str(n, 2, ""), dv)) {
            d.has_digits = true;
            d.digits = dv;
        }
        return d;
    };
    const UnitDecl res_decl = read_unit_decl("resolution");
    const UnitDecl unit_decl = read_unit_decl("unit");
    // digit decade -> count scale; clamped so a hostile literal cannot
    // underflow to zero (18 decades is already sub-femtometre).
    auto decl_scale = [](const UnitDecl& d, double unit_mm) {
        if (!d.has_digits) return unit_mm;
        double digits = std::clamp(d.digits, 0.0, 18.0);
        return unit_mm * std::pow(10.0, -digits);
    };

    double unit_scale_mm = 1.0;
    std::string unit_name = "mm";
    if (unit_decl.present) {
        unit_name = unit_decl.unit.empty() ? "mm" : unit_decl.unit;
        double s = unit_to_mm(unit_name);
        if (s < 0)
            throw BoardError(InputKind::kInvalid, "DSN: unknown (unit " + unit_name + ")");
        unit_scale_mm = decl_scale(unit_decl, s);
    }
    double res_scale_mm = unit_scale_mm;
    if (res_decl.present) {
        std::string rname = res_decl.unit.empty() ? unit_name : res_decl.unit;
        double s = unit_to_mm(rname);
        if (s < 0)
            throw BoardError(InputKind::kInvalid,
                             "DSN: unknown (resolution " + rname + " ...)");
        res_scale_mm = decl_scale(res_decl, s);
    }

    ImportResult result;
    Board& board = result.board;
    board.source_format = "dsn";
    board.source_file = path;
    board.defaults = BoardDefaults();

    // ---- structure: layers / boundary / vias / rules / copper ----
    const SexprNode* structure = pcb->find_child("structure");
    if (!structure) throw BoardError(InputKind::kInvalid, "DSN: missing (structure ...)");
    std::map<std::string, LayerId> layer_by_norm;  // norm_name -> layer id
    {
        LayerId next_id = 0;
        for (const SexprNode* ln : structure->find_all("layer")) {
            Layer layer;
            layer.id = next_id++;
            layer.name = atom_str(ln, 1, "L" + std::to_string(layer.id));
            std::string type = "signal";
            if (const SexprNode* t = ln->find_child("type")) type = atom_str(t, 1, "signal");
            layer.layer_type = (type == "plane" || type == "power") ? "plane" : "signal";
            // Specctra routing direction is a soft preference (never DRC):
            // record it verbatim so reports can show what the file asked for,
            // and fold horizontal/vertical into the Layer cost hint.
            if (const SexprNode* d = ln->find_child("direction")) {
                layer.direction = atom_str(d, 1, "");
                if (layer.direction == "horizontal") layer.preferred_horizontal = true;
                // Specctra's direction vocabulary. Anything else is still
                // recorded verbatim (never fatal) but named, so a dialect
                // surprise cannot look like a hint the router honoured.
                static const char* kDirections[] = {"horizontal", "vertical", "45", "in", "out", "any"};
                bool known = false;
                for (const char* k : kDirections)
                    if (layer.direction == k) known = true;
                if (!known)
                    result.warnings.push_back("dsn: unknown (direction " + layer.direction +
                                              ") on layer '" + layer.name +
                                              "'; recorded verbatim");
            }
            if (const SexprNode* g = ln->find_child("grid"))
                layer.grid_on = atom_str(g, 1, "off") == "on";
            board.layers.push_back(layer);
            layer_by_norm[norm_name(layer.name)] = layer.id;
        }
        if (board.layers.empty()) {
            // Permissive default: a 2-layer board so minimal DSNs still route.
            Layer a, b;
            a.id = 0;
            a.name = "Top";
            b.id = 1;
            b.name = "Bottom";
            board.layers = {a, b};
            layer_by_norm = {{"TOP", 0}, {"BOTTOM", 1}};
            result.warnings.push_back("dsn: no (structure (layer ...)) entries; assumed Top/Bottom");
        }
    }
    // Direction/grid are soft hints: recorded and round-tripped, but the A*
    // step cost is still axis-neutral, so name the gap instead of implying the
    // file's preference was applied.
    {
        std::size_t directed = 0;
        for (const auto& l : board.layers)
            if (!l.direction.empty()) ++directed;
        if (directed > 0)
            result.warnings.push_back(
                "dsn: " + std::to_string(directed) +
                " layer(s) declare a routing direction; recorded as a soft preference "
                "(the router does not bias step cost by axis yet)");
    }
    auto find_layer = [&](const std::string& name) -> LayerId {
        auto it = layer_by_norm.find(norm_name(name));
        return it == layer_by_norm.end() ? -1 : it->second;
    };

    // ---- boundary: (rect ...) or (polygon ...) in FILE COUNTS ------------
    // Parsed before the scale is finalized because a resolution decade that
    // cannot describe these coordinates must be rejected up front (below).
    struct Boundary {
        bool ok = false;
        bool is_rect = false;
        bool non_rect_outline = false;
        double x1 = 0, y1 = 0, x2 = 0, y2 = 0;  // bbox in file counts
    } bound;
    if (const SexprNode* bnode = structure->find_child("boundary")) {
        if (const SexprNode* rc = bnode->find_child("rect")) {
            std::size_t i = first_numeric_atom(rc);
            double x1 = atom_num(rc, i), y1 = atom_num(rc, i + 1);
            double x2 = atom_num(rc, i + 2), y2 = atom_num(rc, i + 3);
            bound.is_rect = true;
            bound.ok = (x1 != x2 && y1 != y2);
            bound.x1 = std::min(x1, x2);
            bound.y1 = std::min(y1, y2);
            bound.x2 = std::max(x1, x2);
            bound.y2 = std::max(y1, y2);
        } else if (const SexprNode* pg = bnode->find_child("polygon")) {
            std::size_t i = first_numeric_atom(pg);
            bool first = true;
            std::size_t pts = 0;
            for (; i + 1 < pg->children.size(); i += 2) {
                if (!is_number_atom(pg->children[i].get())) break;
                double x = atom_num(pg, i), y = atom_num(pg, i + 1);
                if (first) {
                    bound.x1 = bound.x2 = x;
                    bound.y1 = bound.y2 = y;
                    first = false;
                } else {
                    bound.x1 = std::min(bound.x1, x);
                    bound.y1 = std::min(bound.y1, y);
                    bound.x2 = std::max(bound.x2, x);
                    bound.y2 = std::max(bound.y2, y);
                }
                ++pts;
            }
            bound.ok = pts >= 3 && bound.x2 > bound.x1 && bound.y2 > bound.y1;
            // A 4-point polygon whose bbox area matches its own area is a
            // rectangle; anything else is a real outline the rect board model
            // can only approximate.
            double area = 0.0;
            std::vector<std::pair<double, double>> v;
            for (std::size_t k = first_numeric_atom(pg); k + 1 < pg->children.size(); k += 2) {
                if (!is_number_atom(pg->children[k].get())) break;
                v.push_back({atom_num(pg, k), atom_num(pg, k + 1)});
            }
            for (std::size_t k = 0; k < v.size(); ++k) {
                const auto& a = v[k];
                const auto& b = v[(k + 1) % v.size()];
                area += a.first * b.second - b.first * a.second;
            }
            double bbox = (bound.x2 - bound.x1) * (bound.y2 - bound.y1);
            bound.non_rect_outline = v.size() > 4 || std::fabs(std::fabs(area) / 2.0 - bbox) > bbox * 1e-6;
        }
    }

    // ---- scale selection ---------------------------------------------------
    double scale_mm = res_decl.present ? res_scale_mm : unit_scale_mm;
    if (res_decl.present) {
        // A plausible PCB is 0.05 mm .. 5 m. When the resolution decade cannot
        // describe the file's own coordinates (real files that declare um-10
        // while writing coarser integers) the unit scale is the honest
        // fallback; the dialect notes document exactly this rule.
        auto plausible = [](double mm) { return mm >= 0.05 && mm <= 5000.0; };
        if (bound.ok) {
            double span = std::max(bound.x2 - bound.x1, bound.y2 - bound.y1);
            if (!plausible(span * scale_mm)) {
                if (plausible(span * unit_scale_mm)) {
                    char buf[220];
                    std::snprintf(buf, sizeof(buf),
                                  "dsn: (resolution %s) does not fit these coordinates "
                                  "(boundary would span %.6g mm); used the unit scale "
                                  "(%.6g mm per count) instead",
                                  res_decl.unit.c_str(), span * scale_mm, unit_scale_mm);
                    result.warnings.push_back(buf);
                    scale_mm = unit_scale_mm;
                } else {
                    char buf[220];
                    std::snprintf(buf, sizeof(buf),
                                  "dsn: (resolution %s) puts the boundary at %.6g mm, which "
                                  "is not a plausible board; coordinates imported as declared",
                                  res_decl.unit.c_str(), span * scale_mm);
                    result.warnings.push_back(buf);
                }
            }
        }
        char buf[160];
        std::snprintf(buf, sizeof(buf), "dsn: resolution scale = %.6g mm per file count", scale_mm);
        result.warnings.push_back(buf);
    }
    auto u2mm = [&](double v) { return v * scale_mm; };

    if (!bound.ok)
        throw BoardError(InputKind::kInvalid, "DSN: missing (structure (boundary ...)) extents");
    board.width_nm = mm_to_nm(u2mm(bound.x2) - u2mm(bound.x1));
    board.height_nm = mm_to_nm(u2mm(bound.y2) - u2mm(bound.y1));
    if (board.width_nm <= 0 || board.height_nm <= 0)
        throw BoardError(InputKind::kInvalid, "DSN: missing (structure (boundary ...)) extents");
    if (bound.non_rect_outline)
        result.warnings.push_back(
            "dsn: non-rectangular board outline approximated by its bounding box "
            "(the board model is rect-bounded)");
    // Board origin: DSN coordinates may start anywhere; translate to (0,0).
    // Keep the offset to shift pads/pours into board space. The boundary
    // minimum corner is the natural origin.
    const double origin_x_mm = u2mm(bound.x1);
    const double origin_y_mm = u2mm(bound.y1);
    std::map<std::string, std::pair<Coord, Coord>> via_geom;  // name -> (outer, hole)
    for (const SexprNode* vn : structure->find_all("via")) {
        std::string name = atom_str(vn, 1, "");
        if (name.empty()) continue;
        via_geom[name] = {mm_to_nm(u2mm(atom_num(vn, 2, 0.6))),
                          mm_to_nm(u2mm(atom_num(vn, 3, 0.3)))};
    }
    if (const SexprNode* rule = structure->find_child("rule")) {
        if (const SexprNode* w = rule->find_child("width"))
            board.defaults.trace_width_nm = mm_to_nm(u2mm(atom_num(w, 1, 0.2)));
        if (const SexprNode* c = rule->find_child("clearance"))
            board.defaults.clearance_nm = mm_to_nm(u2mm(atom_num(c, 1, 0.15)));
    }

    // ---- network: classes + nets ----
    const SexprNode* network = pcb->find_child("network");
    if (!network) throw BoardError(InputKind::kInvalid, "DSN: missing (network ...)");
    std::map<std::string, DsnClassRule> classes;
    for (const SexprNode* cn : network->find_all("class")) {
        DsnClassRule cr;
        cr.name = atom_str(cn, 1, "");
        if (cr.name.empty()) continue;
        if (const SexprNode* rule = cn->find_child("rule")) {
            if (const SexprNode* w = rule->find_child("width")) {
                WidthTriplet wt = parse_width_rule(w, scale_mm);
                if (wt.min_nm > 0) {
                    cr.has_width = true;
                    cr.width_nm = wt.min_nm;
                    cr.width_source = wt.source;
                } else {
                    result.warnings.push_back(
                        "dsn: class '" + cr.name +
                        "' (width ...) rule carries no usable value; class width ignored");
                }
                if (wt.has_pref) {
                    cr.has_pref_width = true;
                    cr.pref_width_nm = wt.pref_nm;
                }
            }
            if (const SexprNode* c = rule->find_child("clearance")) {
                cr.has_clearance = true;
                cr.clearance_nm = mm_to_nm(u2mm(atom_num(c, 1, 0.15)));
            }
            // Item 12: Electra-style via policy knobs. These appear as
            // (via_share on) / (via_at_smd on (grid on)) / (smd_to_via_gap x)
            // style flags inside the class rule. Parsed + carried as soft
            // via-placement inputs (never hard DRC here).
            if (const SexprNode* vs = rule->find_child("via_share"))
                cr.via_share = !atom_str(vs, 1, "on").empty() &&
                               atom_str(vs, 1, "on") != "off";
            if (const SexprNode* vas = rule->find_child("via_at_smd")) {
                cr.via_at_smd = !atom_str(vas, 1, "off").empty();
                if (const SexprNode* grid = vas->find_child("grid"))
                    cr.via_at_smd_grid = !atom_str(grid, 1, "off").empty();
            }
            if (const SexprNode* sg = rule->find_child("smd_to_via_gap"))
                cr.smd_to_via_gap_nm = mm_to_nm(u2mm(atom_num(sg, 1, 0.0)));
            if (const SexprNode* vg = rule->find_child("via_to_via_gap"))
                cr.via_to_via_gap_nm = mm_to_nm(u2mm(atom_num(vg, 1, 0.0)));
            if (const SexprNode* bvg = rule->find_child("buried_via_gap"))
                cr.buried_via_gap_nm = mm_to_nm(u2mm(atom_num(bvg, 1, 0.0)));
            // Item 13: Altium PolygonConnect-style relief settings. Altium
            // exports these as rule fragments; the DSN path may name them
            // (poly_connect (style relief) (airgap x) (spoke_w x) (entries n))
            // or (connect_style relief ...). Parsed + carried for the EM
            // thermal tier; the router records them, never invents relief.
            if (const SexprNode* pc = rule->find_child("poly_connect")) {
                if (const SexprNode* st = pc->find_child("style"))
                    cr.poly_connect_style = atom_str(st, 1, "");
                else
                    cr.poly_connect_style = atom_str(pc, 1, "");
                if (const SexprNode* ag = pc->find_child("airgap"))
                    cr.relief_airgap_nm = mm_to_nm(u2mm(atom_num(ag, 1, 0.0)));
                if (const SexprNode* sw = pc->find_child("spoke_w"))
                    cr.relief_spoke_w_nm = mm_to_nm(u2mm(atom_num(sw, 1, 0.0)));
                if (const SexprNode* en = pc->find_child("entries"))
                    cr.relief_entries = static_cast<int>(atom_num(en, 1, 0.0));
            }
            if (const SexprNode* cs = rule->find_child("connect_style")) {
                if (const SexprNode* st = cs->find_child("style"))
                    cr.poly_connect_style = atom_str(st, 1, cr.poly_connect_style);
                else
                    cr.poly_connect_style = atom_str(cs, 1, cr.poly_connect_style);
                if (const SexprNode* ag = cs->find_child("airgap"))
                    cr.relief_airgap_nm = mm_to_nm(u2mm(atom_num(ag, 1, 0.0)));
                if (const SexprNode* sw = cs->find_child("spoke_w"))
                    cr.relief_spoke_w_nm = mm_to_nm(u2mm(atom_num(sw, 1, 0.0)));
                if (const SexprNode* en = cs->find_child("entries"))
                    cr.relief_entries = static_cast<int>(atom_num(en, 1, 0.0));
            }
            // Item 9: layer-scoped variants (scope priority: layer wins).
            // Specctra/Altium exporters nest (layer "X" (width ...)
            // (clearance ...)) inside the class rule for per-layer rules.
            for (const SexprNode* lay : rule->find_all("layer")) {
                std::string lay_name = atom_str(lay, 1, "");
                if (lay_name.empty()) continue;
                std::string nrm = norm_name(lay_name);
                if (const SexprNode* lw = lay->find_child("width")) {
                    cr.layer_width[nrm] = parse_width_rule(lw, scale_mm);
                }
                if (const SexprNode* lc = lay->find_child("clearance")) {
                    cr.layer_clearance[nrm] =
                        mm_to_nm(u2mm(atom_num(lc, 1, 0.15)));
                }
            }
        }
        if (const SexprNode* cir = cn->find_child("circuit")) {
            for (const SexprNode* uv : cir->find_all("use_via"))
                cr.use_via.push_back(atom_str(uv, 1, ""));
        }
        classes[cr.name] = cr;
    }
    struct PendingNet {
        std::string name;
        std::vector<std::string> pins;
        std::string cls;
    };
    std::vector<PendingNet> pending;
    for (const SexprNode* nn : network->find_all("net")) {
        PendingNet p;
        p.name = atom_str(nn, 1, "");
        if (p.name.empty())
            throw BoardError(InputKind::kInvalid, "DSN: (net ...) without a name");
        if (const SexprNode* pins = nn->find_child("pins")) {
            for (std::size_t i = 1; i < pins->children.size(); ++i)
                if (pins->children[i]->is_atom) p.pins.push_back(pins->children[i]->atom);
        }
        if (const SexprNode* cl = nn->find_child("class")) p.cls = atom_str(cl, 1, "");
        pending.push_back(std::move(p));
    }
    if (pending.empty()) throw BoardError(InputKind::kInvalid, "DSN: no (net ...) entries");

    // ---- library + placement: pad positions and pad geometry --------------
    // Altium/KiCad/FreeRouting describe non-circular pads with a padstack that
    // carries `(shapes (rect|circle|octagon|polygon ...))`. Without it every
    // pad collapses to a fixed square, which under-reports copper in the
    // verifier (clearance and pad-pad checks see phantom gaps).
    struct PadShapeGeom {
        Coord w_nm = 0;
        Coord h_nm = 0;
        Coord dx_nm = 0;  // shape centre offset from the pin origin
        Coord dy_nm = 0;
        std::string form;
        std::string layer;  // norm_name; empty = any layer
    };
    struct PadGeom {
        double x = 0, y = 0;       // pin origin in design mm
        bool has_shape = false;
        Coord w_nm = 0, h_nm = 0;  // chosen shape extent
        Coord dx_nm = 0, dy_nm = 0;
    };
    // part -> padstack id -> shapes, and part -> pad -> padstack id.
    std::map<std::string, std::map<std::string, std::vector<PadShapeGeom>>> padstacks;
    std::map<std::string, std::map<std::string, PadGeom>> images;
    int pads_without_shape = 0;
    bool warned_octagon_pad = false, warned_polygon_pad = false;
    std::map<std::string, std::vector<std::string>> unknown_padstacks;
    if (const SexprNode* lib = pcb->find_child("library")) {
        for (const SexprNode* img : lib->find_all("image")) {
            std::string part = atom_str(img, 1, "");
            // Padstacks inside an image. Shape lists accept the canonical
            // layer token (`(rect Top 0 0 1 1)`) or omit it, and a padstack
            // may hold one shape per layer.
            auto add_shape = [&](const SexprNode* s, std::vector<PadShapeGeom>& out) {
                std::string head = s->head();
                std::size_t i = first_numeric_atom(s);
                PadShapeGeom g;
                g.form = head;
                if (i > 1) g.layer = norm_name(atom_str(s, 1, ""));
                if (head == "rect" || head == "rect_keepout" || head == "roundrect") {
                    double x1 = u2mm(atom_num(s, i)), y1 = u2mm(atom_num(s, i + 1));
                    double x2 = u2mm(atom_num(s, i + 2)), y2 = u2mm(atom_num(s, i + 3));
                    g.w_nm = mm_to_nm(std::fabs(x2 - x1));
                    g.h_nm = mm_to_nm(std::fabs(y2 - y1));
                    g.dx_nm = mm_to_nm((x1 + x2) / 2.0);
                    g.dy_nm = mm_to_nm((y1 + y2) / 2.0);
                } else if (head == "circle" || head == "octagon" || head == "round") {
                    // (circle <layer> <diameter> [<x> <y>])
                    double d = u2mm(atom_num(s, i));
                    g.w_nm = g.h_nm = mm_to_nm(std::fabs(d));
                    if (i + 2 < s->children.size() && is_number_atom(s->children[i + 1].get())) {
                        g.dx_nm = mm_to_nm(u2mm(atom_num(s, i + 1)));
                        g.dy_nm = mm_to_nm(u2mm(atom_num(s, i + 2)));
                    }
                    if (head == "octagon") {
                        if (!warned_octagon_pad) {
                            result.warnings.push_back(
                                "dsn: padstack (octagon ...) pads approximated by their "
                                "bounding box (8-gon copper not modeled)");
                            warned_octagon_pad = true;
                        }
                    }
                } else if (head == "polygon") {
                    // (polygon <layer> <width> <x1> <y1> ...): the width atom
                    // is the outline width, not a vertex.
                    double x1 = 1e18, y1 = 1e18, x2 = -1e18, y2 = -1e18;
                    std::size_t pts = 0;
                    for (std::size_t k = i + 1; k + 1 < s->children.size(); k += 2) {
                        if (!is_number_atom(s->children[k].get()) ||
                            !is_number_atom(s->children[k + 1].get()))
                            break;
                        double x = u2mm(atom_num(s, k)), y = u2mm(atom_num(s, k + 1));
                        x1 = std::min(x1, x); y1 = std::min(y1, y);
                        x2 = std::max(x2, x); y2 = std::max(y2, y);
                        ++pts;
                    }
                    if (pts < 3) return;
                    g.w_nm = mm_to_nm(x2 - x1);
                    g.h_nm = mm_to_nm(y2 - y1);
                    g.dx_nm = mm_to_nm((x1 + x2) / 2.0);
                    g.dy_nm = mm_to_nm((y1 + y2) / 2.0);
                    if (!warned_polygon_pad) {
                        result.warnings.push_back(
                            "dsn: padstack (polygon ...) pads approximated by their "
                            "bounding box (exact polygon copper not modeled)");
                        warned_polygon_pad = true;
                    }
                } else {
                    return;
                }
                out.push_back(g);
            };
            for (const SexprNode* ps : img->find_all("padstack")) {
                std::string id = atom_str(ps, 1, "");
                if (id.empty()) continue;
                std::vector<PadShapeGeom>& shapes = padstacks[part][norm_name(id)];
                for (const SexprNode* sh : ps->find_all("shape"))
                    for (const auto& s : sh->children)
                        if (!s->is_atom) add_shape(s.get(), shapes);
                // Some writers put the shape lists directly on the padstack.
                for (const auto& s : ps->children)
                    if (!s->is_atom && s->head() != "shape" && s->head() != "padstack")
                        add_shape(s.get(), shapes);
            }
            for (const SexprNode* pin : img->find_all("pin")) {
                std::string pad = atom_str(pin, 1, "");
                // (pin <pad> [<padstack>] <x> <y> [<rot>]): a padstack id sits
                // between the pad name and the coordinates when present.
                bool has_ps = pin->children.size() >= 5 && !is_number_atom(pin->children[2].get());
                std::string ps = has_ps ? atom_str(pin, 2, "") : "";
                std::size_t xi = has_ps ? 3 : 2;
                PadGeom g;
                g.x = u2mm(atom_num(pin, xi));
                g.y = u2mm(atom_num(pin, xi + 1));
                // Choose the shape for the layer the pad lives on (first match,
                // deterministic document order), else the first declared.
                if (!ps.empty()) {
                    const std::vector<PadShapeGeom>* shapes = nullptr;
                    auto pit = padstacks.find(part);
                    if (pit != padstacks.end()) {
                        auto sit = pit->second.find(norm_name(ps));
                        if (sit != pit->second.end()) shapes = &sit->second;
                    }
                    if (shapes && !shapes->empty()) {
                        const PadShapeGeom* chosen = &shapes->front();
                        const std::string top_norm = norm_name(board.layers.front().name);
                        for (const auto& cand : *shapes) {
                            if (!cand.layer.empty() && cand.layer == top_norm) {
                                chosen = &cand;
                                break;
                            }
                        }
                        g.has_shape = true;
                        g.w_nm = chosen->w_nm;
                        g.h_nm = chosen->h_nm;
                        g.dx_nm = chosen->dx_nm;
                        g.dy_nm = chosen->dy_nm;
                    } else {
                        unknown_padstacks[part].push_back(ps);
                    }
                }
                images[part][pad] = g;
            }
        }
    }
    struct Placed {
        std::string part;
        double x = 0, y = 0;
        double rot_deg = 0.0;
    };
    std::map<std::string, Placed> placed;  // ref -> placement
    bool warned_rotation = false;
    if (const SexprNode* pl = pcb->find_child("placement")) {
        for (const SexprNode* comp : pl->find_all("component")) {
            std::string ref = atom_str(comp, 1, "");
            if (const SexprNode* plc = comp->find_child("place")) {
                Placed p;
                p.part = atom_str(plc, 1, "");
                p.x = u2mm(atom_num(plc, 2));
                p.y = u2mm(atom_num(plc, 3));
                // (place <part> <x> <y> <front|back> <rotation>)
                p.rot_deg = atom_num(plc, 5, 0.0);
                if (!warned_rotation && std::fabs(p.rot_deg) > 1e-9) {
                    result.warnings.push_back(
                        "dsn: component rotation is imported as placement metadata only; "
                        "pad geometry is used axis-aligned (rotated pad offsets not modeled)");
                    warned_rotation = true;
                }
                placed[ref] = p;
            }
        }
    }

    // ---- materialize nets + terminals ----
    NetId next_net = 0;
    TermId next_term = 0;
    LayerId top_layer = board.layers.front().id;
    for (const auto& p : pending) {
        NetInfo net;
        net.id = next_net++;
        net.name = p.name;
        auto cit = classes.find(p.cls);
        if (cit != classes.end()) {
            const DsnClassRule& cr = cit->second;
            if (cr.has_width) {
                net.has_min_width = true;
                net.min_width_nm = cr.width_nm;
            }
            if (cr.has_pref_width) {
                net.has_pref_width = true;
                net.pref_width_nm = cr.pref_width_nm;
                // M9: the router routes at the preferred width (fallback:
                // min). Say which value drives routing so agents can see
                // the choice, and floor the preferred by the min (a file
                // with pref < min is a defect; prefer is still >= min).
                if (cr.width_source == "pref" && cr.pref_width_nm >= cr.width_nm) {
                    result.warnings.push_back(
                        "dsn: class '" + cr.name + "' width rule uses the preferred value "
                        + std::to_string(nm_to_mm(cr.pref_width_nm)) + "mm (min " +
                        std::to_string(nm_to_mm(cr.width_nm)) + "mm)");
                } else if (cr.width_source != "single") {
                    result.warnings.push_back(
                        "dsn: class '" + cr.name + "' width rule preferred below min; "
                        "routing at min " + std::to_string(nm_to_mm(cr.width_nm)) + "mm");
                }
            }
            if (cr.has_clearance) {
                net.has_min_clearance = true;
                net.min_clearance_nm = cr.clearance_nm;
            }
            if (!cr.use_via.empty()) net.via_class = cr.use_via.front();
            // Items 9/12/13: parsed + carried, surfaced honestly. The core
            // width/clearance model stays class-wide (the router resolves
            // per-layer via RuleResolver); layer-scoped widths, via policy
            // knobs and relief settings are recorded on the class and
            // reported so agents see them without the router silently
            // inventing enforcement it does not implement.
            if (!cr.layer_width.empty() || !cr.layer_clearance.empty()) {
                result.warnings.push_back(
                    "dsn: class '" + cr.name + "' carries " +
                    std::to_string(cr.layer_width.size() + cr.layer_clearance.size()) +
                    " layer-scoped rule(s); recorded (layer-specific widths are not "
                    "enforced by the class-wide model — see docs/router_comparison.md M9)");
            }
            if (cr.via_share || cr.via_at_smd || cr.via_at_smd_grid ||
                cr.smd_to_via_gap_nm > 0 || cr.via_to_via_gap_nm > 0 ||
                cr.buried_via_gap_nm > 0) {
                result.warnings.push_back(
                    "dsn: class '" + cr.name + "' carries via-policy knobs "
                    "(via_share/via_at_smd/smd_to_via_gap/...); recorded as soft "
                    "via-placement inputs, not hard DRC");
            }
            if (!cr.poly_connect_style.empty()) {
                result.warnings.push_back(
                    "dsn: class '" + cr.name + "' PolygonConnect style '" +
                    cr.poly_connect_style + "' recorded (relief/spoke enforcement "
                    "is the EM/thermal tier's job — em/validate_thermal.py --relief)");
            }
        }
        for (const auto& rp : p.pins) {
            // Pin token is "REF-PAD" (pad names with '-' are unsupported and
            // reported explicitly rather than mis-split).
            std::size_t dash = rp.find('-');
            if (dash == std::string::npos)
                throw BoardError(InputKind::kInvalid, "DSN: bad pin token '" + rp + "'");
            std::string ref = rp.substr(0, dash), pad = rp.substr(dash + 1);
            auto pit = placed.find(ref);
            if (pit == placed.end())
                throw BoardError(InputKind::kInvalid,
                                 "DSN: pin '" + rp + "' names an unplaced component");
            auto iit = images.find(pit->second.part);
            if (iit == images.end())
                throw BoardError(InputKind::kInvalid, "DSN: no (image " + pit->second.part + ")");
            auto padit = iit->second.find(pad);
            if (padit == iit->second.end())
                throw BoardError(InputKind::kInvalid,
                                 "DSN: image '" + pit->second.part + "' has no pin '" + pad + "'");
            double x_mm = pit->second.x + padit->second.x - origin_x_mm;
            double y_mm = pit->second.y + padit->second.y - origin_y_mm;
            Terminal t;
            t.id = next_term++;
            t.net = net.id;
            // Pad copper comes from the padstack shape when the file declares
            // one; the offset moves an asymmetric shape's centre.
            if (padit->second.has_shape) {
                x_mm += nm_to_mm(padit->second.dx_nm);
                y_mm += nm_to_mm(padit->second.dy_nm);
                t.pad_w_nm = padit->second.w_nm;
                t.pad_h_nm = padit->second.h_nm;
            } else {
                ++pads_without_shape;
                t.pad_w_nm = mm_to_nm(0.3);
                t.pad_h_nm = mm_to_nm(0.3);
            }
            t.pos = {mm_to_nm(x_mm), mm_to_nm(y_mm)};
            t.layer = top_layer;
            t.component = ref;
            t.pin = pad;
            board.terminals.push_back(t);
            net.terminals.push_back(t.id);
        }
        if (net.terminals.size() < 2)
            throw BoardError(InputKind::kInvalid,
                             "DSN: net '" + net.name + "' needs >= 2 pins");
        board.nets.push_back(std::move(net));
    }

    // ---- copper: planes + pours map to PlaneZone; keepouts to Keepout ----
    // Every pour names its owning net and layer; unresolvable copper keeps
    // an explicit warning (never silent). Each pour gets its own island id
    // (no declared stitching across pours): overlapping pours still bridge
    // geometrically in the verifier, disjoint same-net pours need copper.
    int next_plane = 0;
    // Shape -> polygon. Specctra list shapes may lead with a layer token
    // (`(polygon Bottom 1 8 19 8 ...)`, `(rect pcb 0 0 20 20)`); the numeric
    // start is detected, and a shape-carried layer name is reported back so a
    // pour resolves its layer from either `(layer ...)` or the shape itself.
    auto shape_to_poly = [&](const SexprNode* shape, const std::string& what,
                             std::vector<Point>& out, std::string* shape_layer) -> bool {
        if (shape_layer) shape_layer->clear();
        if (const SexprNode* pg = shape->find_child("polygon")) {
            std::size_t i0 = first_numeric_atom(pg);
            if (i0 > 1 && shape_layer) *shape_layer = atom_str(pg, 1, "");
            std::size_t nverts = i0 < pg->children.size() ? (pg->children.size() - i0) / 2 : 0;
            if (nverts < 3 || nverts > kMaxPourVertices) return false;
            for (std::size_t i = i0; i + 1 < pg->children.size(); i += 2) {
                double x = u2mm(atom_num(pg, i)) - origin_x_mm;
                double y = u2mm(atom_num(pg, i + 1)) - origin_y_mm;
                out.push_back({mm_to_nm(x), mm_to_nm(y)});
            }
            return out.size() >= 3;
        }
        if (const SexprNode* rc = shape->find_child("rect")) {
            std::size_t i = first_numeric_atom(rc);
            if (i > 1 && shape_layer) *shape_layer = atom_str(rc, 1, "");
            double x1 = u2mm(atom_num(rc, i)) - origin_x_mm;
            double y1 = u2mm(atom_num(rc, i + 1)) - origin_y_mm;
            double x2 = u2mm(atom_num(rc, i + 2)) - origin_x_mm;
            double y2 = u2mm(atom_num(rc, i + 3)) - origin_y_mm;
            out = {{mm_to_nm(std::min(x1, x2)), mm_to_nm(std::min(y1, y2))},
                   {mm_to_nm(std::max(x1, x2)), mm_to_nm(std::min(y1, y2))},
                   {mm_to_nm(std::max(x1, x2)), mm_to_nm(std::max(y1, y2))},
                   {mm_to_nm(std::min(x1, x2)), mm_to_nm(std::max(y1, y2))}};
            return true;
        }
        if (const SexprNode* ci = shape->find_child("circle")) {
            // Octagon approximation of a round pour/cutout (documented).
            std::size_t i = first_numeric_atom(ci);
            if (i > 1 && shape_layer) *shape_layer = atom_str(ci, 1, "");
            double cx = u2mm(atom_num(ci, i)) - origin_x_mm;
            double cy = u2mm(atom_num(ci, i + 1)) - origin_y_mm;
            double r = u2mm(atom_num(ci, i + 2)) / 2.0;
            if (!(r > 0)) return false;
            for (int k = 0; k < 8; ++k) {
                double a = k * 3.141592653589793 / 4.0;
                out.push_back({mm_to_nm(cx + r * std::cos(a)), mm_to_nm(cy + r * std::sin(a))});
            }
            result.warnings.push_back("dsn: (" + what +
                                      " (circle ...)) approximated by an octagon");
            return true;
        }
        if (const SexprNode* pa = shape->find_child("path")) {
            // A path is a polyline, not a region: close it into a polygon.
            // `(path <layer> x1 y1 ...)`; the width token (if any) is the
            // first numeric that is followed by a non-numeric or the list end.
            std::size_t i0 = first_numeric_atom(pa);
            if (i0 > 1 && shape_layer) *shape_layer = atom_str(pa, 1, "");
            std::vector<Point> pts;
            for (std::size_t i = i0; i + 1 < pa->children.size(); i += 2) {
                if (!is_number_atom(pa->children[i].get()) ||
                    !is_number_atom(pa->children[i + 1].get()))
                    break;
                double x = u2mm(atom_num(pa, i)) - origin_x_mm;
                double y = u2mm(atom_num(pa, i + 1)) - origin_y_mm;
                pts.push_back({mm_to_nm(x), mm_to_nm(y)});
                if (pts.size() > static_cast<std::size_t>(kMaxPourVertices)) return false;
            }
            if (pts.size() < 3) return false;
            out = std::move(pts);
            result.warnings.push_back("dsn: (" + what +
                                      " (path ...)) closed into a polygon (approximation)");
            return true;
        }
        return false;
    };
    auto resolve_pour_net = [&](const SexprNode* node) -> const NetInfo* {
        if (const SexprNode* nn = node->find_child("net")) {
            std::string name = atom_str(nn, 1, "");
            for (const auto& n : board.nets)
                if (n.name == name) return &n;
            return nullptr;
        }
        std::string name = atom_str(node, 1, "");
        if (name.empty()) return nullptr;
        for (const auto& n : board.nets)
            if (n.name == name) return &n;
        return nullptr;
    };
    auto resolve_pour_layer = [&](const SexprNode* node, bool& present,
                                  const std::string& shape_layer) -> LayerId {
        present = false;
        const SexprNode* ln = node->find_child("layer");
        if (ln) {
            present = true;
            return find_layer(atom_str(ln, 1, ""));
        }
        // Specctra also names the layer on the shape itself
        // (`(polygon Bottom 1 8 ...)`); either spelling resolves the pour.
        if (!shape_layer.empty()) {
            present = true;
            return find_layer(shape_layer);
        }
        return top_layer;
    };
    auto parse_one_pour = [&](const SexprNode* node, const std::string& what) {
        const NetInfo* ni = resolve_pour_net(node);
        std::vector<Point> poly;
        std::string shape_layer;
        bool shape_ok = shape_to_poly(node, what, poly, &shape_layer);
        bool layer_present = false;
        LayerId layer = resolve_pour_layer(node, layer_present, shape_layer);
        if (!ni || (layer_present && layer < 0) || !shape_ok || poly.size() < 3) {
            result.warnings.push_back(
                "dsn: (" + what + " " + atom_str(node, 1, "?") +
                ") ignored with warning: unknown net/layer, degenerate polygon, or "
                "over vertex cap (" +
                std::to_string(kMaxPourVertices) +
                "); declare pours via native JSON planes[] for full control");
            return;
        }
        if (node->find_child("window"))
            result.warnings.push_back("dsn: (" + what +
                                      ") cutout windows are not modeled and were ignored");
        PlaneZone zone;
        zone.id = next_plane++;
        zone.net = ni->id;
        zone.layer = layer;
        zone.island = zone.id;  // own island: no silent stitching across pours
        zone.routable = true;
        zone.poly = std::move(poly);
        board.planes.push_back(std::move(zone));
    };
    for (const SexprNode* pl : structure->find_all("plane")) {
        parse_one_pour(pl, "plane");
    }
    for (const SexprNode* cp : structure->find_all("copper_pour")) {
        parse_one_pour(cp, "copper_pour");
    }
    // Specctra keepout shapes become rect Keepouts (polygons by bbox, with
    // an explicit warning; circles by bbox the same way). Unknown layers
    // are skipped with a warning rather than applied board-wide.
    // The DSN head names WHAT is forbidden inside the region
    // ((wire_keepout) = no traces, (via_keepout) = no vias, ...); that
    // classification becomes the Keepout kind, so the verifier/router
    // enforce each region per the file's intent (a via keepout never
    // silently becomes a wall that also blocks wires).
    {
        bool warned_poly = false;
        const char* kinds[] = {"wire_keepout", "via_keepout", "place_keepout",
                               "bend_keepout", "elongate_keepout", "keepout"};
        for (const char* kind : kinds) {
            // Head name -> enforcement kind: strip the "_keepout" suffix so
            // keepout_kind_from_name maps "wire" -> kWire, "via" -> kVia,
            // "place" -> kPlace, "bend" -> kBend, "elongate" -> kElongate;
            // the bare "keepout" head stays kAll (blocks everything).
            std::string kind_name(kind);
            static const std::string kKeepoutSuffix = "_keepout";
            if (kind_name.size() > kKeepoutSuffix.size() &&
                kind_name.compare(kind_name.size() - kKeepoutSuffix.size(),
                                  kKeepoutSuffix.size(), kKeepoutSuffix) == 0)
                kind_name.resize(kind_name.size() - kKeepoutSuffix.size());
            KeepoutKind ko_kind = keepout_kind_from_name(kind_name);
            for (const SexprNode* ko : structure->find_all(kind)) {
                std::vector<Point> poly;
                std::string shape_layer;
                if (!shape_to_poly(ko, kind, poly, &shape_layer) || poly.empty()) {
                    result.warnings.push_back(std::string("dsn: (") + kind +
                                              " ...) ignored with warning: no usable shape");
                    continue;
                }
                const SexprNode* ln = ko->find_child("layer");
                LayerId layer = kAllLayers;
                std::string lname = ln ? atom_str(ln, 1, "") : shape_layer;
                if (!lname.empty()) {
                    LayerId resolved = find_layer(lname);
                    if (resolved < 0) {
                        result.warnings.push_back(std::string("dsn: (") + kind +
                                                  " ...) ignored with warning: unknown layer");
                        continue;
                    }
                    layer = resolved;
                }
                Coord x1 = poly[0].x, y1 = poly[0].y, x2 = poly[0].x, y2 = poly[0].y;
                for (const auto& p : poly) {
                    x1 = std::min(x1, p.x);
                    y1 = std::min(y1, p.y);
                    x2 = std::max(x2, p.x);
                    y2 = std::max(y2, p.y);
                }
                if (x2 == x1 || y2 == y1) {
                    result.warnings.push_back(std::string("dsn: (") + kind +
                                              " ...) ignored with warning: degenerate shape");
                    continue;
                }
                if (!ln && !warned_poly) {
                    result.warnings.push_back(
                        "dsn: keepout polygons approximated by bounding box");
                    warned_poly = true;
                }
                Keepout k;
                k.rect = {x1, y1, x2, y2};
                k.layer = layer;
                k.kind = ko_kind;
                k.reason = std::string("dsn-") + kind;
                board.keepouts.push_back(k);
            }
        }
    }
    if (pcb->find_child("wiring"))
        result.warnings.push_back(
            "dsn: (wiring ...) pre-routes ignored on import; the router re-routes from pads");

    // ---- microvia / via_rule: parse + record + warn (honest subset) ------
    // Specctra declares micro-vias and via rotation/offset rules inside
    // `(structure ...)`:
    //   (microvia <name> <layer1> <layer2> <diameter> <hole> ...)
    //   (via_rule <name> (rotate <deg>) (offset <dx> <dy>) ...)
    // The Board model has only through vias, so these declarations are
    // parsed and RECORDED (names listed in the import warnings) but never
    // silently converted into through-via copper: a blind/buried/micro span
    // materialized as a through via would fabricate copper the file did not
    // declare. Never fatal. Some writers nest the rule under a via node
    // (`(via_rule (via <name> ...) ...)`); both dialects are accepted.
    std::vector<std::string> microvia_names, viarule_names;
    for (const SexprNode* mv : structure->find_all("microvia")) {
        std::string name = atom_str(mv, 1, "");
        microvia_names.push_back(name.empty() ? "<unnamed>" : name);
    }
    for (const SexprNode* vr : structure->find_all("via_rule")) {
        std::string name = atom_str(vr, 1, "");
        if (name.empty()) {
            for (const SexprNode* v : vr->find_all("via")) {
                std::string vname = atom_str(v, 1, "");
                if (!vname.empty()) viarule_names.push_back(vname);
            }
        } else {
            viarule_names.push_back(name);
        }
    }
    if (!microvia_names.empty() || !viarule_names.empty()) {
        std::string detail;
        auto append_list = [&](const char* what, const std::vector<std::string>& names) {
            if (names.empty()) return;
            if (!detail.empty()) detail += "; ";
            detail += std::string(what) + ":";
            for (const auto& n : names) detail += " " + n;
        };
        append_list("microvia", microvia_names);
        append_list("via_rule", viarule_names);
        result.warnings.push_back(
            "dsn: recorded " + detail +
            " (blind/buried/micro spans are not modeled; parsed and recorded, "
            "no via copper fabricated)");
    }
    // Blind/buried spans also ride on ordinary via declarations in some
    // writers (a `(blind ...)`/`(buried ...)` property on the via); the
    // keyword scan below catches those without a grammar table.
    if (text.find("blind") != std::string::npos || text.find("buried") != std::string::npos)
        result.warnings.push_back(
            "dsn: blind/buried via spans are not modeled and were ignored");

    // Default via geometry note for agents.
    if (!via_geom.empty()) {
        std::string names;
        for (const auto& kv : via_geom) {
            if (!names.empty()) names += ",";
            names += kv.first;
        }
        result.warnings.push_back("dsn: via geometries available: " + names);
    }
    // Pad geometry honesty: a pad without a padstack shape (or naming a stack
    // the library never declared) falls back to a 0.3 mm square. Reported once
    // per import instead of silently under- or over-stating copper.
    if (pads_without_shape > 0) {
        result.warnings.push_back("dsn: " + std::to_string(pads_without_shape) +
                                  " pad(s) carry no padstack shape; assumed 0.3mm squares");
    }
    for (const auto& kv : unknown_padstacks) {
        std::string names;
        for (const auto& s : kv.second) {
            if (!names.empty()) names += ",";
            names += s;
        }
        result.warnings.push_back("dsn: image '" + kv.first +
                                  "' references unknown padstack(s) " + names +
                                  "; those pads assumed 0.3mm squares");
    }
    (void)design;
    return result;
}

std::string board_to_ses(const Board& board, const std::string& base_design) {
    std::ostringstream os;
    std::string design = base_design.empty() ? board.source_file : base_design;
    if (design.empty()) design = "copperline_board";
    os << "(session \"" << design << "\"\n";
    os << "  (base_design \"" << design << "\")\n";
    os << "  (unit mm)\n";
    os << "  (route\n";
    os << "    (library\n";
    // Via geometries actually used on this board.
    std::map<std::string, std::pair<Coord, Coord>> padstacks;
    for (const auto& v : board.vias) {
        std::string name = v.via_class.empty() ? "STD" : v.via_class;
        padstacks[name] = {v.outer_d_nm, v.hole_d_nm};
    }
    if (padstacks.empty())
        padstacks["STD"] = {board.defaults.via_outer_nm, board.defaults.via_hole_nm};
    for (const auto& kv : padstacks) {
        char ob[64], hb[64];
        std::snprintf(ob, sizeof(ob), "%.6f", nm_to_mm(kv.second.first));
        std::snprintf(hb, sizeof(hb), "%.6f", nm_to_mm(kv.second.second));
        os << "      (padstack \"" << kv.first << "\" " << ob << " " << hb << ")\n";
    }
    os << "    )\n";
    os << "    (network\n";
    std::map<LayerId, std::string> layer_name;
    for (const auto& l : board.layers) layer_name[l.id] = l.name;
    // Deterministic net order: by net id.
    std::vector<const NetInfo*> nets;
    for (const auto& n : board.nets) nets.push_back(&n);
    std::sort(nets.begin(), nets.end(),
              [](const NetInfo* a, const NetInfo* b) { return a->id < b->id; });
    for (const NetInfo* n : nets) {
        os << "      (net \"" << n->name << "\"\n";
        // Group this net's segments into path runs (same layer, chained).
        std::vector<TraceSeg> segs;
        for (const auto& t : board.traces)
            if (t.net == n->id) segs.push_back(t);
        std::sort(segs.begin(), segs.end(), [](const TraceSeg& a, const TraceSeg& b) {
            if (a.layer != b.layer) return a.layer < b.layer;
            if (a.a.x != b.a.x) return a.a.x < b.a.x;
            if (a.a.y != b.a.y) return a.a.y < b.a.y;
            if (a.b.x != b.b.x) return a.b.x < b.b.x;
            return a.b.y < b.b.y;
        });
        for (const auto& s : segs) {
            char w[64];
            std::snprintf(w, sizeof(w), "%.6f", nm_to_mm(s.width_nm));
            auto lit = layer_name.find(s.layer);
            std::string lname = lit == layer_name.end() ? std::to_string(s.layer) : lit->second;
            os << "        (wire (path \"" << lname << "\" ";
            write_mm(os, s.a.x);
            os << " ";
            write_mm(os, s.a.y);
            os << " ";
            write_mm(os, s.b.x);
            os << " ";
            write_mm(os, s.b.y);
            os << " " << w << ")) \n";
            (void)w;
        }
        for (const auto& v : board.vias) {
            if (v.net != n->id) continue;
            std::string name = v.via_class.empty() ? "STD" : v.via_class;
            os << "        (via \"" << name << "\" ";
            write_mm(os, v.pos.x);
            os << " ";
            write_mm(os, v.pos.y);
            os << ")\n";
        }
        os << "      )\n";
    }
    os << "    )\n";
    os << "  )\n";
    os << ")\n";
    return os.str();
}

std::vector<std::string> ses_merge_into(Board& board, const std::string& text,
                                        const std::string& path) {
    std::vector<std::string> warnings;
    std::unique_ptr<SexprNode> root;
    try {
        root = parse_sexpr(text);
    } catch (const std::exception& e) {
        throw BoardError(InputKind::kInvalid, std::string("bad SES: ") + e.what());
    }
    // parse_sexpr returns a wrapper root holding all top-level forms.
    // Two session dialects are in the wild (docs/reverse/circuitmaker/
    // autorouter_modules.md M2):
    //   classic:     (session (unit ..) (route (library ..) (network ..)))
    //   FreeRouting: (session (resolution ..) (library_out ..) (network_out ..))
    // and some writers wrap the FreeRouting payload in (routes ..). The payload
    // itself - wires and vias per net - is identical, so accept every
    // placement instead of failing on nesting.
    const SexprNode* session = root->find_child("session");
    const SexprNode* top = session ? session : root.get();
    const SexprNode* routes = top->find_child("route");
    if (!routes) routes = top->find_child("routes");
    auto find_payload = [&](std::initializer_list<const char*> heads) -> const SexprNode* {
        for (const char* head : heads) {
            if (routes)
                if (const SexprNode* n = routes->find_child(head)) return n;
            if (const SexprNode* n = top->find_child(head)) return n;
        }
        return nullptr;
    };
    const SexprNode* network = find_payload({"network", "network_out"});
    if (!network)
        throw BoardError(InputKind::kInvalid,
                         "SES: missing (route (network ...)) or (network_out ...)");
    const SexprNode* library = find_payload({"library", "library_out"});

    // ---- coordinate scale --------------------------------------------------
    // (resolution <unit> [<digits>]) wins over (unit <unit> [<digits>]) - the
    // same rule as the DSN importer. The chosen scale is always reported.
    const SexprNode* decl = nullptr;
    std::string decl_head;
    for (const char* head : {"resolution", "unit"}) {
        if (routes)
            if ((decl = routes->find_child(head))) {
                decl_head = head;
                break;
            }
        if ((decl = top->find_child(head))) {
            decl_head = head;
            break;
        }
    }
    double unit_scale_mm = 1.0;
    double decl_scale_mm = 1.0;
    if (decl) {
        std::string uname = atom_str(decl, 1, "mm");
        if (uname.empty()) uname = "mm";
        double unit_mm = unit_to_mm(uname);
        if (unit_mm < 0)
            throw BoardError(InputKind::kInvalid, "SES: unknown (" + decl_head + " " + uname + ")");
        unit_scale_mm = unit_mm;
        decl_scale_mm = unit_mm;
        double digits = 0.0;
        if (decl->children.size() > 2 && parse_number(atom_str(decl, 2, ""), digits))
            decl_scale_mm = unit_mm * std::pow(10.0, -std::clamp(digits, 0.0, 18.0));
    } else {
        warnings.push_back(
            "ses: no (resolution ...) or (unit ...) declared; coordinates read as millimetres");
    }

    std::map<std::string, LayerId> layer_by_norm;
    for (const auto& l : board.layers) layer_by_norm[norm_name(l.name)] = l.id;
    auto lookup_layer = [&](const std::string& raw) -> LayerId {
        auto it = layer_by_norm.find(norm_name(raw));
        if (it != layer_by_norm.end()) return it->second;
        // FreeRouting/KiCad name the outer layers F.Cu/B.Cu. Map those aliases
        // onto this board's outer layers and say so rather than dropping
        // copper on a layer-name technicality.
        const std::string n = norm_name(raw);
        const bool front = (n == "F.CU" || n == "FCU" || n == "FRONT" || n == "TOP");
        const bool back = (n == "B.CU" || n == "BCU" || n == "BACK" || n == "BOTTOM");
        if (front || back) {
            const Layer& l = front ? board.layers.front() : board.layers.back();
            warnings.push_back("ses: layer '" + raw + "' matched outer layer '" + l.name + "'");
            return l.id;
        }
        throw BoardError(InputKind::kInvalid, "SES: unknown layer '" + raw + "'");
    };

    // ---- padstacks ---------------------------------------------------------
    // Shape form:   (padstack "ps" (shape (circle <layer> <dia>)) (hole <dia>))
    // Numeric form: (padstack "ps" <outer> <hole>)      [classic library]
    // FreeRouting nests its padstacks inside (image ...) elements. Geometry is
    // kept in file counts; conversion happens once the scale is settled.
    struct SesPadstack {
        double outer = 0.0;  // file counts; <= 0 means "no usable shape"
        double hole = 0.0;
    };
    std::map<std::string, SesPadstack> padstacks;  // norm_name -> geometry
    if (library) {
        std::vector<const SexprNode*> defs;
        std::function<void(const SexprNode*, int)> walk = [&](const SexprNode* n, int depth) {
            for (const auto& ch : n->children) {
                if (!ch || ch->is_atom) continue;
                const std::string head = ch->head();
                if (head == "padstack") {
                    defs.push_back(ch.get());
                } else if ((head == "image" || head == "library") && depth < 3) {
                    walk(ch.get(), depth + 1);
                }
            }
        };
        walk(library, 0);
        for (const SexprNode* ps : defs) {
            const std::string name = atom_str(ps, 1, "STD");
            SesPadstack g;
            bool shaped = false;
            for (const SexprNode* shape : ps->find_all("shape")) {
                const SexprNode* geom = (shape->children.size() > 1 && !shape->children[1]->is_atom)
                                            ? shape->children[1].get()
                                            : nullptr;
                if (!geom) continue;
                const std::string gh = geom->head();
                double extent = 0.0;
                if (gh == "circle") {
                    extent = atom_num(geom, 2, 0.0);
                } else if (gh == "rect" || gh == "octagon" || gh == "polygon") {
                    // Enclosing extent of the shape's vertex bbox.
                    std::size_t i = first_numeric_atom(geom);
                    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
                    bool first = true;
                    for (; i + 1 < geom->children.size(); i += 2) {
                        if (!is_number_atom(geom->children[i].get())) break;
                        const double x = atom_num(geom, i), y = atom_num(geom, i + 1);
                        if (first) {
                            x1 = x2 = x;
                            y1 = y2 = y;
                            first = false;
                        }
                        x1 = std::min(x1, x);
                        y1 = std::min(y1, y);
                        x2 = std::max(x2, x);
                        y2 = std::max(y2, y);
                    }
                    if (!first) extent = std::max(std::fabs(x2 - x1), std::fabs(y2 - y1));
                }
                if (extent > 0) {
                    g.outer = std::max(g.outer, extent);
                    shaped = true;
                }
            }
            if (const SexprNode* h = ps->find_child("hole")) {
                const double hv = atom_num(h, 1, 0.0);
                if (hv > 0) g.hole = hv;
            }
            if (!shaped) {
                // Classic numeric padstack: (padstack "STD" <outer> <hole>).
                double o = 0.0, h = 0.0;
                if (parse_number(atom_str(ps, 2, ""), o)) g.outer = o;
                if (parse_number(atom_str(ps, 3, ""), h)) g.hole = h;
            }
            if (g.outer <= 0) {
                warnings.push_back("ses: padstack '" + name +
                                   "' declares no usable shape; board default via geometry used");
            } else if (g.hole <= 0) {
                warnings.push_back("ses: padstack '" + name +
                                   "' has no (hole ...); assumed half the outer diameter");
                g.hole = g.outer * 0.5;
            }
            padstacks[norm_name(name)] = g;
        }
    }

    // ---- payload: wires + vias per net, still in file counts ---------------
    // Collect everything before scaling so the scale can be chosen from the
    // geometry itself (see below).
    struct RawWire {
        NetId net = -1;
        LayerId layer = 0;
        double ax = 0, ay = 0, bx = 0, by = 0, w = 0;
    };
    struct RawVia {
        NetId net = -1;
        std::string ps;
        double x = 0, y = 0;
    };
    std::vector<RawWire> wires;
    std::vector<RawVia> raw_vias;
    // Both dialects group per net as (wire ...) / (via ...); classic routers
    // additionally wrap them in (wires ...) / (vias ...) containers.
    auto collect = [](const SexprNode* net, const char* head, const char* group) {
        std::vector<const SexprNode*> out = net->find_all(head);
        for (const SexprNode* g : net->find_all(group)) {
            for (const SexprNode* n : g->find_all(head)) out.push_back(n);
        }
        return out;
    };
    for (const SexprNode* nn : network->find_all("net")) {
        const std::string name = atom_str(nn, 1, "");
        const NetInfo* ni = board.find_net_by_name(name);
        if (!ni)
            throw BoardError(InputKind::kInvalid,
                             "SES: net '" + name + "' not in board " + path);
        for (const SexprNode* w : collect(nn, "wire", "wires")) {
            // Width may ride inside the path (classic) or in a sibling
            // (width ...) node (FreeRouting).
            const SexprNode* width_node = w->find_child("width");
            const double w_count = width_node ? atom_num(width_node, 1, 0.0) : 0.0;
            for (const SexprNode* p : w->find_all("path")) {
                const std::string lname = atom_str(p, 1, "");
                const LayerId layer = lookup_layer(lname);
                std::vector<double> nums;
                for (std::size_t i = 2; i < p->children.size(); ++i) {
                    if (!p->children[i]->is_atom)
                        throw BoardError(InputKind::kInvalid, "SES: bad path coordinates");
                    double v = 0.0;
                    if (!parse_number(p->children[i]->atom, v))
                        throw BoardError(InputKind::kInvalid, "SES: bad path number");
                    nums.push_back(v);
                }
                double width = w_count;
                if (!width_node) {
                    // (path layer x1 y1 x2 y2 ... width): even count incl. width.
                    if (nums.size() < 5 || (nums.size() % 2) == 0)
                        throw BoardError(InputKind::kInvalid, "SES: path needs x1 y1 x2 y2 width");
                    width = nums.back();
                    nums.pop_back();
                }
                if (width <= 0)
                    throw BoardError(InputKind::kRule, "SES: path width must be positive");
                if (nums.size() < 4 || (nums.size() % 2) != 0)
                    throw BoardError(InputKind::kInvalid, "SES: path needs >= 2 points");
                // Coordinate list is a chained polyline p0..pk; keep the
                // consecutive pairs (p0,p1), (p1,p2), ... as segments.
                const std::size_t npts = nums.size() / 2;
                for (std::size_t pi = 0; pi + 1 < npts; ++pi) {
                    RawWire rw;
                    rw.net = ni->id;
                    rw.layer = layer;
                    rw.ax = nums[2 * pi];
                    rw.ay = nums[2 * pi + 1];
                    rw.bx = nums[2 * pi + 2];
                    rw.by = nums[2 * pi + 3];
                    rw.w = width;
                    wires.push_back(rw);
                }
            }
        }
        for (const SexprNode* v : collect(nn, "via", "vias")) {
            RawVia rv;
            rv.net = ni->id;
            rv.ps = atom_str(v, 1, "STD");
            rv.x = atom_num(v, 2);
            rv.y = atom_num(v, 3);
            raw_vias.push_back(rv);
        }
    }

    // ---- scale selection ---------------------------------------------------
    // A declared decade that misses this board by orders of magnitude is a
    // dialect trap (real writers emit integer micrometres next to a
    // `(resolution um 10)` header), not a routing fact. Choose between the
    // declared scale and the plain unit scale by asking which one lands the
    // session's copper on this board, and report the choice.
    double lo_x = 0, hi_x = 0, lo_y = 0, hi_y = 0;
    bool have_geo = false;
    auto note = [&](double x, double y) {
        if (!have_geo) {
            lo_x = hi_x = x;
            lo_y = hi_y = y;
            have_geo = true;
        }
        lo_x = std::min(lo_x, x);
        hi_x = std::max(hi_x, x);
        lo_y = std::min(lo_y, y);
        hi_y = std::max(hi_y, y);
    };
    for (const auto& w : wires) {
        note(w.ax, w.ay);
        note(w.bx, w.by);
    }
    for (const auto& v : raw_vias) note(v.x, v.y);
    double scale_mm = decl ? decl_scale_mm : unit_scale_mm;
    if (decl && have_geo) {
        const double board_mm = std::max(nm_to_mm(board.width_nm), nm_to_mm(board.height_nm));
        const double span = std::max(hi_x - lo_x, hi_y - lo_y);
        auto fits = [&](double s) {
            const double mm = span * s;
            return mm <= board_mm * 100.0 && mm >= board_mm / 1000.0;
        };
        if (board_mm > 0 && span > 0 && !fits(decl_scale_mm) && fits(unit_scale_mm)) {
            char buf[220];
            std::snprintf(buf, sizeof(buf),
                          "ses: (%s ...) puts this session's copper %.6g mm across; used the "
                          "unit scale (%.6g mm per count) instead",
                          decl_head.c_str(), span * decl_scale_mm, unit_scale_mm);
            warnings.push_back(buf);
            scale_mm = unit_scale_mm;
        }
    }
    if (decl) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "ses: coordinates scaled by %.6g mm per file count (from (%s ...))",
                      scale_mm, decl_head.c_str());
        warnings.push_back(buf);
    }
    auto s2nm = [&](double v) { return mm_to_nm(v * scale_mm); };

    // ---- commit ------------------------------------------------------------
    bool width_warned = false;
    for (const auto& rw : wires) {
        TraceSeg s;
        s.net = rw.net;
        s.layer = rw.layer;
        s.a = {s2nm(rw.ax), s2nm(rw.ay)};
        s.b = {s2nm(rw.bx), s2nm(rw.by)};
        s.width_nm = s2nm(rw.w);
        if (s.width_nm <= 0) {
            s.width_nm = board.defaults.trace_width_nm;
            if (!width_warned) {
                warnings.push_back(
                    "ses: wire widths round to 0nm at this resolution; board default width used");
                width_warned = true;
            }
        }
        board.traces.push_back(s);
    }
    for (const auto& rv : raw_vias) {
        Via via;
        via.net = rv.net;
        via.pos = {s2nm(rv.x), s2nm(rv.y)};
        via.top_layer = board.layers.front().id;
        via.bottom_layer = board.layers.back().id;
        auto pit = padstacks.find(norm_name(rv.ps));
        if (pit != padstacks.end() && pit->second.outer > 0) {
            via.outer_d_nm = s2nm(pit->second.outer);
            via.hole_d_nm = pit->second.hole > 0 ? s2nm(pit->second.hole) : via.outer_d_nm / 2;
        } else {
            via.outer_d_nm = board.defaults.via_outer_nm;
            via.hole_d_nm = board.defaults.via_hole_nm;
            warnings.push_back("ses: unknown padstack '" + rv.ps +
                               "'; used board default via geometry");
        }
        via.via_class = rv.ps;
        board.vias.push_back(via);
    }
    return warnings;
}

// KiCad importer lives in kicad.cpp (declared here to avoid header weight).
extern const BoardImporter& kicad_importer_singleton();

std::vector<std::string> merge_routes_file(Board& board, const std::string& path) {
    std::string text = read_file_text(path);    std::string head = text.substr(0, std::min<std::size_t>(text.size(), 4096));
    if (ends_with(path, ".ses") || head.find("(session") != std::string::npos)
        return ses_merge_into(board, text, path);
    if (ends_with(path, ".kicad_pcb") || head.find("(kicad_pcb") != std::string::npos) {
        // KiCad board: copy traces/vias with net-name remapping (same as JSON).
        ImportResult r = kicad_importer_singleton().import_file(path);
        std::map<NetId, NetId> net_map;
        for (const auto& n : r.board.nets) {
            const NetInfo* mine = board.find_net_by_name(n.name);
            if (!mine)
                throw BoardError(InputKind::kInvalid,
                                 "routes: net '" + n.name + "' not in board");
            net_map[n.id] = mine->id;
        }
        std::vector<std::string> warnings = r.warnings;
        for (const auto& t : r.board.traces) {
            auto it = net_map.find(t.net);
            if (it == net_map.end()) continue;
            TraceSeg s = t;
            s.net = it->second;
            board.traces.push_back(s);
        }
        for (const auto& v : r.board.vias) {
            auto it = net_map.find(v.net);
            if (it == net_map.end()) continue;
            Via vv = v;
            vv.net = it->second;
            board.vias.push_back(vv);
        }
        return warnings;
    }
    // Native JSON board: copy traces/vias with net-name remapping.
    JsonValue root;
    try {
        root = parse_json(text);
    } catch (const std::exception& e) {
        throw BoardError(InputKind::kInvalid, std::string("bad routes JSON: ") + e.what());
    }
    JsonBoardImporter json;
    ImportResult r = json.import_value(root, path);
    std::map<NetId, NetId> net_map;  // routes-file net id -> board net id
    for (const auto& n : r.board.nets) {
        const NetInfo* mine = board.find_net_by_name(n.name);
        if (!mine)
            throw BoardError(InputKind::kInvalid,
                             "routes: net '" + n.name + "' not in board");
        net_map[n.id] = mine->id;
    }
    std::vector<std::string> warnings = r.warnings;
    for (const auto& t : r.board.traces) {
        auto it = net_map.find(t.net);
        if (it == net_map.end()) continue;
        TraceSeg s = t;
        s.net = it->second;
        board.traces.push_back(s);
    }
    for (const auto& v : r.board.vias) {
        auto it = net_map.find(v.net);
        if (it == net_map.end()) continue;
        Via vv = v;
        vv.net = it->second;
        board.vias.push_back(vv);
    }
    return warnings;
}

void apply_sidecar_nets(Board& board, const JsonValue& config) {
    // Global sidecar defaults complement net-specific metadata. KiCad keeps
    // project-level netclass defaults in .kicad_pro (not .kicad_pcb), so a
    // standalone board import otherwise falls back to BoardDefaults' generic
    // clearance. Allow callers to carry the project clearance alongside the
    // net sidecar without fabricating per-net overrides.
    if (const JsonValue* defaults = config.find("defaults")) {
        if (!defaults->is_object())
            throw BoardError(InputKind::kRule, "config 'defaults' must be an object");
        if (defaults->has("clearance_mm")) {
            double c = defaults->get_number("clearance_mm", -1);
            if (!(c >= 0))
                throw BoardError(InputKind::kRule, "defaults.clearance_mm must be >= 0");
            board.defaults.clearance_nm = mm_to_nm(c);
        }
    }
    const JsonValue* nets = config.find("nets");
    if (!nets) return;
    if (!nets->is_object())
        throw BoardError(InputKind::kRule, "config 'nets' must be an object");
    for (const auto& kv : nets->as_object()) {
        const std::string& name = kv.first;
        const JsonValue& e = kv.second;
        NetInfo* n = nullptr;
        for (auto& cand : board.nets)
            if (cand.name == name) n = &cand;
        if (!n)
            throw BoardError(InputKind::kRule, "config nets: unknown net '" + name + "'");
        if (!e.is_object())
            throw BoardError(InputKind::kRule, "config nets." + name + " must be an object");
        if (e.has("current_a")) {
            double c = e.get_number("current_a", -1);
            if (!(c > 0)) throw BoardError(InputKind::kRule, "nets." + name + ".current_a > 0");
            n->has_current = true;
            n->current_a = c;
        }
        if (e.has("peak_a")) {
            double c = e.get_number("peak_a", -1);
            if (!(c > 0)) throw BoardError(InputKind::kRule, "nets." + name + ".peak_a > 0");
            n->has_peak = true;
            n->peak_a = c;
        }
        if (e.has("voltage_v")) {
            if (!e.find("voltage_v")->is_number())
                throw BoardError(InputKind::kRule, "nets." + name + ".voltage_v must be a number");
            n->has_voltage = true;
            n->voltage_v = e.get_number("voltage_v", 0);
        }
        if (e.has("trace_width_min_mm")) {
            double w = e.get_number("trace_width_min_mm", -1);
            if (!(w > 0))
                throw BoardError(InputKind::kRule, "nets." + name + ".trace_width_min_mm > 0");
            n->has_min_width = true;
            n->min_width_nm = mm_to_nm(w);
        }
        if (e.has("class") || e.has("net_class")) {
            // Informational class label; width/via classes stay explicit.
            std::string cls = e.has("class") ? e.get_string("class") : e.get_string("net_class");
            if (cls.empty())
                throw BoardError(InputKind::kRule, "nets." + name + ".class must be a string");
            n->voltage_class = n->voltage_class.empty() ? cls : n->voltage_class;
        }
        if (e.has("width_class")) {
            std::string wc = e.get_string("width_class");
            if (wc.empty())
                throw BoardError(InputKind::kRule, "nets." + name + ".width_class must be a string");
            n->width_class = wc;
        }
        if (e.has("via_class")) {
            std::string vc = e.get_string("via_class");
            if (vc.empty())
                throw BoardError(InputKind::kRule, "nets." + name + ".via_class must be a string");
            n->via_class = vc;
        }
        if (e.has("voltage_class")) {
            std::string vc = e.get_string("voltage_class");
            if (vc.empty())
                throw BoardError(InputKind::kRule,
                                 "nets." + name + ".voltage_class must be a string");
            n->voltage_class = vc;
        }
        if (e.has("clearance_mm")) {
            double c = e.get_number("clearance_mm", -1);
            if (!(c >= 0))
                throw BoardError(InputKind::kRule, "nets." + name + ".clearance_mm >= 0");
            n->has_min_clearance = true;
            n->min_clearance_nm = mm_to_nm(c);
        }
    }
}

const BoardImporter& dsn_importer_singleton() {
    static const DsnImporter kInstance;
    return kInstance;
}

}  // namespace copperline
