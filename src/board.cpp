#include "router/board.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace copperline {

const NetInfo* Board::find_net(NetId id) const {
    for (const auto& n : nets)
        if (n.id == id) return &n;
    return nullptr;
}

NetInfo* Board::find_net(NetId id) {
    for (auto& n : nets)
        if (n.id == id) return &n;
    return nullptr;
}

const NetInfo* Board::find_net_by_name(const std::string& name) const {
    for (const auto& n : nets)
        if (n.name == name) return &n;
    return nullptr;
}

const Terminal* Board::find_terminal(TermId id) const {
    for (const auto& t : terminals)
        if (t.id == id) return &t;
    return nullptr;
}

bool Board::valid_layer(LayerId id) const {
    for (const auto& l : layers)
        if (l.id == id) return true;
    return false;
}

Rect PlaneZone::bounds() const {
    if (poly.empty()) return {0, 0, 0, 0};
    Rect r{poly[0].x, poly[0].y, poly[0].x, poly[0].y};
    for (const auto& p : poly) {
        if (p.x < r.x1) r.x1 = p.x;
        if (p.y < r.y1) r.y1 = p.y;
        if (p.x > r.x2) r.x2 = p.x;
        if (p.y > r.y2) r.y2 = p.y;
    }
    return r;
}

// ---- Issue #16: integer-nm polygon predicates (exact, deterministic) ----

namespace {

// Orientation test with 128-bit intermediates.
int orient(const Point& a, const Point& b, const Point& c) {
    __int128 v = (__int128)(b.y - a.y) * (c.x - b.x) - (__int128)(b.x - a.x) * (c.y - b.y);
    if (v == 0) return 0;
    return v > 0 ? 1 : 2;
}

bool on_seg(const Point& a, const Point& b, const Point& c) {
    return b.x >= std::min(a.x, c.x) && b.x <= std::max(a.x, c.x) &&
           b.y >= std::min(a.y, c.y) && b.y <= std::max(a.y, c.y);
}

bool segs_cross(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
    int o1 = orient(p1, p2, p3), o2 = orient(p1, p2, p4);
    int o3 = orient(p3, p4, p1), o4 = orient(p3, p4, p2);
    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && on_seg(p1, p3, p2)) return true;
    if (o2 == 0 && on_seg(p1, p4, p2)) return true;
    if (o3 == 0 && on_seg(p3, p1, p4)) return true;
    if (o4 == 0 && on_seg(p3, p2, p4)) return true;
    return false;
}

// Squared distance from point to segment, rounded to integer (exact for
// axis-aligned segments; deterministic otherwise).
__int128 pt_seg_d2(Point p, Point a, Point b) {
    __int128 vx = (__int128)b.x - a.x, vy = (__int128)b.y - a.y;
    __int128 wx = (__int128)p.x - a.x, wy = (__int128)p.y - a.y;
    __int128 len2 = vx * vx + vy * vy;
    if (len2 == 0) return wx * wx + wy * wy;
    long double t = (long double)(wx * vx + wy * vy) / (long double)len2;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    long double cx = (long double)a.x + t * (long double)vx;
    long double cy = (long double)a.y + t * (long double)vy;
    long double dx = (long double)p.x - cx, dy = (long double)p.y - cy;
    __int128 ix = (__int128)llround(dx), iy = (__int128)llround(dy);
    return ix * ix + iy * iy;
}

}  // namespace

bool plane_poly_contains(const std::vector<Point>& poly, Point p) {
    std::size_t n = poly.size();
    if (n < 3) return false;
    // Boundary counts as inside: check edges first.
    for (std::size_t i = 0; i < n; ++i) {
        const Point& a = poly[i];
        const Point& b = poly[(i + 1) % n];
        if (orient(a, b, p) == 0 && on_seg(a, p, b)) return true;
    }
    // Ray casting (+x ray) with exact orientation tests.
    bool inside = false;
    for (std::size_t i = 0; i < n; ++i) {
        const Point& a = poly[i];
        const Point& b = poly[(i + 1) % n];
        if ((a.y > p.y) != (b.y > p.y)) {
            // x of the edge at height p.y, compared without division:
            // x_int > p.x  <=>  (b.x-a.x)*(p.y-a.y)/(b.y-a.y) > p.x-a.x
            __int128 lhs = (__int128)(b.x - a.x) * (p.y - a.y);
            __int128 rhs = (__int128)(p.x - a.x) * (b.y - a.y);
            bool cross_right;
            if (b.y > a.y) cross_right = lhs > rhs;
            else cross_right = lhs < rhs;
            if (cross_right) inside = !inside;
        }
    }
    return inside;
}

Point plane_poly_nearest(const std::vector<Point>& poly, Point p) {
    if (poly.empty()) return p;
    if (plane_poly_contains(poly, p)) return p;
    __int128 best = -1;
    Point out = poly[0];
    std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Point& a = poly[i];
        const Point& b = poly[(i + 1) % n];
        __int128 vx = (__int128)b.x - a.x, vy = (__int128)b.y - a.y;
        __int128 wx = (__int128)p.x - a.x, wy = (__int128)p.y - a.y;
        __int128 len2 = vx * vx + vy * vy;
        long double t = 0;
        if (len2 != 0) {
            t = (long double)(wx * vx + wy * vy) / (long double)len2;
            if (t < 0) t = 0;
            if (t > 1) t = 1;
        }
        Point cand{static_cast<Coord>(llround((long double)a.x + t * (long double)vx)),
                   static_cast<Coord>(llround((long double)a.y + t * (long double)vy))};
        __int128 d = pt_seg_d2(p, a, b);
        if (best < 0 || d < best || (d == best && cand < out)) {
            best = d;
            out = cand;
        }
    }
    return out;
}

bool plane_seg_hits_poly(const Segment& s, const std::vector<Point>& poly) {
    std::size_t n = poly.size();
    if (n < 3) return false;
    if (plane_poly_contains(poly, s.a) || plane_poly_contains(poly, s.b)) return true;
    for (std::size_t i = 0; i < n; ++i) {
        if (segs_cross(s.a, s.b, poly[i], poly[(i + 1) % n])) return true;
    }
    return false;
}

bool plane_rect_hits_poly(const Rect& r, const std::vector<Point>& poly) {
    if (poly.empty()) return false;
    // Any polygon vertex inside the rect (boundary inclusive).
    for (const auto& p : poly) {
        if (r.contains(p)) return true;
    }
    // Any rect corner inside the polygon.
    Point corners[4] = {{r.x1, r.y1}, {r.x2, r.y1}, {r.x2, r.y2}, {r.x1, r.y2}};
    for (auto c : corners) {
        if (plane_poly_contains(poly, c)) return true;
    }
    // Any edge crossing.
    Segment edges[4] = {{{r.x1, r.y1}, {r.x2, r.y1}},
                        {{r.x2, r.y1}, {r.x2, r.y2}},
                        {{r.x2, r.y2}, {r.x1, r.y2}},
                        {{r.x1, r.y2}, {r.x1, r.y1}}};
    std::size_t n = poly.size();
    for (const auto& e : edges) {
        for (std::size_t i = 0; i < n; ++i) {
            if (segs_cross(e.a, e.b, poly[i], poly[(i + 1) % n])) return true;
        }
    }
    return false;
}

__int128 plane_seg_poly_dist2(const Segment& s, const std::vector<Point>& poly) {
    if (plane_seg_hits_poly(s, poly)) return 0;
    __int128 best = -1;
    std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        __int128 d = seg_seg_dist2(s, {poly[i], poly[(i + 1) % n]});
        if (best < 0 || d < best) best = d;
    }
    return best < 0 ? 0 : best;
}

__int128 plane_rect_poly_dist2(const Rect& r, const std::vector<Point>& poly) {
    if (plane_rect_hits_poly(r, poly)) return 0;
    __int128 best = -1;
    Segment edges[4] = {{{r.x1, r.y1}, {r.x2, r.y1}},
                        {{r.x2, r.y1}, {r.x2, r.y2}},
                        {{r.x2, r.y2}, {r.x1, r.y2}},
                        {{r.x1, r.y2}, {r.x1, r.y1}}};
    for (const auto& e : edges) {
        __int128 d = plane_seg_poly_dist2(e, poly);
        if (best < 0 || d < best) best = d;
    }
    return best < 0 ? 0 : best;
}

namespace {

std::string read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw BoardError(InputKind::kInvalid, "cannot open file: " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool ends_with(const std::string& s, const std::string& suffix) {
    if (suffix.size() > s.size()) return false;
    std::string low = s, lsuf = suffix;
    std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    std::transform(lsuf.begin(), lsuf.end(), lsuf.begin(), ::tolower);
    return low.compare(low.size() - lsuf.size(), lsuf.size(), lsuf) == 0;
}

// Resolve a net reference that may be an id (number) or a name (string).
NetId resolve_net(const JsonValue& v, const Board& board, const std::string& ctx) {
    if (v.is_number()) {
        NetId id = static_cast<NetId>(v.as_number());
        if (!board.find_net(id))
            throw BoardError(InputKind::kInvalid, "unknown net id in " + ctx);
        return id;
    }
    if (v.is_string()) {
        const NetInfo* n = board.find_net_by_name(v.as_string());
        if (!n) throw BoardError(InputKind::kInvalid, "unknown net name in " + ctx);
        return n->id;
    }
    throw BoardError(InputKind::kInvalid, "bad net reference in " + ctx);
}

LayerId resolve_layer(const JsonValue& v, const Board& board, const std::string& ctx) {
    if (!v.is_number()) throw BoardError(InputKind::kInvalid, "bad layer in " + ctx);
    LayerId id = static_cast<LayerId>(v.as_number());
    if (!board.valid_layer(id)) throw BoardError(InputKind::kInvalid, "unknown layer in " + ctx);
    return id;
}

void parse_terminal_obj(const JsonValue& t, NetId net, Board& board, TermId& next_id) {
    if (!t.is_object()) throw BoardError(InputKind::kInvalid, "terminal must be an object");
    Terminal term;
    term.id = t.has("id") ? static_cast<TermId>(t.get_number("id", 0)) : next_id++;
    if (t.has("id")) next_id = std::max(next_id, term.id + 1);
    if (board.find_terminal(term.id))
        throw BoardError(InputKind::kInvalid, "duplicate terminal id");
    term.net = net;
    if (!t.has("x_mm") || !t.has("y_mm"))
        throw BoardError(InputKind::kInvalid, "terminal missing x_mm/y_mm");
    term.pos = {mm_to_nm(t.get_number("x_mm", 0)), mm_to_nm(t.get_number("y_mm", 0))};
    term.layer = t.has("layer") ? resolve_layer(*t.find("layer"), board, "terminal") : 0;
    if (!board.valid_layer(term.layer))
        throw BoardError(InputKind::kInvalid, "terminal on unknown layer");
    double pw = t.get_number("pad_w_mm", 0.5);
    double ph = t.get_number("pad_h_mm", 0.5);
    if (pw <= 0 || ph <= 0) throw BoardError(InputKind::kRule, "terminal pad size must be positive");
    term.pad_w_nm = mm_to_nm(pw);
    term.pad_h_nm = mm_to_nm(ph);
    term.component = t.get_string("component");
    term.pin = t.get_string("pin");
    board.terminals.push_back(term);
    if (NetInfo* n = board.find_net(net)) n->terminals.push_back(term.id);
}

void parse_net(const JsonValue& n, Board& board, TermId& next_term) {
    if (!n.is_object()) throw BoardError(InputKind::kInvalid, "net must be an object");
    NetInfo net;
    net.id = static_cast<NetId>(n.get_number("id", -1));
    if (net.id < 0) throw BoardError(InputKind::kInvalid, "net missing id");
    if (board.find_net(net.id)) throw BoardError(InputKind::kInvalid, "duplicate net id");
    net.name = n.get_string("name", "N" + std::to_string(net.id));
    if (n.has("current_a")) {
        net.has_current = true;
        net.current_a = n.get_number("current_a", 0);
        if (net.current_a < 0) throw BoardError(InputKind::kRule, "negative current_a");
    }
    if (n.has("peak_a")) {
        net.has_peak = true;
        net.peak_a = n.get_number("peak_a", 0);
        if (net.peak_a < 0) throw BoardError(InputKind::kRule, "negative peak_a");
    }
    if (n.has("min_width_mm")) {
        net.has_min_width = true;
        double w = n.get_number("min_width_mm", 0);
        if (w <= 0) throw BoardError(InputKind::kRule, "min_width_mm must be positive");
        net.min_width_nm = mm_to_nm(w);
    }
    if (n.has("pref_width_mm")) {
        net.has_pref_width = true;
        double w = n.get_number("pref_width_mm", 0);
        if (w <= 0) throw BoardError(InputKind::kRule, "pref_width_mm must be positive");
        net.pref_width_nm = mm_to_nm(w);
    }
    net.width_class = n.get_string("width_class");
    net.via_class = n.get_string("via_class");
    net.allow_neckdown = n.get_bool("allow_neckdown", false);
    if (n.has("neck_width_mm")) {
        double w = n.get_number("neck_width_mm", 0);
        if (w <= 0) throw BoardError(InputKind::kRule, "neck_width_mm must be positive");
        net.neck_width_nm = mm_to_nm(w);
    }
    if (n.has("neck_max_len_mm")) net.neck_max_len_nm = mm_to_nm(n.get_number("neck_max_len_mm", 0));
    if (n.has("voltage_v")) {
        net.has_voltage = true;
        net.voltage_v = n.get_number("voltage_v", 0);
    }
    net.voltage_class = n.get_string("voltage_class");
    if (n.has("min_clearance_mm")) {
        double c = n.get_number("min_clearance_mm", -1);
        if (c < 0) throw BoardError(InputKind::kRule, "min_clearance_mm must be >= 0");
        net.has_min_clearance = true;
        net.min_clearance_nm = mm_to_nm(c);
    }
    // Issue #11: per-net controlled-impedance metadata.
    if (n.has("target_impedance_ohms")) {
        double z = n.get_number("target_impedance_ohms", -1);
        if (z <= 0)
            throw BoardError(InputKind::kRule, "target_impedance_ohms must be positive");
        net.has_impedance = true;
        net.target_impedance_ohms = z;
        if (n.has("impedance_tolerance_pct")) {
            double tol = n.get_number("impedance_tolerance_pct", -1);
            if (tol <= 0 || tol >= 100)
                throw BoardError(InputKind::kRule,
                                 "impedance_tolerance_pct must be in (0, 100)");
            net.has_impedance_tolerance = true;
            net.impedance_tolerance_frac = tol / 100.0;
        } else if (n.has("impedance_tolerance_frac")) {
            double tol = n.get_number("impedance_tolerance_frac", -1);
            if (tol <= 0 || tol >= 1)
                throw BoardError(InputKind::kRule,
                                 "impedance_tolerance_frac must be in (0, 1)");
            net.has_impedance_tolerance = true;
            net.impedance_tolerance_frac = tol;
        }
        if (n.has("impedance_layers")) {
            const JsonValue* il = n.find("impedance_layers");
            if (!il->is_array())
                throw BoardError(InputKind::kInvalid, "impedance_layers must be array");
            for (const auto& e : il->as_array()) {
                if (!e.is_number())
                    throw BoardError(InputKind::kInvalid, "bad layer in impedance_layers");
                LayerId lid = static_cast<LayerId>(e.as_number(0));
                if (!board.valid_layer(lid))
                    throw BoardError(InputKind::kInvalid,
                                     "impedance_layers on unknown layer");
                net.impedance_layers.push_back(lid);
            }
        }
        if (n.has("impedance_ref_plane")) {
            const JsonValue* rp = n.find("impedance_ref_plane");
            if (!rp->is_number())
                throw BoardError(InputKind::kInvalid, "bad impedance_ref_plane");
            LayerId rid = static_cast<LayerId>(rp->as_number(0));
            if (!board.valid_layer(rid))
                throw BoardError(InputKind::kInvalid,
                                 "impedance_ref_plane on unknown layer");
            net.has_impedance_ref_plane = true;
            net.impedance_ref_plane = rid;
        }
    }
    // Issue #15: single-ended length-tuning intent.
    if (n.has("target_length_mm")) {
        double t = n.get_number("target_length_mm", -1);
        if (!(t > 0))
            throw BoardError(InputKind::kRule, "target_length_mm must be positive");
        net.has_target_length = true;
        net.target_length_nm = mm_to_nm(t);
        double tol = n.get_number("length_tol_mm", n.get_number("length_tolerance_mm", 0.0));
        if (!(tol >= 0))
            throw BoardError(InputKind::kRule, "length_tol_mm must be >= 0");
        net.length_tol_nm = mm_to_nm(tol);
        if (net.length_tol_nm > net.target_length_nm)
            throw BoardError(InputKind::kRule, "length tolerance exceeds target length");
    }
    board.nets.push_back(net);
    if (n.has("terminals")) {
        const JsonValue* terms = n.find("terminals");
        if (!terms->is_array()) throw BoardError(InputKind::kInvalid, "net terminals must be array");
        for (const auto& t : terms->as_array()) parse_terminal_obj(t, net.id, board, next_term);
    }
}

}  // namespace

bool JsonBoardImporter::claims(const std::string& path, const std::string& head_bytes) const {
    if (ends_with(path, ".json")) return true;
    std::size_t i = head_bytes.find_first_not_of(" \t\r\n");
    return i != std::string::npos && head_bytes[i] == '{';
}

ImportResult JsonBoardImporter::import_file(const std::string& path) const {
    std::string text = read_file(path);
    JsonValue root;
    try {
        root = parse_json(text);
    } catch (const std::exception& e) {
        throw BoardError(InputKind::kInvalid, std::string("bad JSON: ") + e.what());
    }
    return import_value(root, path);
}

ImportResult JsonBoardImporter::import_value(const JsonValue& root, const std::string& path) const {
    if (!root.is_object()) throw BoardError(InputKind::kInvalid, "board root must be an object");
    ImportResult result;
    Board& board = result.board;
    board.source_format = "json";
    board.source_file = path;

    const JsonValue* b = root.find("board");
    if (!b || !b->is_object()) throw BoardError(InputKind::kInvalid, "missing 'board' object");
    double w = b->get_number("width_mm", -1), h = b->get_number("height_mm", -1);
    if (w <= 0 || h <= 0) throw BoardError(InputKind::kInvalid, "board width_mm/height_mm missing");
    board.width_nm = mm_to_nm(w);
    board.height_nm = mm_to_nm(h);

    const JsonValue* layers = root.find("layers");
    if (layers && layers->is_array()) {
        for (const auto& l : layers->as_array()) {
            Layer layer;
            layer.id = static_cast<LayerId>(l.get_number("id", -1));
            if (layer.id < 0) throw BoardError(InputKind::kInvalid, "layer missing id");
            layer.name = l.get_string("name", "L" + std::to_string(layer.id));
            layer.preferred_horizontal = l.get_bool("preferred_horizontal", false);
            layer.cost_multiplier = l.get_number("cost_multiplier", 1.0);
            if (l.has("copper_weight_oz")) {
                double oz = l.get_number("copper_weight_oz", -1);
                if (oz <= 0)
                    throw BoardError(InputKind::kRule, "copper_weight_oz must be positive");
                layer.copper_weight_oz = oz;
            }
            if (l.has("is_internal")) {
                layer.has_internal_flag = true;
                layer.is_internal = l.get_bool("is_internal", false);
            } else if (l.has("internal")) {
                layer.has_internal_flag = true;
                layer.is_internal = l.get_bool("internal", false);
            }
            // Issue #11: stackup metadata for impedance-aware routing.
            layer.layer_type = l.get_string("layer_type", l.get_string("type", "signal"));
            if (layer.layer_type != "signal" && layer.layer_type != "plane")
                throw BoardError(InputKind::kRule,
                                 "layer_type must be 'signal' or 'plane'");
            if (l.has("dielectric_thickness_mm")) {
                double h = l.get_number("dielectric_thickness_mm", -1);
                if (h <= 0)
                    throw BoardError(InputKind::kRule,
                                     "dielectric_thickness_mm must be positive");
                layer.has_dielectric_thickness = true;
                layer.dielectric_thickness_nm = mm_to_nm(h);
            }
            if (l.has("dielectric_er")) {
                double er = l.get_number("dielectric_er", -1);
                if (er <= 0)
                    throw BoardError(InputKind::kRule, "dielectric_er must be positive");
                layer.has_dielectric_er = true;
                layer.dielectric_er = er;
            }
            if (l.has("ref_plane")) {
                const JsonValue* rp = l.find("ref_plane");
                if (!rp->is_number())
                    throw BoardError(InputKind::kInvalid, "bad ref_plane in layers[]");
                LayerId rid = static_cast<LayerId>(rp->as_number(0));
                bool known = false;
                for (const auto& ol : board.layers)
                    if (ol.id == rid) known = true;
                // Forward references allowed: validate after all layers load.
                (void)known;
                layer.has_ref_plane = true;
                layer.ref_plane_layer = rid;
            }
            if (l.has("copper_thickness_mm")) {
                double t = l.get_number("copper_thickness_mm", -1);
                if (t <= 0)
                    throw BoardError(InputKind::kRule,
                                     "copper_thickness_mm must be positive");
                layer.has_copper_thickness = true;
                layer.copper_thickness_nm = mm_to_nm(t);
            }
            if (l.has("impedance_model")) {
                layer.impedance_model = l.get_string("impedance_model");
                if (layer.impedance_model != "microstrip" &&
                    layer.impedance_model != "stripline" && layer.impedance_model != "auto")
                    throw BoardError(InputKind::kRule,
                                     "impedance_model must be 'microstrip', 'stripline' or 'auto'");
                if (layer.impedance_model == "auto") layer.impedance_model.clear();
            }
            board.layers.push_back(layer);
        }
    }
    if (board.layers.empty()) {
        board.layers.push_back({0, "Top", false, 1.0});
        board.layers.push_back({1, "Bottom", true, 1.0});
    }
    // Issue #11: validate reference-plane identities now that every layer id
    // is known (forward references were allowed during the loop above).
    for (const auto& l : board.layers) {
        if (l.has_ref_plane && l.ref_plane_layer != kAllLayers) {
            bool known = false;
            for (const auto& o : board.layers)
                if (o.id == l.ref_plane_layer) known = true;
            if (!known)
                throw BoardError(InputKind::kInvalid, "layer ref_plane on unknown layer");
        }
    }

    const JsonValue* def = root.find("defaults");
    if (def && def->is_object()) {
        auto need_pos = [&](const char* key, Coord& out) {
            if (def->has(key)) {
                double v = def->get_number(key, -1);
                if (v <= 0) throw BoardError(InputKind::kRule, std::string(key) + " must be positive");
                out = mm_to_nm(v);
            }
        };
        need_pos("trace_width_mm", board.defaults.trace_width_nm);
        need_pos("clearance_mm", board.defaults.clearance_nm);
        need_pos("via_outer_mm", board.defaults.via_outer_nm);
        need_pos("via_hole_mm", board.defaults.via_hole_nm);
        if (def->has("default_current_a")) {
            double c = def->get_number("default_current_a", -1);
            if (c < 0) throw BoardError(InputKind::kRule, "default_current_a must be >= 0");
            board.defaults.default_current_a = c;
        }
        if (def->has("default_voltage_v")) board.defaults.default_voltage_v = def->get_number("default_voltage_v", 0);
        if (def->has("copper_weight_oz")) {
            double oz = def->get_number("copper_weight_oz", -1);
            if (oz <= 0) throw BoardError(InputKind::kRule, "copper_weight_oz must be positive");
            board.defaults.copper_weight_oz = oz;
        }
        if (def->has("temp_rise_c")) {
            double dt = def->get_number("temp_rise_c", -1);
            if (dt <= 0) throw BoardError(InputKind::kRule, "temp_rise_c must be positive");
            board.defaults.temp_rise_c = dt;
        }
    }

    const JsonValue* nets = root.find("nets");
    if (!nets || !nets->is_array()) throw BoardError(InputKind::kInvalid, "missing 'nets' array");
    TermId next_term = 0;
    for (const auto& n : nets->as_array()) parse_net(n, board, next_term);

    const JsonValue* terms = root.find("terminals");
    if (terms) {
        if (!terms->is_array()) throw BoardError(InputKind::kInvalid, "'terminals' must be array");
        for (const auto& t : terms->as_array()) {
            if (!t.has("net")) throw BoardError(InputKind::kInvalid, "terminal missing net");
            NetId net = resolve_net(*t.find("net"), board, "terminals[]");
            parse_terminal_obj(t, net, board, next_term);
        }
    }

    auto in_bounds = [&](Point p) {
        return p.x >= 0 && p.y >= 0 && p.x <= board.width_nm && p.y <= board.height_nm;
    };
    for (const auto& t : board.terminals) {
        if (!in_bounds(t.pos)) {
            result.warnings.push_back("terminal " + std::to_string(t.id) + " outside board bounds");
        }
    }

    const JsonValue* kos = root.find("keepouts");
    if (kos) {
        if (!kos->is_array()) throw BoardError(InputKind::kInvalid, "'keepouts' must be array");
        for (const auto& k : kos->as_array()) {
            Keepout ko;
            Rect r{mm_to_nm(k.get_number("x1_mm", 0)), mm_to_nm(k.get_number("y1_mm", 0)),
                   mm_to_nm(k.get_number("x2_mm", 0)), mm_to_nm(k.get_number("y2_mm", 0))};
            if (r.x2 < r.x1 || r.y2 < r.y1)
                throw BoardError(InputKind::kInvalid, "keepout has inverted corners");
            ko.rect = r;
            ko.layer = k.has("layer") ? static_cast<LayerId>(k.get_number("layer", -1)) : kAllLayers;
            if (ko.layer != kAllLayers && !board.valid_layer(ko.layer))
                throw BoardError(InputKind::kInvalid, "keepout on unknown layer");
            ko.reason = k.get_string("reason");
            board.keepouts.push_back(ko);
        }
    }

    // Issue #16: declared plane/zone objects ("planes" or legacy "zones").
    const JsonValue* planes = root.find("planes");
    if (!planes) planes = root.find("zones");
    if (planes) {
        if (!planes->is_array()) throw BoardError(InputKind::kInvalid, "'planes' must be array");
        int next_plane = 0;
        for (const auto& z : planes->as_array()) {
            if (!z.is_object()) throw BoardError(InputKind::kInvalid, "plane must be an object");
            PlaneZone plane;
            plane.id = z.has("id") ? static_cast<int>(z.get_number("id", 0)) : next_plane;
            if (z.has("id")) next_plane = std::max(next_plane, plane.id + 1);
            else next_plane++;
            for (const auto& other : board.planes) {
                if (other.id == plane.id)
                    throw BoardError(InputKind::kInvalid, "duplicate plane id");
            }
            if (!z.has("net")) throw BoardError(InputKind::kInvalid, "plane missing net");
            plane.net = resolve_net(*z.find("net"), board, "planes[]");
            plane.layer = z.has("layer") ? resolve_layer(*z.find("layer"), board, "planes[]") : 0;
            plane.island = z.has("island") ? static_cast<int>(z.get_number("island", 0)) : 0;
            plane.routable = z.get_bool("routable", z.get_bool("usable", true));
            const JsonValue* poly = z.find("polygon_mm");
            if (!poly) poly = z.find("polygon");
            if (poly && poly->is_array()) {
                for (const auto& pt : poly->as_array()) {
                    if (!pt.is_array() || pt.as_array().size() < 2)
                        throw BoardError(InputKind::kInvalid, "plane polygon points need [x, y]");
                    double x = pt.as_array()[0].as_number(0);
                    double y = pt.as_array()[1].as_number(0);
                    plane.poly.push_back({mm_to_nm(x), mm_to_nm(y)});
                }
            } else if (z.has("x1_mm")) {
                // Rect shorthand: x1/y1/x2/y2 in mm.
                Rect r{mm_to_nm(z.get_number("x1_mm", 0)), mm_to_nm(z.get_number("y1_mm", 0)),
                       mm_to_nm(z.get_number("x2_mm", 0)), mm_to_nm(z.get_number("y2_mm", 0))};
                if (r.x2 < r.x1 || r.y2 < r.y1)
                    throw BoardError(InputKind::kInvalid, "plane has inverted corners");
                plane.poly = {{r.x1, r.y1}, {r.x2, r.y1}, {r.x2, r.y2}, {r.x1, r.y2}};
            } else {
                throw BoardError(InputKind::kInvalid, "plane needs polygon_mm or rect corners");
            }
            if (plane.poly.size() < 3)
                throw BoardError(InputKind::kInvalid, "plane polygon needs >= 3 points");
            board.planes.push_back(plane);
        }
        std::sort(board.planes.begin(), board.planes.end(),
                  [](const PlaneZone& a, const PlaneZone& b) { return a.id < b.id; });
    }

    // Issue #12: differential-pair declarations ("diffpairs").
    const JsonValue* pairs = root.find("diffpairs");
    if (!pairs) pairs = root.find("diff_pairs");
    if (pairs) {
        if (!pairs->is_array()) throw BoardError(InputKind::kInvalid, "'diffpairs' must be array");
        int next_pair = 0;
        for (const auto& e : pairs->as_array()) {
            if (!e.is_object()) throw BoardError(InputKind::kInvalid, "diffpair must be an object");
            DiffPair pr;
            pr.id = e.has("id") ? static_cast<int>(e.get_number("id", 0)) : next_pair;
            if (e.has("id")) next_pair = std::max(next_pair, pr.id + 1);
            else next_pair++;
            for (const auto& other : board.diffpairs) {
                if (other.id == pr.id)
                    throw BoardError(InputKind::kInvalid, "duplicate diffpair id");
            }
            pr.name = e.get_string("name", "PAIR" + std::to_string(pr.id));
            const JsonValue* pv = e.find("p");
            if (!pv) pv = e.find("net_p");
            if (!pv) pv = e.find("P");
            const JsonValue* nv = e.find("n");
            if (!nv) nv = e.find("net_n");
            if (!nv) nv = e.find("N");
            if (!pv || !nv) throw BoardError(InputKind::kInvalid, "diffpair needs p/n nets");
            pr.net_p = resolve_net(*pv, board, "diffpairs[]");
            pr.net_n = resolve_net(*nv, board, "diffpairs[]");
            if (pr.net_p == pr.net_n)
                throw BoardError(InputKind::kRule, "diffpair p and n must differ");
            if (!e.has("gap_mm"))
                throw BoardError(InputKind::kInvalid, "diffpair needs gap_mm");
            double gap = e.get_number("gap_mm", -1);
            if (!(gap > 0)) throw BoardError(InputKind::kRule, "diffpair gap_mm must be positive");
            pr.gap_nm = mm_to_nm(gap);
            double tol = e.get_number("gap_tol_mm", e.get_number("gap_tolerance_mm", 0.0));
            if (!(tol >= 0)) throw BoardError(InputKind::kRule, "diffpair gap_tol_mm must be >= 0");
            pr.gap_tol_nm = mm_to_nm(tol);
            if (pr.gap_tol_nm > pr.gap_nm)
                throw BoardError(InputKind::kRule, "diffpair gap tolerance exceeds gap");
            if (e.has("width_mm")) {
                double w = e.get_number("width_mm", -1);
                if (!(w > 0)) throw BoardError(InputKind::kRule, "diffpair width_mm must be positive");
                pr.has_width = true;
                pr.width_nm = mm_to_nm(w);
            }
            if (e.has("layers")) {
                const JsonValue* ly = e.find("layers");
                if (!ly->is_array())
                    throw BoardError(InputKind::kInvalid, "diffpair layers must be array");
                for (const auto& le : ly->as_array())
                    pr.preferred_layers.push_back(resolve_layer(le, board, "diffpairs[]"));
            }
            if (e.has("preferred_layers")) {
                const JsonValue* ly = e.find("preferred_layers");
                if (!ly->is_array())
                    throw BoardError(InputKind::kInvalid, "diffpair preferred_layers must be array");
                for (const auto& le : ly->as_array())
                    pr.preferred_layers.push_back(resolve_layer(le, board, "diffpairs[]"));
            }
            if (e.has("target_impedance_ohms")) {
                double z = e.get_number("target_impedance_ohms", -1);
                if (!(z > 0))
                    throw BoardError(InputKind::kRule,
                                     "diffpair target_impedance_ohms must be positive");
                pr.has_impedance = true;
                pr.target_impedance_ohms = z;
            }
            if (e.has("max_skew_mm")) {
                double s = e.get_number("max_skew_mm", -1);
                if (!(s >= 0)) throw BoardError(InputKind::kRule, "diffpair max_skew_mm must be >= 0");
                pr.has_max_skew = true;
                pr.max_skew_nm = mm_to_nm(s);
            }
            pr.via_policy = e.get_string("via_policy", "paired");
            if (pr.via_policy != "paired" && pr.via_policy != "independent")
                throw BoardError(InputKind::kRule,
                                 "diffpair via_policy must be 'paired' or 'independent'");
            // Issue #15: symmetric pair tuning ("symmetric": both members to
            // a common length; default "shorter": minimum added on shorter).
            if (e.has("symmetric_tuning")) {
                pr.symmetric_tuning = e.get_bool("symmetric_tuning", false);
            } else if (e.has("tuning_mode")) {
                std::string tm = e.get_string("tuning_mode", "shorter");
                if (tm != "shorter" && tm != "symmetric")
                    throw BoardError(InputKind::kRule,
                                     "diffpair tuning_mode must be 'shorter' or 'symmetric'");
                pr.symmetric_tuning = (tm == "symmetric");
            }
            board.diffpairs.push_back(pr);
        }
        std::sort(board.diffpairs.begin(), board.diffpairs.end(),
                  [](const DiffPair& a, const DiffPair& b) { return a.id < b.id; });
    }

    const JsonValue* traces = root.find("traces");
    if (traces) {
        if (!traces->is_array()) throw BoardError(InputKind::kInvalid, "'traces' must be array");
        for (const auto& t : traces->as_array()) {
            TraceSeg seg;
            seg.net = resolve_net(*t.find("net"), board, "traces[]");
            seg.layer = t.has("layer") ? resolve_layer(*t.find("layer"), board, "traces[]") : 0;
            seg.a = {mm_to_nm(t.get_number("x1_mm", 0)), mm_to_nm(t.get_number("y1_mm", 0))};
            seg.b = {mm_to_nm(t.get_number("x2_mm", 0)), mm_to_nm(t.get_number("y2_mm", 0))};
            // Issue #17: committed traces are exact arbitrary-angle segments;
            // native JSON round-trips them verbatim (integer nm, no snapping
            // beyond the mm boundary quantization shared by all coordinates).
            double tw = t.get_number("width_mm", nm_to_mm(board.defaults.trace_width_nm));
            if (tw <= 0) throw BoardError(InputKind::kRule, "trace width must be positive");
            seg.width_nm = mm_to_nm(tw);
            board.traces.push_back(seg);
        }
    }

    const JsonValue* vias = root.find("vias");
    if (vias) {
        if (!vias->is_array()) throw BoardError(InputKind::kInvalid, "'vias' must be array");
        for (const auto& t : vias->as_array()) {
            Via via;
            via.net = resolve_net(*t.find("net"), board, "vias[]");
            via.pos = {mm_to_nm(t.get_number("x_mm", 0)), mm_to_nm(t.get_number("y_mm", 0))};
            via.top_layer = t.has("top_layer")
                                ? resolve_layer(*t.find("top_layer"), board, "vias[]")
                                : board.layers.front().id;
            via.bottom_layer = t.has("bottom_layer")
                                   ? resolve_layer(*t.find("bottom_layer"), board, "vias[]")
                                   : board.layers.back().id;
            double outer = t.get_number("outer_mm", nm_to_mm(board.defaults.via_outer_nm));
            double hole = t.get_number("hole_mm", nm_to_mm(board.defaults.via_hole_nm));
            if (outer <= 0 || hole <= 0 || hole >= outer)
                throw BoardError(InputKind::kRule, "via needs 0 < hole < outer diameter");
            via.outer_d_nm = mm_to_nm(outer);
            via.hole_d_nm = mm_to_nm(hole);
            via.via_class = t.get_string("class");
            board.vias.push_back(via);
        }
    }

    return result;
}

JsonValue board_to_json(const Board& board) {
    JsonValue root = JsonValue::object();
    JsonValue b = JsonValue::object();
    b["width_mm"] = nm_to_mm(board.width_nm);
    b["height_mm"] = nm_to_mm(board.height_nm);
    root["board"] = b;

    JsonValue layers = JsonValue::array();
    for (const auto& l : board.layers) {
        JsonValue o = JsonValue::object();
        o["id"] = static_cast<double>(l.id);
        o["name"] = l.name;
        // Issue #11: round-trip stackup metadata (needed for route --output).
        if (l.layer_type != "signal") o["layer_type"] = l.layer_type;
        if (l.copper_weight_oz > 0) o["copper_weight_oz"] = l.copper_weight_oz;
        if (l.has_internal_flag) o["is_internal"] = l.is_internal;
        if (l.has_dielectric_thickness)
            o["dielectric_thickness_mm"] = nm_to_mm(l.dielectric_thickness_nm);
        if (l.has_dielectric_er) o["dielectric_er"] = l.dielectric_er;
        if (l.has_ref_plane) o["ref_plane"] = static_cast<double>(l.ref_plane_layer);
        if (l.has_copper_thickness)
            o["copper_thickness_mm"] = nm_to_mm(l.copper_thickness_nm);
        if (!l.impedance_model.empty()) o["impedance_model"] = l.impedance_model;
        layers.as_array().push_back(o);
    }
    root["layers"] = layers;

    JsonValue nets = JsonValue::array();
    for (const auto& n : board.nets) {
        JsonValue o = JsonValue::object();
        o["id"] = static_cast<double>(n.id);
        o["name"] = n.name;
        if (n.has_current) o["current_a"] = n.current_a;
        if (n.has_voltage) o["voltage_v"] = n.voltage_v;
        if (!n.voltage_class.empty()) o["voltage_class"] = n.voltage_class;
        if (n.has_min_width) o["min_width_mm"] = nm_to_mm(n.min_width_nm);
        // Issue #15: round-trip length-tuning intent.
        if (n.has_target_length) {
            o["target_length_mm"] = nm_to_mm(n.target_length_nm);
            o["length_tol_mm"] = nm_to_mm(n.length_tol_nm);
        }
        // Issue #11: round-trip impedance intent.
        if (n.has_impedance) {
            o["target_impedance_ohms"] = n.target_impedance_ohms;
            o["impedance_tolerance_pct"] = n.impedance_tolerance_frac * 100.0;
            if (!n.impedance_layers.empty()) {
                JsonValue il = JsonValue::array();
                for (LayerId lid : n.impedance_layers)
                    il.as_array().push_back(JsonValue(static_cast<double>(lid)));
                o["impedance_layers"] = il;
            }
            if (n.has_impedance_ref_plane)
                o["impedance_ref_plane"] = static_cast<double>(n.impedance_ref_plane);
        }
        JsonValue terms = JsonValue::array();
        for (TermId tid : n.terminals) {
            const Terminal* t = board.find_terminal(tid);
            if (!t) continue;
            JsonValue to = JsonValue::object();
            to["id"] = static_cast<double>(t->id);
            to["x_mm"] = nm_to_mm(t->pos.x);
            to["y_mm"] = nm_to_mm(t->pos.y);
            to["layer"] = static_cast<double>(t->layer);
            // Issue #12: round-trip pad geometry + identity so
            // `router verify routed.json` sees the same copper the router
            // verified (non-default pads otherwise inflate to 0.5 mm and
            // report phantom pad-pad violations).
            to["pad_w_mm"] = nm_to_mm(t->pad_w_nm);
            to["pad_h_mm"] = nm_to_mm(t->pad_h_nm);
            if (!t->component.empty()) to["component"] = t->component;
            if (!t->pin.empty()) to["pin"] = t->pin;
            terms.as_array().push_back(to);
        }
        o["terminals"] = terms;
        nets.as_array().push_back(o);
    }
    root["nets"] = nets;

    // Issue #16: round-trip declared planes (needed for route --output).
    JsonValue planes = JsonValue::array();
    for (const auto& z : board.planes) {
        JsonValue o = JsonValue::object();
        o["id"] = static_cast<double>(z.id);
        o["net"] = static_cast<double>(z.net);
        o["layer"] = static_cast<double>(z.layer);
        o["island"] = static_cast<double>(z.island);
        o["routable"] = z.routable;
        JsonValue poly = JsonValue::array();
        for (const auto& p : z.poly) {
            JsonValue pt = JsonValue::array();
            pt.as_array().push_back(JsonValue(nm_to_mm(p.x)));
            pt.as_array().push_back(JsonValue(nm_to_mm(p.y)));
            poly.as_array().push_back(pt);
        }
        o["polygon_mm"] = poly;
        planes.as_array().push_back(o);
    }
    root["planes"] = planes;

    // Issue #12: round-trip pair declarations (needed for route --output).
    if (!board.diffpairs.empty()) {
        JsonValue pairs = JsonValue::array();
        for (const auto& pr : board.diffpairs) {
            JsonValue o = JsonValue::object();
            o["id"] = static_cast<double>(pr.id);
            o["name"] = pr.name;
            o["p"] = static_cast<double>(pr.net_p);
            o["n"] = static_cast<double>(pr.net_n);
            o["gap_mm"] = nm_to_mm(pr.gap_nm);
            o["gap_tol_mm"] = nm_to_mm(pr.gap_tol_nm);
            if (pr.has_width) o["width_mm"] = nm_to_mm(pr.width_nm);
            if (!pr.preferred_layers.empty()) {
                JsonValue ly = JsonValue::array();
                for (LayerId l : pr.preferred_layers)
                    ly.as_array().push_back(JsonValue(static_cast<double>(l)));
                o["layers"] = ly;
            }
            if (pr.has_impedance) o["target_impedance_ohms"] = pr.target_impedance_ohms;
            if (pr.has_max_skew) o["max_skew_mm"] = nm_to_mm(pr.max_skew_nm);
            o["via_policy"] = pr.via_policy;
            // Issue #15: round-trip symmetric tuning mode.
            if (pr.symmetric_tuning) o["symmetric_tuning"] = true;
            pairs.as_array().push_back(o);
        }
        root["diffpairs"] = pairs;
    }

    JsonValue traces = JsonValue::array();
    for (const auto& t : board.traces) {
        JsonValue o = JsonValue::object();
        o["net"] = static_cast<double>(t.net);
        o["layer"] = static_cast<double>(t.layer);
        o["x1_mm"] = nm_to_mm(t.a.x);
        o["y1_mm"] = nm_to_mm(t.a.y);
        o["x2_mm"] = nm_to_mm(t.b.x);
        o["y2_mm"] = nm_to_mm(t.b.y);
        o["width_mm"] = nm_to_mm(t.width_nm);
        traces.as_array().push_back(o);
    }
    root["traces"] = traces;

    JsonValue vias = JsonValue::array();
    for (const auto& v : board.vias) {
        JsonValue o = JsonValue::object();
        o["net"] = static_cast<double>(v.net);
        o["x_mm"] = nm_to_mm(v.pos.x);
        o["y_mm"] = nm_to_mm(v.pos.y);
        o["top_layer"] = static_cast<double>(v.top_layer);
        o["bottom_layer"] = static_cast<double>(v.bottom_layer);
        o["outer_mm"] = nm_to_mm(v.outer_d_nm);
        o["hole_mm"] = nm_to_mm(v.hole_d_nm);
        // Issue #16: parallel-via bundles (notably plane entries) verify as
        // a cluster only when every member names its class; dropping it
        // made saved plane routes fail standalone `router verify`.
        if (!v.via_class.empty()) o["class"] = v.via_class;
        vias.as_array().push_back(o);
    }
    root["vias"] = vias;
    return root;
}

std::vector<std::string> supported_formats() {
    return {"json", "kicad_pcb", "dsn", "gerber", "ipc-2581"};
}

ImportResult import_board_auto(const std::string& path) {
    std::string text = read_file(path);
    std::string head = text.substr(0, std::min<std::size_t>(text.size(), 4096));
    // Static importers to avoid repeating construction.
    static const JsonBoardImporter kJson;
    // KiCad / DSN / Gerber / IPC-2581 importers live in their own TUs;
    // declared here to keep board.cpp independent of their header weight.
    extern const BoardImporter& kicad_importer_singleton();
    extern const BoardImporter& dsn_importer_singleton();
    extern const BoardImporter& gerber_importer_singleton();
    extern const BoardImporter& ipc2581_importer_singleton();
    const BoardImporter* importers[] = {&kJson, &kicad_importer_singleton(),
                                        &dsn_importer_singleton(), &gerber_importer_singleton(),
                                        &ipc2581_importer_singleton()};
    for (const BoardImporter* imp : importers) {
        if (imp->claims(path, head)) return imp->import_file(path);
    }
    throw BoardError(InputKind::kInvalid,
                     "unsupported board format; supported: json, kicad_pcb (.kicad_pcb), "
                     "dsn (.dsn), gerber (.gbr/.gtl/.gbl/...), ipc-2581 (.xml).");
}

}  // namespace copperline
