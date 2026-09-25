// Copperline: Specctra DSN import + SES export (Prompt 5 adapters).
//
// Adapters live here, separate from the routing core: they translate between
// EDA file formats and the in-memory Board every engine/verifier caller
// shares. No routing decisions are made in this file.
#include "router/dsn.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
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
    if (unit == "inch") return 25.4;
    return -1.0;
}

bool ends_with(const std::string& s, const std::string& suf) {
    return s.size() >= suf.size() &&
           s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
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

struct DsnClassRule {
    std::string name;
    bool has_width = false;
    Coord width_nm = 0;
    bool has_clearance = false;
    Coord clearance_nm = 0;
    std::vector<std::string> use_via;
};

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

    double unit_mm = 1.0;  // mm per file unit
    if (const SexprNode* u = pcb->find_child("unit")) {
        std::string name = atom_str(u, 1, "mm");
        double s = unit_to_mm(name);
        if (s < 0) throw BoardError(InputKind::kInvalid, "DSN: unknown (unit " + name + ")");
        unit_mm = s;
    }
    auto u2mm = [&](double v) { return v * unit_mm; };

    ImportResult result;
    Board& board = result.board;
    board.source_format = "dsn";
    board.source_file = path;
    board.defaults = BoardDefaults();

    // ---- structure: layers / boundary / vias / rules / copper ----
    const SexprNode* structure = pcb->find_child("structure");
    if (!structure) throw BoardError(InputKind::kInvalid, "DSN: missing (structure ...)");
    std::map<std::string, LayerId> layer_by_name;
    {
        LayerId next_id = 0;
        for (const SexprNode* ln : structure->find_all("layer")) {
            Layer layer;
            layer.id = next_id++;
            layer.name = atom_str(ln, 1, "L" + std::to_string(layer.id));
            std::string type = "signal";
            if (const SexprNode* t = ln->find_child("type")) type = atom_str(t, 1, "signal");
            layer.layer_type = (type == "plane") ? "plane" : "signal";
            board.layers.push_back(layer);
            layer_by_name[layer.name] = layer.id;
        }
        if (board.layers.empty()) {
            // Permissive default: a 2-layer board so minimal DSNs still route.
            Layer a, b;
            a.id = 0;
            a.name = "Top";
            b.id = 1;
            b.name = "Bottom";
            board.layers = {a, b};
            layer_by_name = {{"Top", 0}, {"Bottom", 1}};
            result.warnings.push_back("dsn: no (structure (layer ...)) entries; assumed Top/Bottom");
        }
    }
    if (const SexprNode* bound = structure->find_child("boundary")) {
        if (const SexprNode* rc = bound->find_child("rect")) {
            double x1 = u2mm(atom_num(rc, 1)), y1 = u2mm(atom_num(rc, 2));
            double x2 = u2mm(atom_num(rc, 3)), y2 = u2mm(atom_num(rc, 4));
            board.width_nm = mm_to_nm(std::fabs(x2 - x1));
            board.height_nm = mm_to_nm(std::fabs(y2 - y1));
        } else if (const SexprNode* pg = bound->find_child("polygon")) {
            double x0 = 1e18, y0 = 1e18, x9 = -1e18, y9 = -1e18;
            for (std::size_t i = 1; i + 1 < pg->children.size(); i += 2) {
                double x = u2mm(atom_num(pg, i)), y = u2mm(atom_num(pg, i + 1));
                x0 = std::min(x0, x);
                y0 = std::min(y0, y);
                x9 = std::max(x9, x);
                y9 = std::max(y9, y);
            }
            if (x9 > x0 && y9 > y0) {
                board.width_nm = mm_to_nm(x9 - x0);
                board.height_nm = mm_to_nm(y9 - y0);
            }
        }
    }
    if (board.width_nm <= 0 || board.height_nm <= 0)
        throw BoardError(InputKind::kInvalid, "DSN: missing (structure (boundary ...)) extents");
    // Board origin: DSN coordinates may start anywhere; translate to (0,0).
    // Keep the offset to shift pads/pours into board space. The rect/polygon
    // minimum corner is the natural origin.
    double origin_x_mm = 0.0, origin_y_mm = 0.0;
    if (const SexprNode* bound = structure->find_child("boundary")) {
        if (const SexprNode* rc = bound->find_child("rect")) {
            origin_x_mm = std::min(u2mm(atom_num(rc, 1)), u2mm(atom_num(rc, 3)));
            origin_y_mm = std::min(u2mm(atom_num(rc, 2)), u2mm(atom_num(rc, 4)));
        } else if (const SexprNode* pg = bound->find_child("polygon")) {
            bool first = true;
            for (std::size_t i = 1; i + 1 < pg->children.size(); i += 2) {
                double x = u2mm(atom_num(pg, i)), y = u2mm(atom_num(pg, i + 1));
                if (first) {
                    origin_x_mm = x;
                    origin_y_mm = y;
                    first = false;
                } else {
                    origin_x_mm = std::min(origin_x_mm, x);
                    origin_y_mm = std::min(origin_y_mm, y);
                }
            }
        }
    }
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
                cr.has_width = true;
                cr.width_nm = mm_to_nm(u2mm(atom_num(w, 1, 0.2)));
            }
            if (const SexprNode* c = rule->find_child("clearance")) {
                cr.has_clearance = true;
                cr.clearance_nm = mm_to_nm(u2mm(atom_num(c, 1, 0.15)));
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

    // ---- library + placement: pad positions ----
    // pad_pos[ref][pad] = (x_mm, y_mm) in design coordinates.
    std::map<std::string, std::map<std::string, std::pair<double, double>>> images;
    if (const SexprNode* lib = pcb->find_child("library")) {
        for (const SexprNode* img : lib->find_all("image")) {
            std::string part = atom_str(img, 1, "");
            for (const SexprNode* pin : img->find_all("pin")) {
                std::string pad = atom_str(pin, 1, "");
                double x = u2mm(atom_num(pin, 2)), y = u2mm(atom_num(pin, 3));
                images[part][pad] = {x, y};
            }
        }
    }
    struct Placed {
        std::string part;
        double x = 0, y = 0;
    };
    std::map<std::string, Placed> placed;  // ref -> placement
    if (const SexprNode* pl = pcb->find_child("placement")) {
        for (const SexprNode* comp : pl->find_all("component")) {
            std::string ref = atom_str(comp, 1, "");
            if (const SexprNode* plc = comp->find_child("place")) {
                Placed p;
                p.part = atom_str(plc, 1, "");
                p.x = u2mm(atom_num(plc, 2));
                p.y = u2mm(atom_num(plc, 3));
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
            if (cr.has_clearance) {
                net.has_min_clearance = true;
                net.min_clearance_nm = cr.clearance_nm;
            }
            if (!cr.use_via.empty()) net.via_class = cr.use_via.front();
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
            double x_mm = pit->second.x + padit->second.first - origin_x_mm;
            double y_mm = pit->second.y + padit->second.second - origin_y_mm;
            Terminal t;
            t.id = next_term++;
            t.net = net.id;
            t.pos = {mm_to_nm(x_mm), mm_to_nm(y_mm)};
            t.layer = top_layer;
            t.pad_w_nm = mm_to_nm(0.3);
            t.pad_h_nm = mm_to_nm(0.3);
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
    auto shape_to_poly = [&](const SexprNode* shape, const std::string& what,
                             std::vector<Point>& out) -> bool {
        if (const SexprNode* pg = shape->find_child("polygon")) {
            if (pg->children.size() < 7) return false;
            if ((pg->children.size() - 1) / 2 > kMaxPourVertices) return false;
            for (std::size_t i = 1; i + 1 < pg->children.size(); i += 2) {
                double x = u2mm(atom_num(pg, i)) - origin_x_mm;
                double y = u2mm(atom_num(pg, i + 1)) - origin_y_mm;
                out.push_back({mm_to_nm(x), mm_to_nm(y)});
            }
            return out.size() >= 3;
        }
        if (const SexprNode* rc = shape->find_child("rect")) {
            double x1 = u2mm(atom_num(rc, 1)) - origin_x_mm;
            double y1 = u2mm(atom_num(rc, 2)) - origin_y_mm;
            double x2 = u2mm(atom_num(rc, 3)) - origin_x_mm;
            double y2 = u2mm(atom_num(rc, 4)) - origin_y_mm;
            out = {{mm_to_nm(std::min(x1, x2)), mm_to_nm(std::min(y1, y2))},
                   {mm_to_nm(std::max(x1, x2)), mm_to_nm(std::min(y1, y2))},
                   {mm_to_nm(std::max(x1, x2)), mm_to_nm(std::max(y1, y2))},
                   {mm_to_nm(std::min(x1, x2)), mm_to_nm(std::max(y1, y2))}};
            return true;
        }
        if (const SexprNode* ci = shape->find_child("circle")) {
            // Octagon approximation of a round pour/cutout (documented).
            double cx = u2mm(atom_num(ci, 1)) - origin_x_mm;
            double cy = u2mm(atom_num(ci, 2)) - origin_y_mm;
            double r = u2mm(atom_num(ci, 3)) / 2.0;
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
            std::vector<Point> pts;
            for (std::size_t i = 2; i + 1 < pa->children.size(); i += 2) {
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
    auto resolve_pour_layer = [&](const SexprNode* node, bool& present) -> LayerId {
        present = false;
        const SexprNode* ln = node->find_child("layer");
        if (!ln) return top_layer;
        present = true;
        std::string lname = atom_str(ln, 1, "");
        auto lit = layer_by_name.find(lname);
        if (lit == layer_by_name.end()) return -1;
        return lit->second;
    };
    auto parse_one_pour = [&](const SexprNode* node, const std::string& what) {
        const NetInfo* ni = resolve_pour_net(node);
        bool layer_present = false;
        LayerId layer = resolve_pour_layer(node, layer_present);
        std::vector<Point> poly;
        bool shape_ok = shape_to_poly(node, what, poly);
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
    {
        bool warned_poly = false;
        const char* kinds[] = {"wire_keepout", "via_keepout", "place_keepout",
                               "bend_keepout", "elongate_keepout", "keepout"};
        for (const char* kind : kinds) {
            for (const SexprNode* ko : structure->find_all(kind)) {
                std::vector<Point> poly;
                if (!shape_to_poly(ko, kind, poly) || poly.empty()) {
                    result.warnings.push_back(std::string("dsn: (") + kind +
                                              " ...) ignored with warning: no usable shape");
                    continue;
                }
                const SexprNode* ln = ko->find_child("layer");
                LayerId layer = kAllLayers;
                if (ln) {
                    auto lit = layer_by_name.find(atom_str(ln, 1, ""));
                    if (lit == layer_by_name.end()) {
                        result.warnings.push_back(std::string("dsn: (") + kind +
                                                  " ...) ignored with warning: unknown layer");
                        continue;
                    }
                    layer = lit->second;
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
                k.reason = std::string("dsn-") + kind;
                board.keepouts.push_back(k);
            }
        }
    }
    if (pcb->find_child("wiring"))
        result.warnings.push_back(
            "dsn: (wiring ...) pre-routes ignored on import; the router re-routes from pads");

    // Default via geometry note for agents.
    if (!via_geom.empty()) {
        std::string names;
        for (const auto& kv : via_geom) {
            if (!names.empty()) names += ",";
            names += kv.first;
        }
        result.warnings.push_back("dsn: via geometries available: " + names);
    }
    // Micro-via / blind-buried spans are out of scope: record honestly.
    if (text.find("microvia") != std::string::npos || text.find("blind") != std::string::npos)
        result.warnings.push_back("dsn: micro/blind/buried vias are not modeled and were ignored");
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
    const SexprNode* session = root->find_child("session");
    if (!session)
        throw BoardError(InputKind::kInvalid, "SES: missing (session ...) root");
    const SexprNode* route = session->find_child("route");
    if (!route) throw BoardError(InputKind::kInvalid, "SES: missing (route ...)");
    const SexprNode* network = route->find_child("network");
    if (!network) throw BoardError(InputKind::kInvalid, "SES: missing (route (network ...))");
    std::map<std::string, LayerId> layer_by_name;
    for (const auto& l : board.layers) layer_by_name[l.name] = l.id;
    std::map<std::string, std::pair<Coord, Coord>> padstacks;
    if (const SexprNode* lib = route->find_child("library")) {
        for (const SexprNode* ps : lib->find_all("padstack")) {
            std::string name = atom_str(ps, 1, "STD");
            padstacks[name] = {mm_to_nm(atom_num(ps, 2, 0.6)), mm_to_nm(atom_num(ps, 3, 0.3))};
        }
    }
    for (const SexprNode* nn : network->find_all("net")) {
        std::string name = atom_str(nn, 1, "");
        const NetInfo* ni = board.find_net_by_name(name);
        if (!ni)
            throw BoardError(InputKind::kInvalid,
                             "SES: net '" + name + "' not in board " + path);
        for (const SexprNode* w : nn->find_all("wire")) {
            for (const SexprNode* p : w->find_all("path")) {
                std::string lname = atom_str(p, 1, "");
                auto lit = layer_by_name.find(lname);
                if (lit == layer_by_name.end())
                    throw BoardError(InputKind::kInvalid,
                                     "SES: unknown layer '" + lname + "'");
                // (path layer x1 y1 x2 y2 ... width): even count incl. width.
                std::vector<double> nums;
                for (std::size_t i = 2; i < p->children.size(); ++i) {
                    if (!p->children[i]->is_atom)
                        throw BoardError(InputKind::kInvalid, "SES: bad path coordinates");
                    try {
                        nums.push_back(std::stod(p->children[i]->atom));
                    } catch (...) {
                        throw BoardError(InputKind::kInvalid, "SES: bad path number");
                    }
                }
                if (nums.size() < 5 || (nums.size() % 2) == 0)
                    throw BoardError(InputKind::kInvalid, "SES: path needs x1 y1 x2 y2 width");
                double w_mm = nums.back();
                nums.pop_back();
                if (w_mm <= 0)
                    throw BoardError(InputKind::kRule, "SES: path width must be positive");
                // Coordinate list is a chained polyline p0..pk; emit the
                // consecutive pairs (p0,p1), (p1,p2), ... as segments.
                std::size_t npts = nums.size() / 2;
                if (npts < 2)
                    throw BoardError(InputKind::kInvalid, "SES: path needs >= 2 points");
                for (std::size_t pi = 0; pi + 1 < npts; ++pi) {
                    TraceSeg s;
                    s.net = ni->id;
                    s.layer = lit->second;
                    s.a = {mm_to_nm(nums[2 * pi]), mm_to_nm(nums[2 * pi + 1])};
                    s.b = {mm_to_nm(nums[2 * pi + 2]), mm_to_nm(nums[2 * pi + 3])};
                    s.width_nm = mm_to_nm(w_mm);
                    board.traces.push_back(s);
                }
            }
            for (const SexprNode* v : w->find_all("via")) {
                std::string vs = atom_str(v, 1, "STD");
                Via via;
                via.net = ni->id;
                via.pos = {mm_to_nm(atom_num(v, 2)), mm_to_nm(atom_num(v, 3))};
                via.top_layer = board.layers.front().id;
                via.bottom_layer = board.layers.back().id;
                auto pit = padstacks.find(vs);
                if (pit != padstacks.end()) {
                    via.outer_d_nm = pit->second.first;
                    via.hole_d_nm = pit->second.second;
                } else {
                    via.outer_d_nm = board.defaults.via_outer_nm;
                    via.hole_d_nm = board.defaults.via_hole_nm;
                    warnings.push_back("ses: unknown padstack '" + vs + "'; used defaults");
                }
                via.via_class = vs;
                board.vias.push_back(via);
            }
        }
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
