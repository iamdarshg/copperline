// Copperline: KiCad .kicad_pcb exporter (round-trip writer).
//
// Implements board_to_kicad_pcb (declared in router/board.h). The writer
// is the exact inverse of KicadPcbImporter for everything the importer
// models, so `router route board.kicad_pcb --output routed.kicad_pcb`
// followed by `router verify routed.kicad_pcb` passes, and
// import -> export -> re-import preserves nets/pads/tracks/vias/zones.
#include "router/board.h"

#include <algorithm>
#include <cstdio>
#include <map>
#include <sstream>

namespace copperline {
namespace {

std::string mm_str(Coord nm) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6f", nm_to_mm(nm));
    return buf;
}

std::string esc(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else out += c;
    }
    return out;
}

std::string sanitize_class(const std::string& s) {
    std::string out;
    for (char c : s) {
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
            c == '_')
            out += c;
        else out += '_';
    }
    if (out.empty()) out = "NET";
    return out;
}

struct PadKey {
    std::string comp;
    std::string pin;
    Point pos{};
    Coord w = 0, h = 0;
    bool operator<(const PadKey& o) const {
        if (comp != o.comp) return comp < o.comp;
        if (pin != o.pin) return pin < o.pin;
        if (pos < o.pos || o.pos < pos) return pos < o.pos;
        if (w != o.w) return w < o.w;
        return h < o.h;
    }
};

}  // namespace

std::string board_to_kicad_pcb(const Board& board) {
    std::ostringstream os;
    std::map<LayerId, std::string> layer_name;
    for (const auto& l : board.layers) layer_name[l.id] = l.name;

    auto layer_of = [&](LayerId id) -> std::string {
        auto it = layer_name.find(id);
        return it == layer_name.end() ? ("L" + std::to_string(id)) : it->second;
    };

    // KiCad net numbers: 0 = unconnected, board nets in id order from 1.
    std::vector<const NetInfo*> nets;
    for (const auto& n : board.nets) nets.push_back(&n);
    std::sort(nets.begin(), nets.end(),
              [](const NetInfo* a, const NetInfo* b) { return a->id < b->id; });
    std::map<NetId, int> net_num;
    for (std::size_t i = 0; i < nets.size(); ++i) net_num[nets[i]->id] = static_cast<int>(i) + 1;

    os << "(kicad_pcb (version 20230101) (generator copperline) (general)\n";
    os << "  (paper \"A4\")\n";
    os << "  (layers\n";
    for (std::size_t i = 0; i < board.layers.size(); ++i) {
        os << "    (" << i << " \"" << esc(board.layers[i].name) << "\" signal)\n";
    }
    os << "  )\n";
    os << "  (setup)\n";
    os << "  (net 0 \"\")\n";
    for (const NetInfo* n : nets) os << "  (net " << net_num[n->id] << " \"" << esc(n->name) << "\")\n";

    // Net classes: Default from board defaults; per-net classes for nets
    // carrying explicit width/clearance floors (re-import restores them).
    os << "  (net_class \"Default\" \"\" (clearance " << mm_str(board.defaults.clearance_nm)
       << ") (trace_width " << mm_str(board.defaults.trace_width_nm) << ") (via_dia "
       << mm_str(board.defaults.via_outer_nm) << ") (via_drill "
       << mm_str(board.defaults.via_hole_nm) << ")";
    for (const NetInfo* n : nets) os << " \"" << esc(n->name) << "\"";
    os << ")\n";
    for (const NetInfo* n : nets) {
        if (!n->has_min_width && !n->has_min_clearance) continue;
        Coord w = n->has_min_width ? n->min_width_nm : board.defaults.trace_width_nm;
        Coord c = n->has_min_clearance ? n->min_clearance_nm : board.defaults.clearance_nm;
        os << "  (net_class \"Copperline_" << sanitize_class(n->name) << "\" \"" << esc(n->name)
           << "\" (clearance " << mm_str(c) << ") (trace_width " << mm_str(w) << ") (via_dia "
           << mm_str(board.defaults.via_outer_nm) << ") (via_drill "
           << mm_str(board.defaults.via_hole_nm) << "))\n";
    }

    // ---- footprints: group terminals by component ----
    // Collapse (component, pin, pos, size) fans across layers into one
    // thru-hole pad; single-layer pads stay smd.
    std::map<std::string, std::vector<const Terminal*>> fps;
    std::map<std::string, std::vector<const Terminal*>> virtual_fp;  // per net
    for (const auto& t : board.terminals) {
        if (t.component.empty())
            virtual_fp[board.find_net(t.net) ? board.find_net(t.net)->name : ("N" +
                                                                              std::to_string(
                                                                                  t.net))]
                .push_back(&t);
        else
            fps[t.component].push_back(&t);
    }
    std::size_t tstamp = 1;
    auto emit_footprint = [&](const std::string& ref, std::vector<const Terminal*> terms) {
        std::sort(terms.begin(), terms.end(), [](const Terminal* a, const Terminal* b) {
            if (a->pin != b->pin) return a->pin < b->pin;
            if (a->layer != b->layer) return a->layer < b->layer;
            return a->id < b->id;
        });
        // Merge same (pin, pos, size) across layers -> thru-hole.
        struct Merged {
            std::string pin;
            NetId net = -1;
            Point pos{};
            Coord w = 0, h = 0;
            std::vector<LayerId> layers;
        };
        std::vector<Merged> merged;
        for (const Terminal* t : terms) {
            bool joined = false;
            for (auto& m : merged) {
                if (m.pin == t->pin && m.pos == t->pos && m.w == t->pad_w_nm &&
                    m.h == t->pad_h_nm) {
                    if (std::find(m.layers.begin(), m.layers.end(), t->layer) ==
                        m.layers.end())
                        m.layers.push_back(t->layer);
                    joined = true;
                    break;
                }
            }
            if (!joined) {
                Merged m;
                m.pin = t->pin.empty() ? ("P" + std::to_string(t->id)) : t->pin;
                m.net = t->net;
                m.pos = t->pos;
                m.w = t->pad_w_nm;
                m.h = t->pad_h_nm;
                m.layers.push_back(t->layer);
                merged.push_back(m);
            }
        }
        std::sort(merged.begin(), merged.end(),
                  [](const Merged& a, const Merged& b) { return a.pin < b.pin; });
        const Point org = merged.empty() ? Point{0, 0} : merged.front().pos;
        std::string fplayer = "F.Cu";
        if (!merged.empty()) fplayer = layer_of(merged.front().layers.front());
        os << "  (footprint \"Copperline:PAD\" (layer \"" << esc(fplayer) << "\") (tstamp cl-"
           << (tstamp++) << ") (at " << mm_str(org.x) << " " << mm_str(org.y) << ")\n";
        os << "    (fp_text reference \"" << esc(ref) << "\" (at 0 -1.5) (layer \"F.SilkS\"))\n";
        os << "    (fp_text value \"copperline\" (at 0 1.5) (layer \"F.Fab\"))\n";
        for (const auto& m : merged) {
            auto nit = net_num.find(m.net);
            int nnum = nit == net_num.end() ? 0 : nit->second;
            const NetInfo* ni = board.find_net(m.net);
            std::string nname = ni ? ni->name : "";
            double px = nm_to_mm(m.pos.x - org.x), py = nm_to_mm(m.pos.y - org.y);
            char xb[64], yb[64];
            std::snprintf(xb, sizeof(xb), "%.6f", px);
            std::snprintf(yb, sizeof(yb), "%.6f", py);
            std::sort(const_cast<std::vector<LayerId>&>(m.layers).begin(),
                      const_cast<std::vector<LayerId>&>(m.layers).end());
            if (m.layers.size() >= 2) {
                os << "    (pad \"" << esc(m.pin) << "\" thru_hole rect (at " << xb << " " << yb
                   << ") (size " << mm_str(m.w) << " " << mm_str(m.h) << ") (layers";
                for (LayerId l : m.layers) os << " \"" << esc(layer_of(l)) << "\"";
                os << ") (net " << nnum << " \"" << esc(nname) << "\") (tstamp cl-"
                   << (tstamp++) << "))\n";
            } else {
                std::string ln = layer_of(m.layers.front());
                os << "    (pad \"" << esc(m.pin) << "\" smd rect (at " << xb << " " << yb
                   << ") (size " << mm_str(m.w) << " " << mm_str(m.h) << ") (layers \"" << esc(ln)
                   << "\"";
                if (ln == "F.Cu") os << " \"F.Paste\" \"F.Mask\"";
                if (ln == "B.Cu") os << " \"B.Paste\" \"B.Mask\"";
                os << ") (net " << nnum << " \"" << esc(nname) << "\") (tstamp cl-"
                   << (tstamp++) << "))\n";
            }
        }
        os << "  )\n";
    };
    for (auto& kv : fps) emit_footprint(kv.first, kv.second);
    for (auto& kv : virtual_fp) {
        // One virtual footprint per net for component-less terminals.
        emit_footprint("COPPERLINE_" + sanitize_class(kv.first), kv.second);
    }

    // ---- outline ----
    os << "  (gr_rect (start 0 0) (end " << mm_str(board.width_nm) << " "
       << mm_str(board.height_nm) << ") (layer \"Edge.Cuts\") (width 0.05) (fill none) (tstamp cl-"
       << (tstamp++) << "))\n";

    // ---- committed copper: deterministic net/id order ----
    std::vector<const TraceSeg*> segs;
    for (const auto& t : board.traces) segs.push_back(&t);
    std::sort(segs.begin(), segs.end(), [](const TraceSeg* a, const TraceSeg* b) {
        if (a->net != b->net) return a->net < b->net;
        if (a->layer != b->layer) return a->layer < b->layer;
        if (a->a < b->a || b->a < a->a) return a->a < b->a;
        if (a->b < b->b || b->b < a->b) return a->b < b->b;
        return a->width_nm < b->width_nm;
    });
    for (const TraceSeg* s : segs) {
        auto nit = net_num.find(s->net);
        int nnum = nit == net_num.end() ? 0 : nit->second;
        os << "  (segment (start " << mm_str(s->a.x) << " " << mm_str(s->a.y) << ") (end "
           << mm_str(s->b.x) << " " << mm_str(s->b.y) << ") (width " << mm_str(s->width_nm)
           << ") (layer \"" << esc(layer_of(s->layer)) << "\") (net " << nnum << ") (tstamp cl-"
           << (tstamp++) << "))\n";
    }
    std::vector<const Via*> vias;
    for (const auto& v : board.vias) vias.push_back(&v);
    std::sort(vias.begin(), vias.end(), [](const Via* a, const Via* b) {
        if (a->net != b->net) return a->net < b->net;
        if (a->pos < b->pos || b->pos < a->pos) return a->pos < b->pos;
        return a->outer_d_nm < b->outer_d_nm;
    });
    for (const Via* v : vias) {
        auto nit = net_num.find(v->net);
        int nnum = nit == net_num.end() ? 0 : nit->second;
        os << "  (via (at " << mm_str(v->pos.x) << " " << mm_str(v->pos.y) << ") (size "
           << mm_str(v->outer_d_nm) << ") (drill " << mm_str(v->hole_d_nm) << ") (layers \""
           << esc(layer_of(v->top_layer)) << "\" \"" << esc(layer_of(v->bottom_layer))
           << "\") (net " << nnum << ") (tstamp cl-" << (tstamp++) << "))\n";
    }

    // ---- copper zones from PlaneZone ----
    std::vector<const PlaneZone*> planes;
    for (const auto& z : board.planes) planes.push_back(&z);
    std::sort(planes.begin(), planes.end(),
              [](const PlaneZone* a, const PlaneZone* b) { return a->id < b->id; });
    for (const PlaneZone* z : planes) {
        auto nit = net_num.find(z->net);
        int nnum = nit == net_num.end() ? 0 : nit->second;
        const NetInfo* ni = board.find_net(z->net);
        std::string nname = ni ? ni->name : "";
        if (z->poly.size() < 3 || nnum == 0) continue;  // never emit net-less copper
        os << "  (zone (net " << nnum << ") (net_name \"" << esc(nname) << "\") (layers \""
           << esc(layer_of(z->layer)) << "\") (tstamp cl-" << (tstamp++) << ") (hatch edge 0.5)\n";
        os << "    (connect_pads (clearance 0.2))\n    (min_thickness 0.254) (filled_areas_thickness no)\n";
        os << "    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n";
        os << "    (polygon (pts";
        for (const auto& p : z->poly) os << " (xy " << mm_str(p.x) << " " << mm_str(p.y) << ")";
        os << "))\n";
        os << "    (filled_polygon (layer \"" << esc(layer_of(z->layer)) << "\") (pts";
        for (const auto& p : z->poly) os << " (xy " << mm_str(p.x) << " " << mm_str(p.y) << ")";
        os << "))\n  )\n";
    }

    // ---- keepout rule areas ----
    for (const auto& k : board.keepouts) {
        std::vector<LayerId> klayers;
        if (k.layer == kAllLayers) {
            for (const auto& l : board.layers) klayers.push_back(l.id);
        } else {
            klayers.push_back(k.layer);
        }
        os << "  (zone (net 0) (net_name \"\") (layers";
        for (LayerId l : klayers) os << " \"" << esc(layer_of(l)) << "\"";
        os << ") (tstamp cl-" << (tstamp++) << ") (hatch edge 0.5)\n";
        os << "    (connect_pads (clearance 0.2))\n    (min_thickness 0.254) (filled_areas_thickness no)\n";
        os << "    (keepout (tracks allowed) (vias allowed) (pads allowed) (copperpour allowed) "
              "(footprints allowed))\n";
        os << "    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n";
        os << "    (polygon (pts (xy " << mm_str(k.rect.x1) << " " << mm_str(k.rect.y1)
           << ") (xy " << mm_str(k.rect.x2) << " " << mm_str(k.rect.y1) << ") (xy "
           << mm_str(k.rect.x2) << " " << mm_str(k.rect.y2) << ") (xy " << mm_str(k.rect.x1)
           << " " << mm_str(k.rect.y2) << ")))\n  )\n";
    }

    os << ")\n";
    return os.str();
}

}  // namespace copperline
