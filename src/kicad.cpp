// Copperline: KiCad .kicad_pcb (s-expression) board ingest.
//
// Phase-1 subset: board outline from Edge.Cuts drawings, copper layers,
// nets + net classes (width/clearance/via rules), footprints + pads
// (rect/circle/oval/trapezoid/roundrect approximated by bounding boxes),
// tracks, vias, and keepout rule areas. Everything approximated or skipped
// is reported in ImportResult::warnings; copper zones are NOT yet modeled
// (Prompt 5) and produce a warning instead of silent acceptance.
#include "router/board.h"

#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>

#include "router/sexpr.h"

namespace copperline {

namespace {

bool ends_with_ci(const std::string& s, const std::string& suffix) {
    if (suffix.size() > s.size()) return false;
    for (std::size_t i = 0; i < suffix.size(); ++i) {
        if (std::tolower(s[s.size() - suffix.size() + i]) != std::tolower(suffix[i]))
            return false;
    }
    return true;
}

double num(const std::string& tok, const std::string& ctx) {
    try {
        std::size_t n = 0;
        double v = std::stod(tok, &n);
        if (n != tok.size()) throw std::runtime_error("trailing");
        return v;
    } catch (...) {
        throw BoardError(InputKind::kInvalid, "bad number '" + tok + "' in " + ctx);
    }
}

// Atom children of a list node, skipping the head (children[0]).
std::vector<std::string> tail_atoms(const SexprNode* n) {
    std::vector<std::string> out;
    for (std::size_t i = 1; i < n->children.size(); ++i) {
        if (n->children[i]->is_atom) out.push_back(n->children[i]->atom);
    }
    return out;
}

struct KicadNetClass {
    std::string name;
    double clearance_mm = -1;
    double trace_width_mm = -1;
    double via_dia_mm = -1;
    double via_drill_mm = -1;
    std::vector<std::string> nets;
};

struct PendingPad {
    std::string number;
    std::string type;   // smd | thru_hole | ...
    std::string shape;  // rect | circle | oval | trapezoid | roundrect | custom
    double dx = 0, dy = 0, rot = 0;
    double sx = 0, sy = 0;  // size
    bool has_net = false;
    int net = -1;
    std::vector<std::string> layers;
    bool has_size = false;
};

class KicadImport {
  public:
    ImportResult run(const std::string& text, const std::string& path) {
        path_ = path;
        std::unique_ptr<SexprNode> root;
        try {
            root = parse_sexpr(text);
        } catch (const std::exception& e) {
            throw BoardError(InputKind::kInvalid, std::string("bad s-expr: ") + e.what());
        }
        const SexprNode* pcb = nullptr;
        for (const auto& c : root->children) {
            if (!c->is_atom && c->head() == "kicad_pcb") {
                pcb = c.get();
                break;
            }
        }
        if (!pcb) throw BoardError(InputKind::kInvalid, "not a kicad_pcb file");
        result_.board.source_format = "kicad_pcb";
        result_.board.source_file = path;

        parse_layers(pcb);
        parse_nets(pcb);
        parse_net_classes(pcb);
        parse_footprints(pcb);
        parse_tracks_vias(pcb);
        parse_drawings_outline(pcb);
        parse_zones(pcb);
        finalize_bounds();
        return std::move(result_);
    }

  private:
    std::string path_;
    ImportResult result_;
    std::map<int, LayerId> layer_num_to_id_;     // KiCad layer number -> internal id
    std::map<std::string, LayerId> layer_name_to_id_;  // "F.Cu" -> internal id
    std::map<int, std::string> net_names_;       // kicad net number -> name
    std::map<std::string, NetId> net_name_to_id_;
    std::map<int, NetId> net_num_to_id_;
    std::vector<KicadNetClass> net_classes_;
    // Edge.Cuts extents for the board outline.
    bool have_edge_ = false;
    double edge_x1_ = 0, edge_y1_ = 0, edge_x2_ = 0, edge_y2_ = 0;
    int next_net_id_ = 0;
    int next_term_id_ = 0;
    bool warned_zone_ = false;
    bool warned_tht_ = false;

    void warn(const std::string& w) { result_.warnings.push_back(w); }

    void parse_layers(const SexprNode* pcb) {
        const SexprNode* layers = pcb->find_child("layers");
        Board& b = result_.board;
        if (!layers) {
            // Fall back to a default 2-layer stackup.
            b.layers.push_back({0, "F.Cu"});
            b.layers.push_back({1, "B.Cu"});
            layer_name_to_id_["F.Cu"] = 0;
            layer_name_to_id_["B.Cu"] = 1;
            warn("no (layers) section; assumed F.Cu/B.Cu");
            return;
        }
        for (const auto& c : layers->children) {
            if (c->is_atom) continue;
            auto atoms = tail_atoms(c.get());
            if (c->children.empty() || !c->children[0]->is_atom) continue;
            int layer_num = static_cast<int>(num(c->children[0]->atom, "layers"));
            if (atoms.empty()) continue;
            std::string name = atoms[0];
            std::string type = atoms.size() > 1 ? atoms[1] : "";
            bool copper = (type == "signal") || ends_with_ci(name, ".Cu");
            if (!copper) continue;
            LayerId id = static_cast<LayerId>(b.layers.size());
            b.layers.push_back({id, name});
            layer_num_to_id_[layer_num] = id;
            layer_name_to_id_[name] = id;
        }
        if (b.layers.empty()) throw BoardError(InputKind::kInvalid, "no copper layers found");
    }

    LayerId copper_layer(const std::string& name, bool silent = false) {
        auto it = layer_name_to_id_.find(name);
        if (it == layer_name_to_id_.end()) {
            if (!silent) warn("skipping copper on unknown layer '" + name + "'");
            return -1;
        }
        return it->second;
    }

    void parse_nets(const SexprNode* pcb) {
        Board& b = result_.board;
        for (const SexprNode* n : pcb->find_all("net")) {
            auto atoms = tail_atoms(n);
            if (atoms.size() < 1) continue;
            int number = static_cast<int>(num(atoms[0], "net"));
            std::string name = atoms.size() > 1 ? atoms[1] : "";
            net_names_[number] = name;
            if (number == 0) continue;  // unconnected
            NetInfo net;
            net.id = next_net_id_++;
            net.name = name.empty() ? ("N" + std::to_string(number)) : name;
            b.nets.push_back(net);
            net_num_to_id_[number] = net.id;
            net_name_to_id_[net.name] = net.id;
        }
    }

    void parse_net_classes(const SexprNode* pcb) {
        Board& b = result_.board;
        for (const SexprNode* nc : pcb->find_all("net_class")) {
            KicadNetClass kc;
            auto atoms = tail_atoms(nc);
            if (!atoms.empty()) kc.name = atoms[0];
            // Assigned net names are bare string children after the head.
            for (const auto& c : nc->children) {
                if (c->is_atom && c.get() != &*nc->children.front()) {
                    // tail_atoms already collected; match by position is complex,
                    // so re-derive: bare atoms that are not part of sublists.
                    kc.nets.push_back(c->atom);
                }
            }
            // Remove the class name itself (first bare atom).
            if (!kc.nets.empty() && kc.nets.front() == kc.name) kc.nets.erase(kc.nets.begin());
            if (const SexprNode* c = nc->find_child("clearance"))
                kc.clearance_mm = num(tail_atoms(c).at(0), "net_class clearance");
            if (const SexprNode* c = nc->find_child("trace_width"))
                kc.trace_width_mm = num(tail_atoms(c).at(0), "net_class trace_width");
            if (const SexprNode* c = nc->find_child("via_dia"))
                kc.via_dia_mm = num(tail_atoms(c).at(0), "net_class via_dia");
            if (const SexprNode* c = nc->find_child("via_drill"))
                kc.via_drill_mm = num(tail_atoms(c).at(0), "net_class via_drill");
            net_classes_.push_back(kc);
        }
        // Apply: Default class -> board defaults; named classes -> member nets.
        for (const auto& kc : net_classes_) {
            if (kc.name == "Default") {
                if (kc.trace_width_mm > 0) b.defaults.trace_width_nm = mm_to_nm(kc.trace_width_mm);
                if (kc.clearance_mm > 0) b.defaults.clearance_nm = mm_to_nm(kc.clearance_mm);
                if (kc.via_dia_mm > 0) b.defaults.via_outer_nm = mm_to_nm(kc.via_dia_mm);
                if (kc.via_drill_mm > 0) b.defaults.via_hole_nm = mm_to_nm(kc.via_drill_mm);
                continue;
            }
            for (const auto& name : kc.nets) {
                auto it = net_name_to_id_.find(name);
                if (it == net_name_to_id_.end()) continue;
                NetInfo* net = b.find_net(it->second);
                if (!net) continue;
                if (kc.trace_width_mm > 0) {
                    net->has_min_width = true;
                    net->min_width_nm = mm_to_nm(kc.trace_width_mm);
                }
                if (kc.clearance_mm > 0) {
                    net->has_min_clearance = true;
                    net->min_clearance_nm = mm_to_nm(kc.clearance_mm);
                }
            }
        }
    }

    void parse_footprints(const SexprNode* pcb) {
        for (const SexprNode* fp : pcb->find_all("footprint")) {
            parse_footprint(fp);
        }
    }

    void parse_footprint(const SexprNode* fp) {
        Board& b = result_.board;
        auto atoms = tail_atoms(fp);
        std::string fp_name = atoms.empty() ? "?" : atoms[0];
        double fx = 0, fy = 0, frot = 0;
        if (const SexprNode* at = fp->find_child("at")) {
            auto a = tail_atoms(at);
            if (a.size() > 0) fx = num(a[0], "footprint at");
            if (a.size() > 1) fy = num(a[1], "footprint at");
            if (a.size() > 2) frot = num(a[2], "footprint at");
        }
        std::string ref = fp_name;
        for (const SexprNode* t : fp->find_all("fp_text")) {
            auto a = tail_atoms(t);
            if (a.size() >= 2 && a[0] == "reference") ref = a[1];
        }
        bool bottom = false;
        if (const SexprNode* layer = fp->find_child("layer")) {
            auto a = tail_atoms(layer);
            if (!a.empty() && a[0] == "B.Cu") bottom = true;
        }
        for (const SexprNode* pad : fp->find_all("pad")) {
            parse_pad(pad, ref, fx, fy, frot, bottom);
        }
    }

    void parse_pad(const SexprNode* pad, const std::string& ref, double fx, double fy,
                   double frot, bool bottom) {
        Board& b = result_.board;
        auto a = tail_atoms(pad);
        if (a.size() < 3) {
            warn("skipping malformed pad in " + ref);
            return;
        }
        PendingPad p;
        p.number = a[0];
        p.type = a[1];
        p.shape = a[2];
        if (const SexprNode* at = pad->find_child("at")) {
            auto v = tail_atoms(at);
            if (v.size() > 0) p.dx = num(v[0], "pad at");
            if (v.size() > 1) p.dy = num(v[1], "pad at");
            if (v.size() > 2) p.rot = num(v[2], "pad at");
        }
        if (const SexprNode* size = pad->find_child("size")) {
            auto v = tail_atoms(size);
            if (!v.empty()) {
                p.sx = num(v[0], "pad size");
                p.sy = v.size() > 1 ? num(v[1], "pad size") : p.sx;
                p.has_size = true;
            }
        }
        if (const SexprNode* net = pad->find_child("net")) {
            auto v = tail_atoms(net);
            if (!v.empty()) {
                p.net = static_cast<int>(num(v[0], "pad net"));
                p.has_net = true;
            }
        }
        if (const SexprNode* layers = pad->find_child("layers")) {
            p.layers = tail_atoms(layers);
            // Drop wildcard-only entries.
            std::vector<std::string> keep;
            for (auto& l : p.layers) {
                if (l == "*.Cu" || l == "*.Paste" || l == "*.Mask" || l == "*.SilkS") continue;
                keep.push_back(l);
            }
            if (!keep.empty()) p.layers = keep;
        }
        if (!p.has_net || p.net == 0) return;  // unconnected / mechanical pad
        auto nit = net_num_to_id_.find(p.net);
        if (nit == net_num_to_id_.end()) {
            warn("pad " + ref + ":" + p.number + " on unknown net; skipped");
            return;
        }
        if (!p.has_size) {
            warn("pad " + ref + ":" + p.number + " has no size; using 0.5x0.5mm");
            p.sx = p.sy = 0.5;
        }
        // Rotated bounding box of the pad shape.
        double total_rot = (frot + p.rot) * std::numbers::pi / 180.0;
        double cr = std::fabs(std::cos(total_rot)), sr = std::fabs(std::sin(total_rot));
        double bw = p.sx * cr + p.sy * sr;
        double bh = p.sx * sr + p.sy * cr;
        // Rotate the offset by the footprint rotation.
        double fr = frot * std::numbers::pi / 180.0;
        double ox = p.dx * std::cos(fr) - p.dy * std::sin(fr);
        double oy = p.dx * std::sin(fr) + p.dy * std::cos(fr);
        double px = fx + ox, py = fy + oy;

        std::vector<LayerId> copper;
        for (const auto& l : p.layers) {
            if (!ends_with_ci(l, ".Cu")) continue;
            LayerId id = copper_layer(l, true);
            if (id >= 0) copper.push_back(id);
        }
        if (p.type == "thru_hole") {
            if (!warned_tht_) {
                warn("thru-hole pads fanned to every copper layer (one terminal each)");
                warned_tht_ = true;
            }
            copper.clear();
            for (const auto& l : b.layers) copper.push_back(l.id);
        }
        if (copper.empty()) {
            // Default: footprint side.
            copper.push_back(copper_layer(bottom ? "B.Cu" : "F.Cu", true));
            if (copper.back() < 0) copper.back() = b.layers.front().id;
        }
        for (LayerId lid : copper) {
            Terminal t;
            t.id = next_term_id_++;
            t.net = nit->second;
            t.pos = {mm_to_nm(px), mm_to_nm(py)};
            t.layer = lid;
            t.pad_w_nm = mm_to_nm(bw);
            t.pad_h_nm = mm_to_nm(bh);
            t.component = ref;
            t.pin = p.number;
            b.terminals.push_back(t);
            if (NetInfo* n = b.find_net(t.net)) n->terminals.push_back(t.id);
        }
    }

    void parse_tracks_vias(const SexprNode* pcb) {
        Board& b = result_.board;
        for (const SexprNode* s : pcb->find_all("segment")) {
            const SexprNode* start = s->find_child("start");
            const SexprNode* end = s->find_child("end");
            const SexprNode* width = s->find_child("width");
            const SexprNode* layer = s->find_child("layer");
            const SexprNode* net = s->find_child("net");
            if (!start || !end || !width || !layer || !net) {
                warn("skipping malformed segment");
                continue;
            }
            auto sa = tail_atoms(start), ea = tail_atoms(end);
            auto la = tail_atoms(layer), na = tail_atoms(net);
            if (sa.size() < 2 || ea.size() < 2 || la.empty() || na.empty()) {
                warn("skipping malformed segment");
                continue;
            }
            LayerId lid = copper_layer(la[0]);
            if (lid < 0) continue;
            int netnum = static_cast<int>(num(na[0], "segment net"));
            auto nit = net_num_to_id_.find(netnum);
            if (nit == net_num_to_id_.end() || netnum == 0) {
                warn("segment on unknown net; skipped");
                continue;
            }
            TraceSeg t;
            t.net = nit->second;
            t.layer = lid;
            t.a = {mm_to_nm(num(sa[0], "segment start")), mm_to_nm(num(sa[1], "segment start"))};
            t.b = {mm_to_nm(num(ea[0], "segment end")), mm_to_nm(num(ea[1], "segment end"))};
            if (!t.segment().axis_aligned()) {
                warn("non-Manhattan segment imported as-is (arbitrary-angle copper)");
            }
            t.width_nm = mm_to_nm(num(tail_atoms(width).at(0), "segment width"));
            b.traces.push_back(t);
        }
        for (const SexprNode* s : pcb->find_all("via")) {
            const SexprNode* at = s->find_child("at");
            const SexprNode* size = s->find_child("size");
            const SexprNode* layers = s->find_child("layers");
            const SexprNode* net = s->find_child("net");
            if (!at || !size || !layers || !net) {
                warn("skipping malformed via");
                continue;
            }
            auto aa = tail_atoms(at), la = tail_atoms(layers), na = tail_atoms(net);
            if (aa.size() < 2 || la.size() < 2 || na.empty()) {
                warn("skipping malformed via");
                continue;
            }
            LayerId top = copper_layer(la[0]), bot = copper_layer(la[1]);
            if (top < 0 || bot < 0) continue;
            int netnum = static_cast<int>(num(na[0], "via net"));
            auto nit = net_num_to_id_.find(netnum);
            if (nit == net_num_to_id_.end() || netnum == 0) {
                warn("via on unknown net; skipped");
                continue;
            }
            Via v;
            v.net = nit->second;
            v.pos = {mm_to_nm(num(aa[0], "via at")), mm_to_nm(num(aa[1], "via at"))};
            v.top_layer = top;
            v.bottom_layer = bot;
            v.outer_d_nm = mm_to_nm(num(tail_atoms(size).at(0), "via size"));
            if (const SexprNode* drill = s->find_child("drill"))
                v.hole_d_nm = mm_to_nm(num(tail_atoms(drill).at(0), "via drill"));
            else
                v.hole_d_nm = v.outer_d_nm / 2;
            b.vias.push_back(v);
        }
    }

    void note_edge_point(double x, double y) {
        if (!have_edge_) {
            edge_x1_ = edge_x2_ = x;
            edge_y1_ = edge_y2_ = y;
            have_edge_ = true;
            return;
        }
        edge_x1_ = std::min(edge_x1_, x);
        edge_y1_ = std::min(edge_y1_, y);
        edge_x2_ = std::max(edge_x2_, x);
        edge_y2_ = std::max(edge_y2_, y);
    }

    static bool on_edge_cuts(const SexprNode* n) {
        const SexprNode* l = n->find_child("layer");
        if (!l) return false;
        auto a = tail_atoms(l);
        return !a.empty() && a[0] == "Edge.Cuts";
    }

    void parse_drawings_outline(const SexprNode* pcb) {
        for (const SexprNode* g : pcb->find_all("gr_line")) {
            if (!on_edge_cuts(g)) continue;
            const SexprNode* s = g->find_child("start");
            const SexprNode* e = g->find_child("end");
            if (!s || !e) continue;
            auto sa = tail_atoms(s), ea = tail_atoms(e);
            if (sa.size() < 2 || ea.size() < 2) continue;
            note_edge_point(num(sa[0], "gr_line"), num(sa[1], "gr_line"));
            note_edge_point(num(ea[0], "gr_line"), num(ea[1], "gr_line"));
        }
        for (const SexprNode* g : pcb->find_all("gr_rect")) {
            if (!on_edge_cuts(g)) continue;
            const SexprNode* s = g->find_child("start");
            const SexprNode* e = g->find_child("end");
            if (!s || !e) continue;
            auto sa = tail_atoms(s), ea = tail_atoms(e);
            if (sa.size() < 2 || ea.size() < 2) continue;
            note_edge_point(num(sa[0], "gr_rect"), num(sa[1], "gr_rect"));
            note_edge_point(num(ea[0], "gr_rect"), num(ea[1], "gr_rect"));
        }
    }

    void parse_zones(const SexprNode* pcb) {
        for (const SexprNode* z : pcb->find_all("zone")) {
            bool is_keepout = z->find_child("keepout") != nullptr;
            if (!is_keepout) {
                if (!warned_zone_) {
                    warn("copper zones are not modeled in phase 1 and were ignored "
                         "(declare plane-aware pours via native JSON planes[], issue #16; "
                         "full KiCad zone import lands in Prompt 5)");
                    warned_zone_ = true;
                }
                continue;
            }
            // Keepout rule area: bbox of its polygon points, per listed layer.
            std::vector<std::string> layers;
            if (const SexprNode* l = z->find_child("layers")) layers = tail_atoms(l);
            const SexprNode* poly = z->find_child("polygon");
            if (!poly) {
                if (const SexprNode* fill = z->find_child("fill"))
                    if (const SexprNode* island = fill->find_child("polygon")) poly = island;
            }
            const SexprNode* pts = poly ? poly->find_child("pts") : nullptr;
            if (!pts) {
                warn("keepout zone without polygon; skipped");
                continue;
            }
            bool first = true;
            double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
            for (const SexprNode* xy : pts->find_all("xy")) {
                auto a = tail_atoms(xy);
                if (a.size() < 2) continue;
                double x = num(a[0], "zone pts"), y = num(a[1], "zone pts");
                if (first) {
                    x1 = x2 = x;
                    y1 = y2 = y;
                    first = false;
                } else {
                    x1 = std::min(x1, x);
                    y1 = std::min(y1, y);
                    x2 = std::max(x2, x);
                    y2 = std::max(y2, y);
                }
            }
            if (first) continue;
            warn("keepout rule area approximated by bounding box");
            for (const auto& l : layers) {
                LayerId lid = copper_layer(l, true);
                if (lid < 0) continue;
                Keepout k;
                k.rect = {mm_to_nm(x1), mm_to_nm(y1), mm_to_nm(x2), mm_to_nm(y2)};
                k.layer = lid;
                k.reason = "kicad-keepout";
                result_.board.keepouts.push_back(k);
            }
        }
    }

    void finalize_bounds() {
        Board& b = result_.board;
        double x0, y0, x1, y1;
        if (have_edge_) {
            x0 = edge_x1_;
            y0 = edge_y1_;
            x1 = edge_x2_;
            y1 = edge_y2_;
        } else {
            // Fall back to copper bbox + 5mm margin.
            bool first = true;
            x0 = y0 = x1 = y1 = 0;
            auto note = [&](double x, double y) {
                if (first) {
                    x0 = x1 = x;
                    y0 = y1 = y;
                    first = false;
                } else {
                    x0 = std::min(x0, x);
                    y0 = std::min(y0, y);
                    x1 = std::max(x1, x);
                    y1 = std::max(y1, y);
                }
            };
            for (const auto& t : b.terminals) note(nm_to_mm(t.pos.x), nm_to_mm(t.pos.y));
            for (const auto& t : b.traces) {
                note(nm_to_mm(t.a.x), nm_to_mm(t.a.y));
                note(nm_to_mm(t.b.x), nm_to_mm(t.b.y));
            }
            if (first) throw BoardError(InputKind::kInvalid, "empty board: no geometry found");
            x0 -= 5;
            y0 -= 5;
            x1 += 5;
            y1 += 5;
            warn("no Edge.Cuts outline; bounds derived from copper bbox + 5mm");
        }
        double ox = x0, oy = y0;
        // Translate so the board origin is (0,0).
        for (auto& t : b.terminals) {
            t.pos.x -= mm_to_nm(ox);
            t.pos.y -= mm_to_nm(oy);
        }
        for (auto& t : b.traces) {
            t.a.x -= mm_to_nm(ox);
            t.a.y -= mm_to_nm(oy);
            t.b.x -= mm_to_nm(ox);
            t.b.y -= mm_to_nm(oy);
        }
        for (auto& v : b.vias) {
            v.pos.x -= mm_to_nm(ox);
            v.pos.y -= mm_to_nm(oy);
        }
        for (auto& k : b.keepouts) {
            k.rect = {k.rect.x1 - mm_to_nm(ox), k.rect.y1 - mm_to_nm(oy),
                      k.rect.x2 - mm_to_nm(ox), k.rect.y2 - mm_to_nm(oy)};
        }
        b.width_nm = mm_to_nm(x1 - x0);
        b.height_nm = mm_to_nm(y1 - y0);
        if (b.width_nm <= 0 || b.height_nm <= 0)
            throw BoardError(InputKind::kInvalid, "degenerate board outline");
    }
};

class KicadPcbImporterImpl : public BoardImporter {
  public:
    std::string format_name() const override { return "kicad_pcb"; }
    bool claims(const std::string& path, const std::string& head_bytes) const override {
        if (ends_with_ci(path, ".kicad_pcb")) return true;
        return head_bytes.find("(kicad_pcb") != std::string::npos;
    }
    ImportResult import_file(const std::string& path) const override {
        std::ifstream f(path, std::ios::binary);
        if (!f) throw BoardError(InputKind::kInvalid, "cannot open file: " + path);
        std::ostringstream ss;
        ss << f.rdbuf();
        return import_text(ss.str(), path);
    }
    ImportResult import_text(const std::string& text, const std::string& path) const {
        return KicadImport().run(text, path);
    }
};

}  // namespace

const BoardImporter& kicad_importer_singleton() {
    static const KicadPcbImporterImpl kInstance;
    return kInstance;
}

bool KicadPcbImporter::claims(const std::string& path, const std::string& head_bytes) const {
    return kicad_importer_singleton().claims(path, head_bytes);
}

ImportResult KicadPcbImporter::import_file(const std::string& path) const {
    return kicad_importer_singleton().import_file(path);
}

ImportResult KicadPcbImporter::import_text(const std::string& text, const std::string& path) const {
    return static_cast<const KicadPcbImporterImpl&>(kicad_importer_singleton())
        .import_text(text, path);
}

}  // namespace copperline
