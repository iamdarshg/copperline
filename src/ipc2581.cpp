// Copperline: IPC-2581C import (minimal but real; see ipc2581.h).
//
// Streaming scanner: the file is read in 64KB chunks and `<...>` tags are
// handled incrementally (a small pending tail covers tags split across
// chunks), so large transfers never build a DOM. Buffered state is
// bounded (pending <= 256KB, polygon vertices <= 32k each, primitives <=
// 2M, unknown-tag summary <= 12 names).
#include "router/ipc2581.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>

namespace copperline {
namespace {

constexpr std::size_t kStreamChunk = 65536;
constexpr std::size_t kMaxPending = 262144;
constexpr std::size_t kMaxIpcVertices = 32768;
constexpr std::size_t kMaxIpcPrimitives = 2000000;

std::string lower_str(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(c));
    return s;
}

bool ends_with_ci(const std::string& s, const std::string& suffix) {
    if (suffix.size() > s.size()) return false;
    for (std::size_t i = 0; i < suffix.size(); ++i) {
        if (std::tolower(s[s.size() - suffix.size() + i]) != std::tolower(suffix[i]))
            return false;
    }
    return true;
}

std::string trim(const std::string& s) {
    std::size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t\r\n") + 1 - a);
}

std::string decode_entities(const std::string& s) {
    std::string out;
    for (std::size_t i = 0; i < s.size();) {
        if (s[i] == '&') {
            std::size_t e = s.find(';', i);
            std::string ent = e == std::string::npos ? "" : s.substr(i + 1, e - i - 1);
            if (ent == "amp") {
                out += '&';
                i = e + 1;
            } else if (ent == "lt") {
                out += '<';
                i = e + 1;
            } else if (ent == "gt") {
                out += '>';
                i = e + 1;
            } else if (ent == "quot") {
                out += '"';
                i = e + 1;
            } else if (ent == "apos") {
                out += '\'';
                i = e + 1;
            } else {
                out += s[i++];
            }
        } else {
            out += s[i++];
        }
    }
    return out;
}

bool parse_num(const std::string& s, double& out) {
    try {
        std::size_t n = 0;
        double v = std::stod(trim(s), &n);
        if (n != trim(s).size()) return false;
        out = v;
        return true;
    } catch (...) {
        return false;
    }
}

// unit scale to mm.
double unit_to_mm(const std::string& u) {
    std::string l = lower_str(trim(u));
    if (l == "mm" || l == "millimeter" || l == "millimetre") return 1.0;
    if (l == "inch" || l == "in") return 25.4;
    if (l == "mil" || l == "thou") return 0.0254;
    if (l == "um" || l == "micron" || l == "micrometer" || l == "micrometre") return 0.001;
    if (l == "nm" || l == "nanometer" || l == "nanometre") return 1e-6;
    if (l == "cm" || l == "centimeter" || l == "centimetre") return 10.0;
    return -1.0;
}

struct Tag {
    std::string name;  // lowercased
    std::map<std::string, std::string> attrs;  // lowercased keys
    bool is_end = false;
    bool self_close = false;
};

// Minimal tag parser: <name a="v" b='v' c=v/> (quotes preferred, bare
// tolerated). Returns false when the tag is not well-formed.
bool parse_tag(const std::string& raw, Tag& tag) {
    std::string s = trim(raw);
    if (s.size() < 3 || s.front() != '<' || s.back() != '>') return false;
    s = s.substr(1, s.size() - 2);
    s = trim(s);
    if (s.empty()) return false;
    if (s[0] == '/') {
        tag.is_end = true;
        s = trim(s.substr(1));
        std::size_t e = 0;
        while (e < s.size() && s[e] != ' ' && s[e] != '\t' && s[e] != '\r' && s[e] != '\n')
            ++e;
        tag.name = lower_str(s.substr(0, e));
        return !tag.name.empty();
    }
    if (!s.empty() && s.back() == '/') {
        tag.self_close = true;
        s = trim(s.substr(0, s.size() - 1));
    }
    std::size_t i = 0;
    while (i < s.size() && s[i] != ' ' && s[i] != '\t' && s[i] != '\r' && s[i] != '\n')
        ++i;
    tag.name = lower_str(s.substr(0, i));
    if (tag.name.empty()) return false;
    if (!tag.name.empty() && (tag.name[0] == '?' || tag.name[0] == '!')) return false;
    while (i < s.size()) {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n'))
            ++i;
        if (i >= s.size()) break;
        std::size_t ks = i;
        while (i < s.size() && s[i] != '=' && s[i] != ' ' && s[i] != '\t') ++i;
        std::string key = lower_str(s.substr(ks, i - ks));
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
        if (i >= s.size() || s[i] != '=') {
            if (!key.empty()) tag.attrs[key] = "true";
            continue;
        }
        ++i;
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
        std::string val;
        if (i < s.size() && (s[i] == '"' || s[i] == '\'')) {
            char q = s[i++];
            std::size_t vs = i;
            while (i < s.size() && s[i] != q) ++i;
            val = s.substr(vs, i - vs);
            if (i < s.size()) ++i;
        } else {
            std::size_t vs = i;
            while (i < s.size() && s[i] != ' ' && s[i] != '\t' && s[i] != '\r' &&
                   s[i] != '\n')
                ++i;
            val = s.substr(vs, i - vs);
        }
        if (!key.empty()) tag.attrs[key] = decode_entities(val);
    }
    return true;
}

}  // namespace

class Ipc2581Import {
  public:
    ImportResult run_stream(std::istream& in, const std::string& path) {
        path_ = path;
        std::string pending;
        char buf[kStreamChunk];
        bool eof = false;
        while (!eof) {
            in.read(buf, sizeof(buf));
            std::streamsize got = in.gcount();
            eof = got <= 0 || static_cast<std::size_t>(got) < sizeof(buf);
            if (got > 0) pending.append(buf, static_cast<std::size_t>(got));
            // Extract complete tags; skip comments/PIs/doctype wholesale.
            while (true) {
                std::size_t lt = pending.find('<');
                if (lt == std::string::npos) {
                    pending.clear();
                    break;
                }
                if (lt > 0) pending.erase(0, lt);
                if (pending.compare(0, 4, "<!--") == 0) {
                    std::size_t e = pending.find("-->");
                    if (e == std::string::npos) break;  // need more input
                    pending.erase(0, e + 3);
                    continue;
                }
                if (pending.compare(0, 2, "<?") == 0) {
                    std::size_t e = pending.find("?>");
                    if (e == std::string::npos) break;
                    pending.erase(0, e + 2);
                    continue;
                }
                if (pending.compare(0, 9, "<!DOCTYPE") == 0 ||
                    pending.compare(0, 8, "<!ENTITY") == 0) {
                    std::size_t e = pending.find('>');
                    if (e == std::string::npos) break;
                    pending.erase(0, e + 1);
                    continue;
                }
                if (pending.compare(0, 9, "<![CDATA[") == 0) {
                    std::size_t e = pending.find("]]>");
                    if (e == std::string::npos) break;
                    pending.erase(0, e + 3);
                    continue;
                }
                std::size_t gt = pending.find('>');
                if (gt == std::string::npos) break;  // need more input
                std::string tag = pending.substr(0, gt + 1);
                pending.erase(0, gt + 1);
                handle_tag(tag);
            }
            if (pending.size() > kMaxPending)
                throw BoardError(InputKind::kInvalid,
                                 "ipc-2581: single tag over 256KB; file malformed");
        }
        return build();
    }

    ImportResult run_text(const std::string& text, const std::string& path) {
        std::istringstream in(text);
        return run_stream(in, path);
    }

  private:
    std::string path_;
    ImportResult result_;
    double unit_mm_ = 1.0;
    bool saw_unit_ = false;
    // Datum.
    bool has_datum_ = false;
    double datum_w_ = 0, datum_h_ = 0;  // file units
    // Layers in file order.
    struct IpcLayer {
        std::string name;
        bool plane = false;
    };
    std::vector<IpcLayer> layers_;
    // Nets in file order.
    std::vector<std::string> nets_;
    // Net classes: name -> (width, clearance, members).
    struct IpcClass {
        double width = -1, clearance = -1;  // file units
        std::vector<std::string> members;
    };
    std::map<std::string, IpcClass> classes_;  // lowercased name -> class
    std::string open_class_;                   // currently open class element
    // Components.
    struct IpcPad {
        std::string comp;
        std::string net;
        double x = 0, y = 0, w = 0.5, h = 0.5;  // file units
        std::string layer;
    };
    std::vector<IpcPad> pads_;
    std::string open_comp_;
    // Traces / vias.
    struct IpcTrace {
        std::string net, layer;
        double x1 = 0, y1 = 0, x2 = 0, y2 = 0, w = 0.2;
    };
    std::vector<IpcTrace> traces_;
    struct IpcVia {
        std::string net;
        double x = 0, y = 0, outer = 0.6, hole = 0.3;
        std::string top, bottom;
    };
    std::vector<IpcVia> vias_;
    // Profile polygon (file units).
    std::vector<std::pair<double, double>> profile_;
    bool in_profile_ = false;
    bool profile_truncated_ = false;
    // Diagnostics.
    std::vector<std::string> unknown_tags_;
    bool warned_step_ = false, warned_layer_default_ = false;
    bool saw_step_ = false, in_second_step_ = false;
    std::size_t prim_count_ = 0;

    void warn(const std::string& w) { result_.warnings.push_back(w); }

    void bump_prims(const std::string& what) {
        if (++prim_count_ > kMaxIpcPrimitives)
            throw BoardError(InputKind::kInvalid,
                             "ipc-2581: primitive cap (2000000) exceeded in " + what);
    }

    static std::string attr(const Tag& t, std::initializer_list<const char*> keys) {
        for (const char* k : keys) {
            auto it = t.attrs.find(k);
            if (it != t.attrs.end()) return it->second;
        }
        return "";
    }

    void note_unknown(const std::string& name) {
        if (unknown_tags_.size() >= 12) return;
        if (std::find(unknown_tags_.begin(), unknown_tags_.end(), name) ==
            unknown_tags_.end())
            unknown_tags_.push_back(name);
    }

    void handle_tag(const std::string& raw) {
        Tag t;
        if (!parse_tag(raw, t)) return;
        if (t.name.empty() || t.name[0] == '?' || t.name[0] == '!') return;
        // Unit scope: root containers may carry it.
        if (!saw_unit_) {
            std::string u = attr(t, {"unit", "units", "uom"});
            if (!u.empty()) {
                double s = unit_to_mm(u);
                if (s > 0) {
                    unit_mm_ = s;
                    saw_unit_ = true;
                } else {
                    warn("ipc-2581: unknown unit '" + u + "'; assumed mm");
                    saw_unit_ = true;
                }
            }
        }
        const std::string& n = t.name;
        if (n == "step") {
            if (!t.is_end) {
                if (saw_step_) {
                    in_second_step_ = true;
                    if (!warned_step_) {
                        warned_step_ = true;
                        warn("ipc-2581: multi-step panels are not modeled; only the "
                             "first <Step> is imported");
                    }
                }
                saw_step_ = true;
            } else if (in_second_step_) {
                in_second_step_ = false;
            }
            return;
        }
        if (in_second_step_) return;  // skip panel steps beyond the first
        if (n == "datum") {
            if (!t.is_end) {
                double w = 0, h = 0;
                bool okw = parse_num(attr(t, {"width", "size_x", "sizex", "boardwidth",
                                              "board_width"}),
                                     w);
                bool okh = parse_num(attr(t, {"height", "size_y", "sizey", "boardheight",
                                              "board_height"}),
                                     h);
                if (okw && okh && w > 0 && h > 0) {
                    has_datum_ = true;
                    datum_w_ = w;
                    datum_h_ = h;
                } else {
                    warn("ipc-2581: <Datum> without positive width/height ignored");
                }
            }
            return;
        }
        if (n == "layer" || n == "signallayer" || n == "copperlayer" ||
            n == "conductorlayer" || n == "powerlayer" || n == "groundlayer") {
            if (!t.is_end) {
                std::string name = attr(t, {"name", "layer", "id"});
                if (name.empty()) {
                    warn("ipc-2581: <Layer> without a name skipped with warning");
                    return;
                }
                std::string ty = lower_str(attr(t, {"type", "kind", "function"}));
                bool plane = ty == "plane" || ty == "power" || ty == "ground" ||
                             n == "powerlayer" || n == "groundlayer";
                layers_.push_back({name, plane});
                bump_prims("layers");
            }
            return;
        }
        if (n == "net" || n == "logicalnet" || n == "physicalnet" || n == "signal") {
            if (in_profile_) return;
            if (!t.is_end) {
                // Inside a NetClass element this declares membership.
                if (!open_class_.empty() &&
                    (n == "net" || n == "member" || n == "netref")) {
                    std::string m = attr(t, {"name", "net", "ref"});
                    if (!m.empty()) classes_[open_class_].members.push_back(m);
                    return;
                }
                if (!open_class_.empty()) return;
                std::string name = attr(t, {"name", "net", "id"});
                if (!name.empty() && std::find(nets_.begin(), nets_.end(), name) ==
                                        nets_.end()) {
                    nets_.push_back(name);
                    bump_prims("nets");
                }
            }
            return;
        }
        if (n == "member" || n == "netref") {
            if (!t.is_end && !open_class_.empty()) {
                std::string m = attr(t, {"name", "net", "ref"});
                if (!m.empty()) classes_[open_class_].members.push_back(m);
            }
            return;
        }
        if (n == "netclass" || n == "class" || n == "net_class" || n == "ecennetclass") {
            if (!t.is_end) {
                std::string name = lower_str(attr(t, {"name", "id"}));
                if (name.empty()) {
                    warn("ipc-2581: <NetClass> without a name skipped with warning");
                    return;
                }
                IpcClass c;
                double w = 0, cl = 0;
                if (parse_num(attr(t, {"width", "tracewidth", "trace_width", "minwidth",
                                       "min_width"}),
                              w) &&
                    w > 0)
                    c.width = w;
                if (parse_num(attr(t, {"clearance", "spacing", "minclearance",
                                       "min_clearance", "gap"}),
                              cl) &&
                    cl >= 0)
                    c.clearance = cl;
                classes_[name] = c;
                open_class_ = t.self_close ? "" : name;
            } else {
                open_class_.clear();
            }
            return;
        }
        if (n == "component" || n == "part" || n == "footprint" || n == "placement" ||
            n == "placed" || n == "reference") {
            if (!t.is_end) {
                open_comp_ = attr(t, {"refdes", "ref", "designator", "name", "id"});
                if (open_comp_.empty()) open_comp_ = "U?";
                if (t.self_close) open_comp_.clear();
            } else {
                open_comp_.clear();
            }
            return;
        }
        if (n == "pad" || n == "pin" || n == "terminal" || n == "smdpad" ||
            n == "throughhole") {
            if (t.is_end) return;
            IpcPad p;
            p.comp = attr(t, {"refdes", "ref", "component", "designator"});
            if (p.comp.empty()) p.comp = open_comp_.empty() ? "U?" : open_comp_;
            p.net = attr(t, {"net", "netname", "net_name", "signal"});
            double v = 0;
            if (parse_num(attr(t, {"x", "posx", "pos_x", "cx", "centerx", "xpos"}), v))
                p.x = v;
            if (parse_num(attr(t, {"y", "posy", "pos_y", "cy", "centery", "ypos"}), v))
                p.y = v;
            bool has_w = parse_num(attr(t, {"width", "sizex", "size_x", "sx", "dx"}), v);
            if (has_w) p.w = v;
            bool has_h =
                parse_num(attr(t, {"height", "sizey", "size_y", "sy", "dy"}), v);
            if (has_h) p.h = v;
            double dia = 0;
            if (parse_num(attr(t, {"dia", "diameter", "size", "drill"}), v)) dia = v;
            if (dia > 0 && !has_w && !has_h) p.w = p.h = dia;
            else if (dia > 0 && !has_w) p.w = dia;
            else if (dia > 0 && !has_h) p.h = dia;
            p.layer = attr(t, {"layer", "side", "surface"});
            pads_.push_back(p);
            bump_prims("pads");
            return;
        }
        if (n == "trace" || n == "wire" || n == "track" || n == "conductor" ||
            n == "route" || n == "segment") {
            if (t.is_end) return;
            IpcTrace tr;
            tr.net = attr(t, {"net", "netname", "net_name", "signal"});
            tr.layer = attr(t, {"layer", "side"});
            double v = 0;
            if (parse_num(attr(t, {"x1", "startx", "start_x", "xstart"}), v)) tr.x1 = v;
            if (parse_num(attr(t, {"y1", "starty", "start_y", "ystart"}), v)) tr.y1 = v;
            if (parse_num(attr(t, {"x2", "endx", "end_x", "xend"}), v)) tr.x2 = v;
            if (parse_num(attr(t, {"y2", "endy", "end_y", "yend"}), v)) tr.y2 = v;
            if (parse_num(attr(t, {"width", "w", "size", "thickness"}), v)) tr.w = v;
            traces_.push_back(tr);
            bump_prims("traces");
            return;
        }
        if (n == "via" || n == "platedhole" || n == "hole" || n == "microvia") {
            if (t.is_end) return;
            if (n == "microvia" || lower_str(attr(t, {"type"})) == "micro") {
                warn("ipc-2581: microvias are not modeled and were ignored");
                return;
            }
            IpcVia vv;
            vv.net = attr(t, {"net", "netname", "net_name", "signal"});
            double v = 0;
            if (parse_num(attr(t, {"x", "posx", "cx"}), v)) vv.x = v;
            if (parse_num(attr(t, {"y", "posy", "cy"}), v)) vv.y = v;
            if (parse_num(attr(t, {"outer", "diameter", "size", "dia"}), v)) vv.outer = v;
            if (parse_num(attr(t, {"hole", "drill", "inner"}), v)) vv.hole = v;
            vv.top = attr(t, {"top", "toplayer", "top_layer"});
            vv.bottom = attr(t, {"bottom", "bottomlayer", "bottom_layer"});
            vias_.push_back(vv);
            bump_prims("vias");
            return;
        }
        if (n == "profile" || n == "outline" || n == "boardoutline" || n == "perimeter" ||
            n == "boardedge") {
            if (!t.is_end) {
                in_profile_ = true;
                // Inline rect form: <Profile x1 y1 x2 y2/>.
                double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
                bool ok = parse_num(attr(t, {"x1"}), x1) && parse_num(attr(t, {"y1"}), y1) &&
                          parse_num(attr(t, {"x2"}), x2) && parse_num(attr(t, {"y2"}), y2);
                if (ok) {
                    profile_.push_back({x1, y1});
                    profile_.push_back({x2, y1});
                    profile_.push_back({x2, y2});
                    profile_.push_back({x1, y2});
                }
                if (t.self_close) in_profile_ = false;
            } else {
                in_profile_ = false;
            }
            return;
        }
        if (n == "polygon" || n == "contour" || n == "path") {
            if (!t.is_end && in_profile_) {
                std::string pts = attr(t, {"points", "pts", "vertices", "xy"});
                if (!pts.empty()) {
                    for (auto& pr : split_pair_list2(pts)) {
                        if (profile_.size() > kMaxIpcVertices) {
                            if (!profile_truncated_) {
                                profile_truncated_ = true;
                                warn("ipc-2581: profile over vertex cap (32768); "
                                     "truncated with warning");
                            }
                            break;
                        }
                        profile_.push_back(pr);
                    }
                }
            }
            return;
        }
        if (n == "rect" || n == "rectangle" || n == "box") {
            if (!t.is_end && in_profile_) {
                double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
                if (parse_num(attr(t, {"x1"}), x1) && parse_num(attr(t, {"y1"}), y1) &&
                    parse_num(attr(t, {"x2"}), x2) && parse_num(attr(t, {"y2"}), y2)) {
                    profile_.push_back({x1, y1});
                    profile_.push_back({x2, y1});
                    profile_.push_back({x2, y2});
                    profile_.push_back({x1, y2});
                } else {
                    warn("ipc-2581: <Rect> without x1/y1/x2/y2 ignored");
                }
            }
            return;
        }
        if (n == "point" || n == "vertex" || n == "xy" || n == "corner" || n == "pt") {
            if (!t.is_end && in_profile_) {
                double x = 0, y = 0;
                if (parse_num(attr(t, {"x"}), x) && parse_num(attr(t, {"y"}), y)) {
                    if (profile_.size() > kMaxIpcVertices) {
                        if (!profile_truncated_) {
                            profile_truncated_ = true;
                            warn("ipc-2581: profile over vertex cap (32768); "
                                 "truncated with warning");
                        }
                    } else {
                        profile_.push_back({x, y});
                    }
                }
            }
            return;
        }
        // Structural wrappers with nothing to extract: descend silently.
        if (n == "ipc2581" || n == "ipc-2581" || n == "ecads" || n == "ecad" ||
            n == "content" || n == "header" || n == "cadheader" || n == "caddata" ||
            n == "stackup" || n == "steps" || n == "network" || n == "components" ||
            n == "traces" || n == "vias" || n == "pads" || n == "layers" || n == "nets" ||
            n == "classes" || n == "netclasses" || n == "board" || n == "design" ||
            n == "edd" || n == "package" || n == "padstack" || n == "features" ||
            n == "layerfeature" || n == "conductorfeatures") {
            return;
        }
        if (!t.is_end) note_unknown(n);
    }

    static std::vector<double> split_pair_list(const std::string& s) {
        std::vector<double> out;
        std::string cur;
        for (char c : s) {
            if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') cur += c;
            else if (!cur.empty()) {
                try {
                    out.push_back(std::stod(cur));
                } catch (...) {
                }
                cur.clear();
            }
        }
        if (!cur.empty()) {
            try {
                out.push_back(std::stod(cur));
            } catch (...) {
            }
        }
        return out;
    }

    static std::vector<std::pair<double, double>> split_pair_list2(const std::string& s) {
        std::vector<std::pair<double, double>> out;
        auto nums = split_pair_list(s);
        for (std::size_t i = 0; i + 1 < nums.size(); i += 2) out.push_back({nums[i], nums[i+1]});
        return out;
    }

    ImportResult build() {
        ImportResult& out = result_;
        Board& b = out.board;
        b.source_format = "ipc-2581";
        b.source_file = path_;
        if (!saw_unit_) warn("ipc-2581: no unit attribute; assumed mm");
        if (!unknown_tags_.empty()) {
            std::string names;
            for (const auto& u : unknown_tags_) {
                if (!names.empty()) names += ", ";
                names += "<" + u + ">";
            }
            warn("ipc-2581: unmodeled elements ignored: " + names +
                 " (flat-subset import; see README limits)");
        }
        auto mm = [&](double v) { return v * unit_mm_; };

        // Layers (default 2-layer stackup when absent).
        std::map<std::string, LayerId, std::less<>> layer_id;
        if (layers_.empty()) {
            b.layers.push_back({0, "Top", false, 1.0});
            b.layers.push_back({1, "Bottom", true, 1.0});
            warn("ipc-2581: no <Layer> entries; assumed Top/Bottom");
        } else {
            LayerId next = 0;
            for (const auto& l : layers_) {
                if (layer_id.count(l.name)) continue;
                Layer layer;
                layer.id = next++;
                layer.name = l.name;
                layer.layer_type = l.plane ? "plane" : "signal";
                b.layers.push_back(layer);
                layer_id[l.name] = layer.id;
            }
        }
        auto find_layer = [&](const std::string& name, bool& warned) -> LayerId {
            if (name.empty()) return b.layers.front().id;
            auto it = layer_id.find(name);
            if (it != layer_id.end()) return it->second;
            // Case-insensitive fallback (Top/top, Bottom/bottom).
            for (const auto& kv : layer_id) {
                std::string a = lower_str(kv.first), c = lower_str(name);
                if (a == c) return kv.second;
            }
            if (!warned) {
                warned = true;
                warn("ipc-2581: unknown layer '" + name + "'; defaulted to " +
                     b.layers.front().name + " with warning");
            }
            return b.layers.front().id;
        };

        // Nets.
        if (nets_.empty() && !pads_.empty()) {
            // Derive net names from pad references (documented fallback).
            for (const auto& p : pads_) {
                if (!p.net.empty() &&
                    std::find(nets_.begin(), nets_.end(), p.net) == nets_.end())
                    nets_.push_back(p.net);
            }
            if (!nets_.empty())
                warn("ipc-2581: no <Net> entries; net names derived from pad refs");
        }
        if (nets_.empty())
            throw BoardError(InputKind::kInvalid, "ipc-2581: no nets found (<Net> or pad refs)");
        NetId next_net = 0;
        std::map<std::string, NetId, std::less<>> net_id;
        for (const auto& name : nets_) {
            if (net_id.count(name)) continue;
            NetInfo n;
            n.id = next_net++;
            n.name = name;
            b.nets.push_back(n);
            net_id[name] = n.id;
        }
        // Net classes -> width/clearance floors.
        for (const auto& kv : classes_) {
            const IpcClass& c = kv.second;
            for (const auto& m : c.members) {
                auto it = net_id.find(m);
                if (it == net_id.end()) {
                    warn("ipc-2581: NetClass '" + kv.first + "' member '" + m +
                         "' names an unknown net; skipped with warning");
                    continue;
                }
                NetInfo* n = b.find_net(it->second);
                if (c.width > 0) {
                    n->has_min_width = true;
                    n->min_width_nm = mm_to_nm(mm(c.width));
                }
                if (c.clearance >= 0) {
                    n->has_min_clearance = true;
                    n->min_clearance_nm = mm_to_nm(mm(c.clearance));
                }
            }
        }

        // Outline: datum > profile bbox > copper bbox + margin.
        double ox = 0, oy = 0, bw = 0, bh = 0;
        bool have_outline = false;
        if (has_datum_) {
            bw = mm(datum_w_);
            bh = mm(datum_h_);
            have_outline = true;
        } else if (profile_.size() >= 3) {
            double x1 = profile_[0].first, y1 = profile_[0].second;
            double x2 = x1, y2 = y1;
            for (const auto& p : profile_) {
                x1 = std::min(x1, p.first);
                y1 = std::min(y1, p.second);
                x2 = std::max(x2, p.first);
                y2 = std::max(y2, p.second);
            }
            ox = mm(x1);
            oy = mm(y1);
            bw = mm(x2 - x1);
            bh = mm(y2 - y1);
            have_outline = true;
            warn("ipc-2581: no <Datum>; outline from <Profile> bbox");
        }
        auto note_cu = [&](double x, double y, double& x1, double& y1, double& x2,
                           double& y2, bool& any) {
            if (!any) {
                x1 = x2 = x;
                y1 = y2 = y;
                any = true;
            } else {
                x1 = std::min(x1, x);
                y1 = std::min(y1, y);
                x2 = std::max(x2, x);
                y2 = std::max(y2, y);
            }
        };
        double cx1 = 0, cy1 = 0, cx2 = 0, cy2 = 0;
        bool any_cu = false;
        for (const auto& p : pads_) {
            note_cu(mm(p.x), mm(p.y), cx1, cy1, cx2, cy2, any_cu);
        }
        for (const auto& t : traces_) {
            note_cu(mm(t.x1), mm(t.y1), cx1, cy1, cx2, cy2, any_cu);
            note_cu(mm(t.x2), mm(t.y2), cx1, cy1, cx2, cy2, any_cu);
        }
        for (const auto& v : vias_) note_cu(mm(v.x), mm(v.y), cx1, cy1, cx2, cy2, any_cu);
        if (!have_outline) {
            if (!any_cu)
                throw BoardError(InputKind::kInvalid, "ipc-2581: no geometry found");
            ox = cx1 - 1.0;
            oy = cy1 - 1.0;
            bw = (cx2 - cx1) + 2.0;
            bh = (cy2 - cy1) + 2.0;
            warn("ipc-2581: no <Datum>/<Profile>; outline = copper bbox + 1mm margin");
        }
        b.width_nm = mm_to_nm(bw);
        b.height_nm = mm_to_nm(bh);
        if (b.width_nm <= 0 || b.height_nm <= 0)
            throw BoardError(InputKind::kInvalid, "ipc-2581: degenerate board outline");
        auto qx = [&](double v) { return mm_to_nm(mm(v) - ox); };
        auto qy = [&](double v) { return mm_to_nm(mm(v) - oy); };

        // Terminals.
        TermId next_term = 0;
        bool warned_net = false, warned_pad_layer = false;
        for (const auto& p : pads_) {
            if (p.net.empty()) {
                if (!warned_net) {
                    warned_net = true;
                    warn("ipc-2581: pad without a net skipped with warning (first: " +
                         p.comp + ")");
                }
                continue;
            }
            auto nit = net_id.find(p.net);
            if (nit == net_id.end()) {
                warn("ipc-2581: pad on unknown net '" + p.net + "' skipped with warning");
                continue;
            }
            if (!(p.w > 0) || !(p.h > 0)) {
                warn("ipc-2581: pad with non-positive size skipped with warning");
                continue;
            }
            Terminal t;
            t.id = next_term++;
            t.net = nit->second;
            t.pos = {qx(p.x), qy(p.y)};
            t.layer = find_layer(p.layer, warned_pad_layer);
            t.pad_w_nm = mm_to_nm(mm(p.w));
            t.pad_h_nm = mm_to_nm(mm(p.h));
            t.component = p.comp;
            t.pin = "1";
            b.terminals.push_back(t);
            b.find_net(t.net)->terminals.push_back(t.id);
        }
        // Pin numbers: stable per-component sequence (documented; IPC pad
        // order is file order, so numbering is deterministic).
        {
            std::map<std::string, int> seq;
            for (auto& t : b.terminals) t.pin = std::to_string(++seq[t.component]);
        }
        // Traces.
        bool warned_trace_layer = false;
        for (const auto& tr : traces_) {
            auto nit = net_id.find(tr.net);
            if (nit == net_id.end()) {
                warn("ipc-2581: trace on unknown net '" + tr.net + "' skipped with warning");
                continue;
            }
            if (!(tr.w > 0)) {
                warn("ipc-2581: trace with non-positive width skipped with warning");
                continue;
            }
            TraceSeg s;
            s.net = nit->second;
            s.layer = find_layer(tr.layer, warned_trace_layer);
            s.a = {qx(tr.x1), qy(tr.y1)};
            s.b = {qx(tr.x2), qy(tr.y2)};
            s.width_nm = mm_to_nm(mm(tr.w));
            b.traces.push_back(s);
        }
        // Vias.
        for (const auto& vv : vias_) {
            auto nit = net_id.find(vv.net);
            if (nit == net_id.end()) {
                warn("ipc-2581: via on unknown net '" + vv.net + "' skipped with warning");
                continue;
            }
            if (!(vv.outer > 0) || !(vv.hole > 0) || !(vv.hole < vv.outer)) {
                warn("ipc-2581: via with bad geometry skipped with warning");
                continue;
            }
            Via v;
            v.net = nit->second;
            v.pos = {qx(vv.x), qy(vv.y)};
            bool w1 = false, w2 = false;
            v.top_layer =
                vv.top.empty() ? b.layers.front().id : find_layer(vv.top, w1);
            v.bottom_layer =
                vv.bottom.empty() ? b.layers.back().id : find_layer(vv.bottom, w2);
            v.outer_d_nm = mm_to_nm(mm(vv.outer));
            v.hole_d_nm = mm_to_nm(mm(vv.hole));
            b.vias.push_back(v);
        }
        if (b.terminals.empty())
            throw BoardError(InputKind::kInvalid, "ipc-2581: no usable pads found");
        warn("ipc-2581: flat-subset import (see README limits): stackup "
             "dielectrics, padstack libraries, embedded parts, DFM/BOM, panel "
             "steps beyond the first are not modeled");
        return std::move(out);
    }
};

bool Ipc2581Importer::claims(const std::string& path, const std::string& head_bytes) const {
    std::string head = lower_str(head_bytes);
    bool marker = head.find("ipc2581") != std::string::npos ||
                  head.find("ipc-2581") != std::string::npos ||
                  head.find("<ecad") != std::string::npos;
    if (!marker) return false;
    std::string l = lower_str(path);
    if (ends_with_ci(l, ".json") || ends_with_ci(l, ".dsn") || ends_with_ci(l, ".ses") ||
        ends_with_ci(l, ".kicad_pcb") || ends_with_ci(l, ".gbr") || ends_with_ci(l, ".ger"))
        return false;
    return true;
}

ImportResult Ipc2581Importer::import_file(const std::string& path) const {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw BoardError(InputKind::kInvalid, "cannot open file: " + path);
    return Ipc2581Import().run_stream(f, path);
}

ImportResult Ipc2581Importer::import_text(const std::string& text,
                                          const std::string& path) const {
    std::istringstream in(text);
    return Ipc2581Import().run_stream(in, path);
}

const BoardImporter& ipc2581_importer_singleton() {
    static const Ipc2581Importer kInstance;
    return kInstance;
}

}  // namespace copperline
