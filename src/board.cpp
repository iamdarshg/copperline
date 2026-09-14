#include "router/board.h"

#include <algorithm>
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
            board.layers.push_back(layer);
        }
    }
    if (board.layers.empty()) {
        board.layers.push_back({0, "Top", false, 1.0});
        board.layers.push_back({1, "Bottom", true, 1.0});
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

    const JsonValue* traces = root.find("traces");
    if (traces) {
        if (!traces->is_array()) throw BoardError(InputKind::kInvalid, "'traces' must be array");
        for (const auto& t : traces->as_array()) {
            TraceSeg seg;
            seg.net = resolve_net(*t.find("net"), board, "traces[]");
            seg.layer = t.has("layer") ? resolve_layer(*t.find("layer"), board, "traces[]") : 0;
            seg.a = {mm_to_nm(t.get_number("x1_mm", 0)), mm_to_nm(t.get_number("y1_mm", 0))};
            seg.b = {mm_to_nm(t.get_number("x2_mm", 0)), mm_to_nm(t.get_number("y2_mm", 0))};
            if (!seg.segment().axis_aligned())
                throw BoardError(InputKind::kInvalid, "only Manhattan traces supported in phase 1");
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
        JsonValue terms = JsonValue::array();
        for (TermId tid : n.terminals) {
            const Terminal* t = board.find_terminal(tid);
            if (!t) continue;
            JsonValue to = JsonValue::object();
            to["id"] = static_cast<double>(t->id);
            to["x_mm"] = nm_to_mm(t->pos.x);
            to["y_mm"] = nm_to_mm(t->pos.y);
            to["layer"] = static_cast<double>(t->layer);
            terms.as_array().push_back(to);
        }
        o["terminals"] = terms;
        nets.as_array().push_back(o);
    }
    root["nets"] = nets;

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
        vias.as_array().push_back(o);
    }
    root["vias"] = vias;
    return root;
}

std::vector<std::string> supported_formats() { return {"json", "kicad_pcb"}; }

ImportResult import_board_auto(const std::string& path) {
    std::string text = read_file(path);
    std::string head = text.substr(0, std::min<std::size_t>(text.size(), 4096));
    // Static importers to avoid repeating construction.
    static const JsonBoardImporter kJson;
    // KiCad importer is defined in kicad.cpp; declared here to keep
    // board.cpp independent of its header weight.
    extern const BoardImporter& kicad_importer_singleton();
    const BoardImporter* importers[] = {&kJson, &kicad_importer_singleton()};
    for (const BoardImporter* imp : importers) {
        if (imp->claims(path, head)) return imp->import_file(path);
    }
    throw BoardError(InputKind::kInvalid,
                     "unsupported board format; supported: json, kicad_pcb (.kicad_pcb). "
                     "Specctra DSN / SES / IPC-2581 land in Prompt 5.");
}

}  // namespace copperline
