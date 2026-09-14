#include "router/rules.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace copperline {

Coord WidthClassModel::evaluate(const NetInfo& net, const BoardDefaults& def,
                                const ElectricalContext& ctx) const {
    if (net.width_class.empty()) return -1;
    auto it = classes_.find(net.width_class);
    if (it == classes_.end()) return -1;
    bool dummy = false;
    // Width classes carry their own current ceiling; current still flows from
    // the net so that reporting stays honest.
    (void)def;
    (void)ctx;
    (void)dummy;
    return it->second.min_width_nm;
}

IpcEstimateModel::IpcEstimateModel(double mm_per_amp, double min_mm, double max_mm)
    : mm_per_amp_(mm_per_amp), min_nm_(mm_to_nm(min_mm)), max_nm_(mm_to_nm(max_mm)) {}

Coord IpcEstimateModel::evaluate(const NetInfo& net, const BoardDefaults& def,
                                  const ElectricalContext&) const {
    if (!enabled) return -1;
    // Absent current metadata means board defaults (spec): the estimator must
    // not invent current from the default. It only scales stated current.
    if (!net.has_peak && !net.has_current) return -1;
    (void)def;
    double current = net.has_peak ? net.peak_a : net.current_a;
    if (current <= 0) return -1;
    Coord w = mm_to_nm(current * mm_per_amp_);
    return std::clamp(w, min_nm_, max_nm_);
}

// ---- IPC-2221 ampacity model (issue #7) ----

namespace {

// IPC-2221 section 6.2 constants: I = k * dT^0.44 * A^0.725.
constexpr double kAmpacityKExternal = 0.048;
constexpr double kAmpacityKInternal = 0.024;
constexpr double kAmpacityExpB = 0.44;
constexpr double kAmpacityExpC = 0.725;
constexpr double kMilsPerOz = 1.37;   // 1 oz Cu ~= 1.37 mil ~= 34.8 um
constexpr double kMmPerMil = 0.0254;

}  // namespace

AmpacityModel::AmpacityModel(double min_mm, double max_mm)
    : min_nm_(mm_to_nm(min_mm)), max_nm_(mm_to_nm(max_mm)) {}

double AmpacityModel::width_mm_for(double current_a, double copper_oz, double temp_rise_c,
                                   bool internal) {
    if (current_a <= 0 || copper_oz <= 0 || temp_rise_c <= 0) return -1.0;
    double k = internal ? kAmpacityKInternal : kAmpacityKExternal;
    double area_sq_mils =
        std::pow(current_a / (k * std::pow(temp_rise_c, kAmpacityExpB)), 1.0 / kAmpacityExpC);
    double width_mils = area_sq_mils / copper_mils(copper_oz);
    return width_mils * kMmPerMil;
}

Coord AmpacityModel::evaluate_for(double current_a, double copper_oz, double temp_rise_c,
                                  bool internal) const {
    double w_mm = width_mm_for(current_a, copper_oz, temp_rise_c, internal);
    if (w_mm < 0) return -1;
    return std::clamp(mm_to_nm(w_mm), min_nm_, max_nm_);
}

Coord AmpacityModel::evaluate(const NetInfo& net, const BoardDefaults& def,
                              const ElectricalContext& ctx) const {
    if (!enabled) return -1;
    // Same honesty rule as the legacy model: only stated current is scaled.
    if (!net.has_peak && !net.has_current) return -1;
    double current = net.has_peak ? net.peak_a : net.current_a;
    if (current <= 0) return -1;
    double oz = ctx.copper_weight_oz > 0 ? ctx.copper_weight_oz : def.copper_weight_oz;
    double dt = ctx.temp_rise_c > 0 ? ctx.temp_rise_c : def.temp_rise_c;
    return evaluate_for(current, oz, dt, /*internal=*/false);
}

CurrentCapacitySystem::CurrentCapacitySystem() : classes_(std::map<std::string, WidthClass>()) {}

void CurrentCapacitySystem::set_classes(std::map<std::string, WidthClass> classes) {
    classes_ = WidthClassModel(std::move(classes));
}

double CurrentCapacitySystem::effective_current(const NetInfo& net, const BoardDefaults& def,
                                                bool& default_used) const {
    default_used = false;
    if (net.has_peak) return net.peak_a;
    if (net.has_current) return net.current_a;
    default_used = true;
    return def.default_current_a;
}

bool CurrentCapacitySystem::ampacity_enabled() const {
    if (!ampacity_.enabled) return false;
    // The ampacity model is the default inferred estimator. An explicit
    // legacy "ipc" sidecar block (without a companion "ampacity" block)
    // keeps selecting the legacy linear model for backward compatibility.
    if (has_ipc_config_ && !has_ampacity_config_) return false;
    return true;
}

double CurrentCapacitySystem::effective_temp_rise(const Board& board,
                                                  const ElectricalContext& ctx) const {
    if (ctx.temp_rise_c > 0) return ctx.temp_rise_c;
    if (thermal_temp_c_ > 0) return thermal_temp_c_;
    return board.defaults.temp_rise_c;
}

double CurrentCapacitySystem::effective_copper_oz(const Board& board, LayerId layer,
                                                   const ElectricalContext& ctx) const {
    for (const auto& l : board.layers) {
        if (l.id == layer && l.copper_weight_oz > 0) return l.copper_weight_oz;
    }
    if (ctx.copper_weight_oz > 0) return ctx.copper_weight_oz;
    if (thermal_copper_oz_ > 0) return thermal_copper_oz_;
    return board.defaults.copper_weight_oz;
}

bool CurrentCapacitySystem::effective_internal(const Board& board, LayerId layer) const {
    for (const auto& l : board.layers) {
        if (l.id != layer) continue;
        if (l.has_internal_flag) return l.is_internal;
    }
    if (board.layers.size() <= 2) return false;
    // Stackup inference: the first/last entries are the outer (external)
    // copper; anything sandwiched between them is internal.
    if (!board.layers.empty() &&
        (layer == board.layers.front().id || layer == board.layers.back().id))
        return false;
    for (const auto& l : board.layers) {
        if (l.id == layer) return true;  // known middle layer
    }
    return false;  // unknown layer: conservative external assumption
}

ElectricalContext CurrentCapacitySystem::default_context(const Board& board) const {
    ElectricalContext ctx;
    ctx.temp_rise_c = effective_temp_rise(board, ctx);
    // Copper is layer-dependent; report the board-level fallback here and
    // let effective_copper_oz() refine per layer.
    ctx.copper_weight_oz =
        thermal_copper_oz_ > 0 ? thermal_copper_oz_ : board.defaults.copper_weight_oz;
    return ctx;
}

WidthDetails CurrentCapacitySystem::width_details(const NetInfo& net, const Board& board,
                                                  LayerId layer,
                                                  const ElectricalContext& ctx) const {
    WidthDetails d;
    bool def_used = false;
    d.current_a = effective_current(net, board.defaults, def_used);
    d.default_current_used = def_used;
    d.temp_rise_c = effective_temp_rise(board, ctx);
    d.copper_weight_oz = effective_copper_oz(board, layer, ctx);
    d.internal_layer = effective_internal(board, layer);

    Coord w = explicit_.evaluate(net, board.defaults, ctx);
    if (w >= 0) {
        d.model = "explicit";
        d.width_nm = w;
        return d;
    }
    w = classes_.evaluate(net, board.defaults, ctx);
    if (w >= 0) {
        d.model = "width_class";
        d.width_nm = w;
        return d;
    }
    if (ampacity_enabled()) {
        if (net.has_peak || net.has_current) {
            double current = net.has_peak ? net.peak_a : net.current_a;
            if (current > 0) {
                d.model = "ampacity";
                d.width_nm = ampacity_.evaluate_for(current, d.copper_weight_oz, d.temp_rise_c,
                                                   d.internal_layer);
                return d;
            }
        }
    } else {
        w = ipc_.evaluate(net, board.defaults, ctx);
        if (w >= 0) {
            d.model = "ipc_estimate";
            d.width_nm = w;
            return d;
        }
    }
    d.model = "board_default";
    d.width_nm = board.defaults.trace_width_nm;
    return d;
}

Coord CurrentCapacitySystem::required_min_width(const NetInfo& net, const BoardDefaults& def,
                                                 const ElectricalContext& ctx,
                                                 std::string& source) const {
    Coord w = explicit_.evaluate(net, def, ctx);
    if (w >= 0) {
        source = "explicit";
        return w;
    }
    w = classes_.evaluate(net, def, ctx);
    if (w >= 0) {
        source = "width_class";
        return w;
    }
    if (ampacity_enabled()) {
        w = ampacity_.evaluate(net, def, ctx);
        if (w >= 0) {
            source = "ampacity";
            return w;
        }
    } else {
        w = ipc_.evaluate(net, def, ctx);
        if (w >= 0) {
            source = "ipc_estimate";
            return w;
        }
    }
    source = "board_default";
    return def.trace_width_nm;
}

Coord CurrentCapacitySystem::required_min_width(const NetInfo& net, const Board& board,
                                                 LayerId layer, const ElectricalContext& ctx,
                                                 std::string& source) const {
    WidthDetails d = width_details(net, board, layer, ctx);
    source = d.model;
    return d.width_nm;
}

bool CurrentCapacitySystem::neckdown_legal(const NetInfo& net, Coord neck_width_nm,
                                           Coord length_nm, const BoardDefaults& def,
                                           const ElectricalContext& ctx) const {
    std::string source;
    Coord required = required_min_width(net, def, ctx, source);
    if (neck_width_nm >= required) return true;  // not actually a neckdown
    if (!net.allow_neckdown) return false;       // never invent narrowing
    if (neck_width_nm < net.neck_width_nm) return false;
    if (net.neck_max_len_nm > 0 && length_nm > net.neck_max_len_nm) return false;
    return true;
}

bool CurrentCapacitySystem::via_style_ok(const ViaStyle& style, const NetInfo& net,
                                         const BoardDefaults& def,
                                         const ElectricalContext&) const {
    bool dummy = false;
    double i = effective_current(net, def, dummy);
    return i <= style.max_current_a;
}

int CurrentCapacitySystem::vias_required(const ViaStyle& style, const NetInfo& net,
                                         const BoardDefaults& def,
                                         const ElectricalContext&) const {
    bool dummy = false;
    double i = effective_current(net, def, dummy);
    if (style.max_current_a <= 0) return 1;
    return std::max(1, static_cast<int>(std::ceil(i / style.max_current_a)));
}

VoltageClearanceModel::VoltageClearanceModel() {
    table_.push_back({0.0, mm_to_nm(0.15)});
}

void VoltageClearanceModel::set_table(std::vector<VoltageTableEntry> table) {
    table_ = std::move(table);
    std::sort(table_.begin(), table_.end(),
              [](const auto& a, const auto& b) { return a.delta_v_min < b.delta_v_min; });
}

void VoltageClearanceModel::add_class_pair(const std::string& a, const std::string& b,
                                           Coord clearance_nm) {
    auto key = std::minmax(a, b);
    class_pairs_[key] = clearance_nm;
}

void VoltageClearanceModel::add_net_pair(const std::string& a_net, const std::string& b_net,
                                         Coord clearance_nm) {
    net_pairs_.push_back({{a_net, b_net}, clearance_nm});
}

ClearanceResolution VoltageClearanceModel::resolve(
    double v_a, bool has_a, const std::string& class_a, const std::string& name_a, double v_b,
    bool has_b, const std::string& class_b, const std::string& name_b, const NetInfo* info_a,
    const NetInfo* info_b) const {
    // Stage 1: candidate selection with fixed precedence
    //   explicit pair rule > class pair > voltage-difference table > board default.
    ClearanceResolution r;
    bool have_pair = false;
    Coord pair_c = 0;
    for (const auto& [pair, c] : net_pairs_) {
        if ((pair.first == name_a && pair.second == name_b) ||
            (pair.first == name_b && pair.second == name_a)) {
            have_pair = true;
            pair_c = c;
            break;
        }
    }
    if (have_pair) {
        r.candidate_nm = pair_c;
        r.candidate_source = "pair_rule";
    } else {
        Coord c = default_nm_;
        std::string src = "board_default";
        if (has_a && has_b && !table_.empty()) {
            // The voltage-difference table is authoritative when both voltages are
            // known: most specific applicable rule wins over the board default.
            double delta = std::fabs(v_a - v_b);
            const VoltageTableEntry* best = nullptr;
            for (const auto& e : table_) {
                if (delta >= e.delta_v_min) best = &e;
            }
            if (best) {
                c = best->clearance_nm;
                src = "voltage_table";
            }
        }
        if (!class_a.empty() && !class_b.empty()) {
            auto it = class_pairs_.find(std::minmax(class_a, class_b));
            if (it != class_pairs_.end()) {
                c = it->second;
                src = "class_pair";
            }
        }
        r.candidate_nm = c;
        r.candidate_source = src;
    }
    // Stage 2: hard-floor enforcement. Neither net's explicit minimum may be
    // undercut, regardless of which stage-1 source was selected.
    Coord floor = 0;
    if (info_a && info_a->has_min_clearance) floor = std::max(floor, info_a->min_clearance_nm);
    if (info_b && info_b->has_min_clearance) floor = std::max(floor, info_b->min_clearance_nm);
    r.floor_nm = floor;
    r.floor_applied = (floor > r.candidate_nm);
    r.value_nm = r.floor_applied ? floor : r.candidate_nm;
    r.source = r.floor_applied ? "net_floor" : r.candidate_source;
    return r;
}

Coord VoltageClearanceModel::required(double v_a, bool has_a, const std::string& class_a,
                                      const std::string& name_a, double v_b, bool has_b,
                                      const std::string& class_b, const std::string& name_b,
                                      const NetInfo* info_a, const NetInfo* info_b,
                                      std::string& source) const {
    ClearanceResolution r = resolve(v_a, has_a, class_a, name_a, v_b, has_b, class_b, name_b,
                                    info_a, info_b);
    source = r.source;
    return r.value_nm;
}

RuleResolver::RuleResolver(const Board* board, CurrentCapacitySystem current,
                           VoltageClearanceModel voltage, std::vector<ViaStyle> via_styles)
    : board_(board), current_(std::move(current)), voltage_(std::move(voltage)),
      via_styles_(std::move(via_styles)) {}

RuleResolver RuleResolver::defaults_for(const Board& board) {
    CurrentCapacitySystem current;
    VoltageClearanceModel voltage;
    voltage.set_default(board.defaults.clearance_nm);
    ViaStyle std_style;
    std_style.name = "STD";
    std_style.outer_nm = board.defaults.via_outer_nm;
    std_style.hole_nm = board.defaults.via_hole_nm;
    std_style.max_current_a = 2.0;
    return RuleResolver(&board, std::move(current), std::move(voltage), {std_style});
}

RuleResolver RuleResolver::from_config(const Board& board, const JsonValue& config) {
    if (!config.is_object()) throw BoardError(InputKind::kRule, "config root must be an object");
    CurrentCapacitySystem current;
    VoltageClearanceModel voltage;
    voltage.set_default(board.defaults.clearance_nm);
    std::vector<ViaStyle> styles;

    if (const JsonValue* wc = config.find("width_classes")) {
        if (!wc->is_array()) throw BoardError(InputKind::kRule, "width_classes must be array");
        std::map<std::string, WidthClass> classes;
        for (const auto& e : wc->as_array()) {
            WidthClass c;
            c.name = e.get_string("name");
            if (c.name.empty()) throw BoardError(InputKind::kRule, "width class missing name");
            double w = e.get_number("min_width_mm", -1);
            if (w <= 0) throw BoardError(InputKind::kRule, "width class needs min_width_mm > 0");
            c.min_width_nm = mm_to_nm(w);
            c.max_current_a = e.get_number("max_current_a", 1e9);
            classes[c.name] = c;
        }
        current.set_classes(std::move(classes));
    }
    if (const JsonValue* ipc = config.find("ipc")) {
        double k = ipc->get_number("mm_per_amp", 0.75);
        double lo = ipc->get_number("min_mm", 0.15);
        double hi = ipc->get_number("max_mm", 10.0);
        if (!(k > 0) || !(lo > 0) || !(hi >= lo))
            throw BoardError(InputKind::kRule, "bad ipc estimator parameters");
        IpcEstimateModel model(k, lo, hi);
        model.enabled = ipc->get_bool("enabled", true);
        current.set_ipc(model);
    }
    if (const JsonValue* amp = config.find("ampacity")) {
        if (!amp->is_object()) throw BoardError(InputKind::kRule, "ampacity must be an object");
        std::string model_name = amp->get_string("model", "ipc2221");
        if (model_name != "ipc2221")
            throw BoardError(InputKind::kRule,
                             "unknown ampacity model '" + model_name + "' (supported: ipc2221)");
        double lo = amp->get_number("min_mm", 0.15);
        double hi = amp->get_number("max_mm", 10.0);
        if (!(lo > 0) || !(hi >= lo))
            throw BoardError(InputKind::kRule, "bad ampacity width clamps");
        AmpacityModel model(lo, hi);
        model.enabled = amp->get_bool("enabled", true);
        current.set_ampacity(model);
        double dt = amp->get_number("temp_rise_c", -1);
        double oz = amp->get_number("copper_weight_oz", -1);
        if (amp->has("temp_rise_c") && !(dt > 0))
            throw BoardError(InputKind::kRule, "ampacity temp_rise_c must be positive");
        if (amp->has("copper_weight_oz") && !(oz > 0))
            throw BoardError(InputKind::kRule, "ampacity copper_weight_oz must be positive");
        current.set_thermal(amp->has("temp_rise_c") ? dt : -1,
                            amp->has("copper_weight_oz") ? oz : -1);
    }
    if (const JsonValue* vt = config.find("voltage_table")) {
        if (!vt->is_array()) throw BoardError(InputKind::kRule, "voltage_table must be array");
        std::vector<VoltageTableEntry> table;
        for (const auto& e : vt->as_array()) {
            VoltageTableEntry entry;
            entry.delta_v_min = e.get_number("delta_v_min", 0);
            double cl = e.get_number("clearance_mm", -1);
            if (cl < 0) throw BoardError(InputKind::kRule, "voltage table needs clearance_mm");
            entry.clearance_nm = mm_to_nm(cl);
            table.push_back(entry);
        }
        voltage.set_table(std::move(table));
    }
    if (const JsonValue* cp = config.find("class_pairs")) {
        if (!cp->is_array()) throw BoardError(InputKind::kRule, "class_pairs must be array");
        for (const auto& e : cp->as_array()) {
            std::string a = e.get_string("a"), b = e.get_string("b");
            double cl = e.get_number("clearance_mm", -1);
            if (a.empty() || b.empty() || cl < 0)
                throw BoardError(InputKind::kRule, "class pair needs a/b/clearance_mm");
            voltage.add_class_pair(a, b, mm_to_nm(cl));
        }
    }
    if (const JsonValue* pr = config.find("pair_rules")) {
        if (!pr->is_array()) throw BoardError(InputKind::kRule, "pair_rules must be array");
        for (const auto& e : pr->as_array()) {
            std::string a = e.get_string("a"), b = e.get_string("b");
            double cl = e.get_number("clearance_mm", -1);
            if (a.empty() || b.empty() || cl < 0)
                throw BoardError(InputKind::kRule, "pair rule needs a/b/clearance_mm");
            voltage.add_net_pair(a, b, mm_to_nm(cl));
        }
    }
    if (const JsonValue* vs = config.find("via_classes")) {
        if (!vs->is_array()) throw BoardError(InputKind::kRule, "via_classes must be array");
        for (const auto& e : vs->as_array()) {
            ViaStyle s;
            s.name = e.get_string("name");
            if (s.name.empty()) throw BoardError(InputKind::kRule, "via class missing name");
            double outer = e.get_number("outer_mm", -1), hole = e.get_number("hole_mm", -1);
            if (!(outer > 0) || !(hole > 0) || !(hole < outer))
                throw BoardError(InputKind::kRule, "via class needs 0 < hole < outer");
            s.outer_nm = mm_to_nm(outer);
            s.hole_nm = mm_to_nm(hole);
            s.max_current_a = e.get_number("max_current_a", 2.0);
            styles.push_back(s);
        }
    }
    if (styles.empty()) {
        ViaStyle std_style;
        std_style.name = "STD";
        std_style.outer_nm = board.defaults.via_outer_nm;
        std_style.hole_nm = board.defaults.via_hole_nm;
        styles.push_back(std_style);
    }
    return RuleResolver(&board, std::move(current), std::move(voltage), std::move(styles));
}

TraceRule RuleResolver::traceRule(NetId net, LayerId layer, RegionId region) const {
    (void)region;
    const NetInfo* n = board_->find_net(net);
    if (!n) throw std::runtime_error("traceRule: unknown net");
    ElectricalContext ctx = current_.default_context(*board_);
    TraceRule rule;
    rule.min_width_nm = current_.required_min_width(*n, *board_, layer, ctx, rule.width_source);
    rule.pref_width_nm = n->has_pref_width ? std::max(n->pref_width_nm, rule.min_width_nm)
                                           : rule.min_width_nm;
    rule.allow_neckdown = n->allow_neckdown;
    rule.neck_width_nm = n->neck_width_nm;
    rule.neck_max_len_nm = n->neck_max_len_nm;
    return rule;
}

Coord RuleResolver::requiredTraceWidth(NetId net, LayerId layer, const ElectricalContext& ctx,
                                       std::string* source_out) const {
    const NetInfo* n = board_->find_net(net);
    if (!n) throw std::runtime_error("requiredTraceWidth: unknown net");
    std::string source;
    Coord w = current_.required_min_width(*n, *board_, layer, ctx, source);
    if (source_out) *source_out = source;
    return w;
}

WidthDetails RuleResolver::widthDetails(NetId net, LayerId layer,
                                        const ElectricalContext& ctx) const {
    const NetInfo* n = board_->find_net(net);
    if (!n) throw std::runtime_error("widthDetails: unknown net");
    return current_.width_details(*n, *board_, layer, ctx);
}

ElectricalContext RuleResolver::defaultContext() const {
    return current_.default_context(*board_);
}

ClearanceResolution RuleResolver::clearanceResolution(NetId a, NetId b, LayerId layer,
                                                      const ElectricalContext& ctx) const {
    (void)layer;
    (void)ctx;
    const NetInfo* na = board_->find_net(a);
    const NetInfo* nb = board_->find_net(b);
    if (!na || !nb) throw std::runtime_error("requiredClearance: unknown net");
    return voltage_.resolve(na->voltage_v, na->has_voltage, na->voltage_class, na->name,
                            nb->voltage_v, nb->has_voltage, nb->voltage_class, nb->name, na, nb);
}

Coord RuleResolver::requiredClearance(NetId a, NetId b, LayerId layer, const ElectricalContext& ctx,
                                      std::string* source_out) const {
    ClearanceResolution r = clearanceResolution(a, b, layer, ctx);
    if (source_out) *source_out = r.source;
    return r.value_nm;
}

std::vector<ViaStyle> RuleResolver::allowedVias(NetId net, LayerSpan span) const {
    (void)span;
    const NetInfo* n = board_->find_net(net);
    if (!n) throw std::runtime_error("allowedVias: unknown net");
    ElectricalContext ctx;
    std::vector<ViaStyle> preferred, other;
    for (const auto& s : via_styles_) {
        if (!current_.via_style_ok(s, *n, board_->defaults, ctx)) continue;  // rejected
        if (!n->via_class.empty() && s.name == n->via_class) preferred.push_back(s);
        else other.push_back(s);
    }
    preferred.insert(preferred.end(), other.begin(), other.end());
    return preferred;
}

bool RuleResolver::select_via(NetId net, LayerSpan span, ViaStyle& out) const {
    auto allowed = allowedVias(net, span);
    if (allowed.empty()) return false;
    out = allowed.front();
    return true;
}

bool RuleResolver::lookup_via_style(const std::string& name, ViaStyle& out) const {
    for (const auto& s : via_styles_) {
        if (s.name == name) {
            out = s;
            return true;
        }
    }
    return false;
}

bool RuleResolver::default_current_used(NetId net) const {
    const NetInfo* n = board_->find_net(net);
    if (!n) return false;
    return !n->has_current && !n->has_peak;
}

}  // namespace copperline
