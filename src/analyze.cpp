#include "router/analyze.h"

#include <algorithm>
#include <map>
#include <set>

#include "router/density.h"
#include "router/escape.h"
#include "router/route_tree.h"

namespace copperline {

AnalysisResult analyze_board(const Board& board, const RuleResolver& resolver,
                             const ElectricalContext& ctx,
                             const std::vector<std::string>& import_warnings) {
    AnalysisResult out;
    JsonValue r = JsonValue::object();
    r["schema"] = "copperline/analyze-report/1";

    DensityEstimator estimator;
    DensityResult density = estimator.analyze(board);

    JsonValue b = JsonValue::object();
    b["source_format"] = board.source_format;
    b["width_mm"] = nm_to_mm(board.width_nm);
    b["height_mm"] = nm_to_mm(board.height_nm);
    b["layers"] = static_cast<double>(board.layers.size());
    b["nets"] = static_cast<double>(board.nets.size());
    b["terminals"] = static_cast<double>(board.terminals.size());
    b["keepouts"] = static_cast<double>(board.keepouts.size());
    b["prerouted_traces"] = static_cast<double>(board.traces.size());
    b["prerouted_vias"] = static_cast<double>(board.vias.size());
    r["board"] = b;

    std::map<std::size_t, double> term_dens;
    for (std::size_t i = 0; i < board.terminals.size(); ++i)
        term_dens[board.terminals[i].id] = density.terminal_density[i];

    JsonValue nets = JsonValue::array();
    std::vector<std::string> defaults_used;
    std::set<std::string> current_classes, voltage_classes;
    for (const auto& n : board.nets) {
        JsonValue o = JsonValue::object();
        o["id"] = static_cast<double>(n.id);
        o["name"] = n.name;
        o["terminals"] = static_cast<double>(n.terminals.size());
        bool def_used = resolver.default_current_used(n.id);
        bool dummy = false;
        double eff = resolver.current().effective_current(n, board.defaults, dummy);
        o["current_a"] = n.has_current ? n.current_a : board.defaults.default_current_a;
        o["effective_current_a"] = eff;
        o["default_current_used"] = def_used;
        if (def_used) defaults_used.push_back(n.name);
        if (n.has_peak) o["peak_a"] = n.peak_a;
        if (n.has_voltage) o["voltage_v"] = n.voltage_v;
        if (!n.voltage_class.empty()) o["voltage_class"] = n.voltage_class;
        std::string wsource;
        Coord w = resolver.requiredTraceWidth(n.id, 0, ctx, &wsource);
        o["required_width_mm"] = nm_to_mm(w);
        o["width_source"] = wsource;
        double peak = 0;
        for (TermId tid : n.terminals) {
            auto it = term_dens.find(tid);
            if (it != term_dens.end()) peak = std::max(peak, it->second);
        }
        o["peak_local_density_per_mm2"] = peak;
        if (!n.width_class.empty()) current_classes.insert(n.width_class + "@" + std::to_string(eff));
        else current_classes.insert(wsource + "@" + std::to_string(eff));
        voltage_classes.insert(n.voltage_class.empty() ? "(none)" : n.voltage_class);
        nets.as_array().push_back(o);
    }
    r["nets"] = nets;

    JsonValue fps = JsonValue::array();
    for (const auto& fp : density.dense_footprints) {
        JsonValue o = JsonValue::object();
        o["component"] = fp.component;
        o["pins"] = static_cast<double>(fp.pin_count);
        o["pins_per_mm2"] = fp.pins_per_mm2;
        o["x_mm"] = nm_to_mm(fp.centroid.x);
        o["y_mm"] = nm_to_mm(fp.centroid.y);
        fps.as_array().push_back(o);
    }
    r["dense_footprints"] = fps;

    // Fine-pitch escape inputs (Prompt 2): which components deserve dedicated
    // escape handling, with pitch/channel/depth context for agents.
    {
        FinePitchDetector detector;
        CentreDepthAnalyzer cda;
        JsonValue fe = JsonValue::array();
        for (const auto& fp : detector.detect(board, resolver, ctx)) {
            JsonValue o = JsonValue::object();
            o["component"] = fp.component;
            o["pins"] = static_cast<double>(fp.pad_count);
            o["pitch_mm"] = nm_to_mm(fp.pitch_nm);
            o["channel_count"] = static_cast<double>(fp.channel_count);
            o["pins_per_mm2"] = fp.pins_per_mm2;
            o["reason"] = fp.reason;
            auto depth = cda.analyze(board, fp);
            int max_d = 0;
            for (const auto& [tid, d] : depth) max_d = std::max(max_d, d);
            o["max_centre_depth"] = static_cast<double>(max_d);
            fe.as_array().push_back(o);
        }
        r["fine_pitch"] = fe;
    }

    JsonValue cc = JsonValue::array();
    for (const auto& c : current_classes) cc.as_array().push_back(JsonValue(c));
    r["current_classes"] = cc;
    JsonValue vc = JsonValue::array();
    for (const auto& c : voltage_classes) vc.as_array().push_back(JsonValue(c));
    r["voltage_classes"] = vc;

    JsonValue du = JsonValue::array();
    for (const auto& n : defaults_used) du.as_array().push_back(JsonValue(n));
    r["defaults_used_for_current"] = du;

    // Bottlenecks: all tasks scored, hardest first (top 20).
    std::vector<ConnectionTask> tasks;
    for (const auto& n : board.nets) {
        if (n.terminals.size() < 2) continue;
        RouteTree tree = build_route_tree(board, n.id);
        for (auto& t : tree.tasks) {
            t.difficulty = task_difficulty(board, resolver, t, ctx, density.terminal_density);
            tasks.push_back(t);
        }
    }
    sort_tasks_deterministic(tasks);
    JsonValue bn = JsonValue::array();
    for (std::size_t i = 0; i < tasks.size() && i < 20; ++i) {
        const auto& t = tasks[i];
        const NetInfo* n = board.find_net(t.net);
        const Terminal* ta = board.find_terminal(t.a);
        const Terminal* tb = board.find_terminal(t.b);
        JsonValue o = JsonValue::object();
        o["net"] = static_cast<double>(t.net);
        o["net_name"] = n ? n->name : "?";
        o["terminal_a"] = static_cast<double>(t.a);
        o["terminal_b"] = static_cast<double>(t.b);
        o["difficulty"] = t.difficulty;
        if (ta && tb) o["span_mm"] = nm_to_mm(manhattan(ta->pos, tb->pos));
        bn.as_array().push_back(o);
    }
    r["likely_bottlenecks"] = bn;

    JsonValue w = JsonValue::array();
    for (const auto& s : import_warnings) w.as_array().push_back(JsonValue(s));
    r["import_warnings"] = w;

    out.data = r;
    return out;
}

}  // namespace copperline
