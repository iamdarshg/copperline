#include "router/analyze.h"

#include <algorithm>
#include <map>
#include <set>

#include "router/density.h"
#include "router/diffpair.h"
#include "router/escape.h"
#include "router/route_tree.h"
#include "router/via_bundle.h"

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

    // Issue #16: declared plane/zone inventory for agents and the
    // scheduler. Per-plane owning net, layer, island and routability, plus
    // per-net plane backing (drives plane-access tasks, not pad-to-pad).
    JsonValue planes = JsonValue::array();
    std::set<NetId> plane_backed;
    for (const auto& z : board.planes) {
        JsonValue o = JsonValue::object();
        o["id"] = static_cast<double>(z.id);
        const NetInfo* zn = board.find_net(z.net);
        o["net"] = static_cast<double>(z.net);
        o["net_name"] = zn ? zn->name : "?";
        o["layer"] = static_cast<double>(z.layer);
        o["island"] = static_cast<double>(z.island);
        o["routable"] = z.routable;
        Rect zb = z.bounds();
        o["x1_mm"] = nm_to_mm(zb.x1);
        o["y1_mm"] = nm_to_mm(zb.y1);
        o["x2_mm"] = nm_to_mm(zb.x2);
        o["y2_mm"] = nm_to_mm(zb.y2);
        planes.as_array().push_back(o);
        if (z.routable) plane_backed.insert(z.net);
    }
    r["planes"] = planes;

    JsonValue nets = JsonValue::array();
    std::vector<std::string> defaults_used;
    std::set<std::string> current_classes, voltage_classes;
    for (const auto& n : board.nets) {
        JsonValue o = JsonValue::object();
        o["id"] = static_cast<double>(n.id);
        o["name"] = n.name;
        o["terminals"] = static_cast<double>(n.terminals.size());
        o["plane_backed"] = plane_backed.count(n.id) > 0;
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
        WidthDetails wd = resolver.widthDetails(n.id, 0, ctx);
        o["required_width_mm"] = nm_to_mm(wd.width_nm);
        o["width_source"] = wd.model;
        // Issue #11: for impedance-controlled nets the routable width is
        // the reconciled (impedance + ampacity floor) width, not the bare
        // current floor above.
        if (resolver.impedance().has_target(n)) {
            std::string rsrc;
            Coord rw = resolver.requiredTraceWidth(n.id, 0, ctx, &rsrc);
            o["required_width_mm"] = nm_to_mm(rw);
            o["width_source"] = rsrc;
        }
        // Ampacity accounting (issue #7): which physical model and inputs
        // produced the required width, so agents can audit it.
        o["width_model"] = wd.model;
        o["copper_weight_oz"] = wd.copper_weight_oz;
        o["copper_thickness_mm"] = wd.copper_weight_oz * 0.0348;
        o["temp_rise_c"] = wd.temp_rise_c;
        o["width_internal_layer"] = wd.internal_layer;
        // Issue #11: controlled-impedance accounting (selected layer/width,
        // model, target, estimate, tolerance error, conflict).
        o["impedance"] = resolver.impedanceResolution(n.id, ctx).to_json();
        // Parallel-via diagnostics (issue #5): the current each layer
        // transition must carry and how many parallel vias that needs.
        {
            LayerSpan full{board.layers.front().id, board.layers.back().id};
            auto ordered = ViaBundlePlanner::ordered_styles(resolver, n.id, full);
            ViaStyle single;
            if (resolver.select_via(n.id, full, single)) {
                o["via_style"] = single.name;
                o["vias_required"] = 1.0;
                o["via_reason"] = "ok";
            } else if (!ordered.empty()) {
                o["via_style"] = ordered.front().name;
                o["vias_required"] = static_cast<double>(
                    ViaBundlePlanner::required_count(resolver, ordered.front(), n.id));
                o["via_reason"] = "needs_parallel_bundle";
            } else {
                o["via_style"] = "";
                o["vias_required"] = 1.0;
                o["via_reason"] = "no_via_class";
            }
        }
        double peak = 0;
        for (TermId tid : n.terminals) {
            auto it = term_dens.find(tid);
            if (it != term_dens.end()) peak = std::max(peak, it->second);
        }
        o["peak_local_density_per_mm2"] = peak;
        if (!n.width_class.empty()) current_classes.insert(n.width_class + "@" + std::to_string(eff));
        else current_classes.insert(wd.model + "@" + std::to_string(eff));
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
    // Issue #12: pair members never bottleneck individually; the atomic
    // corridor task represents the pair's shared resource demand.
    std::vector<ConnectionTask> tasks;
    {
        std::vector<std::string> pair_reasons;
        std::vector<int> pair_bad;
        if (board.diffpairs.empty()) {
            for (const auto& n : board.nets) {
                if (n.terminals.size() < 2) continue;
                RouteTree tree = build_route_tree(board, n.id);
                for (auto& t : tree.tasks) tasks.push_back(t);
            }
        } else {
            tasks = build_global_tasks_with_pairs(board, pair_reasons, pair_bad);
        }
        for (auto& t : tasks) {
            t.difficulty = task_difficulty(board, resolver, t, ctx,
                                           density.terminal_density);
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
        // Issue #12: corridor bottlenecks name the pair and both members.
        o["is_pair_corridor"] = t.is_pair_corridor;
        if (t.is_pair_corridor) {
            o["pair_id"] = static_cast<double>(t.pair_id);
            o["pair_other_net"] = static_cast<double>(t.pair_other_net);
        }
        if (ta && tb) o["span_mm"] = nm_to_mm(manhattan(ta->pos, tb->pos));
        bn.as_array().push_back(o);
    }
    r["likely_bottlenecks"] = bn;

    JsonValue w = JsonValue::array();
    for (const auto& s : import_warnings) w.as_array().push_back(JsonValue(s));
    r["import_warnings"] = w;

    // Issue #12: pair inventory for agents (members, gap, occupied width).
    JsonValue dp = JsonValue::array();
    for (const auto& pr : board.diffpairs) {
        JsonValue o = JsonValue::object();
        o["pair_id"] = static_cast<double>(pr.id);
        o["name"] = pr.name;
        o["net_p"] = static_cast<double>(pr.net_p);
        o["net_n"] = static_cast<double>(pr.net_n);
        const NetInfo* npp = board.find_net(pr.net_p);
        const NetInfo* nnn = board.find_net(pr.net_n);
        o["net_p_name"] = npp ? npp->name : "?";
        o["net_n_name"] = nnn ? nnn->name : "?";
        o["gap_mm"] = nm_to_mm(pr.gap_nm);
        o["gap_tol_mm"] = nm_to_mm(pr.gap_tol_nm);
        o["occupied_width_mm"] = nm_to_mm(diffpair_occupied_width(board, resolver, pr, ctx));
        o["via_policy"] = pr.via_policy;
        if (pr.has_max_skew) o["max_skew_mm"] = nm_to_mm(pr.max_skew_nm);
        if (pr.has_impedance) o["target_impedance_ohms"] = pr.target_impedance_ohms;
        std::string reason;
        o["valid"] = diffpair_valid(board, pr, reason);
        o["valid_reason"] = reason;
        dp.as_array().push_back(o);
    }
    r["diffpairs"] = dp;

    out.data = r;
    return out;
}

}  // namespace copperline
