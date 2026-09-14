#include "router/parallel.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "router/via_bundle.h"

namespace copperline {

namespace {

// Exact squared edge-to-edge gap between rects (0 when touching/overlapping).
__int128 rect_gap2(const Rect& a, const Rect& b) {
    Coord dx = 0, dy = 0;
    if (a.x2 < b.x1) dx = b.x1 - a.x2;
    else if (b.x2 < a.x1) dx = a.x1 - b.x2;
    if (a.y2 < b.y1) dy = b.y1 - a.y2;
    else if (b.y2 < a.y1) dy = a.y1 - b.y2;
    return (__int128)dx * dx + (__int128)dy * dy;
}

// Fast conservative test: exact gap >= need? Rectangular pre-check first.
bool gap_ok_rect(const Rect& a, const Rect& b, Coord need) {
    if (!a.expanded(need).intersects(b)) return true;
    return rect_gap2(a, b) >= (__int128)need * need;
}

bool seg_ok_rect(const Segment& s, const Rect& raw, Coord need) {
    if (!s.bounds().expanded(need).intersects(raw)) return true;
    __int128 d2 = seg_rect_dist2(s, raw);
    return d2 >= (__int128)need * need;
}

const Terminal* find_term(const Board& b, TermId id) { return b.find_terminal(id); }

Coord max_clear_for(const Board& board, const RuleResolver& resolver, NetId net,
                    const ElectricalContext& ctx) {
    Coord m = 0;
    std::string cs;
    for (const auto& o : board.nets) {
        if (o.id == net) continue;
        m = std::max(m, resolver.requiredClearance(net, o.id, 0, ctx, &cs));
    }
    return m;
}

double term_density_at(const Board& board, TermId tid, const std::vector<double>& dens) {
    for (std::size_t i = 0; i < board.terminals.size(); ++i)
        if (board.terminals[i].id == tid && i < dens.size()) return dens[i];
    return 0.0;
}

}  // namespace

// ---- Difficulty ----

DifficultyVector compute_difficulty(const Board& board, const RuleResolver& resolver,
                                    const ConnectionTask& task, const ElectricalContext& ctx,
                                    const std::vector<double>& terminal_density,
                                    const std::map<TermId, int>& centre_depth, int prev_failures) {
    DifficultyVector d;
    const Terminal* ta = find_term(board, task.a);
    const Terminal* tb = find_term(board, task.b);
    const NetInfo* net = board.find_net(task.net);
    if (!ta || !tb) return d;

    d.span_mm = nm_to_mm(manhattan(ta->pos, tb->pos));
    std::string wsource;
    Coord width = resolver.requiredTraceWidth(task.net, ta->layer, ctx, &wsource);
    d.width_mm = nm_to_mm(width);
    d.clearance_mm = nm_to_mm(max_clear_for(board, resolver, task.net, ctx));
    d.endpoint_density =
        std::max(term_density_at(board, task.a, terminal_density),
                 term_density_at(board, task.b, terminal_density));

    // Free routing space inside the probable corridor.
    Rect corr = Rect::from_points(ta->pos, tb->pos).expanded(width / 2 + mm_to_nm(0.5));
    double corr_area = static_cast<double>(corr.width()) * static_cast<double>(corr.height());
    double blocked = 0.0;
    if (corr_area > 0) {
        for (const auto& ko : board.keepouts) {
            if (!ko.rect.intersects(corr)) continue;
            Rect inter{std::max(ko.rect.x1, corr.x1), std::max(ko.rect.y1, corr.y1),
                       std::min(ko.rect.x2, corr.x2), std::min(ko.rect.y2, corr.y2)};
            blocked += static_cast<double>(inter.width()) * static_cast<double>(inter.height());
        }
        for (const auto& t : board.terminals) {
            if (t.net == task.net) continue;
            Rect pr = t.pad_rect();
            if (!pr.intersects(corr)) continue;
            Rect inter{std::max(pr.x1, corr.x1), std::max(pr.y1, corr.y1),
                       std::min(pr.x2, corr.x2), std::min(pr.y2, corr.y2)};
            blocked += static_cast<double>(inter.width()) * static_cast<double>(inter.height());
        }
    }
    double cover = corr_area > 0 ? std::min(1.0, blocked / corr_area) : 0.0;
    d.free_space = 1.0 - cover;
    d.corridor_count = 2.0 * std::max<std::size_t>(1, board.layers.size());
    d.corridor_scarcity = cover + 1.0 / d.corridor_count;

    // Layer restrictions: costly layers shrink options.
    int costly = 0;
    for (const auto& l : board.layers)
        if (l.cost_multiplier > 1.25) ++costly;
    d.layer_restriction =
        board.layers.empty() ? 0 : 0.5 * static_cast<double>(costly) / board.layers.size();

    // Via restrictions: multi-layer span needs a via; over-current styles
    // force parallel vias; a missing preferred class hurts.
    d.via_restriction = (ta->layer == tb->layer) ? 0.0 : 0.5;
    {
        ViaStyle style;
        LayerSpan full{board.layers.front().id, board.layers.back().id};
        if (!resolver.select_via(task.net, full, style)) {
            d.via_restriction += 2.0;
        } else {
            ElectricalContext c2 = ctx;
            int n = resolver.current().vias_required(style, *board.find_net(task.net),
                                                     board.defaults, c2);
            if (n > 1) d.via_restriction += 1.5 * (n - 1);
        }
    }

    d.prev_failures = static_cast<double>(std::max(0, prev_failures));
    {
        auto it = centre_depth.find(task.a);
        int da = it != centre_depth.end() ? it->second : 0;
        it = centre_depth.find(task.b);
        int db = it != centre_depth.end() ? it->second : 0;
        d.fine_pitch_depth = static_cast<double>(std::max(da, db));
    }

    d.total = d.span_mm + 40.0 * d.width_mm + 30.0 * d.clearance_mm + 0.5 * d.endpoint_density +
              4.0 * (1.0 - d.free_space) + 2.0 * d.corridor_scarcity + d.layer_restriction +
              d.via_restriction + 3.0 * d.prev_failures + 5.0 * d.fine_pitch_depth;
    if (wsource == "ipc_estimate" || wsource == "ampacity") d.total += 0.5;
    if (net && net->terminals.size() > 2) d.total += 0.25 * net->terminals.size();
    return d;
}

// ---- Corridors + interference ----

Corridor probable_corridor(const Board& board, const RuleResolver& resolver,
                           const ConnectionTask& task, const ElectricalContext& ctx) {
    Corridor c;
    const Terminal* ta = find_term(board, task.a);
    const Terminal* tb = find_term(board, task.b);
    if (!ta || !tb) return c;
    std::string ws;
    c.width_nm = resolver.requiredTraceWidth(task.net, ta->layer, ctx, &ws);
    c.clear_nm = max_clear_for(board, resolver, task.net, ctx);
    c.rect = Rect::from_points(ta->pos, tb->pos).expanded(c.width_nm / 2 + c.clear_nm);
    return c;
}

std::vector<std::vector<double>> build_interference(const std::vector<ConnectionTask>& tasks,
                                                    const std::vector<Corridor>& corridors) {
    std::size_t n = tasks.size();
    std::vector<std::vector<double>> w(n, std::vector<double>(n, 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const Rect& a = corridors[i].rect;
            const Rect& b = corridors[j].rect;
            if (!a.intersects(b)) continue;
            Rect inter{std::max(a.x1, b.x1), std::max(a.y1, b.y1), std::min(a.x2, b.x2),
                       std::min(a.y2, b.y2)};
            double overlap_mm2 = nm_to_mm(inter.width()) * nm_to_mm(inter.height());
            // Wider traces + larger clearances consume more shared resource.
            double wi = nm_to_mm(corridors[i].width_nm), wj = nm_to_mm(corridors[j].width_nm);
            double ci = nm_to_mm(corridors[i].clear_nm), cj = nm_to_mm(corridors[j].clear_nm);
            double v = overlap_mm2 * (1.0 + 2.0 * (ci + cj) + (wi + wj));
            w[i][j] = w[j][i] = v;
        }
    }
    return w;
}

std::vector<std::vector<int>> schedule_batches(const std::vector<ConnectionTask>& tasks,
                                                int batch_size) {
    if (batch_size < 1) batch_size = 1;
    std::vector<int> order(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) order[i] = static_cast<int>(i);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        if (tasks[a].difficulty != tasks[b].difficulty)
            return tasks[a].difficulty > tasks[b].difficulty;
        if (tasks[a].net != tasks[b].net) return tasks[a].net < tasks[b].net;
        if (tasks[a].a != tasks[b].a) return tasks[a].a < tasks[b].a;
        return tasks[a].b < tasks[b].b;
    });
    std::vector<std::vector<int>> batches;
    for (std::size_t i = 0; i < order.size(); i += batch_size) {
        batches.push_back({});
        for (std::size_t j = i; j < order.size() && j < i + (std::size_t)batch_size; ++j)
            batches.back().push_back(order[j]);
    }
    return batches;
}

// ---- Congestion ----

void CongestionMap::init(const Board& board, int cells_per_side) {
    n_ = std::max(4, cells_per_side);
    bounds_ = board.bounds();
    Coord w = std::max<Coord>(1, bounds_.width());
    Coord h = std::max<Coord>(1, bounds_.height());
    cell_w_ = std::max<Coord>(1, (w + n_ - 1) / n_);
    cell_h_ = std::max<Coord>(1, (h + n_ - 1) / n_);
    present_.assign(n_ * n_, 0.0);
    history_.assign(n_ * n_, 0.0);
}

void CongestionMap::reset_present() { std::fill(present_.begin(), present_.end(), 0.0); }

int CongestionMap::cell_of_x(Coord x) const {
    int c = static_cast<int>((x - bounds_.x1) / cell_w_);
    return std::clamp(c, 0, n_ - 1);
}
int CongestionMap::cell_of_y(Coord y) const {
    int c = static_cast<int>((y - bounds_.y1) / cell_h_);
    return std::clamp(c, 0, n_ - 1);
}

void CongestionMap::add_present_corridor(const Rect& rect, double weight) {
    int x0 = cell_of_x(rect.x1), x1 = cell_of_x(rect.x2);
    int y0 = cell_of_y(rect.y1), y1 = cell_of_y(rect.y2);
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) present_[y * n_ + x] += weight;
}

void CongestionMap::add_history_rect(const Rect& rect, double amount) {
    int x0 = cell_of_x(rect.x1), x1 = cell_of_x(rect.x2);
    int y0 = cell_of_y(rect.y1), y1 = cell_of_y(rect.y2);
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) history_[y * n_ + x] += amount;
}

void CongestionMap::add_history_segment(const Segment& seg, double amount) {
    add_history_rect(seg.bounds(), amount);
}

Coord CongestionMap::penalty_for_segment(const Segment& seg) const {
    Rect r = seg.bounds();
    int x0 = cell_of_x(r.x1), x1 = cell_of_x(r.x2);
    int y0 = cell_of_y(r.y1), y1 = cell_of_y(r.y2);
    double cost = 0.0;
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) cost += present_[y * n_ + x] + 3.0 * history_[y * n_ + x];
    // 0.025mm per cost unit, capped: bounded distortion, never a hard lock.
    Coord p = static_cast<Coord>(cost * 25000.0);
    return std::min<Coord>(p, 1500000);
}

double CongestionMap::present_at(Point p) const {
    return present_[cell_of_y(p.y) * n_ + cell_of_x(p.x)];
}
double CongestionMap::history_at(Point p) const {
    return history_[cell_of_y(p.y) * n_ + cell_of_x(p.x)];
}

std::vector<Hotspot> CongestionMap::hotspots(int top_k) const {
    struct Cell {
        double score;
        int idx;
    };
    std::vector<Cell> cells;
    cells.reserve(n_ * n_);
    for (int i = 0; i < n_ * n_; ++i) {
        double s = present_[i] + 3.0 * history_[i];
        if (s > 0) cells.push_back({s, i});
    }
    std::sort(cells.begin(), cells.end(),
              [](const Cell& a, const Cell& b) { return a.score > b.score; });
    std::vector<Hotspot> out;
    for (int i = 0; i < (int)cells.size() && i < top_k; ++i) {
        int cx = cells[i].idx % n_, cy = cells[i].idx / n_;
        Hotspot h;
        h.x_mm = nm_to_mm(bounds_.x1 + cx * cell_w_ + cell_w_ / 2);
        h.y_mm = nm_to_mm(bounds_.y1 + cy * cell_h_ + cell_h_ / 2);
        h.present = present_[cells[i].idx];
        h.history = history_[cells[i].idx];
        out.push_back(h);
    }
    return out;
}

// ---- Reservations ----

void ReservationSet::build(const std::vector<ConnectionTask>& tasks,
                           const std::vector<Corridor>& corridors,
                           const std::vector<double>& difficulties) {
    corridors_ = corridors;
    weights_.resize(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i)
        weights_[i] = 0.5 + (i < difficulties.size() ? difficulties[i] / 8.0 : 0.0);
}

Coord ReservationSet::penalty_for_segment(std::size_t self_task, const Segment& seg) const {
    Rect r = seg.bounds();
    double acc = 0.0;
    for (std::size_t i = 0; i < corridors_.size(); ++i) {
        if (i == self_task) continue;
        if (corridors_[i].rect.intersects(r)) acc += weights_[i];
    }
    Coord p = static_cast<Coord>(acc * 20000.0);
    return std::min<Coord>(p, 800000);
}

// ---- Candidate generation (worker body) ----

CandidateRoute route_candidate_task(const Board& snapshot, const RuleResolver& resolver,
                                    const ConnectionTask& task, std::size_t task_index,
                                    double difficulty, const ElectricalContext& ctx,
                                    const std::vector<double>& layer_mult,
                                    const AStarConfig& astar_cfg, const CongestionMap& congestion,
                                    const ReservationSet& reservations) {
    CandidateRoute cand;
    cand.task = task;
    cand.task_index = task_index;
    cand.difficulty = difficulty;
    const Terminal* ta = snapshot.find_terminal(task.a);
    const Terminal* tb = snapshot.find_terminal(task.b);
    if (!ta || !tb) {
        cand.fail_reason = "bad_task";
        return cand;
    }
    cand.gate_a = ta->pos;
    cand.gate_b = tb->pos;
    TraceRule rule = resolver.traceRule(task.net, ta->layer, kAnyRegion);
    Coord width = rule.pref_width_nm;

    SparseRoutingGraph graph = SparseRoutingGraph::build(snapshot, resolver, task.net, ta->pos,
                                                         tb->pos, ta->layer, tb->layer, width, ctx);
    // Soft costs only: congestion + reservations bias the search, legality is
    // structural (illegal edges were never built).
    graph.add_penalties([&](const SparseNode& n, const SparseEdge& e) -> Coord {
        Coord p = 0;
        if (e.is_via) {
            p += congestion.penalty_for_segment(Segment{n.p, n.p});
            p += reservations.penalty_for_segment(task_index, Segment{n.p, n.p});
        } else if (e.dir2 >= 0) {
            Segment s1{n.p, e.elbow}, s2{e.elbow, graph.nodes()[e.to].p};
            p += congestion.penalty_for_segment(s1) + congestion.penalty_for_segment(s2);
            p += reservations.penalty_for_segment(task_index, s1) +
                 reservations.penalty_for_segment(task_index, s2);
        } else {
            Segment s{n.p, graph.nodes()[e.to].p};
            p += congestion.penalty_for_segment(s);
            p += reservations.penalty_for_segment(task_index, s);
        }
        return p;
    });

    AStarResult res = astar_route(graph, layer_mult, astar_cfg);
    cand.expansions = res.expansions;
    cand.closest_node = res.closest_node;
    cand.closest_goal_dist_nm = res.closest_goal_dist_nm;
    // Baseline via diagnostics from the ordered selection (no geometry).
    {
        const NetInfo* ninfo = snapshot.find_net(task.net);
        bool dummy = false;
        cand.required_current_a =
            ninfo ? resolver.current().effective_current(*ninfo, snapshot.defaults, dummy)
                  : 0.0;
        auto ordered = ViaBundlePlanner::ordered_styles(
            resolver, task.net,
            LayerSpan{snapshot.layers.front().id, snapshot.layers.back().id});
        if (!ordered.empty()) {
            cand.via_style = ordered.front().name;
            cand.vias_required = ViaBundlePlanner::required_count(resolver, ordered.front(),
                                                                  task.net);
        } else {
            cand.via_style.clear();
            cand.vias_required = 1;
            cand.via_reason = "no_via_class";
        }
    }
    if (!res.found) {
        cand.fail_reason = res.fail_reason == "budget_exhausted" ? "budget_exhausted" : "unreachable";
        // Explicit bundle infeasibility (issue #5): a layer-changing task on
        // a graph with zero via edges can never transition. Distinguish a
        // blocked parallel bundle from a plain maze failure so agents get a
        // stable reason category.
        if (ta->layer != tb->layer) {
            int via_edges = 0;
            for (std::size_t ni = 0; ni < graph.nodes().size(); ++ni)
                for (const auto& e : graph.edges(static_cast<int>(ni)))
                    if (e.is_via) ++via_edges;
            if (via_edges == 0) {
                LayerSpan span{ta->layer, tb->layer};
                ViaBundle probe = ViaBundlePlanner::plan(snapshot, resolver, task.net,
                                                         ta->pos, span, width, ctx);
                cand.via_style = probe.via_class;
                cand.vias_required = std::max(1, probe.count);
                cand.required_current_a = probe.required_current_a;
                if (!probe.feasible) {
                    cand.via_reason = probe.reason;
                    cand.fail_reason = probe.reason == "no_via_class"
                                           ? "no_via"
                                           : "via_bundle_infeasible";
                }
            }
        }
        return cand;
    }
    cand.found = true;
    cand.cost_nm = res.cost_nm;
    // Materialize every A* layer transition as one atomic bundle (issue #5):
    // all barrels plus both layers' star stubs, or the whole candidate
    // fails with an explicit reason. Never half-build a transition.
    bool saw_via = false;
    for (std::size_t i = 0; i < res.edge_path.size(); ++i) {
        int u = res.node_path[i];
        const SparseEdge& e = graph.edges(u)[res.edge_path[i]];
        const SparseNode& nu = graph.nodes()[u];
        const SparseNode& nv = graph.nodes()[e.to];
        if (e.is_via) {
            LayerSpan span{nu.layer, nv.layer};
            ViaBundle bundle = ViaBundlePlanner::plan(snapshot, resolver, task.net, nu.p,
                                                      span, width, ctx);
            if (!bundle.feasible) {
                cand.found = false;
                cand.traces.clear();
                cand.vias.clear();
                cand.via_style = bundle.via_class;
                cand.vias_required = std::max(1, bundle.count);
                cand.required_current_a = bundle.required_current_a;
                cand.via_reason = bundle.reason;
                cand.fail_reason =
                    bundle.reason == "no_via_class" ? "no_via" : "via_bundle_infeasible";
                return cand;
            }
            if (!saw_via) {
                cand.via_style = bundle.style.name;
                cand.vias_required = bundle.count;
                cand.via_reason = "ok";
                cand.required_current_a = bundle.required_current_a;
                saw_via = true;
            } else {
                cand.vias_required = std::max(cand.vias_required, bundle.count);
            }
            for (auto p : bundle.positions) {
                Via v;
                v.net = task.net;
                v.pos = p;
                v.top_layer = std::min(nu.layer, nv.layer);
                v.bottom_layer = std::max(nu.layer, nv.layer);
                v.outer_d_nm = bundle.style.outer_nm;
                v.hole_d_nm = bundle.style.hole_nm;
                v.via_class = bundle.style.name;
                cand.vias.push_back(v);
            }
            for (const auto& s : bundle.stubs) cand.traces.push_back(s);
        } else if (e.dir2 >= 0) {
            cand.traces.push_back({task.net, nu.layer, nu.p, e.elbow, width});
            if (!(e.elbow == nv.p)) cand.traces.push_back({task.net, nu.layer, e.elbow, nv.p, width});
        } else {
            if (!(nu.p == nv.p)) cand.traces.push_back({task.net, nu.layer, nu.p, nv.p, width});
        }
    }
    if (!saw_via) cand.via_reason = "ok";
    return cand;
}

// ---- Legality + conflicts (arbiter) ----

namespace {

// All committed copper plus already-accepted candidates, read-only.
struct LegalView {
    const Board& board;
    const std::vector<TraceSeg>* extra_traces = nullptr;
    const std::vector<Via>* extra_vias = nullptr;
};

bool trace_legal(const TraceSeg& s, const LegalView& v, const RuleResolver& resolver,
                 const ElectricalContext& ctx, std::string& why) {
    Coord hw = s.width_nm / 2;
    if (!v.board.bounds().contains(s.segment().bounds().expanded(hw))) {
        why = "off_board";
        return false;
    }
    std::string cs;
    Coord max_clear = max_clear_for(v.board, resolver, s.net, ctx);
    auto check_pad = [&](const Terminal& t) -> bool {
        if (t.net == s.net || t.layer != s.layer) return true;
        Coord c = resolver.requiredClearance(s.net, t.net, s.layer, ctx, &cs);
        return seg_ok_rect(s.segment(), t.pad_rect(), c + hw);
    };
    auto check_trace = [&](const TraceSeg& t) -> bool {
        if (t.net == s.net || t.layer != s.layer) return true;
        Coord c = resolver.requiredClearance(s.net, t.net, s.layer, ctx, &cs);
        Segment a = s.segment(), b = t.segment();
        if (!a.bounds().expanded(c + hw + t.width_nm / 2).intersects(b.bounds())) return true;
        __int128 d2 = seg_seg_dist2(a, b);
        Coord need = c + hw + t.width_nm / 2;
        return d2 >= (__int128)need * need;
    };
    auto check_via = [&](const Via& vv) -> bool {
        if (vv.net == s.net) return true;
        if (s.layer < std::min(vv.top_layer, vv.bottom_layer) ||
            s.layer > std::max(vv.top_layer, vv.bottom_layer))
            return true;
        Coord c = resolver.requiredClearance(s.net, vv.net, s.layer, ctx, &cs);
        Rect vr = Rect::from_center_size(vv.pos, vv.outer_d_nm, vv.outer_d_nm);
        return seg_ok_rect(s.segment(), vr, c + hw);
    };
    for (const auto& ko : v.board.keepouts) {
        if (ko.layer != kAllLayers && ko.layer != s.layer) continue;
        if (!seg_ok_rect(s.segment(), ko.rect, max_clear + hw)) {
            why = "keepout:" + ko.reason;
            return false;
        }
    }
    for (const auto& t : v.board.terminals)
        if (!check_pad(t)) {
            why = "clearance:pad";
            return false;
        }
    for (const auto& t : v.board.traces)
        if (!check_trace(t)) {
            why = "clearance:trace";
            return false;
        }
    for (const auto& vv : v.board.vias)
        if (!check_via(vv)) {
            why = "clearance:via";
            return false;
        }
    if (v.extra_traces)
        for (const auto& t : *v.extra_traces)
            if (!check_trace(t)) {
                why = "clearance:trace";
                return false;
            }
    if (v.extra_vias)
        for (const auto& vv : *v.extra_vias)
            if (!check_via(vv)) {
                why = "clearance:via";
                return false;
            }
    return true;
}

bool via_legal(const Via& vv, const LegalView& v, const RuleResolver& resolver,
               const ElectricalContext& ctx, std::string& why) {
    Rect vr = Rect::from_center_size(vv.pos, vv.outer_d_nm, vv.outer_d_nm);
    if (!v.board.bounds().contains(vr)) {
        why = "off_board";
        return false;
    }
    std::string cs;
    Coord max_clear = max_clear_for(v.board, resolver, vv.net, ctx);
    auto check_vs_net = [&](NetId other, const Rect& raw) -> bool {
        Coord c = resolver.requiredClearance(vv.net, other, vv.top_layer, ctx, &cs);
        return gap_ok_rect(vr, raw, c);
    };
    for (const auto& ko : v.board.keepouts) {
        bool span_hit = ko.layer == kAllLayers ||
                        (ko.layer >= std::min(vv.top_layer, vv.bottom_layer) &&
                         ko.layer <= std::max(vv.top_layer, vv.bottom_layer));
        if (!span_hit) continue;
        if (!gap_ok_rect(vr, ko.rect, max_clear)) {
            why = "keepout:" + ko.reason;
            return false;
        }
    }
    for (const auto& t : v.board.terminals) {
        if (t.net == vv.net) continue;
        bool span_hit =
            t.layer >= std::min(vv.top_layer, vv.bottom_layer) &&
            t.layer <= std::max(vv.top_layer, vv.bottom_layer);
        if (!span_hit) continue;
        if (!check_vs_net(t.net, t.pad_rect())) {
            why = "clearance:pad";
            return false;
        }
    }
    auto check_trace = [&](const TraceSeg& t) -> bool {
        if (t.net == vv.net) return true;
        if (t.layer < std::min(vv.top_layer, vv.bottom_layer) ||
            t.layer > std::max(vv.top_layer, vv.bottom_layer))
            return true;
        Coord c = resolver.requiredClearance(vv.net, t.net, t.layer, ctx, &cs);
        __int128 d2 = seg_rect_dist2(t.segment(), vr);
        Coord need = c + t.width_nm / 2;
        return d2 >= (__int128)need * need;
    };
    for (const auto& t : v.board.traces)
        if (!check_trace(t)) {
            why = "clearance:trace";
            return false;
        }
    if (v.extra_traces)
        for (const auto& t : *v.extra_traces)
            if (!check_trace(t)) {
                why = "clearance:trace";
                return false;
            }
    auto check_via = [&](const Via& o) -> bool {
        if (o.net == vv.net) return true;
        bool overlap = !(o.bottom_layer < std::min(vv.top_layer, vv.bottom_layer) ||
                         o.top_layer > std::max(vv.top_layer, vv.bottom_layer));
        if (!overlap) return true;
        Rect orr = Rect::from_center_size(o.pos, o.outer_d_nm, o.outer_d_nm);
        return check_vs_net(o.net, orr);
    };
    for (const auto& o : v.board.vias)
        if (!check_via(o)) {
            why = "clearance:via";
            return false;
        }
    if (v.extra_vias)
        for (const auto& o : *v.extra_vias)
            if (!check_via(o)) {
                why = "clearance:via";
                return false;
            }
    return true;
}

Rect candidate_bounds(const CandidateRoute& c) {
    if (c.traces.empty() && c.vias.empty()) return {c.gate_a.x, c.gate_a.y, c.gate_a.x, c.gate_a.y};
    Rect r{};
    bool first = true;
    auto grow = [&](const Rect& q) {
        if (first) {
            r = q;
            first = false;
        } else {
            r = {std::min(r.x1, q.x1), std::min(r.y1, q.y1), std::max(r.x2, q.x2),
                 std::max(r.y2, q.y2)};
        }
    };
    for (const auto& t : c.traces) grow(t.segment().bounds().expanded(t.width_nm / 2));
    for (const auto& vv : c.vias)
        grow(Rect::from_center_size(vv.pos, vv.outer_d_nm, vv.outer_d_nm));
    return r;
}

}  // namespace

bool candidate_legal_vs_board(const CandidateRoute& cand, const Board& committed,
                              const RuleResolver& resolver, const ElectricalContext& ctx,
                              std::string& reason_out) {
    LegalView v{committed, nullptr, nullptr};
    for (const auto& t : cand.traces)
        if (!trace_legal(t, v, resolver, ctx, reason_out)) return false;
    for (const auto& vv : cand.vias)
        if (!via_legal(vv, v, resolver, ctx, reason_out)) return false;
    return true;
}

bool candidates_conflict(const CandidateRoute& a, const CandidateRoute& b,
                         const RuleResolver& resolver, const ElectricalContext& ctx) {
    if (!a.found || !b.found || a.task.net == b.task.net) return false;
    // Bounds pre-check with the pair clearance.
    std::string cs;
    Coord c = resolver.requiredClearance(a.task.net, b.task.net, 0, ctx, &cs);
    Coord wa = 0, wb = 0;
    for (const auto& t : a.traces) wa = std::max(wa, t.width_nm / 2);
    for (const auto& t : b.traces) wb = std::max(wb, t.width_nm / 2);
    if (!candidate_bounds(a).expanded(c + wa + wb).intersects(candidate_bounds(b))) return false;
    for (const auto& ta : a.traces) {
        for (const auto& tb : b.traces) {
            if (ta.layer != tb.layer) continue;
            Segment sa = ta.segment(), sb = tb.segment();
            Coord need = c + ta.width_nm / 2 + tb.width_nm / 2;
            if (!sa.bounds().expanded(need).intersects(sb.bounds())) continue;
            if (seg_seg_dist2(sa, sb) < (__int128)need * need) return true;
        }
        for (const auto& vb : b.vias) {
            if (ta.layer < std::min(vb.top_layer, vb.bottom_layer) ||
                ta.layer > std::max(vb.top_layer, vb.bottom_layer))
                continue;
            Rect vr = Rect::from_center_size(vb.pos, vb.outer_d_nm, vb.outer_d_nm);
            if (!seg_ok_rect(ta.segment(), vr, c + ta.width_nm / 2)) return true;
        }
    }
    for (const auto& va : a.vias) {
        Rect vra = Rect::from_center_size(va.pos, va.outer_d_nm, va.outer_d_nm);
        for (const auto& tb : b.traces) {
            if (tb.layer < std::min(va.top_layer, va.bottom_layer) ||
                tb.layer > std::max(va.top_layer, va.bottom_layer))
                continue;
            if (!seg_ok_rect(tb.segment(), vra, c + tb.width_nm / 2)) return true;
        }
        for (const auto& vb : b.vias) {
            bool overlap = !(vb.bottom_layer < std::min(va.top_layer, va.bottom_layer) ||
                             vb.top_layer > std::max(va.top_layer, va.bottom_layer));
            if (!overlap) continue;
            Rect vr = Rect::from_center_size(vb.pos, vb.outer_d_nm, vb.outer_d_nm);
            if (!gap_ok_rect(vra, vr, c)) return true;
        }
    }
    return false;
}

ArbiterResult arbitrate(const std::vector<CandidateRoute>& candidates, const Board& committed,
                        const RuleResolver& resolver, const ElectricalContext& ctx) {
    ArbiterResult out;
    std::vector<std::size_t> order(candidates.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    // Deterministic selection order: difficulty desc, then (net, a, b).
    // Completion order of workers can never change this.
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        if (candidates[a].difficulty != candidates[b].difficulty)
            return candidates[a].difficulty > candidates[b].difficulty;
        const auto& ta = candidates[a].task;
        const auto& tb = candidates[b].task;
        if (ta.net != tb.net) return ta.net < tb.net;
        if (ta.a != tb.a) return ta.a < tb.a;
        return ta.b < tb.b;
    });
    std::vector<TraceSeg> acc_traces;
    std::vector<Via> acc_vias;
    std::vector<std::size_t> acc_idx;
    LegalView v{committed, &acc_traces, &acc_vias};
    for (std::size_t i : order) {
        const CandidateRoute& c = candidates[i];
        if (!c.found) {
            out.rejected.push_back(i);
            out.reject_reason.push_back(c.fail_reason.empty() ? "unreachable" : c.fail_reason);
            continue;
        }
        bool conflict = false;
        for (std::size_t j : acc_idx) {
            if (candidates_conflict(c, candidates[j], resolver, ctx)) {
                conflict = true;
                break;
            }
        }
        if (conflict) {
            out.rejected.push_back(i);
            out.reject_reason.push_back("conflict");
            continue;
        }
        std::string why;
        bool ok = true;
        for (const auto& t : c.traces)
            if (!trace_legal(t, v, resolver, ctx, why)) {
                ok = false;
                break;
            }
        if (ok)
            for (const auto& vv : c.vias)
                if (!via_legal(vv, v, resolver, ctx, why)) {
                    ok = false;
                    break;
                }
        if (!ok) {
            out.rejected.push_back(i);
            out.reject_reason.push_back("illegal_overlap:" + why);
            continue;
        }
        out.accepted.push_back(i);
        acc_idx.push_back(i);
        acc_traces.insert(acc_traces.end(), c.traces.begin(), c.traces.end());
        acc_vias.insert(acc_vias.end(), c.vias.begin(), c.vias.end());
    }
    return out;
}

void commit_candidates(Board& committed, const std::vector<CandidateRoute>& candidates,
                       const ArbiterResult& arb, RouteStats& stats) {
    for (std::size_t i : arb.accepted) {
        const CandidateRoute& c = candidates[i];
        for (const auto& t : c.traces) {
            stats.length_nm += manhattan(t.a, t.b);
            committed.traces.push_back(t);
        }
        for (const auto& vv : c.vias) {
            committed.vias.push_back(vv);
            stats.via_count++;
        }
        stats.tasks_routed++;
        stats.expansions_total += c.expansions;
    }
    // Rejected candidates still consumed search work: count it.
    for (std::size_t i : arb.rejected) stats.expansions_total += candidates[i].expansions;
}

JsonValue EpochInfo::to_json() const {
    JsonValue o = JsonValue::object();
    o["epoch"] = static_cast<double>(epoch);
    o["batch_size"] = static_cast<double>(batch_size);
    o["candidates"] = static_cast<double>(candidates);
    o["accepted"] = static_cast<double>(accepted);
    o["rejected"] = static_cast<double>(rejected);
    o["expansions"] = static_cast<double>(expansions);
    o["workers"] = static_cast<double>(workers);
    o["time_ms"] = static_cast<double>(time_ms);
    return o;
}

std::string geometry_hash(const Board& board) {
    // FNV-1a/64 over committed copper in commit order (deterministic).
    std::uint64_t h = 1469598103934665603ULL;
    auto mix = [&](const void* p, std::size_t n) {
        const auto* b = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) {
            h ^= b[i];
            h *= 1099511628211ULL;
        }
    };
    auto mix64 = [&](std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h ^= static_cast<unsigned char>(v & 0xff);
            h *= 1099511628211ULL;
            v >>= 8;
        }
    };
    for (const auto& t : board.traces) {
        mix64(static_cast<std::uint64_t>(t.net));
        mix64(static_cast<std::uint64_t>(t.layer));
        mix64(static_cast<std::uint64_t>(t.a.x));
        mix64(static_cast<std::uint64_t>(t.a.y));
        mix64(static_cast<std::uint64_t>(t.b.x));
        mix64(static_cast<std::uint64_t>(t.b.y));
        mix64(static_cast<std::uint64_t>(t.width_nm));
    }
    mix("V", 1);
    for (const auto& v : board.vias) {
        mix64(static_cast<std::uint64_t>(v.net));
        mix64(static_cast<std::uint64_t>(v.pos.x));
        mix64(static_cast<std::uint64_t>(v.pos.y));
        mix64(static_cast<std::uint64_t>(v.top_layer));
        mix64(static_cast<std::uint64_t>(v.bottom_layer));
        mix64(static_cast<std::uint64_t>(v.outer_d_nm));
        mix64(static_cast<std::uint64_t>(v.hole_d_nm));
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", (unsigned long long)h);
    return std::string(buf);
}

}  // namespace copperline
