#include "router/portfolio.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "router/route_tree.h"
#include "router/simplify.h"
#include "router/via_bundle.h"

namespace copperline {

namespace {

constexpr Coord kSigCellNm = 2000000;  // 2mm quantization for dedup
constexpr Coord kDirRunMinNm = 1000000;  // 1mm principal runs only
constexpr Coord kDeviatePenaltyNm = 10000000;  // 10mm bottleneck penalty

int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

std::string layers_key(const std::vector<LayerId>& layers) {
    if (layers.empty()) return "L?";
    std::string s;
    for (std::size_t i = 0; i < layers.size(); ++i) {
        if (i) s += "+";
        s += "L" + std::to_string(layers[i]);
    }
    return s;
}

Coord max_clear_for_task(const Board& board, const RuleResolver& resolver, NetId net,
                         const ElectricalContext& ctx) {
    Coord m = 0;
    std::string cs;
    for (const auto& o : board.nets) {
        if (o.id == net) continue;
        m = std::max(m, resolver.requiredClearance(net, o.id, 0, ctx, &cs));
    }
    return m;
}

std::vector<double> eff_mult_for(const Board& snapshot, const RuleResolver& resolver,
                                 const ConnectionTask& task,
                                 const std::vector<double>& base) {
    const NetInfo* n = snapshot.find_net(task.net);
    if (!n || !resolver.impedance().has_target(*n)) return base;
    std::vector<double> mult = base;
    for (const auto& l : snapshot.layers) {
        double b = resolver.impedanceLayerMultiplier(task.net, l.id);
        if (l.id >= 0 && l.id < static_cast<int>(mult.size())) mult[l.id] *= b;
    }
    return mult;
}

// Materialize an A* result into exact copper (ordinary tasks only).
// Mirrors route_candidate_task's non-pair path: atomic via bundles,
// per-layer impedance shrink, #17 simplification. Returns false with reason
// when a bundle is infeasible.
bool materialize_ordinary(const Board& snapshot, const RuleResolver& resolver,
                          const ConnectionTask& task, const SparseRoutingGraph& graph,
                          const AStarResult& res, Coord width, bool imp_active,
                          const ElectricalContext& ctx, CandidateRoute& out) {
    out.traces.clear();
    out.vias.clear();
    bool saw_via = false;
    std::vector<TraceSeg> route_segs;
    std::vector<TraceSeg> stub_segs;
    for (std::size_t i = 0; i < res.edge_path.size(); ++i) {
        int u = res.node_path[i];
        const SparseEdge& e = graph.edges(u)[res.edge_path[i]];
        const SparseNode& nu = graph.nodes()[u];
        const SparseNode& nv = graph.nodes()[e.to];
        if (e.is_via) {
            LayerSpan span{nu.layer, nv.layer};
            ViaBundle bundle =
                ViaBundlePlanner::plan(snapshot, resolver, task.net, nu.p, span, width, ctx);
            if (!bundle.feasible) {
                out.found = false;
                out.traces.clear();
                out.vias.clear();
                out.via_style = bundle.via_class;
                out.vias_required = std::max(1, bundle.count);
                out.required_current_a = bundle.required_current_a;
                out.via_reason = bundle.reason;
                out.fail_reason =
                    bundle.reason == "no_via_class" ? "no_via" : "via_bundle_infeasible";
                return false;
            }
            if (!saw_via) {
                out.via_style = bundle.style.name;
                out.vias_required = bundle.count;
                out.via_reason = "ok";
                out.required_current_a = bundle.required_current_a;
                saw_via = true;
            } else {
                out.vias_required = std::max(out.vias_required, bundle.count);
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
                out.vias.push_back(v);
            }
            for (const auto& s : bundle.stubs) stub_segs.push_back(s);
        } else if (e.dir2 >= 0) {
            route_segs.push_back({task.net, nu.layer, nu.p, e.elbow, width});
            if (!(e.elbow == nv.p))
                route_segs.push_back({task.net, nu.layer, e.elbow, nv.p, width});
        } else {
            if (!(nu.p == nv.p)) route_segs.push_back({task.net, nu.layer, nu.p, nv.p, width});
        }
    }
    if (!saw_via) out.via_reason = "ok";
    if (imp_active) {
        for (auto& s : route_segs)
            s.width_nm = resolver.requiredTraceWidth(task.net, s.layer, ctx);
    }
    out.traces = route_segs;
    std::vector<char> stub_mask(route_segs.size(), 0);
    out.traces.insert(out.traces.end(), stub_segs.begin(), stub_segs.end());
    stub_mask.resize(out.traces.size(), 0);
    for (std::size_t i = route_segs.size(); i < out.traces.size(); ++i) stub_mask[i] = 1;
    simplify_candidate_traces(snapshot, resolver, ctx, task.net, out.traces, stub_mask,
                              simplify_exempt_task(task));
    out.found = true;
    return true;
}

}  // namespace

int clamp_portfolio_k(int requested_k, int max_k) {
    if (max_k < 1) max_k = 1;
    if (max_k > 64) max_k = 64;
    if (requested_k < 1) requested_k = 1;
    if (requested_k > max_k) requested_k = max_k;
    return requested_k;
}

int effective_portfolio_k(int requested_k, int max_k, int budget_route_k,
                          std::size_t memory_budget_bytes, int batch_width,
                          std::size_t per_task_bytes, std::size_t per_alt_bytes) {
    int k = clamp_portfolio_k(requested_k, max_k);
    if (budget_route_k >= 1 && budget_route_k < k) k = budget_route_k;
    if (memory_budget_bytes == 0) return k;
    if (batch_width < 1) batch_width = 1;
    if (per_task_bytes == 0) per_task_bytes = kPerCandidateBytes;
    if (per_alt_bytes == 0) per_alt_bytes = kPerPortfolioAltBytes;
    // K x batch bound: each batch task holds one shared graph + K streamed
    // alternatives (at most K stored, ≤15, so streaming == stored here).
    // share = floor(budget / batch); alternatives fit in (share - graph).
    std::size_t share = memory_budget_bytes / static_cast<std::size_t>(batch_width);
    if (share <= per_task_bytes) return 1;
    std::size_t room = share - per_task_bytes;
    std::size_t fit = room / per_alt_bytes + 1;  // base alternative is free-ish
    if (fit < 1) fit = 1;
    if (static_cast<std::size_t>(k) > fit) k = static_cast<int>(fit);
    if (k < 1) k = 1;
    return k;
}

std::string RouteSignature::to_string() const {
    return corridor_class + "|" + dir_seq + "|" + first_via + "|" + bottlenecks;
}

JsonValue RouteSignature::to_json() const {
    JsonValue o = JsonValue::object();
    o["corridor_class"] = corridor_class;
    o["principal_directions"] = dir_seq;
    o["first_via"] = first_via;
    o["bottlenecks"] = bottlenecks;
    o["signature"] = to_string();
    return o;
}

JsonValue PortfolioResult::to_json() const {
    JsonValue o = JsonValue::object();
    o["schema"] = "copperline/route-portfolio/1";
    o["requested_k"] = static_cast<double>(requested_k);
    o["effective_k"] = static_cast<double>(effective_k);
    o["threads_effective"] = static_cast<double>(threads_effective);
    o["size"] = static_cast<double>(candidates.size());
    o["graph_builds"] = static_cast<double>(graph_builds);
    o["total_expansions"] = static_cast<double>(total_expansions);
    o["coarse_expansions"] = static_cast<double>(coarse_expansions);
    if (!fail_reason.empty()) o["fail_reason"] = fail_reason;
    JsonValue arr = JsonValue::array();
    for (const auto& c : candidates) {
        JsonValue e = JsonValue::object();
        e["signature"] = c.sig_str;
        e["corridor_class"] = c.sig.corridor_class;
        e["principal_directions"] = c.sig.dir_seq;
        e["first_via"] = c.sig.first_via;
        e["bottlenecks"] = c.sig.bottlenecks;
        e["cost_nm"] = static_cast<double>(c.route.cost_nm);
        json_add_mm(e, "cost_mm", nm_to_mm(c.route.cost_nm));
        // Issue #8: expose both costs (cost_nm == materialized for ordering).
        e["search_cost_nm"] = static_cast<double>(c.route.search_cost_nm);
        e["materialized_cost_nm"] = static_cast<double>(c.route.materialized_cost_nm);
        e["expansions"] = static_cast<double>(c.route.expansions);
        e["vias"] = static_cast<double>(c.route.vias.size());
        e["traces"] = static_cast<double>(c.route.traces.size());
        arr.as_array().push_back(e);
    }
    o["candidates"] = arr;
    return o;
}

RouteSignature compute_route_signature(const SparseRoutingGraph& graph,
                                       const AStarResult& res) {
    RouteSignature sig;
    if (!res.found || res.node_path.empty()) {
        sig.corridor_class = "none";
        sig.dir_seq = "none";
        sig.first_via = "none";
        sig.bottlenecks = "none";
        return sig;
    }
    // Layers used.
    std::set<LayerId> layers;
    Coord x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    bool first = true;
    for (int nid : res.node_path) {
        const SparseNode& n = graph.nodes()[nid];
        layers.insert(n.layer);
        if (first) {
            x1 = x2 = n.p.x;
            y1 = y2 = n.p.y;
            first = false;
        } else {
            x1 = std::min(x1, n.p.x);
            y1 = std::min(y1, n.p.y);
            x2 = std::max(x2, n.p.x);
            y2 = std::max(y2, n.p.y);
        }
    }
    std::vector<LayerId> lv(layers.begin(), layers.end());
    auto q = [](Coord v) { return (v / kSigCellNm); };
    sig.corridor_class = std::to_string(q(x2 - x1)) + "x" + std::to_string(q(y2 - y1)) +
                         ":" + layers_key(lv);
    // Principal direction runs (>=1mm), compressed.
    std::vector<std::pair<int, Coord>> runs;
    for (std::size_t i = 1; i < res.node_path.size(); ++i) {
        const Point& a = graph.nodes()[res.node_path[i - 1]].p;
        const Point& b = graph.nodes()[res.node_path[i]].p;
        int d = direction_of(a, b);
        Coord len = manhattan(a, b);
        if (d == 4 || len <= 0) continue;
        if (!runs.empty() && runs.back().first == d) {
            runs.back().second += len;
        } else {
            runs.push_back({d, len});
        }
    }
    const char* names[5] = {"E", "N", "W", "S", "?"};
    std::string ds;
    for (auto& r : runs) {
        if (r.second < kDirRunMinNm) continue;  // tiny jogs are not distinct
        if (!ds.empty()) ds += "-";
        ds += names[r.first >= 0 && r.first < 4 ? r.first : 4];
    }
    sig.dir_seq = ds.empty() ? "via-only" : ds;
    // First via quantized to early/mid/late thirds.
    std::string fv = "none";
    for (std::size_t i = 0; i < res.edge_path.size(); ++i) {
        int u = res.node_path[i];
        const SparseEdge& e = graph.edges(u)[res.edge_path[i]];
        if (e.is_via) {
            const SparseNode& nu = graph.nodes()[u];
            const SparseNode& nv = graph.nodes()[e.to];
            double frac =
                res.edge_path.size() > 0 ? static_cast<double>(i) / res.edge_path.size() : 0;
            const char* third = frac < 1.0 / 3.0 ? "early" : (frac < 2.0 / 3.0 ? "mid" : "late");
            fv = "L" + std::to_string(nu.layer) + "->L" + std::to_string(nv.layer) + "@" +
                 third;
            break;
        }
    }
    sig.first_via = fv;
    // Bottleneck cells: via nodes + middle node, quantized to 2mm.
    std::set<std::string> cells;
    auto cell_of = [](Point p, LayerId l) {
        return "c(" + std::to_string(p.x / kSigCellNm) + "," +
               std::to_string(p.y / kSigCellNm) + ",L" + std::to_string(l) + ")";
    };
    for (std::size_t i = 0; i < res.edge_path.size(); ++i) {
        int u = res.node_path[i];
        const SparseEdge& e = graph.edges(u)[res.edge_path[i]];
        if (e.is_via) cells.insert(cell_of(graph.nodes()[u].p, graph.nodes()[u].layer));
    }
    {
        int mid = res.node_path[res.node_path.size() / 2];
        cells.insert(cell_of(graph.nodes()[mid].p, graph.nodes()[mid].layer));
    }
    std::string bn;
    for (const auto& c : cells) {
        if (!bn.empty()) bn += ",";
        bn += c;
    }
    sig.bottlenecks = bn.empty() ? "none" : bn;
    return sig;
}

// Issue #9: copper-based signature. All fields derive from the final
// materialized traces + via positions (post-simplification), never from
// the A* node path (whose direction_of(a,b) collapses L elbows to one
// cardinal and whose bbox/vias predate bundles + simplification).
namespace {

// 8-way direction of a copper segment (8 = degenerate point).
int copper_dir8(Point a, Point b) {
    Coord dx = b.x - a.x;
    Coord dy = b.y - a.y;
    if (dx == 0 && dy == 0) return 8;
    int sx = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
    int sy = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
    if (sx > 0 && sy == 0) return 0;  // E
    if (sx > 0 && sy > 0) return 1;   // NE
    if (sx == 0 && sy > 0) return 2;  // N
    if (sx < 0 && sy > 0) return 3;   // NW
    if (sx < 0 && sy == 0) return 4;  // W
    if (sx < 0 && sy < 0) return 5;   // SW
    if (sx == 0 && sy < 0) return 6;  // S
    return 7;                         // SE
}

}  // namespace

RouteSignature compute_route_signature(const std::vector<TraceSeg>& traces,
                                       const std::vector<Via>& vias) {
    RouteSignature sig;
    if (traces.empty() && vias.empty()) {
        sig.corridor_class = "none";
        sig.dir_seq = "none";
        sig.first_via = "none";
        sig.bottlenecks = "none";
        return sig;
    }
    // Copper bbox from actual extents (trace width + via barrels).
    bool first = true;
    Coord x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    std::set<LayerId> layers;
    auto grow = [&](Coord ax1, Coord ay1, Coord ax2, Coord ay2) {
        if (first) {
            x1 = ax1;
            y1 = ay1;
            x2 = ax2;
            y2 = ay2;
            first = false;
        } else {
            x1 = std::min(x1, ax1);
            y1 = std::min(y1, ay1);
            x2 = std::max(x2, ax2);
            y2 = std::max(y2, ay2);
        }
    };
    for (const auto& t : traces) {
        layers.insert(t.layer);
        Coord hw = t.width_nm / 2;
        grow(std::min(t.a.x, t.b.x) - hw, std::min(t.a.y, t.b.y) - hw,
             std::max(t.a.x, t.b.x) + hw, std::max(t.a.y, t.b.y) + hw);
    }
    for (const auto& v : vias) {
        for (LayerId l = std::min(v.top_layer, v.bottom_layer);
             l <= std::max(v.top_layer, v.bottom_layer); ++l)
            layers.insert(l);
        Coord h = v.outer_d_nm / 2;
        grow(v.pos.x - h, v.pos.y - h, v.pos.x + h, v.pos.y + h);
    }
    std::vector<LayerId> lv(layers.begin(), layers.end());
    auto q = [](Coord v) { return (v / kSigCellNm); };
    sig.corridor_class = std::to_string(q(x2 - x1)) + "x" + std::to_string(q(y2 - y1)) +
                         ":" + layers_key(lv);
    // Principal direction runs from actual segment directions (8-way,
    // Euclidean lengths, >=1mm runs only).
    static const char* kDir8[9] = {"E", "NE", "N", "NW", "W", "SW", "S", "SE", "?"};
    std::vector<std::pair<int, Coord>> runs;
    for (const auto& t : traces) {
        int d = copper_dir8(t.a, t.b);
        Coord len = euclid_len_nm(t.a, t.b);
        if (d == 8 || len <= 0) continue;
        if (!runs.empty() && runs.back().first == d) {
            runs.back().second += len;
        } else {
            runs.push_back({d, len});
        }
    }
    std::string ds;
    for (auto& r : runs) {
        if (r.second < kDirRunMinNm) continue;
        if (!ds.empty()) ds += "-";
        ds += kDir8[r.first >= 0 && r.first < 8 ? r.first : 8];
    }
    sig.dir_seq = ds.empty() ? "via-only" : ds;
    // First via from actual via positions: copper-length fraction of the
    // nearest route segment midpoint (vias[] is already path-ordered; the
    // front is the first transition).
    std::string fv = "none";
    if (!vias.empty()) {
        const Via& v0 = vias.front();
        Coord total = 0;
        std::vector<Coord> seg_len;
        seg_len.reserve(traces.size());
        for (const auto& t : traces) {
            Coord l = euclid_len_nm(t.a, t.b);
            seg_len.push_back(l);
            total += l;
        }
        double frac = 0.0;
        if (!traces.empty() && total > 0) {
            // Nearest segment to the via position (exact integer dist2).
            std::size_t best_i = 0;
            Coord best_d2 = std::numeric_limits<Coord>::max();
            for (std::size_t i = 0; i < traces.size(); ++i) {
                Coord d2 = point_seg_dist2(v0.pos, Segment{traces[i].a, traces[i].b});
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_i = i;
                }
            }
            Coord before = 0;
            for (std::size_t i = 0; i < best_i; ++i) before += seg_len[i];
            // Midpoint of the owning segment as the along-route proxy.
            frac = static_cast<double>(before + seg_len[best_i] / 2) /
                   static_cast<double>(total);
        }
        const char* third = frac < 1.0 / 3.0 ? "early" : (frac < 2.0 / 3.0 ? "mid" : "late");
        fv = "L" + std::to_string(std::min(v0.top_layer, v0.bottom_layer)) + "->L" +
             std::to_string(std::max(v0.top_layer, v0.bottom_layer)) + "@" + third;
    }
    sig.first_via = fv;
    // Bottlenecks from actual copper: via cells + copper midpoint cell.
    std::set<std::string> cells;
    auto cell_of = [](Point p, LayerId l) {
        return "c(" + std::to_string(p.x / kSigCellNm) + "," +
               std::to_string(p.y / kSigCellNm) + ",L" + std::to_string(l) + ")";
    };
    for (const auto& v : vias)
        cells.insert(cell_of(v.pos, std::min(v.top_layer, v.bottom_layer)));
    if (!traces.empty()) {
        // Midpoint of the route by copper length.
        Coord total = 0;
        for (const auto& t : traces) total += euclid_len_nm(t.a, t.b);
        Coord half = total / 2;
        Coord acc = 0;
        Point mid = traces.front().a;
        LayerId mid_l = traces.front().layer;
        for (const auto& t : traces) {
            Coord l = euclid_len_nm(t.a, t.b);
            if (acc + l >= half) {
                double f = (l > 0) ? static_cast<double>(half - acc) / static_cast<double>(l)
                                   : 0.0;
                mid = {t.a.x + static_cast<Coord>((t.b.x - t.a.x) * f),
                       t.a.y + static_cast<Coord>((t.b.y - t.a.y) * f)};
                mid_l = t.layer;
                break;
            }
            acc += l;
            mid = t.b;
            mid_l = t.layer;
        }
        cells.insert(cell_of(mid, mid_l));
    } else if (!vias.empty()) {
        const Via& v0 = vias.front();
        cells.insert(cell_of(v0.pos, std::min(v0.top_layer, v0.bottom_layer)));
    }
    std::string bn2;
    for (const auto& c : cells) {
        if (!bn2.empty()) bn2 += ",";
        bn2 += c;
    }
    sig.bottlenecks = bn2.empty() ? "none" : bn2;
    return sig;
}

RouteSignature compute_route_signature(const CandidateRoute& cand) {
    return compute_route_signature(cand.traces, cand.vias);
}

PortfolioResult build_portfolio(const Board& snapshot, const RuleResolver& resolver,
                                const ConnectionTask& task, std::size_t task_index,
                                double difficulty, const ElectricalContext& ctx,
                                const std::vector<double>& layer_mult,
                                const AStarConfig& astar_cfg,
                                const CongestionMap& congestion,
                                const ReservationSet& reservations,
                                const HierarchyConfig& hier_cfg,
                                const HierarchyCache* hier_cache,
                                const PortfolioOptions& opts) {
    PortfolioResult out;
    out.requested_k = opts.requested_k;
    out.threads_effective = resolve_worker_threads(opts.threads_requested);
    int max_k = opts.max_k < 1 ? kPortfolioMaxK : opts.max_k;
    if (max_k > 64) max_k = 64;
    out.effective_k = effective_portfolio_k(opts.requested_k, max_k, opts.budget_route_k,
                                            opts.memory_budget_bytes, opts.batch_width,
                                            opts.per_task_bytes, opts.per_alt_bytes);
    if (out.effective_k < 1) out.effective_k = 1;

    const Terminal* ta = snapshot.find_terminal(task.a);
    const Terminal* tb = snapshot.find_terminal(task.b);
    if (!ta || !tb) {
        out.fail_reason = "bad_task";
        return out;
    }
    if (task.is_pair_corridor) {
        // Atomic pair path stays single: wrap the authoritative candidate.
        // Issue #4: forward the maturity graph budget when present.
        const SparseGraphBudget* pair_budget =
            opts.has_graph_budget ? &opts.graph_budget : nullptr;
        CandidateRoute single = route_candidate_task(
            snapshot, resolver, task, task_index, difficulty, ctx, layer_mult, astar_cfg,
            congestion, reservations, hier_cfg, hier_cache, pair_budget);
        out.total_expansions = single.expansions;
        out.coarse_expansions = single.hierarchy.coarse_expansions;
        out.graph_builds = 1;
        if (!single.found) {
            out.fail_reason =
                single.fail_reason.empty() ? "unreachable" : single.fail_reason;
            return out;
        }
        std::string why;
        if (!candidate_legal_vs_board(single, snapshot, resolver, ctx, why)) {
            out.fail_reason = "illegal_overlap:" + why;
            return out;
        }
        // Signature needs a graph; rebuild one lightweight view for the sig.
        // Cost: one extra build only on the pair fallback (rare path).
        std::vector<SparseTarget> dsts = {{task_dst_point(snapshot, task),
                                           task_dst_layer(snapshot, task)}};
        Point src_pt = task_src_point(snapshot, task);
        Coord w = 0;
        {
            std::string ws;
            w = resolver.requiredTraceWidth(task.net, ta->layer, ctx, &ws);
        }
        SparseRoutingGraph g = SparseRoutingGraph::build_multi(
            snapshot, resolver, task.net, src_pt, ta->layer, dsts, w, ctx);
        (void)g;
        PortfolioCandidate pc;
        pc.route = single;
        pc.sig.corridor_class = "pair";
        pc.sig.dir_seq = single.vias.empty() ? "signal" : "via";
        pc.sig.first_via = single.vias.empty() ? "none" : "pair-via";
        pc.sig.bottlenecks = "pair:" + std::to_string(task.pair_id);
        pc.sig_str = pc.sig.to_string();
        out.candidates.push_back(std::move(pc));
        return out;
    }

    // Impedance gate (mirrors the worker body): never route silently.
    const NetInfo* imp_net = snapshot.find_net(task.net);
    bool imp_active = imp_net && resolver.impedance().has_target(*imp_net);
    if (imp_active) {
        ImpedanceResolution ir = resolver.impedanceResolution(task.net, ctx);
        if (ir.has_target && !ir.feasible) {
            out.fail_reason = ir.conflict ? "impedance_current_conflict" : "impedance_infeasible";
            return out;
        }
    }
    TraceRule rule = resolver.traceRule(task.net, ta->layer, kAnyRegion);
    Coord width = imp_active ? resolver.maxRequiredWidth(task.net, ctx) : rule.pref_width_nm;

    // Multi-target set (mirrors worker body, ordinary tasks).
    std::vector<SparseTarget> dsts;
    LayerId primary_dst_layer = task_dst_layer(snapshot, task);
    Point src_pt = ta->pos;
    if (task.has_copper_target) {
        std::vector<TermId> target_comp;
        for (const auto& grp : net_terminal_components(snapshot, task.net)) {
            if (std::find(grp.begin(), grp.end(), task.b) != grp.end()) {
                target_comp = grp;
                break;
            }
        }
        if (target_comp.empty()) target_comp = {task.b};
        std::vector<CopperTarget> contacts =
            copper_contacts_for(snapshot, task.net, task.a, target_comp, 8);
        for (const auto& ct : contacts) dsts.push_back({ct.p, ct.layer});
        if (dsts.empty()) dsts.push_back({tb->pos, tb->layer});
        primary_dst_layer = dsts.front().layer;
    } else {
        dsts.push_back({tb->pos, tb->layer});
    }
    if (task.has_plane_target) {
        int pid = -1, island = 0;
        Point entry{};
        LayerId elayer = task.plane_layer;
        if (nearest_plane_target(snapshot, task.net, ta->pos, ta->layer, pid, entry, elayer,
                                 island)) {
            dsts.clear();
            dsts.push_back({entry, elayer});
            primary_dst_layer = elayer;
        }
    }
    (void)primary_dst_layer;

    std::vector<double> eff_mult = eff_mult_for(snapshot, resolver, task, layer_mult);

    // Hierarchy guidance once (bias + coarse accounting, shared by all K).
    GuidanceResult guide;
    bool guidance_on = hier_cfg.enabled && hier_cache != nullptr;
    std::vector<Point> guide_path;
    if (guidance_on) {
        HierarchyRequest req;
        req.net = task.net;
        req.src = src_pt;
        req.src_layer = ta->layer;
        req.dsts = dsts;
        req.width_nm = width;
        req.clearance_nm = max_clear_for_task(snapshot, resolver, task.net, ctx);
        req.layer_mult = eff_mult;
        req.astar_cfg = astar_cfg;
        req.soft_cost = [&](const Segment& s) -> Coord {
            return congestion.penalty_for_segment(s) +
                   reservations.penalty_for_segment(task_index, s);
        };
        guide = hier_cache->build_guidance(snapshot, resolver, ctx, req, hier_cfg);
        out.coarse_expansions = guide.coarse_expansions;
        if (guide.found && !guide.level_paths.empty()) guide_path = guide.level_paths.back();
    }

    // Build the ONE shared graph (reuse across K).
    // Issue #4: maturity graph budget flows in via opts; legacy 384/16
    // otherwise. The base miss below gets one last-resort rebuild
    // (uncapped bases, K=64) before the portfolio reports unreachable.
    const SparseGraphBudget pf_budget =
        opts.has_graph_budget ? opts.graph_budget : SparseGraphBudget::defaults();
    auto build_pf_graph = [&](const SparseGraphBudget& b) {
        SparseRoutingGraph g = SparseRoutingGraph::build_multi(
            snapshot, resolver, task.net, src_pt, ta->layer, dsts, width, ctx,
            nullptr, 0, b);
        g.add_penalties([&](const SparseNode& n, const SparseEdge& e) -> Coord {
            Coord p = 0;
            if (e.is_via) {
                p += congestion.penalty_for_segment(Segment{n.p, n.p});
                p += reservations.penalty_for_segment(task_index, Segment{n.p, n.p});
            } else if (e.dir2 >= 0) {
                Segment s1{n.p, e.elbow}, s2{e.elbow, g.nodes()[e.to].p};
                p += congestion.penalty_for_segment(s1) + congestion.penalty_for_segment(s2);
                p += reservations.penalty_for_segment(task_index, s1) +
                     reservations.penalty_for_segment(task_index, s2);
            } else {
                Segment s{n.p, g.nodes()[e.to].p};
                p += congestion.penalty_for_segment(s);
                p += reservations.penalty_for_segment(task_index, s);
            }
            return p;
        });
        return g;
    };
    SparseRoutingGraph base_graph = build_pf_graph(pf_budget);
    out.graph_builds = 1;

    auto pull_bias = [&](const SparseRoutingGraph& g) {
        std::vector<Coord> bias(g.nodes().size(), 0);
        if (guide_path.empty()) return bias;
        for (std::size_t i = 0; i < g.nodes().size(); ++i) {
            Coord best = std::numeric_limits<Coord>::max();
            for (const auto& p : guide_path) {
                Coord d = manhattan(g.nodes()[i].p, p);
                if (d < best) best = d;
            }
            if (best == std::numeric_limits<Coord>::max()) best = 0;
            bias[i] = std::min(hier_cfg.max_bias_nm, best / 4);
        }
        return bias;
    };
    std::vector<Coord> base_bias = pull_bias(base_graph);

    // Base path: normal A* with hierarchy bias.
    AStarResult base_res =
        astar_route_masked(base_graph, eff_mult, astar_cfg, {}, base_bias);
    out.total_expansions += base_res.expansions;
    if (!base_res.found && base_res.fail_reason != "budget_exhausted" &&
        !pf_budget.is_last_resort()) {
        // Issue #4 last resort: uncapped bases + K=64 before unreachable.
        SparseRoutingGraph lr_graph = build_pf_graph(SparseGraphBudget::last_resort());
        std::vector<Coord> lr_bias;
        lr_bias.reserve(lr_graph.nodes().size());
        {
            // Recompute the pull-to-path bias against the new node set.
            std::vector<Coord> b2(lr_graph.nodes().size(), 0);
            if (!guide_path.empty()) {
                for (std::size_t i = 0; i < lr_graph.nodes().size(); ++i) {
                    Coord best = std::numeric_limits<Coord>::max();
                    for (const auto& p : guide_path) {
                        Coord d = manhattan(lr_graph.nodes()[i].p, p);
                        if (d < best) best = d;
                    }
                    if (best == std::numeric_limits<Coord>::max()) best = 0;
                    b2[i] = std::min(hier_cfg.max_bias_nm, best / 4);
                }
            }
            lr_bias = std::move(b2);
        }
        AStarResult lr_res =
            astar_route_masked(lr_graph, eff_mult, astar_cfg, {}, lr_bias);
        out.total_expansions += lr_res.expansions;
        out.graph_builds += 1;
        if (lr_res.found || lr_res.fail_reason != "budget_exhausted") {
            base_graph = std::move(lr_graph);
            base_res = std::move(lr_res);
            base_bias = std::move(lr_bias);
        }
    }
    if (!base_res.found) {
        out.fail_reason =
            base_res.fail_reason == "budget_exhausted" ? "budget_exhausted" : "unreachable";
        return out;
    }

    struct Attempt {
        AStarResult res;
        SparseRoutingGraph graph;  // search view (copy of base + deviation)
        std::vector<double> mult;
        AStarConfig cfg;
        std::vector<Coord> bias;
        std::string strategy;
    };
    std::vector<Attempt> attempts;
    attempts.reserve(out.effective_k);
    attempts.push_back({base_res, base_graph, eff_mult, astar_cfg, base_bias, "base"});

    // Bottleneck edge set from the base path (directed from->to pairs).
    std::set<std::pair<int, int>> base_edges;
    for (std::size_t i = 0; i < base_res.edge_path.size(); ++i)
        base_edges.insert({base_res.node_path[i], base_res.node_path[i + 1]});
    std::size_t nsteps = base_res.edge_path.size();
    auto third_of = [&](std::size_t i) -> int {
        if (nsteps == 0) return 0;
        double f = static_cast<double>(i) / static_cast<double>(nsteps);
        return f < 1.0 / 3.0 ? 0 : (f < 2.0 / 3.0 ? 1 : 2);
    };
    // Most/least used layers on the base path.
    std::map<LayerId, int> layer_use;
    for (int nid : base_res.node_path) layer_use[base_graph.nodes()[nid].layer]++;
    LayerId top_layer = -1;
    int top_n = -1;
    for (auto& [l, c] : layer_use) {
        if (c > top_n) {
            top_n = c;
            top_layer = l;
        }
    }
    // Via step indices on the base path.
    std::vector<std::size_t> via_steps;
    for (std::size_t i = 0; i < base_res.edge_path.size(); ++i) {
        int u = base_res.node_path[i];
        if (base_graph.edges(u)[base_res.edge_path[i]].is_via) via_steps.push_back(i);
    }

    // Deterministic deviation strategies (fixed order, first K wins).
    // Each entry builds a graph copy + mult/cfg/bias tweak, then A*.
    int want = out.effective_k;
    auto add_penalized = [&](const std::string& name, int third /*-1=all*/,
                             bool forbid_middle_edge, const std::vector<double>& mult,
                             const AStarConfig& cfg, const std::vector<Coord>& bias,
                             bool penalize_late_vias, bool penalize_early_vias) {
        if (static_cast<int>(attempts.size()) >= want) return;
        SparseRoutingGraph g = base_graph;  // copy: nodes/edges reused, no rebuild
        if (forbid_middle_edge && nsteps > 0) {
            std::size_t mid = nsteps / 2;
            int fu = base_res.node_path[mid];
            int fv = base_res.node_path[mid + 1];
            auto& vec = const_cast<std::vector<SparseEdge>&>(g.edges(fu));
            // Hard forbid the single middle transition (deterministic).
            vec.erase(std::remove_if(vec.begin(), vec.end(),
                                     [&](const SparseEdge& e) { return e.to == fv; }),
                      vec.end());
        } else {
            g.add_penalties([&](const SparseNode& n, const SparseEdge& e) -> Coord {
                (void)n;
                Coord p = 0;
                // Bottleneck thirds of the base corridor.
                if (third >= 0) {
                    for (std::size_t i = 0; i < base_res.edge_path.size(); ++i) {
                        if (third_of(i) != third) continue;
                        // Penalize edges incident to base-path nodes of that third.
                        int bu = base_res.node_path[i];
                        int bv = base_res.node_path[i + 1];
                        const Point& pa = base_graph.nodes()[bu].p;
                        const Point& pb = base_graph.nodes()[bv].p;
                        Coord d = std::min(manhattan(n.p, pa), manhattan(n.p, pb));
                        if (d < 2000000) {
                            p += kDeviatePenaltyNm;
                            break;
                        }
                    }
                } else if (third == -1) {
                    // Whole-corridor pull-away handled via bias; small penalty
                    // near the exact base nodes to encourage side corridors.
                    for (int bn : base_res.node_path) {
                        if (manhattan(n.p, base_graph.nodes()[bn].p) < 1000000) {
                            p += kDeviatePenaltyNm / 2;
                            break;
                        }
                    }
                }
                if (e.is_via) {
                    // Early/late first-via steering.
                    // Position proxy: node coordinate along src->dst axis.
                    if (penalize_late_vias || penalize_early_vias) {
                        Coord span = std::max<Coord>(
                            1, manhattan(base_graph.nodes()[base_res.node_path.front()].p,
                                         base_graph.nodes()[base_res.node_path.back()].p));
                        Coord along = manhattan(
                            base_graph.nodes()[base_res.node_path.front()].p, n.p);
                        double f = static_cast<double>(along) / static_cast<double>(span);
                        if (penalize_late_vias && f > 0.5) p += kDeviatePenaltyNm;
                        if (penalize_early_vias && f <= 0.5) p += kDeviatePenaltyNm;
                    }
                }
                return p;
            });
        }
        AStarResult r = astar_route_masked(g, mult, cfg, {}, bias);
        out.total_expansions += r.expansions;
        if (!r.found) return;
        attempts.push_back({r, std::move(g), mult, cfg, bias, name});
    };

    // Away-bias: push search off the base corridor (quarter-distance bias).
    auto away_bias = [&]() {
        std::vector<Coord> b(base_graph.nodes().size(), 0);
        for (std::size_t i = 0; i < base_graph.nodes().size(); ++i) {
            Coord best = std::numeric_limits<Coord>::max();
            for (int bn : base_res.node_path) {
                Coord d = manhattan(base_graph.nodes()[i].p, base_graph.nodes()[bn].p);
                if (d < best) best = d;
            }
            if (best == std::numeric_limits<Coord>::max()) best = 0;
            // Zero ON the corridor, growing off it is backwards; instead add
            // a flat penalty near the corridor via add_penalized(third=-1).
            // Here keep the hierarchy bias (soft pull) unchanged.
            b[i] = base_bias[i];
        }
        return b;
    };

    if (want > 1) {
        // 1: penalize first third (forces alternate entry corridor).
        add_penalized("penalize-first-third", 0, false, eff_mult, astar_cfg, base_bias,
                      false, false);
        // 2: penalize middle third (forces around the bottleneck).
        add_penalized("penalize-middle-third", 1, false, eff_mult, astar_cfg, base_bias,
                      false, false);
        // 3: penalize last third (forces alternate exit corridor).
        add_penalized("penalize-last-third", 2, false, eff_mult, astar_cfg, base_bias,
                      false, false);
        // 4: forbid the single middle transition (hard deviation).
        add_penalized("forbid-middle-edge", -2, true, eff_mult, astar_cfg, base_bias,
                      false, false);
        // 5: via-cost x4 (no-via preference).
        {
            AStarConfig c = astar_cfg;
            c.via_cost_nm = astar_cfg.via_cost_nm * 4;
            add_penalized("via-expensive", -3, false, eff_mult, c, base_bias, false,
                          false);
        }
        // 6: via-cost /4 (via-tolerant alternative).
        {
            AStarConfig c = astar_cfg;
            c.via_cost_nm = std::max<Coord>(1, astar_cfg.via_cost_nm / 4);
            add_penalized("via-cheap", -3, false, eff_mult, c, base_bias, false, false);
        }
        // 7: ban the dominant layer (forces the via alternative on open boards).
        if (top_layer >= 0) {
            std::vector<double> m = eff_mult;
            if (top_layer >= 0 && top_layer < static_cast<int>(m.size())) m[top_layer] *= 5.0;
            add_penalized("ban-dominant-layer", -3, false, m, astar_cfg, base_bias,
                          false, false);
        }
        // 8: penalize every other used layer (second layer strategy).
        {
            std::vector<double> m = eff_mult;
            bool any = false;
            for (auto& [l, c] : layer_use) {
                if (l == top_layer) continue;
                if (l >= 0 && l < static_cast<int>(m.size())) {
                    m[l] *= 3.0;
                    any = true;
                }
            }
            if (any)
                add_penalized("penalize-alt-layers", -3, false, m, astar_cfg, base_bias,
                              false, false);
        }
        // 9: corridor pull-away (penalize the whole base corridor).
        add_penalized("away-corridor", -1, false, eff_mult, astar_cfg, away_bias(), false,
                      false);
        // 10: first-via early (penalize late vias).
        add_penalized("via-early", -3, false, eff_mult, astar_cfg, base_bias, true,
                      false);
        // 11: first-via late (penalize early vias).
        add_penalized("via-late", -3, false, eff_mult, astar_cfg, base_bias, false,
                      true);
        // 12: bend-expensive (straight preference).
        {
            AStarConfig c = astar_cfg;
            c.bend_cost_nm = astar_cfg.bend_cost_nm * 4;
            add_penalized("bend-expensive", -3, false, eff_mult, c, base_bias, false,
                          false);
        }
        // 13: bend-free (detour-tolerant).
        {
            AStarConfig c = astar_cfg;
            c.bend_cost_nm = 0;
            add_penalized("bend-free", -3, false, eff_mult, c, base_bias, false, false);
        }
        // 14: penalize first+last thirds together (widest corridor shift).
        if (static_cast<int>(attempts.size()) < want) {
            SparseRoutingGraph g = base_graph;
            g.add_penalties([&](const SparseNode& n, const SparseEdge& e) -> Coord {
                (void)e;
                Coord p = 0;
                for (std::size_t i = 0; i < base_res.edge_path.size(); ++i) {
                    if (third_of(i) == 1) continue;
                    int bu = base_res.node_path[i];
                    const Point& pa = base_graph.nodes()[bu].p;
                    if (manhattan(n.p, pa) < 2000000) {
                        p += kDeviatePenaltyNm;
                        break;
                    }
                }
                return p;
            });
            AStarResult r = astar_route_masked(g, eff_mult, astar_cfg, {}, base_bias);
            out.total_expansions += r.expansions;
            if (r.found) attempts.push_back({r, std::move(g), eff_mult, astar_cfg,
                                             base_bias, "penalize-entry-exit"});
        }
        (void)via_steps;
    }

    // Materialize + exact legality + signature, streaming into a bounded set.
    std::map<std::string, PortfolioCandidate> by_sig;  // dedup: cheapest wins
    for (auto& a : attempts) {
        CandidateRoute cand;
        cand.task = task;
        cand.task_index = task_index;
        cand.difficulty = difficulty;
        cand.gate_a = src_pt;
        cand.gate_b = dsts.front().p;
        cand.expansions = a.res.expansions;
        // Issue #8: search cost is debug-only; the ordering key below is the
        // recomputed materialized cost from final copper.
        cand.search_cost_nm = a.res.cost_nm;
        cand.cost_nm = a.res.cost_nm;
        cand.hierarchy.attempted = guidance_on;
        cand.hierarchy.coarse_expansions = 0;  // counted once at result level
        cand.hierarchy.exact_expansions = a.res.expansions;
        const NetInfo* ninfo = snapshot.find_net(task.net);
        bool dummy = false;
        cand.required_current_a =
            ninfo ? resolver.current().effective_current(*ninfo, snapshot.defaults, dummy)
                  : 0.0;
        if (!snapshot.layers.empty()) {
            auto ordered = ViaBundlePlanner::ordered_styles(
                resolver, task.net,
                LayerSpan{snapshot.layers.front().id, snapshot.layers.back().id});
            if (!ordered.empty()) {
                cand.via_style = ordered.front().name;
                cand.vias_required =
                    ViaBundlePlanner::required_count(resolver, ordered.front(), task.net);
            }
        }
        if (!materialize_ordinary(snapshot, resolver, task, a.graph, a.res, width,
                                  imp_active, ctx, cand)) {
            continue;  // bundle infeasible: skip, never half-build
        }
        std::string why;
        if (!candidate_legal_vs_board(cand, snapshot, resolver, ctx, why)) continue;
        // Issue #8: recompute from final copper (simplification shortens it);
        // cost_nm mirrors it for compatibility. Issue #9: signature from the
        // same final copper, not the A* node path.
        cand.materialized_cost_nm =
            materialized_route_cost(cand.traces, cand.vias, eff_mult, astar_cfg);
        cand.cost_nm = cand.materialized_cost_nm;
        RouteSignature sig = compute_route_signature(cand);
        std::string key = sig.to_string();
        auto it = by_sig.find(key);
        if (it == by_sig.end() ||
            cand.materialized_cost_nm < it->second.route.materialized_cost_nm) {
            PortfolioCandidate pc;
            pc.route = std::move(cand);
            pc.sig = sig;
            pc.sig_str = key;
            by_sig[key] = std::move(pc);
        }
        if (static_cast<int>(by_sig.size()) >= out.effective_k * 2) break;  // stream cap
    }
    if (by_sig.empty()) {
        out.fail_reason = "unreachable";
        // Preserve the base fail category when the base itself was the only
        // attempt (e.g. budget exhaustion on the shared graph).
        if (attempts.size() == 1 && !base_res.found)
            out.fail_reason = base_res.fail_reason == "budget_exhausted"
                                  ? "budget_exhausted"
                                  : "unreachable";
        return out;
    }
    out.candidates.reserve(by_sig.size());
    for (auto& kv : by_sig) out.candidates.push_back(std::move(kv.second));
    std::sort(out.candidates.begin(), out.candidates.end(),
              [](const PortfolioCandidate& a, const PortfolioCandidate& b) {
                  if (a.route.materialized_cost_nm != b.route.materialized_cost_nm)
                      return a.route.materialized_cost_nm < b.route.materialized_cost_nm;
                  if (a.route.cost_nm != b.route.cost_nm)
                      return a.route.cost_nm < b.route.cost_nm;
                  return a.sig_str < b.sig_str;
              });
    if (static_cast<int>(out.candidates.size()) > out.effective_k)
        out.candidates.resize(out.effective_k);
    return out;
}

}  // namespace copperline
