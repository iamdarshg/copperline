#include "router/recovery.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <thread>

#include "router/impact.h"
#include "router/simplify.h"

namespace copperline {

std::string recovery_mode_name(RecoveryMode m) {
    switch (m) {
        case RecoveryMode::FAST: return "FAST";
        case RecoveryMode::RECOVERY: return "RECOVERY";
        case RecoveryMode::EXHAUSTIVE_LOCAL: return "EXHAUSTIVE_LOCAL_RECOVERY";
    }
    return "FAST";
}

RecoveryMode recovery_mode_for_generation(int generation) {
    if (generation <= 0) return RecoveryMode::FAST;
    if (generation <= 2) return RecoveryMode::RECOVERY;
    return RecoveryMode::EXHAUSTIVE_LOCAL;
}

AStarConfig astar_config_for_mode(const AStarConfig& base, RecoveryMode mode) {
    AStarConfig out = base;
    if (mode == RecoveryMode::RECOVERY) {
        out.max_expansions = base.max_expansions * 4;
    } else if (mode == RecoveryMode::EXHAUSTIVE_LOCAL) {
        out.max_expansions = base.max_expansions * 16;
        if (out.max_expansions < 800000) out.max_expansions = 800000;
    }
    return out;
}

int branch_width_for_generation(int generation) {
    if (generation <= 0) return 2;
    if (generation <= 2) return 4;
    return 8;
}

int max_rip_breadth_for_generation(int generation) {
    if (generation <= 0) return 1;
    if (generation <= 2) return 2;
    return 4;
}

int max_blocker_nets_for_mode(RecoveryMode mode) {
    switch (mode) {
        case RecoveryMode::FAST: return 1;
        case RecoveryMode::RECOVERY: return 2;
        case RecoveryMode::EXHAUSTIVE_LOCAL: return 3;
    }
    return 1;
}

void StallDetector::note_epoch(int accepted) {
    if (accepted == 0)
        ++stalled_epochs;
    else
        stalled_epochs = 0;
}

JsonValue FrontierDiag::to_json() const {
    JsonValue o = JsonValue::object();
    o["net"] = static_cast<double>(net);
    o["terminal_a"] = static_cast<double>(a);
    o["terminal_b"] = static_cast<double>(b);
    o["reason"] = fail_reason;
    o["expansions"] = static_cast<double>(expansions);
    o["closest_node"] = static_cast<double>(closest_node);
    json_add_mm(o, "closest_goal_dist_mm", nm_to_mm(closest_goal_dist_nm));
    // Issue #21: top frontier blockers/reasons for agents.
    JsonValue tb = JsonValue::array();
    for (const auto& b : top_blockers) {
        JsonValue e = JsonValue::object();
        e["blocker_net"] = static_cast<double>(b.blocker_net);
        e["kind"] = b.kind;
        e["blocker"] = b.desc;
        e["layer"] = static_cast<double>(b.layer);
        json_add_point_mm(e, "x_mm", "y_mm", nm_to_mm(b.pos.x), nm_to_mm(b.pos.y));
        e["count"] = static_cast<double>(b.count);
        tb.as_array().push_back(e);
    }
    o["top_blockers"] = tb;
    return o;
}

FrontierDiag diagnose_task(const ConnectionTask& task, const CandidateRoute& last) {
    FrontierDiag d;
    d.net = task.net;
    d.a = task.a;
    d.b = task.b;
    d.fail_reason = last.fail_reason.empty() ? "unreachable" : last.fail_reason;
    d.expansions = last.expansions;
    d.closest_node = last.closest_node;
    d.closest_goal_dist_nm = last.closest_goal_dist_nm;
    d.top_blockers = last.frontier_blockers;
    return d;
}

std::vector<BlockerHit> attribute_blockers_detailed(const Board& board,
                                                     const ConnectionTask& task,
                                                     Coord width_nm,
                                                     const CandidateRoute* last) {
    // Issue #21: rank actual frontier blockers first; the coarse
    // corridor-overlap heuristic is fallback/supplement only. Frontier area
    // proxies are set huge (2^60 base) so real evidence always outranks
    // rectangular guesses, while preserving frontier count order among
    // themselves.
    std::vector<BlockerHit> frontier_hits;
    if (last) {
        int rank = 0;
        for (const auto& f : last->frontier_blockers) {
            BlockerHit h;
            if (f.kind == "bounds" || f.kind == "frontier") {
                h.desc = "off_board";
                h.net = -1;
                h.kind = "frontier";
            } else {
                h.desc = f.desc;
                h.net = f.blocker_net;
                h.kind = f.kind;  // "trace" | "pad" | "via" | "keepout"
            }
            h.area = static_cast<Coord>((1LL << 60) + (Coord)f.count * 1000000 - rank);
            frontier_hits.push_back(h);
            ++rank;
            if (frontier_hits.size() >= 5) break;
        }
    }
    std::vector<BlockerHit> out;
    for (auto& h : frontier_hits) out.push_back(h);
    const Terminal* ta = board.find_terminal(task.a);
    const Terminal* tb = board.find_terminal(task.b);
    if (!ta || !tb) return out;
    // S8: when frontier evidence already fills the 8-slot budget, the
    // corridor-overlap rescan below contributes nothing (supplement adds
    // zero). Skip the O(copper) keepout/trace/pad/via passes and go
    // straight to the frontier-gap tail — byte-identical output.
    const std::size_t supplement_budget = out.size() >= 8 ? 0 : 8 - out.size();
    std::vector<BlockerHit> hits;
    if (supplement_budget > 0) {
    // Issue #4: corridor covers the actual routing target (copper point).
    Rect corridor = Rect::from_points(ta->pos, task_dst_point(board, task)).expanded(width_nm);
    struct Hit {
        BlockerHit h;
    };
    for (const auto& ko : board.keepouts) {
        if (!ko.rect.intersects(corridor)) continue;
        Rect inter{std::max(ko.rect.x1, corridor.x1), std::max(ko.rect.y1, corridor.y1),
                   std::min(ko.rect.x2, corridor.x2), std::min(ko.rect.y2, corridor.y2)};
        __int128 area = (__int128)(inter.x2 - inter.x1) * (inter.y2 - inter.y1);
        BlockerHit h;
        h.desc = "keepout:" + ko.reason;
        h.net = -1;
        h.kind = "keepout";
        h.area = static_cast<Coord>(area);
        hits.push_back(h);
    }
    for (const auto& t : board.traces) {
        if (t.net == task.net) continue;
        if (!t.segment().bounds().expanded(t.width_nm).intersects(corridor)) continue;
        const NetInfo* on = board.find_net(t.net);
        BlockerHit h;
        h.desc = "trace:net=" + std::string(on ? on->name : "?");
        h.net = t.net;
        h.kind = "trace";
        __int128 a128 = (__int128)t.width_nm * manhattan(t.a, t.b);
        h.area = a128 > (__int128)INT64_MAX ? INT64_MAX : static_cast<Coord>(a128);
        hits.push_back(h);
    }
    for (const auto& t : board.terminals) {
        if (t.net == task.net) continue;
        if (!t.pad_rect().intersects(corridor)) continue;
        const NetInfo* on = board.find_net(t.net);
        BlockerHit h;
        h.desc = "pad:net=" + std::string(on ? on->name : "?") +
                 (t.component.empty() ? "" : ":" + t.component + "." + t.pin);
        h.net = t.net;
        h.kind = "pad";
        __int128 a128 = (__int128)t.pad_w_nm * t.pad_h_nm;
        h.area = a128 > (__int128)INT64_MAX ? INT64_MAX : static_cast<Coord>(a128);
        hits.push_back(h);
    }
    for (const auto& v : board.vias) {
        if (v.net == task.net) continue;
        Rect vr = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        if (!vr.intersects(corridor)) continue;
        const NetInfo* on = board.find_net(v.net);
        BlockerHit h;
        h.desc = "via:net=" + std::string(on ? on->name : "?");
        h.net = v.net;
        h.kind = "via";
        __int128 a128 = (__int128)v.outer_d_nm * v.outer_d_nm;
        h.area = a128 > (__int128)INT64_MAX ? INT64_MAX : static_cast<Coord>(a128);
        hits.push_back(h);
    }
    std::sort(hits.begin(), hits.end(), [](const BlockerHit& a, const BlockerHit& b) {
        if (a.area != b.area) return a.area > b.area;
        return a.desc < b.desc;
    });
    }  // end if (supplement_budget > 0): corridor rescan skipped when full
    // Supplement: corridor guesses for blocker nets not already covered by
    // real frontier evidence. Frontier hits keep their leading positions.
    // S6: the dependency graph itself is built once per recovery generation
    // (engine side) and reused across all speculative branches in that
    // generation; per-branch reroute consumes it read-only via Stream-1
    // candidates_conflict/candidate_legal_vs_board calls (which already
    // carry their own bounds precheck) — no divergent-board graph sharing,
    // which would risk stale legality.
    {
        std::set<NetId> covered;
        for (const auto& h : frontier_hits)
            if (h.net >= 0) covered.insert(h.net);
        std::size_t budget = supplement_budget;
        std::size_t added = 0;
        for (const auto& h : hits) {
            if (added >= budget) break;
            if (h.net >= 0 && covered.count(h.net)) continue;
            out.push_back(h);
            if (h.net >= 0) covered.insert(h.net);
            ++added;
            if (out.size() >= 8) break;
        }
    }
    if (last && last->closest_node >= 0 && out.size() < 9) {
        BlockerHit h;
        h.desc = "frontier_gap_mm=" + std::to_string(nm_to_mm(last->closest_goal_dist_nm));
        h.net = -1;
        h.kind = "frontier";
        h.area = 0;
        out.push_back(h);
    }
    return out;
}

DependencyGraph build_dependency_graph(
    const Board& board, const RuleResolver& resolver, const ElectricalContext& /*ctx*/,
    const std::vector<ConnectionTask>& tasks, const std::vector<int>& remaining,
    const std::map<std::pair<NetId, std::pair<TermId, TermId>>, CandidateRoute>& last_attempt) {
    DependencyGraph g;
    auto key_of = [](const ConnectionTask& t) {
        return std::make_pair(t.net, std::make_pair(std::min(t.a, t.b), std::max(t.a, t.b)));
    };
    for (int ti : remaining) {
        g.failed.push_back(tasks[ti]);
    }
    for (std::size_t fi = 0; fi < remaining.size(); ++fi) {
        int ti = remaining[fi];
        const ConnectionTask& task = tasks[ti];
        const Terminal* ta = board.find_terminal(task.a);
        TraceRule rule = resolver.traceRule(task.net, ta ? ta->layer : 0, kAnyRegion);
        auto it = last_attempt.find(key_of(task));
        const CandidateRoute* last = it != last_attempt.end() ? &it->second : nullptr;
        std::vector<BlockerHit> hits = attribute_blockers_detailed(board, task, rule.pref_width_nm, last);
        // Collapse to one edge per blocker net (max area wins), deterministic.
        // Frontier-derived areas are huge by construction, so real evidence
        // outranks corridor guesses here as well.
        std::map<NetId, DependencyEdge> best;
        for (const auto& h : hits) {
            if (h.kind != "trace" && h.kind != "pad" && h.kind != "via") {
                if (h.kind == "keepout" || h.kind == "frontier" || h.kind == "bounds") {
                    DependencyEdge e;
                    e.failed_pos = static_cast<int>(fi);
                    e.failed_net = task.net;
                    e.blocker_net = -1;
                    e.blocker_desc = h.desc;
                    e.weight = 0.0;
                    g.edges.push_back(e);
                }
                continue;
            }
            auto b = best.find(h.net);
            double w = static_cast<double>(h.area);
            if (b == best.end() || w > b->second.weight) {
                DependencyEdge e;
                e.failed_pos = static_cast<int>(fi);
                e.failed_net = task.net;
                e.blocker_net = h.net;
                e.blocker_desc = h.desc;
                e.weight = w;
                best[h.net] = e;
            }
        }
        for (const auto& [net, e] : best) g.edges.push_back(e);
    }
    std::sort(g.edges.begin(), g.edges.end(), [](const DependencyEdge& a, const DependencyEdge& b) {
        if (a.weight != b.weight) return a.weight > b.weight;
        if (a.failed_net != b.failed_net) return a.failed_net < b.failed_net;
        return a.blocker_net < b.blocker_net;
    });
    return g;
}

JsonValue DependencyGraph::to_json() const {
    JsonValue o = JsonValue::object();
    JsonValue fl = JsonValue::array();
    for (const auto& t : failed) {
        JsonValue e = JsonValue::object();
        e["net"] = static_cast<double>(t.net);
        e["terminal_a"] = static_cast<double>(t.a);
        e["terminal_b"] = static_cast<double>(t.b);
        fl.as_array().push_back(e);
    }
    o["blocked"] = fl;
    JsonValue el = JsonValue::array();
    for (const auto& e : edges) {
        JsonValue ejo = JsonValue::object();
        ejo["failed_pos"] = static_cast<double>(e.failed_pos);
        ejo["failed_net"] = static_cast<double>(e.failed_net);
        ejo["blocker_net"] = static_cast<double>(e.blocker_net);
        ejo["blocker"] = e.blocker_desc;
        ejo["weight"] = e.weight;
        el.as_array().push_back(ejo);
    }
    o["edges"] = el;
    return o;
}

double route_protection_score(bool is_escape_stub, double difficulty, int stable_epochs,
                              bool is_fixed, RecoveryMode mode) {
    if (is_fixed) return kFixedProtection;  // fixed user copper is never ripped
    double s = 1.0 + 0.5 * difficulty + 1.0 * std::max(0, stable_epochs);
    if (is_escape_stub) s += 5.0;  // escape stubs cost more to rip initially
    if (mode == RecoveryMode::EXHAUSTIVE_LOCAL && is_escape_stub) {
        // Still expensive, but no longer prohibitive: exhaustive recovery may
        // reconsider escape bundles when connectivity demands it.
        s *= 0.8;
    }
    return s;
}

void rebuild_board_from_owned(Board& board, const std::vector<TraceSeg>& fixed_traces,
                              const std::vector<Via>& fixed_vias,
                              const std::vector<OwnedRoute>& owned) {
    board.traces = fixed_traces;
    board.vias = fixed_vias;
    // Deterministic commit order: (net, a, b) over owned routes.
    std::vector<const OwnedRoute*> order;
    for (const auto& o : owned) order.push_back(&o);
    std::sort(order.begin(), order.end(), [](const OwnedRoute* a, const OwnedRoute* b) {
        if (a->task.net != b->task.net) return a->task.net < b->task.net;
        if (a->task.a != b->task.a) return a->task.a < b->task.a;
        return a->task.b < b->task.b;
    });
    for (const OwnedRoute* o : order) {
        for (const auto& t : o->traces) board.traces.push_back(t);
        for (const auto& v : o->vias) board.vias.push_back(v);
    }
}

namespace {
std::uint64_t fnv1a_64(const void* data, std::size_t n, std::uint64_t seed) {
    const auto* b = static_cast<const unsigned char*>(data);
    std::uint64_t h = seed;
    for (std::size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 1099511628211ULL;
    }
    return h;
}

void hash_mix64(std::uint64_t& h, std::uint64_t v) {
    // Zobrist-style xor combine with a per-value splitmix.
    v += 0x9e3779b97f4a7c15ULL;
    v = (v ^ (v >> 30)) * 0xbf58476d1ce4e5b9ULL;
    v = (v ^ (v >> 27)) * 0x94d049bb133111ebULL;
    v ^= (v >> 31);
    h ^= v;
    h *= 1099511628211ULL;
}
}  // namespace

StateHash128 state_hash128(const Board& board, const std::vector<ConnectionTask>& tasks,
                           const std::vector<int>& remaining) {
    StateHash128 out{1469598103934665603ULL, 1099511628211ULL ^ 0x9e3779b97f4a7c15ULL};
    for (const auto& t : board.traces) {
        std::uint64_t v = fnv1a_64(&t.net, sizeof(t.net), 1469598103934665603ULL);
        hash_mix64(out.lo, v ^ static_cast<std::uint64_t>(t.a.x * 31 + t.a.y));
        hash_mix64(out.hi, v ^ static_cast<std::uint64_t>(t.b.x * 131 + t.b.y * 17 + t.width_nm));
        hash_mix64(out.lo, static_cast<std::uint64_t>(t.layer) + 0x12345);
    }
    hash_mix64(out.lo, 0x76416d6569676fULL);
    for (const auto& v : board.vias) {
        std::uint64_t vv = fnv1a_64(&v.pos.x, sizeof(v.pos.x), 1469598103934665603ULL);
        hash_mix64(out.lo, vv ^ static_cast<std::uint64_t>(v.pos.y));
        hash_mix64(out.hi, vv ^ static_cast<std::uint64_t>(v.outer_d_nm + v.top_layer * 77));
    }
    // Remaining set, sorted for stability.
    // Issue #12: pair-corridor tasks carry their pair identity so a corridor
    // and a plain task on the same (net, a, b) never alias in the table.
    std::vector<std::string> keys;
    for (int ti : remaining) {
        const ConnectionTask& t = tasks[ti];
        std::string k = std::to_string(t.net) + ":" + std::to_string(std::min(t.a, t.b)) + ":" +
                        std::to_string(std::max(t.a, t.b));
        if (t.is_pair_corridor) k += ":pair" + std::to_string(t.pair_id);
        keys.push_back(k);
    }
    std::sort(keys.begin(), keys.end());
    for (const auto& k : keys) {
        hash_mix64(out.lo, fnv1a_64(k.data(), k.size(), 1469598103934665603ULL));
        hash_mix64(out.hi, fnv1a_64(k.data(), k.size(), 1099511628211ULL));
    }
    return out;
}

std::string StateHash128::to_hex() const {
    char buf[33];
    std::snprintf(buf, sizeof(buf), "%016llx%016llx", (unsigned long long)lo,
                  (unsigned long long)hi);
    return std::string(buf);
}

bool TranspositionTable::should_prune(const StateHash128& h, int remaining) const {
    auto it = table_.find(h.to_hex());
    if (it == table_.end()) return false;
    if (it->second <= remaining) {
        ++hits_;
        return true;
    }
    return false;
}

void TranspositionTable::record(const StateHash128& h, int remaining) {
    std::string k = h.to_hex();
    auto it = table_.find(k);
    if (it == table_.end() || remaining < it->second) table_[k] = remaining;
}

void HistoryHeuristic::reward(const std::string& key, double amount) {
    scores_[key] += amount;
}

double HistoryHeuristic::bonus(const std::string& key) const {
    auto it = scores_.find(key);
    return it != scores_.end() ? it->second : 0.0;
}

std::string HistoryHeuristic::move_key(const ConnectionTask& failed, NetId blocker_net) {
    return std::to_string(failed.net) + ":" + std::to_string(failed.a) + ":" +
           std::to_string(failed.b) + "->" + std::to_string(blocker_net);
}

std::vector<RipupMove> generate_ripup_moves(const std::vector<ConnectionTask>& failed_tasks,
                                            const DependencyGraph& graph,
                                            const std::vector<OwnedRoute>& owned,
                                            const HistoryHeuristic& history,
                                            const std::string& pv_key, RecoveryMode mode,
                                            int max_moves, int max_breadth,
                                            std::chrono::steady_clock::time_point deadline) {
    std::vector<RipupMove> moves;
    const bool bounded = deadline != std::chrono::steady_clock::time_point::max();
    std::set<std::string> seen;  // dedup equivalent owned-route index sets
    auto dedup_key = [](int failed_pos, const std::vector<int>& idx) {
        std::string k = std::to_string(failed_pos) + ":";
        for (int i : idx) k += std::to_string(i) + ",";
        return k;
    };
    // Owned routes by net for blocker lookup.
    std::map<NetId, std::vector<int>> owned_by_net;
    for (std::size_t i = 0; i < owned.size(); ++i) owned_by_net[owned[i].task.net].push_back((int)i);
    auto cheapest_of_net = [&](NetId bn) -> std::vector<int> {
        auto oit = owned_by_net.find(bn);
        if (oit == owned_by_net.end() || oit->second.empty()) return {};
        std::vector<int> cands = oit->second;
        std::sort(cands.begin(), cands.end(), [&](int a, int b) {
            if (owned[a].protection != owned[b].protection)
                return owned[a].protection < owned[b].protection;
            if (owned[a].task.net != owned[b].task.net)
                return owned[a].task.net < owned[b].task.net;
            if (owned[a].task.a != owned[b].task.a) return owned[a].task.a < owned[b].task.a;
            return owned[a].task.b < owned[b].task.b;
        });
        return cands;
    };
    auto is_rippable = [&](int oi) {
        return owned[oi].protection < kFixedProtection / 2;
    };

    // Edges grouped per failed position.
    std::map<int, std::vector<DependencyEdge>> per_failed;
    for (const auto& e : graph.edges) {
        if (e.blocker_net < 0) continue;  // keepout/frontier: nothing to rip
        per_failed[e.failed_pos].push_back(e);
    }
    const int over_gen = std::max(1, max_moves * 3);
    for (std::size_t fi = 0; fi < failed_tasks.size(); ++fi) {
        // Bounded runs: stop enumerating once the global deadline is spent.
        if (bounded && (fi & 63u) == 0 &&
            std::chrono::steady_clock::now() > deadline)
            break;
        const ConnectionTask& failed = failed_tasks[fi];
        auto it = per_failed.find((int)fi);
        if (it == per_failed.end() || it->second.empty()) continue;
        // Distinct blocker nets, weight order (graph already sorted).
        // Ranked blocker list per failed task feeds both layers below.
        std::vector<NetId> blockers;
        std::map<NetId, double> weight_of;
        for (const auto& e : it->second) {
            if (weight_of.find(e.blocker_net) == weight_of.end()) {
                blockers.push_back(e.blocker_net);
                weight_of[e.blocker_net] = e.weight;
            }
        }
        std::sort(blockers.begin(), blockers.end(), [&](NetId a, NetId b) {
            if (weight_of[a] != weight_of[b]) return weight_of[a] > weight_of[b];
            return a < b;
        });
        // ---- Layer 1 (cheapest first): single-blocker moves ----
        for (NetId bn : blockers) {
            std::vector<int> cands = cheapest_of_net(bn);
            if (cands.empty()) continue;
            if (!is_rippable(cands[0])) continue;  // fixed: never include
            int breadth = std::min<int>(max_breadth, (int)cands.size());
            for (int k = 1; k <= breadth; ++k) {
                bool ok = true;
                for (int j = 0; j < k; ++j)
                    if (!is_rippable(cands[j])) ok = false;
                if (!ok) break;
                RipupMove m;
                m.failed_pos = (int)fi;
                m.failed_task = failed;
                m.blocker_net = bn;
                for (int j = 0; j < k; ++j) m.owned_idx.push_back(cands[j]);
                std::sort(m.owned_idx.begin(), m.owned_idx.end());
                std::string dk = dedup_key((int)fi, m.owned_idx);
                if (seen.count(dk)) continue;
                seen.insert(dk);
                m.reason = "blocked_by_net_" + std::to_string(bn);
                m.gain = weight_of[bn];
                m.cost = 0;
                for (int oi : m.owned_idx) m.cost += owned[oi].protection;
                std::string hk = HistoryHeuristic::move_key(failed, bn);
                m.score = m.gain / (1.0 + m.cost) + history.bonus(hk);
                if (!pv_key.empty() && hk == pv_key) m.score += 1e9;  // PV reuse first
                moves.push_back(m);
                if ((int)moves.size() >= over_gen) break;
            }
            if ((int)moves.size() >= over_gen) break;
        }
        // ---- Layer 2 (issue #20): bounded multi-blocker beam ----
        // Combines cheapest routes from 2..N distinct blocker nets. N is
        // capped by mode (FAST=1 stays single); the candidate pool is capped
        // to top-K nets so enumeration cannot explode.
        int max_nets = max_blocker_nets_for_mode(mode);
        if (max_nets >= 2 && blockers.size() >= 2) {
            int top_k = std::min<int>((int)blockers.size(), max_nets + 2);
            if (top_k > 5) top_k = 5;
            std::vector<NetId> pool(blockers.begin(), blockers.begin() + top_k);
            // Filter pool to nets with at least one rippable owned route.
            std::vector<NetId> rip_pool;
            for (NetId bn : pool) {
                std::vector<int> c = cheapest_of_net(bn);
                if (!c.empty() && is_rippable(c[0])) rip_pool.push_back(bn);
            }
            // Enumerate combinations of size 2..max_nets (index lex order).
            int rn = (int)rip_pool.size();
            std::vector<int> idx;
            std::function<void(int, int)> rec = [&](int start, int want) {
                if ((int)idx.size() == want) {
                    // Build combo: cheapest 1 route per net (minimal
                    // disruption; deeper per-net breadth is layer-1 work).
                    std::vector<int> owned_idx;
                    double gain = 0, cost = 0, hist = 0;
                    std::string reason = "blocked_by_nets";
                    for (int ii : idx) {
                        NetId bn = rip_pool[ii];
                        std::vector<int> c = cheapest_of_net(bn);
                        owned_idx.push_back(c[0]);
                        gain += weight_of[bn];
                        cost += owned[c[0]].protection;
                        hist += history.bonus(HistoryHeuristic::move_key(failed, bn));
                        reason += (reason.back() == 's' ? "_" : "+") + std::to_string(bn);
                    }
                    std::sort(owned_idx.begin(), owned_idx.end());
                    if ((int)owned_idx.size() > max_breadth) return;
                    std::string dk = dedup_key((int)fi, owned_idx);
                    if (seen.count(dk)) return;
                    seen.insert(dk);
                    RipupMove m;
                    m.failed_pos = (int)fi;
                    m.failed_task = failed;
                    m.blocker_net = rip_pool[idx[0]];
                    m.owned_idx = owned_idx;
                    m.reason = reason;
                    m.gain = gain;
                    m.cost = cost;
                    m.score = gain / (1.0 + cost) + hist;
                    // No 1e9 PV bonus for combos: singles stay cheapest first.
                    moves.push_back(m);
                    return;
                }
                for (int i = start; i < rn; ++i) {
                    if ((int)moves.size() >= over_gen * 2) return;
                    idx.push_back(i);
                    rec(i + 1, want);
                    idx.pop_back();
                }
            };
            for (int want = 2; want <= max_nets && want <= rn; ++want) {
                idx.clear();
                rec(0, want);
                if ((int)moves.size() >= over_gen * 2) break;
            }
        }
        if ((int)moves.size() >= over_gen * 2) break;
    }
    std::sort(moves.begin(), moves.end(), [](const RipupMove& a, const RipupMove& b) {
        if (a.score != b.score) return a.score > b.score;
        if (a.failed_task.net != b.failed_task.net) return a.failed_task.net < b.failed_task.net;
        if (a.failed_task.a != b.failed_task.a) return a.failed_task.a < b.failed_task.a;
        if (a.failed_task.b != b.failed_task.b) return a.failed_task.b < b.failed_task.b;
        if (a.owned_idx.size() != b.owned_idx.size()) return a.owned_idx.size() < b.owned_idx.size();
        return a.owned_idx < b.owned_idx;
    });
    if ((int)moves.size() > max_moves) moves.resize(max_moves);
    return moves;
}

bool branch_better(const BranchResult& a, const BranchResult& b) {
    if (!a.evaluated) return false;
    if (!b.evaluated) return true;
    // 1. Global connectivity first: fewer unconnected tasks wins. This is
    // the router's lexicographic objective (connectivity > everything).
    if (a.remaining_task_ids.size() != b.remaining_task_ids.size())
        return a.remaining_task_ids.size() < b.remaining_task_ids.size();
    // 2. Legality, then resource overuse.
    if (a.hard_violations != b.hard_violations) return a.hard_violations < b.hard_violations;
    if (a.resource_overuse != b.resource_overuse) return a.resource_overuse < b.resource_overuse;
    // 3. More newly-connected previously-unrouted tasks wins. Reconnecting
    // an already-connected ripped route does not count (issue #19).
    if (a.newly_connected_global != b.newly_connected_global)
        return a.newly_connected_global > b.newly_connected_global;
    // 4. Lower disruption wins: fewer ripped/replaced routes.
    if (a.disrupted_routes != b.disrupted_routes) return a.disrupted_routes < b.disrupted_routes;
    // 5. Fewer vias, then shorter global copper, then lower future
    // obstruction (issue #9/#22), then deterministic hash. Impact never
    // outranks connectivity/legality/disruption: it only breaks ties among
    // globally equivalent boards, so connectivity-first is preserved at
    // every depth. Branches without a score (no rerouted candidates) never
    // win on impact alone.
    if (a.via_count != b.via_count) return a.via_count < b.via_count;
    if (a.length_nm != b.length_nm) return a.length_nm < b.length_nm;
    if (a.has_impact && b.has_impact) {
        double d = a.impact_obstruction - b.impact_obstruction;
        if (std::fabs(d) > 1e-9) return a.impact_obstruction < b.impact_obstruction;
    }
    return a.hash.to_hex() < b.hash.to_hex();
}

int effective_multiply_depth(int requested_depth, const EffectiveSearchBudget& budget,
                             const MaturityCaps& caps) {
    int req = requested_depth <= 0 ? kDefaultMultiplyDepth : requested_depth;
    if (req < 1) req = 1;
    if (req > kMaxMultiplyDepth) req = kMaxMultiplyDepth;
    int eff = req;
    // Maturity allowance is an advisory ceiling (user cap wins).
    if (budget.recovery_depth >= 1 && eff > budget.recovery_depth)
        eff = budget.recovery_depth;
    if (caps.max_recovery_depth >= 1 && eff > caps.max_recovery_depth)
        eff = caps.max_recovery_depth;
    if (eff < 1) eff = 1;
    return eff;
}

int effective_multiply_beam(int requested_beam, const EffectiveSearchBudget& budget,
                            const MaturityCaps& caps, std::size_t memory_budget_bytes) {
    int req = requested_beam <= 0 ? kDefaultMultiplyBeam : requested_beam;
    if (req < 1) req = 1;
    if (req > kMaxMultiplyBeam) req = kMaxMultiplyBeam;
    // Maturity may raise the beam on dense boards (floor); it never lowers
    // the explicit/default request.
    int eff = req;
    if (budget.recovery_beam > eff) eff = budget.recovery_beam;
    if (caps.max_beam >= 1 && eff > caps.max_beam) eff = caps.max_beam;
    if (eff < 1) eff = 1;
    if (eff > kMaxMultiplyBeam) eff = kMaxMultiplyBeam;
    // Memory: one stored beam member ~ kMultiplyMemPerBeamBytes; stream
    // beams so only depth*beam boards coexist. Clamp the beam to the budget.
    if (memory_budget_bytes > 0 && kMultiplyMemPerBeamBytes > 0) {
        std::size_t bound = memory_budget_bytes / kMultiplyMemPerBeamBytes;
        if (bound < 1) bound = 1;
        if (static_cast<std::size_t>(eff) > bound) eff = static_cast<int>(bound);
        if (eff < 1) eff = 1;
    }
    return eff;
}

BranchResult reroute_branch(const Board& base_template, const std::vector<TraceSeg>& fixed_traces,
                            const std::vector<Via>& fixed_vias,
                            const std::vector<OwnedRoute>& surviving,
                            const std::vector<int>& to_route, int failed_ti,
                            const std::vector<ConnectionTask>& tasks,
                            const std::vector<int>& gen_remaining,
                            const std::vector<Corridor>& corridors,
                            const RuleResolver& resolver, const ElectricalContext& ctx,
                            const std::vector<double>& layer_mult, const AStarConfig& astar_cfg,
                            const CongestionMap& congestion_tpl,
                            const std::string& mode_name, const RipupMove& move,
                            const HierarchyConfig& hier_cfg,
                            const HierarchyCache* hier_cache,
                            double reservation_strength,
                            std::chrono::steady_clock::time_point deadline) {
    BranchResult out;
    out.evaluated = true;
    out.mode = mode_name;
    out.move = move;
    const bool time_bounded = deadline != std::chrono::steady_clock::time_point::max();
    // Order change after stalls is itself a decision: the failed task goes
    // first (it owned nothing and needs the corridor most), then difficulty
    // order with stable (net, a, b) tie-breaks.
    std::vector<int> order = to_route;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        bool fa = (a == failed_ti), fb = (b == failed_ti);
        if (fa != fb) return fa > fb;
        if (tasks[a].difficulty != tasks[b].difficulty)
            return tasks[a].difficulty > tasks[b].difficulty;
        if (tasks[a].net != tasks[b].net) return tasks[a].net < tasks[b].net;
        if (tasks[a].a != tasks[b].a) return tasks[a].a < tasks[b].a;
        return tasks[a].b < tasks[b].b;
    });
    Board work = base_template;
    work.traces = fixed_traces;
    work.vias = fixed_vias;
    for (const auto& o : surviving) {
        for (const auto& t : o.traces) work.traces.push_back(t);
        for (const auto& v : o.vias) work.vias.push_back(v);
    }
    RuleResolver r = resolver;
    r.rebind(&work);
    CongestionMap congestion = congestion_tpl;
    ReservationSet reservations;
    std::vector<ConnectionTask> ordered_tasks;
    std::vector<Corridor> ordered_corr;
    std::vector<double> ordered_diff;
    for (int ti : order) {
        ordered_tasks.push_back(tasks[ti]);
        ordered_corr.push_back(corridors[ti]);
        ordered_diff.push_back(tasks[ti].difficulty);
    }
    reservations.build(ordered_tasks, ordered_corr, ordered_diff);
    // Issue #14: maturity-driven reservation strength (soft cost only).
    reservations.set_strength(reservation_strength);

    std::vector<OwnedRoute> new_owned = surviving;
    std::vector<int> done;
    std::vector<CandidateRoute> branch_cands;  // issue #9: scored below for #22
    std::int64_t expansions = 0;
    for (std::size_t k = 0; k < order.size(); ++k) {
        // Global deadline: a branch reroutes many tasks with O(copper)
        // legality checks per candidate; stop once the budget is spent rather
        // than letting one branch overrun the run by minutes.
        if (time_bounded && std::chrono::steady_clock::now() > deadline) break;
        int ti = order[k];
        // S8: pos is the task's index in the branch order — the loop above
        // iterates k over that same order, so pos == k directly (the old
        // linear rescan was O(n^2) for the identical value).
        std::size_t pos = k;
        CandidateRoute cand = route_candidate_task(work, r, tasks[ti], pos, tasks[ti].difficulty,
                                                   ctx, layer_mult, astar_cfg, congestion,
                                                   reservations, hier_cfg, hier_cache);
        expansions += cand.expansions;
        if (!cand.found) continue;
        std::string why;
        if (!candidate_legal_vs_board(cand, work, r, ctx, why)) continue;
        // Commit sequentially inside the branch (revalidation = arbiter).
        bool conflict = false;
        for (const auto& o : new_owned) {
            CandidateRoute other;
            other.task = o.task;
            other.found = true;
            other.traces = o.traces;
            other.vias = o.vias;
            if (candidates_conflict(cand, other, r, ctx)) {
                conflict = true;
                break;
            }
        }
        if (conflict) continue;
        for (const auto& t : cand.traces) work.traces.push_back(t);
        for (const auto& v : cand.vias) work.vias.push_back(v);
        OwnedRoute o;
        o.task = tasks[ti];
        o.task_pos = ti;
        o.traces = cand.traces;
        o.vias = cand.vias;
        o.epoch_committed = -1;
        o.is_escape_stub = false;
        o.protection = 1.0 + tasks[ti].difficulty * 0.1;
        new_owned.push_back(o);
        done.push_back(ti);
        branch_cands.push_back(cand);
        for (const auto& t : cand.traces) congestion.add_history_segment(t.segment(), 0.25);
    }
    out.connected_tasks = (int)done.size();
    out.total_tasks = (int)order.size();
    out.expansions = expansions;
    Coord len = 0;
    int vias = 0;
    for (const auto& o : new_owned) {
        for (const auto& t : o.traces) len += euclid_len_nm(t.a, t.b);
        vias += (int)o.vias.size();
    }
    out.length_nm = len;
    out.via_count = vias;
    // Exact unfinished set: (gen_remaining ∪ to_route) \ newly_done, sorted.
    // No transposition access here (workers never touch the table); the
    // arbiter prunes/records serially after join using this exact identity.
    // Issue #19: also fill the explicit global outcome fields so ranking
    // measures the resulting global board, not branch-local work.
    {
        // O(N) mask passes instead of three std::set builds + lookups: the
        // output (ascending remaining ids) is identical, the allocation and
        // comparison churn is gone. Runs once per branch and the remaining
        // set is large on stalled boards.
        const std::size_t n = tasks.size();
        std::vector<char> done_mask(n, 0), gen_mask(n, 0), present(n, 0);
        for (int ti : done)
            if (ti >= 0 && static_cast<std::size_t>(ti) < n)
                done_mask[static_cast<std::size_t>(ti)] = 1;
        for (int ti : gen_remaining)
            if (ti >= 0 && static_cast<std::size_t>(ti) < n) {
                gen_mask[static_cast<std::size_t>(ti)] = 1;
                present[static_cast<std::size_t>(ti)] = 1;
            }
        for (int ti : to_route)
            if (ti >= 0 && static_cast<std::size_t>(ti) < n)
                present[static_cast<std::size_t>(ti)] = 1;
        std::vector<int> rem;
        for (std::size_t ti = 0; ti < n; ++ti)
            if (present[ti] && !done_mask[ti]) rem.push_back(static_cast<int>(ti));
        out.remaining_task_ids = rem;
        out.hash = state_hash128(work, tasks, rem);
        out.global_connected_tasks = (int)tasks.size() - (int)rem.size();
        int newly_global = 0;
        for (int ti : done)
            if (ti >= 0 && static_cast<std::size_t>(ti) < n &&
                gen_mask[static_cast<std::size_t>(ti)])
                ++newly_global;
        out.newly_connected_global = newly_global;
        out.disrupted_routes = (int)move.owned_idx.size();
        out.hard_violations = 0;  // branch commits only legal-vs-board + conflict-free copper
        out.resource_overuse = 0;
        // Issue #9: expose the branch's future-obstruction to multi-ply
        // recovery (#22). Mean weighted obstruction of this branch's newly
        // committed candidates vs the branch's exact unfinished set. Scored
        // on the final work board with default weights; streaming, O(done *
        // remaining) with O(1) scratch. Never affects branch_better ranking.
        if (!branch_cands.empty()) {
            ImpactContext bctx;
            bctx.board = &work;
            bctx.resolver = &r;
            bctx.ctx = &ctx;
            for (int ti : rem) {
                if (ti < 0 || ti >= (int)tasks.size()) continue;
                bctx.remaining_tasks.push_back(tasks[ti]);
                if (ti < (int)corridors.size()) bctx.remaining_corridors.push_back(corridors[ti]);
            }
            WeightedImpactScorer wsc;
            double sum = 0;
            for (const auto& bc : branch_cands) sum += wsc.score(bc, bctx).total;
            out.impact_obstruction = sum / static_cast<double>(branch_cands.size());
            out.has_impact = true;
        }
    }
    // Move the heavy members in (the copies above were the last uses).
    out.board = std::move(work);
    out.owned = std::move(new_owned);
    out.newly_done = std::move(done);
    return out;
}

MultiPlyResult multiply_beam_search(const std::vector<RipupMove>& first_moves,
                                    const std::vector<BranchResult>& first_results,
                                    const MultiPlyContext& mctx, const MultiPlyConfig& cfg,
                                    TranspositionTable& tt,
                                    std::chrono::steady_clock::time_point deadline) {
    MultiPlyResult out;
    int workers = cfg.threads < 1 ? 1 : cfg.threads;
    auto one_ply_best_index = [&]() -> int {
        int best = -1;
        for (std::size_t i = 0; i < first_results.size(); ++i) {
            if (!first_results[i].evaluated || first_results[i].pruned) continue;
            if (best < 0 || branch_better(first_results[i], first_results[best]))
                best = static_cast<int>(i);
        }
        return best;
    };
    if (cfg.depth <= 1 || first_moves.empty() || first_results.empty()) {
        out.searched = false;
        out.fallback_to_one_ply = true;
        out.best_first_move = one_ply_best_index();
        return out;
    }
    if (!mctx.tasks || !mctx.corridors || !mctx.resolver || !mctx.ectx ||
        !mctx.layer_mult) {
        out.searched = false;
        out.fallback_to_one_ply = true;
        out.best_first_move = one_ply_best_index();
        return out;
    }
    int beam = cfg.beam < 1 ? 1 : cfg.beam;
    if (beam > kMaxMultiplyBeam) beam = kMaxMultiplyBeam;
    std::int64_t max_nodes = cfg.max_nodes < 0 ? 0 : cfg.max_nodes;
    out.searched = true;

    // Seed beam from the non-pruned first-ply outcomes (immutable copies).
    struct BeamEntry {
        BranchResult state;
        std::vector<RipupMove> pv;
        int first_idx = -1;
    };
    std::vector<BeamEntry> cur;
    for (std::size_t i = 0; i < first_moves.size() && i < first_results.size(); ++i) {
        const BranchResult& r = first_results[i];
        if (!r.evaluated || r.pruned) continue;
        BeamEntry e;
        e.state = r;  // immutable branch state copy (bounded: beam width)
        e.pv.push_back(first_moves[i]);
        e.first_idx = static_cast<int>(i);
        cur.push_back(std::move(e));
    }
    if (cur.empty()) {
        out.fallback_to_one_ply = true;
        out.best_first_move = -1;
        return out;
    }
    std::sort(cur.begin(), cur.end(), [](const BeamEntry& a, const BeamEntry& b) {
        if (branch_better(a.state, b.state)) return true;
        if (branch_better(b.state, a.state)) return false;
        return a.first_idx < b.first_idx;
    });
    if (static_cast<int>(cur.size()) > beam) cur.resize(beam);

    int one_ply_best = one_ply_best_index();
    // Best leaf across all depths (starts at the one-ply beam best).
    BeamEntry best_leaf = cur.front();
    // Depth loop: cur holds the beam at depth d (d=0 is first ply).
    for (int d = 1; d < cfg.depth; ++d) {
        if (std::chrono::steady_clock::now() > deadline) {
            out.fallback_to_one_ply = true;
            out.best_first_move = one_ply_best;
            if (one_ply_best >= 0 && one_ply_best < static_cast<int>(first_results.size())) {
                out.best_leaf = first_results[one_ply_best];
                out.has_best_leaf = true;
                out.best_pv.clear();
                out.best_pv.push_back(first_moves[one_ply_best]);
            }
            return out;
        }
        if (out.nodes_evaluated >= max_nodes) {
            out.fallback_to_one_ply = true;
            out.best_first_move = one_ply_best;
            if (one_ply_best >= 0 && one_ply_best < static_cast<int>(first_results.size())) {
                out.best_leaf = first_results[one_ply_best];
                out.has_best_leaf = true;
                out.best_pv.clear();
                out.best_pv.push_back(first_moves[one_ply_best]);
            }
            return out;
        }
        // Wave-parallel parents: expand one memory-bounded wave of
        // (parent, child) pairs at a time, merging children into a global
        // top-beam. Stored boards stay bounded to beam (parents) + one
        // wave of pairs (<= kMaxStoredMultiplyNodes) + beam (merged).
        // S2 wave-parallel parents: per-parent specs (blocker attribution
        // + move-gen + to_route over the parent's immutable board) are
        // built serially in parent order with the identical deadline and
        // node-budget checks as the old streaming loop; all pairs of one
        // memory-bounded wave (<= kMaxStoredMultiplyNodes pairs) are then
        // rerouted in a single flat indexed pool, and TT prune/merge stays
        // serial in (parent, move-index) order. Pruned flags, beam order,
        // node/expansion accounting and fallback behavior are unchanged;
        // worker utilization is no longer capped at children-per-parent.
        // (A deadline trip mid-wave is timing-dependent by nature; with no
        // timeout set the sequence is identical to streaming.)
        struct ChildSpec {
            RipupMove move;
            std::vector<OwnedRoute> surviving;
            std::vector<int> to_route;
            int failed_ti = -1;
        };
        struct WavePair {
            std::size_t pi = 0;  // parent index in cur (merge order)
            ChildSpec spec;
        };
        const std::size_t kWavePairCap =
            static_cast<std::size_t>(kMaxStoredMultiplyNodes);
        std::vector<BeamEntry> next_all;
        bool budget_hit = false;
        std::size_t pi = 0;
        while (pi < cur.size() && !budget_hit) {
            // Accumulate one wave (serial spec build, ordered checks).
            std::vector<WavePair> wave;
            while (pi < cur.size()) {
                if (std::chrono::steady_clock::now() > deadline) {
                    budget_hit = true;
                    break;
                }
                if (out.nodes_evaluated >= max_nodes) {
                    budget_hit = true;
                    break;
                }
                const BeamEntry& parent = cur[pi];
                const std::vector<int>& prem = parent.state.remaining_task_ids;
                if (prem.empty()) {
                    ++pi;
                    continue;  // complete leaf: nothing to expand
                }
                // Rebuild blocker attribution on the parent's immutable board.
                RuleResolver r = *mctx.resolver;
                r.rebind(&parent.state.board);
                DependencyGraph graph = build_dependency_graph(
                    parent.state.board, r, *mctx.ectx, *mctx.tasks, prem,
                    mctx.last_attempt);
                if (graph.edges.empty()) {
                    ++pi;
                    continue;  // keepout-only: dead end
                }
                std::vector<ConnectionTask> failed_tasks = graph.failed;
                int max_moves =
                    mctx.max_moves > 0 ? mctx.max_moves : cfg.max_moves_per_node;
                if (max_moves < 1) max_moves = 1;
                std::vector<RipupMove> moves = generate_ripup_moves(
                    failed_tasks, graph, parent.state.owned, mctx.history,
                    mctx.pv_key, mctx.mode, max_moves, mctx.max_breadth, deadline);
                if (moves.empty()) {
                    ++pi;
                    continue;
                }
                // Cap this parent's fan-out against the remaining node budget
                // deterministically (leading moves only). Exhaustion of the
                // deeper budget falls back to one-ply per spec.
                std::int64_t room = max_nodes - out.nodes_evaluated;
                // Reserve room for the remaining parents' minimal progress: we
                // still expand this parent fully when room allows; otherwise
                // fall back rather than committing a truncated lookahead.
                if (room < static_cast<std::int64_t>(moves.size())) {
                    // Deterministic truncation would bias the beam; fall back.
                    budget_hit = true;
                    break;
                }
                std::vector<ChildSpec> specs;
                specs.reserve(moves.size());
                std::set<int> prem_set(prem.begin(), prem.end());
                for (const auto& m : moves) {
                    ChildSpec s;
                    s.move = m;
                    const std::size_t owned_n = parent.state.owned.size();
                    std::vector<char> rip(owned_n, 0);
                    for (int oi : m.owned_idx)
                        if (oi >= 0 && static_cast<std::size_t>(oi) < owned_n)
                            rip[static_cast<std::size_t>(oi)] = 1;
                    s.surviving.reserve(owned_n);
                    for (std::size_t i = 0; i < owned_n; ++i)
                        if (!rip[i]) s.surviving.push_back(parent.state.owned[i]);
                    if (m.failed_pos < 0 ||
                        m.failed_pos >= static_cast<int>(prem.size()))
                        continue;
                    int failed_ti = prem[m.failed_pos];
                    s.failed_ti = failed_ti;
                    std::vector<int> to_route;
                    to_route.push_back(failed_ti);
                    for (int oi : m.owned_idx) {
                        if (oi < 0 ||
                            oi >= static_cast<int>(parent.state.owned.size()))
                            continue;
                        int tp = parent.state.owned[oi].task_pos;
                        if (tp >= 0) {
                            to_route.push_back(tp);
                        } else {
                            TermId stub_term = parent.state.owned[oi].task.a;
                            NetId stub_net = parent.state.owned[oi].task.net;
                            for (int ti : prem) {
                                if (ti < 0 ||
                                    ti >= static_cast<int>(mctx.tasks->size()))
                                    continue;
                                const ConnectionTask& t = (*mctx.tasks)[ti];
                                if (t.net != stub_net) continue;
                                if (t.a == stub_term || t.b == stub_term)
                                    to_route.push_back(ti);
                            }
                        }
                    }
                    std::sort(to_route.begin(), to_route.end());
                    to_route.erase(
                        std::unique(to_route.begin(), to_route.end()),
                        to_route.end());
                    s.to_route = std::move(to_route);
                    specs.push_back(std::move(s));
                }
                if (specs.empty()) {
                    ++pi;
                    continue;
                }
                if (!wave.empty() && wave.size() + specs.size() > kWavePairCap)
                    break;  // flush this wave first; parent keeps its turn
                for (auto& s : specs) {
                    WavePair wp;
                    wp.pi = pi;
                    wp.spec = std::move(s);
                    wave.push_back(std::move(wp));
                }
                ++pi;
            }
            if (budget_hit || wave.empty()) continue;
            // Flat parallel reroute of the wave's pairs (indexed slots).
            int pair_n = static_cast<int>(wave.size());
            int wcap = std::max(1, std::min(workers, pair_n));
            std::vector<BranchResult> wres(static_cast<std::size_t>(pair_n));
            auto fn = [&](int w) {
                for (int c = w; c < pair_n; c += wcap) {
                    const WavePair& wp = wave[static_cast<std::size_t>(c)];
                    const BeamEntry& parent = cur[wp.pi];
                    const ChildSpec& s = wp.spec;
                    BranchResult r = reroute_branch(
                        parent.state.board, mctx.fixed_traces,
                        mctx.fixed_vias, s.surviving, s.to_route,
                        s.failed_ti, *mctx.tasks,
                        parent.state.remaining_task_ids, *mctx.corridors,
                        *mctx.resolver, *mctx.ectx, *mctx.layer_mult,
                        mctx.astar_cfg, mctx.congestion_tpl, mctx.mode_name,
                        s.move, mctx.hier_cfg, mctx.hier_cache,
                        mctx.reservation_strength, deadline);
                    wres[static_cast<std::size_t>(c)] = std::move(r);
                }
            };
            if (wcap == 1) {
                fn(0);
            } else {
                std::vector<std::thread> pool;
                for (int w = 0; w < wcap; ++w)
                    pool.emplace_back(fn, w);
                for (auto& th : pool) th.join();
            }
            // Serial TT prune in deterministic (parent, move-index) order.
            for (int c = 0; c < pair_n; ++c) {
                BranchResult& br = wres[static_cast<std::size_t>(c)];
                if (!br.evaluated) continue;
                int undone = static_cast<int>(br.remaining_task_ids.size());
                if (tt.should_prune(br.hash, undone)) {
                    br.pruned = true;
                    ++out.nodes_pruned;
                } else {
                    tt.record(br.hash, undone);
                }
            }
            for (int c = 0; c < pair_n; ++c) {
                const WavePair& wp = wave[static_cast<std::size_t>(c)];
                const BeamEntry& parent = cur[wp.pi];
                BranchResult& br = wres[static_cast<std::size_t>(c)];
                out.expansions_total += br.expansions;
                if (!br.evaluated || br.pruned) continue;
                BeamEntry e;
                e.state = std::move(br);
                e.pv = parent.pv;
                e.pv.push_back(wp.spec.move);
                e.first_idx = parent.first_idx;
                next_all.push_back(std::move(e));
            }
            out.nodes_evaluated += pair_n;
        }
        if (budget_hit) {
            out.fallback_to_one_ply = true;
            out.best_first_move = one_ply_best;
            if (one_ply_best >= 0 && one_ply_best < static_cast<int>(first_results.size())) {
                out.best_leaf = first_results[one_ply_best];
                out.has_best_leaf = true;
                out.best_pv.clear();
                out.best_pv.push_back(first_moves[one_ply_best]);
            }
            return out;
        }
        if (next_all.empty()) break;  // no deeper leaves: keep current best
        std::sort(next_all.begin(), next_all.end(),
                  [](const BeamEntry& a, const BeamEntry& b) {
                      if (branch_better(a.state, b.state)) return true;
                      if (branch_better(b.state, a.state)) return false;
                      if (a.first_idx != b.first_idx)
                          return a.first_idx < b.first_idx;
                      return a.state.hash.to_hex() < b.state.hash.to_hex();
                  });
        if (static_cast<int>(next_all.size()) > beam)
            next_all.resize(beam);
        // Track the best leaf across all depths (connectivity-first).
        if (branch_better(next_all.front().state, best_leaf.state))
            best_leaf = next_all.front();
        cur = std::move(next_all);
        // Stored-node cap: depth*beam boards max (streaming bound).
        if (static_cast<int>(cur.size()) > kMaxStoredMultiplyNodes)
            cur.resize(kMaxStoredMultiplyNodes);
    }
    out.best_first_move = best_leaf.first_idx;
    out.best_leaf = best_leaf.state;
    out.has_best_leaf = true;
    out.best_pv = best_leaf.pv;
    out.fallback_to_one_ply = false;
    return out;
}

JsonValue RecoveryInfo::to_json() const {
    JsonValue o = JsonValue::object();
    o["generations"] = static_cast<double>(generations);
    o["branches_evaluated"] = static_cast<double>(branches_evaluated);
    o["branches_pruned"] = static_cast<double>(branches_pruned);
    o["ripups"] = static_cast<double>(ripups);
    o["transposition_hits"] = static_cast<double>(transposition_hits);
    o["modes_attempted"] = json_string_array(modes_attempted);
    o["dependency_graph"] = last_graph.to_json();
    o["multiply_depth_requested"] = static_cast<double>(multiply_depth_requested);
    o["multiply_depth_effective"] = static_cast<double>(multiply_depth_effective);
    o["multiply_beam_requested"] = static_cast<double>(multiply_beam_requested);
    o["multiply_beam_effective"] = static_cast<double>(multiply_beam_effective);
    o["multiply_threads_effective"] = static_cast<double>(multiply_threads_effective);
    o["multiply_nodes_evaluated"] = static_cast<double>(multiply_nodes_evaluated);
    o["multiply_nodes_pruned"] = static_cast<double>(multiply_nodes_pruned);
    o["multiply_fallback_to_one_ply"] = multiply_fallback_to_one_ply;
    o["multiply_pv"] = json_string_array(multiply_pv);
    o["multiply_best_hash"] = multiply_best_hash;
    return o;
}

std::string result_category(const std::string& status) {
    if (status == "COMPLETE") return "COMPLETE";
    if (status == "VIOLATION") return "HARD_RULE_VIOLATION";
    if (status == "BUDGET_EXHAUSTED") return "SEARCH_BUDGET_EXHAUSTED_WITH_UNROUTED_CONNECTIONS";
    if (status == "TIMEOUT") return "TIMEOUT";
    return "UNROUTABLE_UNDER_CONFIGURED_CONSTRAINTS_AND_BUDGET";
}

}  // namespace copperline
