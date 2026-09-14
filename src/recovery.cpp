#include "router/recovery.h"

#include <algorithm>
#include <cstdio>
#include <map>
#include <set>

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
    o["closest_goal_dist_mm"] = nm_to_mm(closest_goal_dist_nm);
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
    return d;
}

std::vector<BlockerHit> attribute_blockers_detailed(const Board& board,
                                                     const ConnectionTask& task,
                                                     Coord width_nm,
                                                     const CandidateRoute* last) {
    std::vector<BlockerHit> out;
    const Terminal* ta = board.find_terminal(task.a);
    const Terminal* tb = board.find_terminal(task.b);
    if (!ta || !tb) return out;
    Rect corridor = Rect::from_points(ta->pos, tb->pos).expanded(width_nm);
    struct Hit {
        BlockerHit h;
    };
    std::vector<BlockerHit> hits;
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
    std::sort(hits.begin(), hits.end(), [](const BlockerHit& a, const BlockerHit& b) {
        if (a.area != b.area) return a.area > b.area;
        return a.desc < b.desc;
    });
    for (std::size_t i = 0; i < hits.size() && i < 5; ++i) out.push_back(hits[i]);
    if (last && last->closest_node >= 0) {
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
        std::map<NetId, DependencyEdge> best;
        for (const auto& h : hits) {
            if (h.kind != "trace" && h.kind != "pad") {
                if (h.kind == "keepout" || h.kind == "frontier") {
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
    std::vector<std::string> keys;
    for (int ti : remaining) {
        const ConnectionTask& t = tasks[ti];
        keys.push_back(std::to_string(t.net) + ":" + std::to_string(std::min(t.a, t.b)) + ":" +
                       std::to_string(std::max(t.a, t.b)));
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
                                            const std::string& pv_key, RecoveryMode /*mode*/,
                                            int max_moves, int max_breadth) {
    std::vector<RipupMove> moves;
    // Owned routes by net for blocker lookup.
    std::map<NetId, std::vector<int>> owned_by_net;
    for (std::size_t i = 0; i < owned.size(); ++i) owned_by_net[owned[i].task.net].push_back((int)i);

    // Edges grouped per failed position.
    std::map<int, std::vector<DependencyEdge>> per_failed;
    for (const auto& e : graph.edges) {
        if (e.blocker_net < 0) continue;  // keepout/frontier: nothing to rip
        per_failed[e.failed_pos].push_back(e);
    }
    for (std::size_t fi = 0; fi < failed_tasks.size(); ++fi) {
        const ConnectionTask& failed = failed_tasks[fi];
        auto it = per_failed.find((int)fi);
        if (it == per_failed.end() || it->second.empty()) continue;
        // Distinct blocker nets, weight order (graph already sorted).
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
        for (NetId bn : blockers) {
            auto oit = owned_by_net.find(bn);
            if (oit == owned_by_net.end() || oit->second.empty()) continue;
            // Candidate owned routes of the blocker net, cheapest protection first.
            std::vector<int> cands = oit->second;
            std::sort(cands.begin(), cands.end(), [&](int a, int b) {
                if (owned[a].protection != owned[b].protection)
                    return owned[a].protection < owned[b].protection;
                if (owned[a].task.net != owned[b].task.net)
                    return owned[a].task.net < owned[b].task.net;
                if (owned[a].task.a != owned[b].task.a) return owned[a].task.a < owned[b].task.a;
                return owned[a].task.b < owned[b].task.b;
            });
            int breadth = std::min<int>(max_breadth, (int)cands.size());
            for (int k = 1; k <= breadth; ++k) {
                RipupMove m;
                m.failed_pos = (int)fi;
                m.failed_task = failed;
                m.blocker_net = bn;
                for (int j = 0; j < k; ++j) m.owned_idx.push_back(cands[j]);
                m.reason = "blocked_by_net_" + std::to_string(bn);
                m.gain = weight_of[bn];
                m.cost = 0;
                for (int oi : m.owned_idx) m.cost += owned[oi].protection;
                std::string hk = HistoryHeuristic::move_key(failed, bn);
                m.score = m.gain / (1.0 + m.cost) + history.bonus(hk);
                if (!pv_key.empty() && hk == pv_key) m.score += 1e9;  // PV reuse first
                moves.push_back(m);
                if ((int)moves.size() >= std::max(1, max_moves * 3)) break;
            }
            if ((int)moves.size() >= std::max(1, max_moves * 3)) break;
        }
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
    // Strict connectivity priority: more connected tasks always wins, even
    // when the board is longer or has more vias.
    if (a.connected_tasks != b.connected_tasks) return a.connected_tasks > b.connected_tasks;
    if (a.via_count != b.via_count) return a.via_count < b.via_count;
    if (a.length_nm != b.length_nm) return a.length_nm < b.length_nm;
    return a.hash.to_hex() < b.hash.to_hex();
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
                            const std::string& mode_name, const RipupMove& move) {
    BranchResult out;
    out.evaluated = true;
    out.mode = mode_name;
    out.move = move;
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

    std::vector<OwnedRoute> new_owned = surviving;
    std::vector<int> done;
    std::int64_t expansions = 0;
    for (std::size_t k = 0; k < order.size(); ++k) {
        int ti = order[k];
        std::size_t pos = 0;
        for (std::size_t j = 0; j < order.size(); ++j)
            if (order[j] == ti) {
                pos = j;
                break;
            }
        CandidateRoute cand = route_candidate_task(work, r, tasks[ti], pos, tasks[ti].difficulty,
                                                   ctx, layer_mult, astar_cfg, congestion,
                                                   reservations);
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
        for (const auto& t : cand.traces) congestion.add_history_segment(t.segment(), 0.25);
    }
    out.board = work;
    out.owned = new_owned;
    out.newly_done = done;
    out.connected_tasks = (int)done.size();
    out.total_tasks = (int)order.size();
    out.expansions = expansions;
    Coord len = 0;
    int vias = 0;
    for (const auto& o : new_owned) {
        for (const auto& t : o.traces) len += manhattan(t.a, t.b);
        vias += (int)o.vias.size();
    }
    out.length_nm = len;
    out.via_count = vias;
    // Exact unfinished set: (gen_remaining ∪ to_route) \ newly_done, sorted.
    // No transposition access here (workers never touch the table); the
    // arbiter prunes/records serially after join using this exact identity.
    {
        std::set<int> done_set(done.begin(), done.end());
        std::set<int> rest(gen_remaining.begin(), gen_remaining.end());
        for (int ti : to_route) rest.insert(ti);
        std::vector<int> rem;
        for (int ti : rest)
            if (!done_set.count(ti)) rem.push_back(ti);
        std::sort(rem.begin(), rem.end());
        out.remaining_task_ids = rem;
        out.hash = state_hash128(work, tasks, rem);
    }
    return out;
}

JsonValue RecoveryInfo::to_json() const {
    JsonValue o = JsonValue::object();
    o["generations"] = static_cast<double>(generations);
    o["branches_evaluated"] = static_cast<double>(branches_evaluated);
    o["branches_pruned"] = static_cast<double>(branches_pruned);
    o["ripups"] = static_cast<double>(ripups);
    o["transposition_hits"] = static_cast<double>(transposition_hits);
    JsonValue modes = JsonValue::array();
    for (const auto& m : modes_attempted) modes.as_array().push_back(JsonValue(m));
    o["modes_attempted"] = modes;
    o["dependency_graph"] = last_graph.to_json();
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
