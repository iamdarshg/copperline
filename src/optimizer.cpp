// Copperline: transactional cleanup optimizer (Prompt 5).
#include "router/optimizer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <map>
#include <set>
#include <thread>
#include <vector>

#include "router/board_stats.h"
#include "router/parallel.h"
#include "router/transaction_gate.h"

namespace copperline {
namespace {

int count_bends(const Board& board) {
    // Bends = direction changes between consecutive same-net/same-layer
    // segments sharing an endpoint. Order-independent: build per-net chains
    // by sorting, so the count is deterministic.
    int bends = 0;
    std::map<NetId, std::vector<const TraceSeg*>> by_net;
    for (const auto& s : board.traces) by_net[s.net].push_back(&s);
    for (const auto& kv : by_net) {
        std::vector<const TraceSeg*> segs = kv.second;
        std::sort(segs.begin(), segs.end(), [](const TraceSeg* a, const TraceSeg* b) {
            if (a->layer != b->layer) return a->layer < b->layer;
            if (a->a.x != b->a.x) return a->a.x < b->a.x;
            if (a->a.y != b->a.y) return a->a.y < b->a.y;
            if (a->b.x != b->b.x) return a->b.x < b->b.x;
            return a->b.y < b->b.y;
        });
        for (std::size_t i = 0; i + 1 < segs.size(); ++i) {
            const TraceSeg* a = segs[i];
            const TraceSeg* b = segs[i + 1];
            if (a->layer != b->layer) continue;
            if (!(a->b == b->a || a->b == b->b || a->a == b->a || a->a == b->b)) continue;
            Coord d1x = a->b.x - a->a.x, d1y = a->b.y - a->a.y;
            Coord d2x = b->b.x - b->a.x, d2y = b->b.y - b->a.y;
            __int128 cross = (__int128)d1x * d2y - (__int128)d1y * d2x;
            __int128 dot = (__int128)d1x * d2x + (__int128)d1y * d2y;
            if (cross != 0 || dot <= 0) bends++;
        }
    }
    return bends;
}

}  // namespace

// B6: batch-parallel candidate evaluation with serial transactional
// commit. One candidate spec: evaluated against a board snapshot in
// parallel, committed in deterministic scan order through the shared
// TransactionGate. The mutate functor must be a pure function of the
// board it is given (indices captured by value at enumeration time);
// the live board is untouched until the serial commit.
struct OptBatchCandidate {
    std::function<void(Board&)> mutate;
    bool must_shorten = false;
    NetId touched_net = -1;
    Rect touched_area{};
};

// Bound on parallel optimizer workers: each worker holds one transient
// Board copy plus verifier scratch, so copies x workers stays far under
// the 2048MB router budget even on large COMPLETE boards.
constexpr int kOptMaxWorkers = 8;

JsonValue OptimizerReport::to_json() const {
    JsonValue r = JsonValue::object();
    r["schema"] = "copperline/optimizer-report/1";
    r["enabled"] = enabled;
    r["ran"] = ran;
    if (!ran) r["gate_reason"] = gate_reason;
    r["passes"] = static_cast<double>(passes);
    r["candidates"] = static_cast<double>(candidates);
    r["applied"] = static_cast<double>(applied);
    r["reverted"] = static_cast<double>(reverted);
    r["bends_before"] = static_cast<double>(bends_before);
    r["bends_after"] = static_cast<double>(bends_after);
    r["vias_before"] = static_cast<double>(vias_before);
    r["vias_after"] = static_cast<double>(vias_after);
    r["length_before_mm"] = length_before_mm;
    r["length_after_mm"] = length_after_mm;
    r["straighten_ran"] = straighten_ran;
    r["via_miser_ran"] = via_miser_ran;
    r["orthogonal_ran"] = orthogonal_ran;
    r["clean_pad_entries_ran"] = clean_pad_entries_ran;
    r["hug_ran"] = hug_ran;
    r["spread_ran"] = spread_ran;
    r["peak_bytes"] = static_cast<double>(peak_bytes);
    return r;
}

CleanupOptimizer::CleanupOptimizer(Board* board, const RuleResolver* resolver,
                                   const ElectricalContext& ctx, OptimizerOptions options)
    : board_(board), resolver_(resolver), ctx_(ctx), opt_(options) {}

OptimizerReport CleanupOptimizer::run() {
    OptimizerReport rep;
    rep.enabled = opt_.enabled;
    if (!opt_.enabled) {
        rep.gate_reason = "disabled";
        return rep;
    }
    // Shared transactional gate (D6): snapshot/mutate/verify-or-revert
    // through the independent BoardVerifier. FINAL full gate always.
    TransactionGate gate(board_, resolver_, &ctx_);
    if (!gate.check()) {
        rep.gate_reason = "gated: input copper is not completely and legally connected";
        return rep;
    }
    rep.ran = true;
    rep.via_miser_ran = opt_.via_miser;
    rep.orthogonal_ran = opt_.orthogonal && !opt_.via_miser;
    rep.bends_before = count_bends(*board_);
    rep.vias_before = static_cast<int>(board_via_count(*board_));
    rep.length_before_mm = board_total_length_mm(*board_);
    double base_len = rep.length_before_mm;
    // Length-tuned and pair-coupled copper is load-bearing electrical
    // geometry (issue #15 targets, issue #12 skew): the optimizer must not
    // touch those nets. This is the "higher-priority electrical
    // requirements" revert rule made structural.
    std::set<NetId> protected_nets;
    for (const auto& n : board_->nets)
        if (n.has_target_length) protected_nets.insert(n.id);
    for (const auto& pr : board_->diffpairs) {
        protected_nets.insert(pr.net_p);
        protected_nets.insert(pr.net_n);
    }
    auto is_protected = [&](NetId net) { return protected_nets.count(net) != 0; };

    // B6: worker pool for candidate evaluation. Self-resolved (the
    // optimizer has no thread plumbing today; see follow-up): serial
    // commit below keeps every decision thread-count independent.
    int opt_workers = resolve_worker_threads(0);
    if (opt_workers < 1) opt_workers = 1;
    if (opt_workers > kOptMaxWorkers) opt_workers = kOptMaxWorkers;
    // Per-worker snapshot scratch, reused across batches (assignment
    // reuses capacity; no per-candidate allocation after warmup). Grown
    // lazily so candidate-free boards pay nothing.
    std::vector<Board> eval_scratch;
    eval_scratch.reserve(static_cast<std::size_t>(opt_workers));

    // Transaction helper (D6 shared gate + S3 delta pre-check): serial
    // transactional commit of one pre-evaluated candidate. Counting and
    // base_len updates are exactly the old attempt() semantics; a
    // pre-rejected candidate only re-counts (the board stays untouched,
    // as before). Returns true when the mutation was kept.
    auto commit_evaluated = [&](const OptBatchCandidate& c, bool pre_kept) -> bool {
        if (rep.candidates >= static_cast<int>(opt_.max_candidates)) return false;
        rep.candidates++;
        double now = base_len;
        bool kept = false;
        if (pre_kept) {
            kept = gate.try_apply_scoped(
                [&]() { c.mutate(*board_); },
                [&]() {
                    now = board_total_length_mm(*board_);
                    return !c.must_shorten || now <= base_len + 1e-9;
                },
                c.touched_net, c.touched_area);
        }
        if (!kept) {
            rep.reverted++;
        } else {
            rep.applied++;
            base_len = now;
        }
        return kept;
    };

    // Parallel legality/verify/length verdicts for one window against the
    // current board, in window order. Workers mutate private snapshots
    // only (indexed slots over eval_scratch); the live board is purely
    // read. Deterministic: identical inputs yield identical verdicts at
    // any worker count.
    auto eval_window = [&](const std::vector<OptBatchCandidate>& window,
                           std::vector<char>& keep) {
        std::size_t n = window.size();
        keep.assign(n, 0);
        if (n == 0) return;
        int w = static_cast<int>(std::min<std::size_t>(n, (std::size_t)opt_workers));
        if (w < 1) w = 1;
        while (eval_scratch.size() < static_cast<std::size_t>(w))
            eval_scratch.push_back(*board_);
        const Board& base = *board_;
        double window_base = base_len;
        auto fn = [&](int tid) {
            BoardVerifier verifier;  // per-thread instance: no shared mutable state
            for (std::size_t k = static_cast<std::size_t>(tid); k < n;
                 k += static_cast<std::size_t>(w)) {
                const OptBatchCandidate& c = window[k];
                Board& tmp = eval_scratch[static_cast<std::size_t>(tid)];
                tmp = base;
                c.mutate(tmp);
                if (verifier.has_local_violation(tmp, *resolver_, ctx_, c.touched_net,
                                                c.touched_area))
                    continue;
                if (!verifier.verify(tmp, *resolver_, ctx_).ok) continue;
                if (c.must_shorten &&
                    !(board_total_length_mm(tmp) <= window_base + 1e-9))
                    continue;
                keep[k] = 1;
            }
        };
        if (w == 1) {
            fn(0);
        } else {
            std::vector<std::thread> pool;
            for (int tid = 0; tid < w; ++tid) pool.emplace_back(fn, tid);
            for (auto& th : pool) th.join();
        }
    };

    // Serial commit of an evaluated window in order, stopping after the
    // first accept (#37: scan restarts only on accept; later verdicts,
    // computed against the pre-accept board, are discarded and
    // re-enumerated from fresh state). Returns the accepted window index,
    // -1 when everything was rejected, -2 on candidate-cap exhaustion.
    auto commit_until_accept = [&](const std::vector<OptBatchCandidate>& window,
                                   const std::vector<char>& keep) -> int {
        for (std::size_t k = 0; k < window.size(); ++k) {
            if (rep.candidates >= static_cast<int>(opt_.max_candidates)) return -2;
            if (commit_evaluated(window[k], keep[k] != 0)) return static_cast<int>(k);
        }
        return -1;
    };

    // Pass 1: collinear merge of consecutive same-net/same-layer segments.
    if (opt_.collinear_merge && !opt_.via_miser) {
        rep.passes++;
        bool changed = true;
        int guard = 0;
        while (changed && guard++ < 4) {
            changed = false;
            // Deterministic scan order: sort indices by (net, layer, a, b).
            std::vector<std::size_t> order(board_->traces.size());
            for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
            std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
                const TraceSeg& A = board_->traces[a];
                const TraceSeg& B = board_->traces[b];
                if (A.net != B.net) return A.net < B.net;
                if (A.layer != B.layer) return A.layer < B.layer;
                if (A.a.x != B.a.x) return A.a.x < B.a.x;
                if (A.a.y != B.a.y) return A.a.y < B.a.y;
                if (A.b.x != B.b.x) return A.b.x < B.b.x;
                return A.b.y < B.b.y;
            });
            // Windowed enumeration in scan order (overlapping adjacent
            // pairs, exactly the old sequence): cheap structural checks
            // stay serial; each window is evaluated in parallel and
            // committed serially. The first accept restarts the scan
            // (re-sort after mutation, as before); #37 holds.
            std::size_t ii = 0;
            while (ii + 1 < order.size() && !changed &&
                   rep.candidates < static_cast<int>(opt_.max_candidates)) {
                std::vector<OptBatchCandidate> window;
                while (ii + 1 < order.size() &&
                       (int)window.size() < opt_workers) {
                    std::size_t i = order[ii], j = order[ii + 1];
                    ++ii;
                    if (i >= board_->traces.size() || j >= board_->traces.size()) break;
                    const TraceSeg A = board_->traces[i], B = board_->traces[j];
                    if (A.net != B.net || A.layer != B.layer) continue;
                    if (is_protected(A.net)) continue;
                    if (!(A.b == B.a)) continue;
                    Coord d1x = A.b.x - A.a.x, d1y = A.b.y - A.a.y;
                    Coord d2x = B.b.x - B.a.x, d2y = B.b.y - B.a.y;
                    __int128 cross = (__int128)d1x * d2y - (__int128)d1y * d2x;
                    __int128 dot = (__int128)d1x * d2x + (__int128)d1y * d2y;
                    if (cross != 0 || dot <= 0) continue;
                    OptBatchCandidate c;
                    c.must_shorten = false;
                    c.touched_net = A.net;
                    c.touched_area =
                        rect_union(A.segment().bounds(), B.segment().bounds());
                    TraceSeg Bc = B;
                    c.mutate = [i, j, Bc](Board& bd) {
                        bd.traces[i].b = Bc.b;
                        bd.traces.erase(bd.traces.begin() +
                                          static_cast<std::ptrdiff_t>(j));
                    };
                    window.push_back(std::move(c));
                }
                if (window.empty()) break;
                std::vector<char> keep;
                eval_window(window, keep);
                if (commit_until_accept(window, keep) >= 0) changed = true;
            }
        }
    }

    // Pass 2: bend removal — replace chained corner A-B, B-C (same net/layer)
    // with a direct A-C segment when it shortens and stays legal.
    // Windowed (i, j) row-major enumeration, exactly the old sequence;
    // first accept restarts the full scan (#37).
    if (opt_.bend_removal && !opt_.via_miser) {
        rep.passes++;
        bool changed = true;
        int guard = 0;
        while (changed && guard++ < 8 &&
               rep.candidates < static_cast<int>(opt_.max_candidates)) {
            changed = false;
            const std::size_t ntr = board_->traces.size();
            const std::size_t nn = ntr * ntr;
            std::size_t t = 0;
            while (t < nn && !changed &&
                   rep.candidates < static_cast<int>(opt_.max_candidates)) {
                std::vector<OptBatchCandidate> window;
                while (t < nn && (int)window.size() < opt_workers) {
                    std::size_t i = t / ntr, j = t % ntr;
                    ++t;
                    if (i == j) continue;
                    const TraceSeg A = board_->traces[i], B = board_->traces[j];
                    if (A.net != B.net || A.layer != B.layer) continue;
                    if (is_protected(A.net)) continue;
                    if (!(A.b == B.a)) continue;
                    // Corner at A.b: direct shortcut A.a -> B.b.
                    double old_len = trace_seg_length_mm(A) + trace_seg_length_mm(B);
                    TraceSeg shortcut = A;
                    shortcut.b = B.b;
                    double new_len = trace_seg_length_mm(shortcut);
                    if (new_len >= old_len - 1e-9) continue;
                    OptBatchCandidate c;
                    c.must_shorten = true;
                    c.touched_net = A.net;
                    c.touched_area =
                        rect_union(A.segment().bounds(), B.segment().bounds());
                    TraceSeg Bc = B;
                    c.mutate = [i, j, Bc](Board& bd) {
                        bd.traces[i].b = Bc.b;
                        bd.traces.erase(bd.traces.begin() +
                                          static_cast<std::ptrdiff_t>(j));
                    };
                    window.push_back(std::move(c));
                }
                if (window.empty()) break;
                std::vector<char> keep;
                eval_window(window, keep);
                if (commit_until_accept(window, keep) >= 0) changed = true;
            }
        }
    }

    // Pass 3: via elimination — a via joining exactly two same-net segments
    // on adjacent layers is removed and both sides joined on the first layer.
    // Windowed vi-order enumeration; an accept restarts from 0 (#37).
    if (opt_.via_elimination) {
        rep.passes++;
        std::size_t vi = 0;
        while (vi < board_->vias.size() &&
               rep.candidates < static_cast<int>(opt_.max_candidates)) {
            std::vector<OptBatchCandidate> window;
            std::size_t cursor = vi;
            while (cursor < board_->vias.size() &&
                   (int)window.size() < opt_workers) {
                Via v = board_->vias[cursor];
                std::size_t cur = cursor;
                ++cursor;
                if (is_protected(v.net)) continue;
                std::size_t s1 = board_->traces.size(), s2 = board_->traces.size();
                int hits = 0;
                for (std::size_t si = 0; si < board_->traces.size(); ++si) {
                    const TraceSeg& s = board_->traces[si];
                    if (s.net != v.net) continue;
                    if (s.a == v.pos || s.b == v.pos) {
                        if (hits == 0) s1 = si;
                        if (hits == 1) s2 = si;
                        hits++;
                    }
                }
                if (hits != 2 || s1 == s2) continue;
                TraceSeg A = board_->traces[s1], B = board_->traces[s2];
                // Join the far endpoints on A's layer.
                Point far_a = (A.a == v.pos) ? A.b : A.a;
                Point far_b = (B.a == v.pos) ? B.b : B.a;
                std::size_t lo = std::min(s1, s2), hi = std::max(s1, s2);
                Rect touched =
                    rect_union(Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm),
                               rect_union(A.segment().bounds(), B.segment().bounds()));
                OptBatchCandidate c;
                c.must_shorten = false;
                c.touched_net = v.net;
                c.touched_area = touched;
                LayerId layer = A.layer;
                c.mutate = [cur, lo, hi, far_a, far_b, layer](Board& bd) {
                    bd.traces[lo].a = far_a;
                    bd.traces[lo].b = far_b;
                    bd.traces[lo].layer = layer;
                    bd.traces.erase(bd.traces.begin() +
                                      static_cast<std::ptrdiff_t>(hi));
                    bd.vias.erase(bd.vias.begin() +
                                    static_cast<std::ptrdiff_t>(cur));
                };
                window.push_back(std::move(c));
            }
            if (window.empty()) break;
            std::vector<char> keep;
            eval_window(window, keep);
            int hit = commit_until_accept(window, keep);
            if (hit >= 0) {
                vi = 0;  // accepted removal shifted indices: rescan
            } else if (hit == -1) {
                vi = cursor;  // all rejected: continue past the window
            } else {
                break;  // candidate-cap exhaustion: report counts as-is
            }
        }
    }

    // Pass 4: preferred-layer — move a net whose copper sits on exactly one
    // non-preferred layer onto the preferred layer (layers[0]).
    // Windowed net-order enumeration (no restart in this pass: an accept
    // continues after the accepted net on fresh state; later window
    // verdicts are discarded and re-enumerated so a stale reject can never
    // strand a candidate the serial scan would accept).
    if (opt_.preferred_layer && !opt_.via_miser && !board_->layers.empty()) {
        rep.passes++;
        LayerId pref = board_->layers.front().id;
        std::map<NetId, std::vector<std::size_t>> segs_by_net;
        for (std::size_t i = 0; i < board_->traces.size(); ++i)
            segs_by_net[board_->traces[i].net].push_back(i);
        // Deterministic net order.
        std::vector<NetId> nets;
        for (const auto& kv : segs_by_net) nets.push_back(kv.first);
        std::sort(nets.begin(), nets.end());
        std::size_t ni = 0;
        while (ni < nets.size() &&
               rep.candidates < static_cast<int>(opt_.max_candidates)) {
            std::vector<OptBatchCandidate> window;
            std::vector<std::size_t> window_ni;
            std::size_t cursor = ni;
            while (cursor < nets.size() && (int)window.size() < opt_workers) {
                NetId n = nets[cursor];
                std::size_t cur_ni = cursor;
                ++cursor;
                if (is_protected(n)) continue;
                // Skip nets with vias (layer transitions are load-bearing).
                bool has_via = false;
                for (const auto& v : board_->vias)
                    if (v.net == n) has_via = true;
                if (has_via) continue;
                LayerId only = segs_by_net[n].empty()
                                   ? pref
                                   : board_->traces[segs_by_net[n][0]].layer;
                bool single = true;
                for (std::size_t si : segs_by_net[n])
                    if (board_->traces[si].layer != only) single = false;
                if (!single || only == pref) continue;
                // Layer-only mutation: indices stay valid across commits,
                // but verdicts are still refreshed after every accept.
                std::vector<std::size_t> segs = segs_by_net[n];
                OptBatchCandidate c;
                c.must_shorten = false;
                c.touched_net = n;
                c.touched_area = board_->bounds();
                c.mutate = [segs, pref](Board& bd) {
                    for (std::size_t si : segs) bd.traces[si].layer = pref;
                };
                window.push_back(std::move(c));
                window_ni.push_back(cur_ni);
            }
            if (window.empty()) break;
            std::vector<char> keep;
            eval_window(window, keep);
            int hit = commit_until_accept(window, keep);
            if (hit >= 0) {
                ni = window_ni[static_cast<std::size_t>(hit)] + 1;
            } else if (hit == -1) {
                ni = cursor;
            } else {
                break;  // candidate-cap exhaustion: report counts as-is
            }
        }
    }

    // Pass 5: straighten (Phase 3 item 19) — corner probe independent of
    // trace endpoint orientation. Exactly-2 same-net/same-layer segments
    // sharing an endpoint (no via at the corner) collapse to a direct
    // shortcut when it shortens; every candidate re-verifies and reverts
    // on harm. OFF by default; enabled by strategy "straighten": true.
    if (opt_.straighten && !opt_.via_miser) {
        rep.passes++;
        rep.straighten_ran = true;
        bool changed = true;
        int guard = 0;
        while (changed && guard++ < 12 &&
               rep.candidates < static_cast<int>(opt_.max_candidates)) {
            changed = false;
            // Endpoint degree per (net, point) for the exact-2 gate.
            std::map<NetId, std::map<Point, int>> degree;
            for (const auto& seg : board_->traces)
                degree[seg.net][seg.a]++, degree[seg.net][seg.b]++;
            const std::size_t ntr = board_->traces.size();
            const std::size_t nn = ntr * ntr;
            std::size_t t = 0;
            while (t < nn && !changed &&
                   rep.candidates < static_cast<int>(opt_.max_candidates)) {
                std::vector<OptBatchCandidate> window;
                while (t < nn && (int)window.size() < opt_workers) {
                    std::size_t i = t / ntr, j = t % ntr;
                    ++t;
                    if (i == j) continue;
                    const TraceSeg A = board_->traces[i], B = board_->traces[j];
                    if (A.net != B.net || A.layer != B.layer) continue;
                    if (is_protected(A.net)) continue;
                    Point corner, from, to;
                    bool joined = false;
                    if (A.a == B.a) {
                        corner = A.a; from = A.b; to = B.b; joined = true;
                    } else if (A.a == B.b) {
                        corner = A.a; from = A.b; to = B.a; joined = true;
                    } else if (A.b == B.a) {
                        corner = A.b; from = A.a; to = B.b; joined = true;
                    } else if (A.b == B.b) {
                        corner = A.b; from = A.a; to = B.a; joined = true;
                    }
                    if (!joined || from == to) continue;
                    if (degree[A.net][corner] != 2) continue;
                    // A via wedged into the corner is a junction: never
                    // shortcut across it (via elimination already had its turn).
                    bool via_at_corner = false;
                    for (const auto& v : board_->vias)
                        if (v.net == A.net && v.pos == corner) via_at_corner = true;
                    if (via_at_corner) continue;
                    double old_len = trace_seg_length_mm(A) + trace_seg_length_mm(B);
                    TraceSeg shortcut = A;
                    shortcut.a = from;
                    shortcut.b = to;
                    double new_len = trace_seg_length_mm(shortcut);
                    if (new_len >= old_len - 1e-9) continue;
                    OptBatchCandidate c;
                    c.must_shorten = true;
                    c.touched_net = A.net;
                    c.touched_area =
                        rect_union(A.segment().bounds(), B.segment().bounds());
                    c.mutate = [i, j, from, to](Board& bd) {
                        bd.traces[i].a = from;
                        bd.traces[i].b = to;
                        bd.traces.erase(bd.traces.begin() +
                                        static_cast<std::ptrdiff_t>(j));
                    };
                    window.push_back(std::move(c));
                }
                if (window.empty()) break;
                std::vector<char> keep;
                eval_window(window, keep);
                if (commit_until_accept(window, keep) >= 0) changed = true;
            }
        }
    }

    // Pass 6: recorner/miter (Phase 3 item 20) — chamfer axis-aligned
    // 90-degree corners with a 45-degree segment when a strategy sets
    // "orthogonal": true. The chamfer shortens the pair and removes the sharp
    // corner; every candidate re-verifies and reverts on harm. OFF by default.
    if (opt_.orthogonal && !opt_.via_miser) {
        rep.passes++;
        bool changed = true;
        int guard = 0;
        while (changed && guard++ < 12 &&
               rep.candidates < static_cast<int>(opt_.max_candidates)) {
            changed = false;
            std::map<NetId, std::map<Point, int>> degree;
            for (const auto& seg : board_->traces)
                degree[seg.net][seg.a]++, degree[seg.net][seg.b]++;
            const std::size_t ntr = board_->traces.size();
            const std::size_t nn = ntr * ntr;
            std::size_t t = 0;
            while (t < nn && !changed &&
                   rep.candidates < static_cast<int>(opt_.max_candidates)) {
                std::vector<OptBatchCandidate> window;
                while (t < nn && (int)window.size() < opt_workers) {
                    std::size_t i = t / ntr, j = t % ntr;
                    ++t;
                    if (i == j) continue;
                    const TraceSeg A = board_->traces[i], B = board_->traces[j];
                    if (A.net != B.net || A.layer != B.layer) continue;
                    if (is_protected(A.net)) continue;
                    // Shared corner, both legs axis-aligned and perpendicular.
                    Point corner, a_far, b_far;
                    bool joined = false;
                    if (A.a == B.a) {
                        corner = A.a; a_far = A.b; b_far = B.b; joined = true;
                    } else if (A.a == B.b) {
                        corner = A.a; a_far = A.b; b_far = B.a; joined = true;
                    } else if (A.b == B.a) {
                        corner = A.b; a_far = A.a; b_far = B.b; joined = true;
                    } else if (A.b == B.b) {
                        corner = A.b; a_far = A.a; b_far = B.a; joined = true;
                    }
                    if (!joined) continue;
                    if (degree[A.net][corner] != 2) continue;
                    const bool a_h = (a_far.y == corner.y);
                    const bool a_v = (a_far.x == corner.x);
                    const bool b_h = (b_far.y == corner.y);
                    const bool b_v = (b_far.x == corner.x);
                    if (!((a_h && b_v) || (a_v && b_h))) continue;
                    // A via at the corner is a junction: never chamfer it.
                    bool via_at_corner = false;
                    for (const auto& v : board_->vias)
                        if (v.net == A.net && v.pos == corner) via_at_corner = true;
                    if (via_at_corner) continue;
                    const Coord la = manhattan(corner, a_far);
                    const Coord lb = manhattan(corner, b_far);
                    if (la <= 0 || lb <= 0) continue;
                    // Chamfer offset bounded by half of each leg.
                    Coord d = A.width_nm / 2;
                    if (d < 1) d = 1;
                    if (d > la / 2) d = la / 2;
                    if (d > lb / 2) d = lb / 2;
                    if (d <= 0) continue;
                    auto step = [d](Coord from, Coord to) {
                        return from + (to > from ? d : (to < from ? -d : 0));
                    };
                    const Point P{step(corner.x, a_far.x), step(corner.y, a_far.y)};
                    const Point Q{step(corner.x, b_far.x), step(corner.y, b_far.y)};
                    if (P == Q) continue;
                    // The chamfer must shorten the pair.
                    const double old_len =
                        trace_seg_length_mm(A) + trace_seg_length_mm(B);
                    const double new_len =
                        static_cast<double>(euclid_len_nm(a_far, P)) / 1e6 +
                        static_cast<double>(euclid_len_nm(P, Q)) / 1e6 +
                        static_cast<double>(euclid_len_nm(Q, b_far)) / 1e6;
                    if (new_len >= old_len - 1e-9) continue;
                    OptBatchCandidate c;
                    c.must_shorten = true;
                    c.touched_net = A.net;
                    c.touched_area =
                        rect_union(A.segment().bounds(), B.segment().bounds());
                    const Coord w = A.width_nm;
                    const LayerId layer = A.layer;
                    const NetId net = A.net;
                    c.mutate = [i, j, a_far, b_far, P, Q, w, layer, net](Board& bd) {
                        bd.traces[i] = TraceSeg{net, layer, a_far, P, w};
                        bd.traces[j] = TraceSeg{net, layer, Q, b_far, w};
                        bd.traces.push_back(TraceSeg{net, layer, P, Q, w});
                    };
                    window.push_back(std::move(c));
                }
                if (window.empty()) break;
                std::vector<char> keep;
                eval_window(window, keep);
                if (commit_until_accept(window, keep) >= 0) changed = true;
            }
        }
    }

    // Pass 7: clean pad entries (Phase 3 item 18) — for asymmetric
    // rectangular pads whose single exit runs along the SHORT axis, reroute
    // the exit along the LONG axis (evidence: asymmetric-pad cleanliness).
    // Verify+revert like every transform; OFF by default ("clean_pad_entries").
    if (opt_.clean_pad_entries && !opt_.via_miser) {
        rep.passes++;
        rep.clean_pad_entries_ran = true;
        Coord clear = board_->defaults.clearance_nm;
        if (clear <= 0) clear = mm_to_nm(0.15);
        bool changed = true;
        int guard = 0;
        while (changed && guard++ < 8 &&
               rep.candidates < static_cast<int>(opt_.max_candidates)) {
            changed = false;
            std::vector<std::size_t> order(board_->terminals.size());
            for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
            std::sort(order.begin(), order.end(), [&](std::size_t ia, std::size_t ib) {
                const Terminal& A = board_->terminals[ia];
                const Terminal& B = board_->terminals[ib];
                if (A.net != B.net) return A.net < B.net;
                if (A.pos.x != B.pos.x) return A.pos.x < B.pos.x;
                if (A.pos.y != B.pos.y) return A.pos.y < B.pos.y;
                return A.id < B.id;
            });
            std::size_t oi = 0;
            while (oi < order.size() && !changed &&
                   rep.candidates < static_cast<int>(opt_.max_candidates)) {
                std::vector<OptBatchCandidate> window;
                while (oi < order.size() && (int)window.size() < opt_workers) {
                    const Terminal& t = board_->terminals[order[oi]];
                    ++oi;
                    if (t.pad_w_nm <= 0 || t.pad_h_nm <= 0) continue;
                    const Coord dw = t.pad_w_nm - t.pad_h_nm;
                    const Coord asym = dw < 0 ? -dw : dw;
                    if (asym < mm_to_nm(0.05)) continue;  // near-square: leave
                    const bool prefer_x = t.pad_w_nm > t.pad_h_nm;
                    // Exactly one trace of this net/layer ends on the pad.
                    std::size_t si = board_->traces.size();
                    int hits = 0;
                    for (std::size_t k = 0; k < board_->traces.size(); ++k) {
                        const TraceSeg& s = board_->traces[k];
                        if (s.net != t.net || s.layer != t.layer) continue;
                        if (s.a == t.pos || s.b == t.pos) {
                            si = k;
                            ++hits;
                        }
                    }
                    if (hits != 1) continue;  // ambiguous entry: leave it
                    const TraceSeg s = board_->traces[si];
                    if (is_protected(s.net)) continue;
                    const Point far = (s.a == t.pos) ? s.b : s.a;
                    const bool cur_x = (s.a.y == s.b.y);
                    const bool cur_y = (s.a.x == s.b.x);
                    if (!cur_x && !cur_y) continue;  // diagonal exit: leave it
                    if (prefer_x && cur_x) continue;   // already along long axis
                    if (!prefer_x && cur_y) continue;
                    const Coord need =
                        (prefer_x ? t.pad_w_nm / 2 : t.pad_h_nm / 2) +
                        s.width_nm / 2 + clear;
                    const Coord delta =
                        prefer_x ? (far.x - t.pos.x) : (far.y - t.pos.y);
                    const Coord sign = delta < 0 ? -1 : 1;
                    Point e = t.pos;
                    if (prefer_x) e.x += sign * need; else e.y += sign * need;
                    if (e == far) continue;
                    OptBatchCandidate c;
                    c.must_shorten = false;
                    c.touched_net = s.net;
                    c.touched_area = rect_union(s.segment().bounds(), t.pad_rect());
                    const Coord w = s.width_nm;
                    const LayerId layer = s.layer;
                    const NetId net = s.net;
                    const Point p = t.pos;
                    const Point ep = e;
                    const Point fp = far;
                    c.mutate = [si, net, layer, p, ep, fp, w](Board& bd) {
                        bd.traces[si] = TraceSeg{net, layer, p, ep, w};
                        bd.traces.push_back(TraceSeg{net, layer, ep, fp, w});
                    };
                    window.push_back(std::move(c));
                }
                if (window.empty()) break;
                std::vector<char> keep;
                eval_window(window, keep);
                if (commit_until_accept(window, keep) >= 0) changed = true;
            }
        }
    }

    // Pass 8: hug / spread (Phase 3 items 22/23) — move a straight parallel
    // segment perpendicular to its run, stitching it back with two short
    // perpendicular stubs:
    //   hug    — move toward the nearest same-net parallel segment to minimum
    //            same-net spacing (frees the opposite side of the channel)
    //   spread — move to the midpoint of the nearest parallel neighbours, so
    //            the two gaps around it become equal
    // Every candidate re-verifies and reverts; the shift is bounded, so the
    // added stub length is bounded too. OFF by default.
    if ((opt_.hug || opt_.spread) && !opt_.via_miser) {
        rep.passes++;
        rep.hug_ran = opt_.hug;
        rep.spread_ran = opt_.spread;
        const Coord max_shift = mm_to_nm(0.5);   // bound per segment
        const Coord min_shift = mm_to_nm(0.01);  // ignore noise
        bool changed = true;
        int guard = 0;
        while (changed && guard++ < 8 &&
               rep.candidates < static_cast<int>(opt_.max_candidates)) {
            changed = false;
            const std::size_t ntr = board_->traces.size();
            std::vector<OptBatchCandidate> window;
            for (std::size_t i = 0; i < ntr && (int)window.size() < opt_workers; ++i) {
                const TraceSeg s = board_->traces[i];
                if (is_protected(s.net)) continue;
                const bool horiz = (s.a.y == s.b.y);
                const bool vert = (s.a.x == s.b.x);
                if (!horiz && !vert) continue;  // only straight runs
                // Only real runs are worth moving; this also stops the pass
                // from churning on the perpendicular stubs it just created.
                if (manhattan(s.a, s.b) < mm_to_nm(1.0)) continue;
                const Coord off = horiz ? s.a.y : s.a.x;
                const Coord lo = horiz ? std::min(s.a.x, s.b.x) : std::min(s.a.y, s.b.y);
                const Coord hi = horiz ? std::max(s.a.x, s.b.x) : std::max(s.a.y, s.b.y);
                // Nearest parallel neighbours (below / above in offset).
                bool has_below = false, has_above = false;
                Coord below_off = 0, above_off = 0;
                bool hug_found = false;
                Coord hug_gap = 0, hug_target = 0;
                Coord hug_dir = 0;
                for (std::size_t j = 0; j < ntr; ++j) {
                    if (j == i) continue;
                    const TraceSeg& o = board_->traces[j];
                    if (o.layer != s.layer) continue;
                    const bool o_h = (o.a.y == o.b.y);
                    const bool o_v = (o.a.x == o.b.x);
                    if ((horiz && !o_h) || (vert && !o_v)) continue;
                    const Coord o_off = horiz ? o.a.y : o.a.x;
                    if (o_off == off) continue;
                    const Coord o_lo =
                        horiz ? std::min(o.a.x, o.b.x) : std::min(o.a.y, o.b.y);
                    const Coord o_hi =
                        horiz ? std::max(o.a.x, o.b.x) : std::max(o.a.y, o.b.y);
                    if (std::min(hi, o_hi) - std::max(lo, o_lo) <= 0) continue;
                    if (o_off < off) {
                        if (!has_below || o_off > below_off) below_off = o_off;
                        has_below = true;
                    } else {
                        if (!has_above || o_off < above_off) above_off = o_off;
                        has_above = true;
                    }
                    // Hug: nearest parallel run on either side, same net or
                    // foreign. Foreign neighbours are hugged to exactly the
                    // required clearance; same-net runs to minimum spacing.
                    if (opt_.hug) {
                        const Coord gap = o_off > off ? o_off - off : off - o_off;
                        Coord target = 0;
                        if (o.net == s.net) {
                            target = std::max<Coord>(
                                s.width_nm * 2,
                                (s.width_nm + o.width_nm) / 2 + 1);
                        } else {
                            Coord need = resolver_
                                             ? resolver_->requiredClearance(
                                                   s.net, o.net, s.layer, ctx_, nullptr)
                                             : 0;
                            if (need <= 0) need = mm_to_nm(0.15);
                            // edge gap == clearance, plus a 10um safety margin
                            target = need + (s.width_nm + o.width_nm) / 2 +
                                     mm_to_nm(0.01);
                        }
                        if (gap <= target) continue;
                        if (!hug_found || gap < hug_gap) {
                            hug_found = true;
                            hug_gap = gap;
                            hug_target = target;
                            hug_dir = o_off < off ? -1 : 1;
                        }
                    }
                }
                Coord shift = 0;
                if (opt_.hug && hug_found) {
                    const Coord step = std::min(hug_gap - hug_target, max_shift);
                    if (step > min_shift) shift = hug_dir * step;
                }
                if (opt_.spread && shift == 0 && has_below && has_above) {
                    const Coord target = (below_off + above_off) / 2;
                    const Coord delta = target - off;
                    const Coord mag = delta < 0 ? -delta : delta;
                    if (mag > min_shift && mag <= max_shift) shift = delta;
                }
                if (shift == 0) continue;
                // Shifted run + two perpendicular stubs, preserving direction.
                const Point A = s.a, B = s.b;
                Point A2 = A, B2 = B;
                if (horiz) {
                    A2.y += shift;
                    B2.y += shift;
                } else {
                    A2.x += shift;
                    B2.x += shift;
                }
                OptBatchCandidate c;
                c.must_shorten = false;  // hugging/equalising may add stub length
                c.touched_net = s.net;
                c.touched_area = rect_union(
                    s.segment().bounds(),
                    rect_union(Rect::from_points(A2, B2), Rect::from_points(A, B)));
                const Coord w = s.width_nm;
                const LayerId layer = s.layer;
                const NetId net = s.net;
                c.mutate = [i, net, layer, A, A2, B2, B, w](Board& bd) {
                    bd.traces[i] = TraceSeg{net, layer, A, A2, w};
                    bd.traces.push_back(TraceSeg{net, layer, A2, B2, w});
                    bd.traces.push_back(TraceSeg{net, layer, B2, B, w});
                };
                window.push_back(std::move(c));
            }
            if (window.empty()) break;
            std::vector<char> keep;
            eval_window(window, keep);
            if (commit_until_accept(window, keep) >= 0) changed = true;
        }
    }

    rep.bends_after = count_bends(*board_);
    rep.vias_after = static_cast<int>(board_via_count(*board_));
    rep.length_after_mm = board_total_length_mm(*board_);
    rep.peak_bytes = rep.candidates * 64;  // bounded scratch accounting
    return rep;
}

}  // namespace copperline
