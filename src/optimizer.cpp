// Copperline: transactional cleanup optimizer (Prompt 5).
#include "router/optimizer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <set>

#include "router/board_stats.h"
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

    // Transaction helper (D6 shared gate + S3 delta pre-check): snapshot,
    // mutate, scoped sound-reject on the touched net/bbox, FINAL full
    // verify, keep-or-revert.
    auto attempt = [&](auto&& mutate, bool must_shorten, NetId touched_net,
                       const Rect& touched_area) {
        if (rep.candidates >= static_cast<int>(opt_.max_candidates)) return;
        rep.candidates++;
        double now = base_len;
        bool kept = gate.try_apply_scoped(
            mutate,
            [&]() {
                now = board_total_length_mm(*board_);
                return !must_shorten || now <= base_len + 1e-9;
            },
            touched_net, touched_area);
        if (!kept) {
            rep.reverted++;
        } else {
            rep.applied++;
            base_len = now;
        }
    };

    // Pass 1: collinear merge of consecutive same-net/same-layer segments.
    if (opt_.collinear_merge) {
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
            for (std::size_t ii = 0; ii + 1 < order.size(); ++ii) {
                std::size_t i = order[ii], j = order[ii + 1];
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
                attempt(
                    [&]() {
                        board_->traces[i].b = B.b;
                        board_->traces.erase(board_->traces.begin() +
                                             static_cast<std::ptrdiff_t>(j));
                    },
                    /*must_shorten=*/false, A.net,
                    rect_union(A.segment().bounds(), B.segment().bounds()));
                changed = true;
                break;  // re-sort after mutation (indices shift)
            }
        }
    }

    // Pass 2: bend removal — replace chained corner A-B, B-C (same net/layer)
    // with a direct A-C segment when it shortens and stays legal.
    if (opt_.bend_removal) {
        rep.passes++;
        bool changed = true;
        int guard = 0;
        while (changed && guard++ < 8 &&
               rep.candidates < static_cast<int>(opt_.max_candidates)) {
            changed = false;
            for (std::size_t i = 0; i < board_->traces.size() && !changed; ++i) {
                for (std::size_t j = 0; j < board_->traces.size() && !changed; ++j) {
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
                    attempt(
                        [&]() {
                            board_->traces[i].b = B.b;
                            board_->traces.erase(board_->traces.begin() +
                                                 static_cast<std::ptrdiff_t>(j));
                        },
                        /*must_shorten=*/true, A.net,
                        rect_union(A.segment().bounds(), B.segment().bounds()));
                    changed = true;
                }
            }
        }
    }

    // Pass 3: via elimination — a via joining exactly two same-net segments
    // on adjacent layers is removed and both sides joined on the first layer.
    if (opt_.via_elimination) {
        rep.passes++;
        for (std::size_t vi = 0; vi < board_->vias.size() &&
                                 rep.candidates < static_cast<int>(opt_.max_candidates);) {
            Via v = board_->vias[vi];
            if (is_protected(v.net)) {
                ++vi;
                continue;
            }
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
            if (hits != 2 || s1 == s2) {
                ++vi;
                continue;
            }
            TraceSeg A = board_->traces[s1], B = board_->traces[s2];
            // Join the far endpoints on A's layer.
            Point far_a = (A.a == v.pos) ? A.b : A.a;
            Point far_b = (B.a == v.pos) ? B.b : B.a;
            std::size_t lo = std::min(s1, s2), hi = std::max(s1, s2);
            int applied_before = rep.applied;
            Rect touched =
                rect_union(Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm),
                           rect_union(A.segment().bounds(), B.segment().bounds()));
            attempt(
                [&]() {
                    board_->traces[lo].a = far_a;
                    board_->traces[lo].b = far_b;
                    board_->traces[lo].layer = A.layer;
                    board_->traces.erase(board_->traces.begin() +
                                         static_cast<std::ptrdiff_t>(hi));
                    board_->vias.erase(board_->vias.begin() +
                                       static_cast<std::ptrdiff_t>(vi));
                },
                /*must_shorten=*/false, v.net, touched);
            if (rep.applied > applied_before) {
                vi = 0;  // accepted removal shifted indices: rescan
            } else {
                ++vi;
            }
        }
    }

    // Pass 4: preferred-layer — move a net whose copper sits on exactly one
    // non-preferred layer onto the preferred layer (layers[0]).
    if (opt_.preferred_layer && !board_->layers.empty()) {
        rep.passes++;
        LayerId pref = board_->layers.front().id;
        std::map<NetId, std::vector<std::size_t>> segs_by_net;
        for (std::size_t i = 0; i < board_->traces.size(); ++i)
            segs_by_net[board_->traces[i].net].push_back(i);
        // Deterministic net order.
        std::vector<NetId> nets;
        for (const auto& kv : segs_by_net) nets.push_back(kv.first);
        std::sort(nets.begin(), nets.end());
        for (NetId n : nets) {
            if (rep.candidates >= static_cast<int>(opt_.max_candidates)) break;
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
            attempt(
                [&]() {
                    for (std::size_t si : segs_by_net[n]) board_->traces[si].layer = pref;
                },
                /*must_shorten=*/false, n, board_->bounds());
        }
    }

    rep.bends_after = count_bends(*board_);
    rep.vias_after = static_cast<int>(board_via_count(*board_));
    rep.length_after_mm = board_total_length_mm(*board_);
    rep.peak_bytes = rep.candidates * 64;  // bounded scratch accounting
    return rep;
}

}  // namespace copperline
