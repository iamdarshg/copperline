// Copperline: post-route LengthTuner (issue #15). See tuning.h for staging
// and determinism contracts.
#include "router/tuning.h"

#include <algorithm>
#include <cmath>

#include "router/diffpair.h"
#include "router/simplify.h"
#include "router/verifier.h"

namespace copperline {
namespace {

Coord net_length(const Board& board, NetId net) {
    Coord total = 0;
    for (const auto& t : board.traces)
        if (t.net == net) total += euclid_len_nm(t.a, t.b);
    return total;
}

// Exact pair-gap floor for one candidate segment against partner copper:
// edge distance >= gap - tol everywhere (mirrors the verifier's too_close
// math: trace-trace folds both widths, rect cases fold the segment width).
// Floor-only by design: trombone teeth are specified (#15) skew jogs that
// legitimately leave the gap band, so the candidate pre-check enforces the
// floor here while the committed teeth carry TraceSeg::tuning_tooth (set by
// the caller below) for the verifier's ceiling exemption. The transactional
// full-verifier accept below still gates every commit.
bool gap_floor_ok(const Segment& s, Coord w, LayerId layer, NetId partner,
                  const Board& board, Coord gap_lo) {
    for (const auto& t : board.traces) {
        if (t.net != partner || t.layer != layer) continue;
        __int128 rhs = (__int128)2 * gap_lo + w + t.width_nm;
        if ((__int128)4 * seg_seg_dist2(s, t.segment()) < rhs * rhs) return false;
    }
    for (const auto& tm : board.terminals) {
        if (tm.net != partner || tm.layer != layer) continue;
        __int128 rhs = (__int128)2 * gap_lo + w;
        if ((__int128)4 * seg_rect_dist2(s, tm.pad_rect()) < rhs * rhs) return false;
    }
    for (const auto& v : board.vias) {
        if (v.net != partner) continue;
        if (layer < std::min(v.top_layer, v.bottom_layer) ||
            layer > std::max(v.top_layer, v.bottom_layer))
            continue;
        Rect vr = Rect::from_center_size(v.pos, v.outer_d_nm, v.outer_d_nm);
        __int128 rhs = (__int128)2 * gap_lo + w;
        if ((__int128)4 * seg_rect_dist2(s, vr) < rhs * rhs) return false;
    }
    return true;
}

struct BaseSeg {
    int trace_idx = -1;  // index into board.traces
    Coord len = 0;
};

bool base_less(const BaseSeg& a, const BaseSeg& b, const Board& board) {
    if (a.len != b.len) return a.len > b.len;  // longest first
    const TraceSeg& ta = board.traces[(std::size_t)a.trace_idx];
    const TraceSeg& tb = board.traces[(std::size_t)b.trace_idx];
    if (ta.a.x != tb.a.x) return ta.a.x < tb.a.x;
    if (ta.a.y != tb.a.y) return ta.a.y < tb.a.y;
    if (ta.b.x != tb.b.x) return ta.b.x < tb.b.x;
    if (ta.b.y != tb.b.y) return ta.b.y < tb.b.y;
    return ta.layer < tb.layer;
}

}  // namespace

Coord tuning_net_length(const Board& board, NetId net) { return net_length(board, net); }

bool tuning_config_from_json(const JsonValue& cfg, TuningConfig& out, std::string& err) {
    const JsonValue* t = cfg.find("tuning");
    if (!t) return true;  // absent = defaults
    if (!t->is_object()) {
        err = "tuning: expected an object";
        return false;
    }
    if (t->has("enabled")) out.enabled = t->get_bool("enabled", true);
    auto pos_mm = [&](const char* key, Coord& slot) -> bool {
        if (t->has(key)) {
            double v = t->get_number(key, -1);
            if (!(v > 0)) {
                err = std::string("tuning.") + key + ": expected a positive number (mm)";
                return false;
            }
            slot = mm_to_nm(v);
        }
        return true;
    };
    if (!pos_mm("amplitude_mm", out.amplitude_nm)) return false;
    if (!pos_mm("pitch_mm", out.pitch_nm)) return false;
    if (!pos_mm("max_added_mm", out.max_added_nm)) return false;
    if (t->has("max_candidates")) {
        long long v = static_cast<long long>(t->get_number("max_candidates", -1));
        if (v < 1 || v > 4096) {
            err = "tuning.max_candidates: expected a number in [1, 4096]";
            return false;
        }
        out.max_candidates = static_cast<int>(v);
    }
    if (t->has("max_regions")) {
        long long v = static_cast<long long>(t->get_number("max_regions", -1));
        if (v < 1 || v > 256) {
            err = "tuning.max_regions: expected a number in [1, 256]";
            return false;
        }
        out.max_regions = static_cast<int>(v);
    }
    if (t->has("max_teeth")) {
        long long v = static_cast<long long>(t->get_number("max_teeth", -1));
        if (v < 1 || v > 64) {
            err = "tuning.max_teeth: expected a number in [1, 64]";
            return false;
        }
        out.max_teeth = static_cast<int>(v);
    }
    if (t->has("style")) {
        out.style = t->get_string("style", "trombone");
        if (out.style != "trombone") {
            err = "tuning.style: only 'trombone' is supported";
            return false;
        }
    }
    if (t->has("symmetric_pairs")) out.symmetric_pairs = t->get_bool("symmetric_pairs", false);
    return true;
}

JsonValue TuningRecord::to_json() const {
    JsonValue o = JsonValue::object();
    o["kind"] = kind;
    o["net"] = static_cast<double>(net);
    o["net_name"] = net_name;
    if (pair_id >= 0) {
        o["pair_id"] = static_cast<double>(pair_id);
        o["pair_name"] = pair_name;
    }
    o["status"] = status;
    o["required_added_mm"] = nm_to_mm(required_added_nm);
    o["added_mm"] = nm_to_mm(added_nm);
    if (kind == "single") {
        o["target_length_mm"] = target_length_mm;
        o["length_tol_mm"] = length_tol_mm;
        o["final_length_mm"] = final_length_mm;
    } else {
        o["skew_before_mm"] = skew_before_mm;
        o["skew_after_mm"] = skew_after_mm;
        o["max_skew_mm"] = max_skew_mm;
        o["final_length_mm"] = final_length_mm;
    }
    o["regions_considered"] = static_cast<double>(regions_considered);
    o["candidates_evaluated"] = static_cast<double>(candidates_evaluated);
    o["teeth"] = static_cast<double>(teeth);
    o["layer"] = static_cast<double>(layer);
    o["tuning_exempt"] = tuning_exempt;
    return o;
}

JsonValue TuningSummary::to_json() const {
    JsonValue o = JsonValue::object();
    o["stage"] = stage;
    o["threads_used"] = static_cast<double>(threads_used);
    o["effective_threads"] = static_cast<double>(effective_threads);
    o["regions_considered"] = static_cast<double>(regions_considered);
    o["candidates_evaluated"] = static_cast<double>(candidates_evaluated);
    o["added_total_mm"] = nm_to_mm(added_total_nm);
    o["peak_scratch_bytes"] = static_cast<double>(peak_scratch_bytes);
    JsonValue recs = JsonValue::array();
    for (const auto& r : records) recs.as_array().push_back(r.to_json());
    o["records"] = recs;
    JsonValue tn = JsonValue::array();
    for (NetId n : tuned_nets) tn.as_array().push_back(JsonValue(static_cast<double>(n)));
    o["tuned_nets"] = tn;
    return o;
}

LengthTuner::LengthTuner(Board* board, const RuleResolver* resolver,
                         const ElectricalContext* ctx, TuningConfig cfg)
    : board_(board), resolver_(resolver), ctx_(ctx), cfg_(cfg) {}

// Build one trombone candidate replacing base trace `base` with a square-wave
// accordion of `teeth` teeth on `side` (+1/-1 normal). Appends the new chain
// (lead + accordion + lead) to `out`. Returns false when the base is too
// short or degenerate. Outside the accordion the chain reuses the original
// arbitrary-angle direction (collinear leads).
bool build_trombone(const TraceSeg& base, int teeth, int side, Coord amplitude_nm,
                    Coord pitch_nm, std::vector<TraceSeg>& out) {
    if (teeth < 1 || amplitude_nm <= 0 || pitch_nm < base.width_nm) return false;
    double ax = (double)base.a.x, ay = (double)base.a.y;
    double bx = (double)base.b.x, by = (double)base.b.y;
    double dx = bx - ax, dy = by - ay;
    double len = std::sqrt(dx * dx + dy * dy);
    if (!(len > 0)) return false;
    double period = 2.0 * (double)pitch_nm;
    double total = (double)teeth * period;
    if (len < total) return false;  // never stretch topology: must fit inside
    double ux = dx / len, uy = dy / len;
    double nx = -uy * (double)side, ny = ux * (double)side;
    double s0 = len / 2.0 - total / 2.0;
    auto at = [&](double s, double off) -> Point {
        double px = ax + ux * s + nx * off;
        double py = ay + uy * s + ny * off;
        return {static_cast<Coord>(std::llround(px)), static_cast<Coord>(std::llround(py))};
    };
    std::vector<Point> pts;
    pts.reserve((std::size_t)teeth * 4 + 2);
    pts.push_back(at(s0, 0.0));
    for (int i = 0; i < teeth; ++i) {
        double sb = s0 + (double)i * period;
        if (i > 0) pts.push_back(at(sb, 0.0));  // flat between teeth
        pts.push_back(at(sb, (double)amplitude_nm));
        pts.push_back(at(sb + (double)pitch_nm, (double)amplitude_nm));
        pts.push_back(at(sb + (double)pitch_nm, 0.0));
    }
    pts.push_back(base.b);
    // Prepend the original start when the accordion does not begin there.
    if (!(pts.front() == base.a)) pts.insert(pts.begin(), base.a);
    // Drop consecutive duplicates (zero-length legs).
    std::vector<Point> clean;
    clean.reserve(pts.size());
    for (auto p : pts) {
        if (clean.empty() || !(clean.back() == p)) clean.push_back(p);
    }
    if (clean.size() < 2) return false;
    out.clear();
    for (std::size_t i = 0; i + 1 < clean.size(); ++i)
        out.push_back({base.net, base.layer, clean[i], clean[i + 1], base.width_nm});
    return !out.empty();
}

TuningSummary LengthTuner::run(bool topology_closed) {
    TuningSummary sum;
    sum.threads_used = 1;
    sum.effective_threads = 1;
    if (!cfg_.enabled) {
        sum.stage = "SKIPPED_DISABLED";
        return sum;
    }
    if (!topology_closed) {
        sum.stage = "SKIPPED_NOT_CLOSED";
        return sum;
    }
    Board& board = *board_;

    // ---- Collect tuning targets in deterministic order ----
    struct SingleNeed {
        NetId net;
        Coord lo = 0;  // required added window [lo, hi]
        Coord hi = 0;
    };
    struct PairNeed {
        int pair_idx = -1;  // index into board.diffpairs
        bool symmetric = false;
    };
    std::vector<PairNeed> pair_needs;
    std::vector<SingleNeed> single_needs;
    {
        // Pair members are handled by the pair phase only.
        std::vector<char> is_pair_member;
        // Map net id -> presence via linear scan (net ids are dense ints).
        auto net_is_member = [&](NetId n) {
            for (const auto& pr : board.diffpairs) {
                if (pr.net_p == n || pr.net_n == n) return true;
            }
            return false;
        };
        for (std::size_t i = 0; i < board.diffpairs.size(); ++i) {
            const DiffPair& pr = board.diffpairs[i];
            // Only routed pairs (both members own copper) are tunable.
            bool has_p = false, has_n = false;
            for (const auto& t : board.traces) {
                if (t.net == pr.net_p) has_p = true;
                if (t.net == pr.net_n) has_n = true;
            }
            if (!has_p || !has_n) continue;
            const NetInfo* np = board.find_net(pr.net_p);
            const NetInfo* nn = board.find_net(pr.net_n);
            bool want = pr.has_max_skew ||
                        (np && np->has_target_length) || (nn && nn->has_target_length);
            if (!want) continue;
            PairNeed pn;
            pn.pair_idx = (int)i;
            pn.symmetric = pr.symmetric_tuning || cfg_.symmetric_pairs;
            pair_needs.push_back(pn);
        }
        std::sort(pair_needs.begin(), pair_needs.end(), [&](const PairNeed& a, const PairNeed& b) {
            return board.diffpairs[(std::size_t)a.pair_idx].id <
                   board.diffpairs[(std::size_t)b.pair_idx].id;
        });
        for (const auto& n : board.nets) {
            if (!n.has_target_length) continue;
            if (net_is_member(n.id)) continue;  // covered by the pair phase
            bool routed = false;
            for (const auto& t : board.traces) {
                if (t.net == n.id) {
                    routed = true;
                    break;
                }
            }
            if (!routed) {
                TuningRecord r;
                r.kind = "single";
                r.net = n.id;
                r.net_name = n.name;
                r.status = "INFEASIBLE:not_routed";
                r.target_length_mm = nm_to_mm(n.target_length_nm);
                r.length_tol_mm = nm_to_mm(n.length_tol_nm);
                r.final_length_mm = 0.0;
                sum.records.push_back(r);
                continue;
            }
            Coord cur = net_length(board, n.id);
            Coord lo_w = n.target_length_nm - n.length_tol_nm;
            Coord hi_w = n.target_length_nm + n.length_tol_nm;
            if (cur >= lo_w && cur <= hi_w) {
                TuningRecord r;
                r.kind = "single";
                r.net = n.id;
                r.net_name = n.name;
                r.status = "ALREADY_WITHIN_WINDOW";
                r.target_length_mm = nm_to_mm(n.target_length_nm);
                r.length_tol_mm = nm_to_mm(n.length_tol_nm);
                r.final_length_mm = nm_to_mm(cur);
                sum.records.push_back(r);
                continue;
            }
            if (cur > hi_w) {
                TuningRecord r;
                r.kind = "single";
                r.net = n.id;
                r.net_name = n.name;
                r.status = "INFEASIBLE:overshoot_cannot_shorten";
                r.target_length_mm = nm_to_mm(n.target_length_nm);
                r.length_tol_mm = nm_to_mm(n.length_tol_nm);
                r.final_length_mm = nm_to_mm(cur);
                sum.records.push_back(r);
                continue;
            }
            SingleNeed sn;
            sn.net = n.id;
            sn.lo = lo_w - cur;
            sn.hi = hi_w - cur;
            single_needs.push_back(sn);
        }
        std::sort(single_needs.begin(), single_needs.end(),
                  [](const SingleNeed& a, const SingleNeed& b) { return a.net < b.net; });
        (void)is_pair_member;
    }

    if (pair_needs.empty() && single_needs.empty() &&
        std::all_of(sum.records.begin(), sum.records.end(), [](const TuningRecord& r) {
            return r.status == "ALREADY_WITHIN_WINDOW";
        })) {
        sum.stage = sum.records.empty() ? "SKIPPED_NO_TARGETS" : "TUNING_COMPLETE";
        return sum;
    }

    BoardVerifier verifier;
    bool any_tuned = false, any_infeasible = false;
    scratch_new_.reserve(256);
    scratch_gap_probe_.reserve(4);

    // ---- Tune one member net by an added-length window ----
    // Returns added length on success, <0 with record status on failure.
    auto tune_member = [&](NetId net, NetId partner, Coord need_lo, Coord need_hi,
                           TuningRecord& rec) -> Coord {
        rec.regions_considered = 0;
        rec.candidates_evaluated = 0;
        if (need_lo > cfg_.max_added_nm) {
            rec.status = "INFEASIBLE:exceeds_max_added";
            return -1;
        }
        if (need_lo <= 0) return 0;
        // Candidate tooth counts: minimum added first, bounded window.
        Coord cap = std::min(need_hi, cfg_.max_added_nm);
        int kmin = (int)((need_lo + 2 * cfg_.amplitude_nm - 1) / (2 * cfg_.amplitude_nm));
        if (kmin < 1) kmin = 1;
        std::vector<int> ks;
        for (int k = kmin; k <= cfg_.max_teeth; ++k) {
            __int128 added = (__int128)2 * cfg_.amplitude_nm * k;
            if (added > cap) break;
            ks.push_back(k);
        }
        if (ks.empty()) {
            rec.status =
                (need_lo > cfg_.max_added_nm)
                    ? "INFEASIBLE:exceeds_max_added"
                    : "INFEASIBLE:no_tooth_count_in_window";
            return -1;
        }
        // Base regions: longest segments first, coordinate tie-break.
        std::vector<BaseSeg> bases;
        for (std::size_t i = 0; i < board.traces.size(); ++i) {
            const TraceSeg& t = board.traces[i];
            if (t.net != net) continue;
            BaseSeg b;
            b.trace_idx = (int)i;
            b.len = euclid_len_nm(t.a, t.b);
            if (b.len > 0) bases.push_back(b);
        }
        std::sort(bases.begin(), bases.end(),
                  [&](const BaseSeg& a, const BaseSeg& b) { return base_less(a, b, board); });
        if ((int)bases.size() > cfg_.max_regions) bases.resize((std::size_t)cfg_.max_regions);
        const NetInfo* np = board.find_net(net);
        Coord gap_lo = 0;
        bool pair_tune = partner >= 0;
        if (pair_tune) {
            for (const auto& pr : board.diffpairs) {
                if (pr.net_p == net || pr.net_n == net) {
                    gap_lo = pr.gap_nm - pr.gap_tol_nm;
                    if (gap_lo < 0) gap_lo = 0;
                    break;
                }
            }
        }
        (void)np;
        int regions = 0;
        for (const auto& b : bases) {
            if (candidates_evaluated_ >= cfg_.max_candidates) break;
            // Re-resolve the base trace (indices shift only on accept, and
            // we return immediately after an accept).
            const TraceSeg base = board.traces[(std::size_t)b.trace_idx];
            if (base.net != net) continue;  // stale after a prior accept
            ++regions;
            ++rec.regions_considered;
            for (int k : ks) {
                for (int side : {+1, -1}) {
                    if (candidates_evaluated_ >= cfg_.max_candidates) break;
                    scratch_new_.clear();
                    if (!build_trombone(base, k, side, cfg_.amplitude_nm,
                                        cfg_.pitch_nm, scratch_new_))
                        continue;
                    // Issue #15/#26: pair-tuning teeth are specified skew
                    // jogs; mark them so the verifier floor-checks (never
                    // ceiling-checks) the committed accordion. Single-ended
                    // teeth need no pair-gap treatment and stay unmarked.
                    if (pair_tune) {
                        for (auto& s : scratch_new_) s.tuning_tooth = true;
                    }
                    ++candidates_evaluated_;
                    ++rec.candidates_evaluated;
                    ++sum.candidates_evaluated;
                    peak_scratch_bytes_ = std::max(
                        peak_scratch_bytes_,
                        scratch_new_.size() * sizeof(TraceSeg) + sizeof(Board));
                    bool legal = true;
                    for (const auto& s : scratch_new_) {
                        if (!simplify_segment_legal_except(board, *resolver_, net,
                                                           s.layer, s.width_nm,
                                                           s.segment(), *ctx_, partner)) {
                            legal = false;
                            break;
                        }
                        if (pair_tune &&
                            !gap_floor_ok(s.segment(), s.width_nm, s.layer,
                                          partner, board, gap_lo)) {
                            legal = false;
                            break;
                        }
                    }
                    if (!legal) continue;
                    // Transactional commit: swap base for the accordion,
                    // measure, verify, revert on any failure.
                    std::size_t erase_at = (std::size_t)b.trace_idx;
                    TraceSeg saved = board.traces[erase_at];
                    board.traces.erase(board.traces.begin() + (long long)erase_at);
                    for (const auto& s : scratch_new_) board.traces.push_back(s);
                    Coord new_total = net_length(board, net);
                    Coord old_total = new_total;
                    // old_total recompute: subtract candidate, add base back
                    Coord cand_len = 0;
                    for (const auto& s : scratch_new_) cand_len += euclid_len_nm(s.a, s.b);
                    Coord base_len = euclid_len_nm(saved.a, saved.b);
                    old_total = new_total - cand_len + base_len;
                    Coord added = new_total - old_total;
                    bool in_window = added >= need_lo && added <= need_hi;
                    VerifyResult vr = verifier.verify(board, *resolver_, *ctx_);
                    if (in_window && vr.ok) {
                        rec.added_nm = added;
                        rec.teeth = k;
                        rec.layer = base.layer;
                        rec.tuning_exempt = true;
                        sum.added_total_nm += added;
                        sum.regions_considered += regions;
                        return added;
                    }
                    // Revert: remove candidate segs, restore base verbatim.
                    for (std::size_t i = 0; i < scratch_new_.size(); ++i)
                        board.traces.pop_back();
                    board.traces.insert(board.traces.begin() + (long long)erase_at,
                                        saved);
                }
            }
        }
        sum.regions_considered += regions;
        if (candidates_evaluated_ >= cfg_.max_candidates)
            rec.status = "INFEASIBLE:resource_capped";
        else
            rec.status = "INFEASIBLE:no_legal_region";
        return -1;
    };

    // ---- Pair phase (shorter-only or symmetric) ----
    for (const auto& pn : pair_needs) {
        const DiffPair& pr = board.diffpairs[(std::size_t)pn.pair_idx];
        TuningRecord rec;
        rec.kind = "pair";
        rec.pair_id = pr.id;
        rec.pair_name = pr.name;
        rec.max_skew_mm = pr.has_max_skew ? nm_to_mm(pr.max_skew_nm) : 0.0;
        std::vector<TraceSeg> tp, tn;
        for (const auto& t : board.traces) {
            if (t.net == pr.net_p) tp.push_back(t);
            if (t.net == pr.net_n) tn.push_back(t);
        }
        Coord lp = pair_total_length(tp), ln = pair_total_length(tn);
        Coord skew = lp >= ln ? lp - ln : ln - lp;
        rec.skew_before_mm = nm_to_mm(skew);
        rec.required_added_nm = 0;
        const NetInfo* np = board.find_net(pr.net_p);
        const NetInfo* nn = board.find_net(pr.net_n);
        bool ok = true;
        std::string note;
        if (pn.symmetric) {
            // Common target: longest member and both single-ended floors.
            Coord common = std::max(lp, ln);
            if (np && np->has_target_length) common = std::max(common, np->target_length_nm);
            if (nn && nn->has_target_length) common = std::max(common, nn->target_length_nm);
            // The common length must not overshoot either member ceiling;
            // copper can only be added, never shortened.
            bool ceil_ok = true;
            if (np && np->has_target_length && common > np->target_length_nm + np->length_tol_nm)
                ceil_ok = false;
            if (nn && nn->has_target_length && common > nn->target_length_nm + nn->length_tol_nm)
                ceil_ok = false;
            Coord need_p = common - lp, need_n = common - ln;
            // Window: symmetric tuning aims exactly at common; accept the
            // member tolerance when declared, else exact (+1 nm rounding).
            Coord hi_p = need_p, hi_n = need_n;
            if (np && np->has_target_length) hi_p = need_p + 2 * np->length_tol_nm;
            if (nn && nn->has_target_length) hi_n = need_n + 2 * nn->length_tol_nm;
            hi_p += 1;
            hi_n += 1;
            rec.required_added_nm = need_p + need_n;
            rec.net = -1;
            rec.net_name = pr.name;
            TuningRecord rp = rec, rn = rec;
            Coord got_p = 0, got_n = 0;
            if (!ceil_ok) {
                ok = false;
                note = "PARTIAL:symmetric_target_conflict";
                rec.status = note;
            }
            if (ok && need_p > 0) {
                got_p = tune_member(pr.net_p, pr.net_n, need_p, hi_p, rp);
                if (got_p < 0) {
                    ok = false;
                    note = "p:" + rp.status;
                }
            }
            if (ok && need_n > 0) {
                // Re-measure P after its transaction (indices/lengths moved).
                got_n = tune_member(pr.net_n, pr.net_p, need_n, hi_n, rn);
                if (got_n < 0) {
                    ok = false;
                    note = "n:" + rn.status;
                }
            }
            rec.added_nm = (got_p > 0 ? got_p : 0) + (got_n > 0 ? got_n : 0);
            rec.candidates_evaluated = rp.candidates_evaluated + rn.candidates_evaluated;
            rec.regions_considered = rp.regions_considered + rn.regions_considered;
            rec.teeth = std::max(rp.teeth, rn.teeth);
            rec.tuning_exempt = ok && (need_p > 0 || need_n > 0);
            (void)got_p;
            (void)got_n;
        } else {
            // Shorter-only: skew first, then member targets that do not
            // re-break skew.
            bool p_shorter = lp <= ln;
            NetId s_net = p_shorter ? pr.net_p : pr.net_n;
            NetId l_net = p_shorter ? pr.net_n : pr.net_p;
            NetId s_partner = l_net;
            Coord s_len = std::min(lp, ln), l_len = std::max(lp, ln);
            Coord skew_need =
                (pr.has_max_skew && skew > pr.max_skew_nm) ? (skew - pr.max_skew_nm) : 0;
            rec.required_added_nm = skew_need;
            rec.net = s_net;
            const NetInfo* s_ni = board.find_net(s_net);
            const NetInfo* l_ni = board.find_net(l_net);
            rec.net_name = s_ni ? s_ni->name : "?";
            // Fold the shorter member's single-ended floor into its need:
            // after the skew fix it must still reach target-tol.
            Coord s_target_need = 0, s_hi_extra = 1;
            if (s_ni && s_ni->has_target_length) {
                Coord floor = s_ni->target_length_nm - s_ni->length_tol_nm;
                Coord want = floor - (s_len + skew_need);
                if (want < 0) want = 0;
                // Overshoot beyond target+tol can never be shortened.
                if (s_len + skew_need > s_ni->target_length_nm + s_ni->length_tol_nm &&
                    skew_need == 0 && want == 0) {
                    // Already above window without any skew need: leave
                    // copper alone, report overshoot below.
                }
                s_target_need = want;
                s_hi_extra = 2 * s_ni->length_tol_nm + 1;
            }
            // Skew headroom: adding up to 2*max_skew beyond the minimum
            // keeps |skew| within max (the shorter member may overshoot
            // the longer one by at most max_skew).
            if (skew_need > 0 && pr.has_max_skew) s_hi_extra += 2 * pr.max_skew_nm;
            Coord need_s = skew_need + s_target_need;
            Coord hi_s = need_s + s_hi_extra;
            // Member target ceiling: never overshoot target+tol even when
            // skew headroom would allow more copper.
            if (s_ni && s_ni->has_target_length)
                hi_s = std::min(hi_s, s_ni->target_length_nm + s_ni->length_tol_nm - s_len);
            rec.required_added_nm = need_s;
            Coord got_s = 0;
            if (need_s > 0) {
                got_s = tune_member(s_net, s_partner, need_s, hi_s, rec);
                if (got_s < 0) {
                    ok = false;
                    note = rec.status;
                } else {
                    s_len += got_s;
                }
            }
            // Longer member target: only when it cannot re-break skew.
            if (ok && l_ni && l_ni->has_target_length) {
                Coord l_floor = l_ni->target_length_nm - l_ni->length_tol_nm;
                Coord l_ceil = l_ni->target_length_nm + l_ni->length_tol_nm;
                if (l_len < l_floor) {
                    Coord want = l_floor - l_len;
                    Coord skew_after = (l_len + want) >= s_len ? (l_len + want - s_len) : 0;
                    bool skew_ok =
                        !pr.has_max_skew || skew_after <= pr.max_skew_nm;
                    if (!skew_ok) {
                        ok = false;
                        note = "PARTIAL:longer_target_skew_conflict";
                        rec.status = note;
                    } else {
                        TuningRecord rl;
                        Coord got_l =
                            tune_member(l_net, s_net, want, want + 2 * l_ni->length_tol_nm + 1, rl);
                        rec.candidates_evaluated += rl.candidates_evaluated;
                        rec.regions_considered += rl.regions_considered;
                        if (got_l < 0) {
                            ok = false;
                            note = "PARTIAL:longer_" + rl.status;
                            rec.status = note;
                        } else {
                            rec.added_nm += got_l;
                            if (rl.teeth > rec.teeth) {
                                rec.teeth = rl.teeth;
                                rec.layer = rl.layer;
                            }
                        }
                    }
                    (void)l_ceil;
                } else if (l_len > l_ceil) {
                    ok = false;
                    note = "PARTIAL:longer_overshoot_cannot_shorten";
                    rec.status = note;
                }
            }
            if (ok && s_ni && s_ni->has_target_length) {
                if (s_len > s_ni->target_length_nm + s_ni->length_tol_nm) {
                    ok = false;
                    note = "PARTIAL:shorter_overshoot_cannot_shorten";
                    rec.status = note;
                }
            }
            if (got_s > 0) rec.added_nm += 0;  // already in rec.added_nm via tune_member
        }
        // Re-measure skew from committed copper (never router metadata).
        {
            std::vector<TraceSeg> qp, qn;
            for (const auto& t : board.traces) {
                if (t.net == pr.net_p) qp.push_back(t);
                if (t.net == pr.net_n) qn.push_back(t);
            }
            Coord nlp = pair_total_length(qp), nln = pair_total_length(qn);
            Coord nskew = nlp >= nln ? nlp - nln : nln - nlp;
            rec.skew_after_mm = nm_to_mm(nskew);
            rec.final_length_mm = nm_to_mm(std::max(nlp, nln));
        }
        if (ok) {
            bool changed = rec.added_nm > 0;
            rec.status = changed ? "TUNED" : "ALREADY_WITHIN_WINDOW";
            // ALREADY_WITHIN_WINDOW also covers "no skew/target need".
            if (changed) {
                any_tuned = true;
                rec.tuning_exempt = true;
                if (std::find(sum.tuned_nets.begin(), sum.tuned_nets.end(), pr.net_p) ==
                    sum.tuned_nets.end()) {
                    // Record both members exempt: the pair is tuned as one.
                    sum.tuned_nets.push_back(pr.net_p);
                    sum.tuned_nets.push_back(pr.net_n);
                }
            }
        } else {
            if (rec.status.rfind("PARTIAL", 0) != 0 && rec.status.rfind("INFEASIBLE", 0) != 0)
                rec.status = "INFEASIBLE:" + note;
            any_infeasible = true;
        }
        sum.records.push_back(rec);
    }

    // ---- Single-ended phase ----
    for (const auto& sn : single_needs) {
        const NetInfo* ni = board.find_net(sn.net);
        TuningRecord rec;
        rec.kind = "single";
        rec.net = sn.net;
        rec.net_name = ni ? ni->name : "?";
        rec.required_added_nm = sn.lo;
        rec.target_length_mm = ni ? nm_to_mm(ni->target_length_nm) : 0.0;
        rec.length_tol_mm = ni ? nm_to_mm(ni->length_tol_nm) : 0.0;
        Coord got = tune_member(sn.net, -1, sn.lo, sn.hi, rec);
        rec.final_length_mm = nm_to_mm(net_length(board, sn.net));
        if (got >= 0) {
            if (got == 0) {
                rec.status = "ALREADY_WITHIN_WINDOW";
            } else {
                rec.status = "TUNED";
                any_tuned = true;
                rec.tuning_exempt = true;
                sum.tuned_nets.push_back(sn.net);
            }
        } else {
            any_infeasible = true;
        }
        sum.records.push_back(rec);
    }

    std::sort(sum.tuned_nets.begin(), sum.tuned_nets.end());
    sum.peak_scratch_bytes = peak_scratch_bytes_;
    if (any_infeasible && any_tuned)
        sum.stage = "TUNING_PARTIAL";
    else if (any_infeasible)
        sum.stage = "TUNING_PARTIAL";
    else if (any_tuned)
        sum.stage = "TUNING_COMPLETE";
    else
        sum.stage = "TUNING_COMPLETE";
    return sum;
}

}  // namespace copperline
