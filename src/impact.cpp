#include "router/impact.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "router/density.h"
#include "router/route_tree.h"

namespace copperline {

namespace {

double clamp01(double v) {
    if (!(v >= 0.0)) return 0.0;  // NaN -> 0
    if (v > 1.0) return 1.0;
    return v;
}

double rect_area_mm2(const Rect& r) {
    double w = nm_to_mm(r.x2 - r.x1);
    double h = nm_to_mm(r.y2 - r.y1);
    if (w <= 0 || h <= 0) return 0.0;
    return w * h;
}

double overlap_area_mm2(const Rect& a, const Rect& b) {
    Coord x1 = std::max(a.x1, b.x1);
    Coord y1 = std::max(a.y1, b.y1);
    Coord x2 = std::min(a.x2, b.x2);
    Coord y2 = std::min(a.y2, b.y2);
    if (x2 <= x1 || y2 <= y1) return 0.0;
    return nm_to_mm(x2 - x1) * nm_to_mm(y2 - y1);
}

// Candidate expanded bbox: traces expanded by half-width + max clearance,
// vias as outer-diameter squares expanded by clearance. Single rect keeps
// scoring O(segments + remaining) with O(1) scratch (streamed).
Rect candidate_footprint(const CandidateRoute& cand, const ImpactContext& ictx,
                         Coord& width_out, Coord& clear_out) {
    width_out = 0;
    clear_out = 0;
    const Board* b = ictx.board;
    const RuleResolver* r = ictx.resolver;
    const ElectricalContext* ctx = ictx.ctx;
    Coord wmax = 0;
    for (const auto& s : cand.traces) wmax = std::max(wmax, s.width_nm);
    if (wmax <= 0 && b && r && ctx) {
        std::string src;
        wmax = r->requiredTraceWidth(cand.task.net, cand.traces.empty() ? 0
                                                                        : cand.traces.front().layer,
                                     *ctx, &src);
    }
    width_out = wmax;
    Coord cmax = 0;
    if (b && r && ctx) {
        for (const auto& o : b->nets) {
            if (o.id == cand.task.net) continue;
            std::string cs;
            cmax = std::max(cmax, r->requiredClearance(cand.task.net, o.id, 0, *ctx, &cs));
        }
    }
    clear_out = cmax;
    Coord expand = wmax / 2 + cmax;
    bool first = true;
    Rect box{0, 0, 0, 0};
    auto grow = [&](Point p, Coord half) {
        Rect q{p.x - half, p.y - half, p.x + half, p.y + half};
        if (first) {
            box = q;
            first = false;
        } else {
            box.x1 = std::min(box.x1, q.x1);
            box.y1 = std::min(box.y1, q.y1);
            box.x2 = std::max(box.x2, q.x2);
            box.y2 = std::max(box.y2, q.y2);
        }
    };
    for (const auto& s : cand.traces) {
        grow(s.a, expand);
        grow(s.b, expand);
    }
    for (const auto& v : cand.vias) {
        grow(v.pos, v.outer_d_nm / 2 + cmax);
    }
    if (first) {
        // Empty geometry: fall back to the task endpoints so an empty
        // candidate still localizes (zero-area box at src).
        Point p = cand.gate_a;
        box = {p.x, p.y, p.x, p.y};
    }
    return box;
}

Point candidate_mid(const CandidateRoute& cand, const ImpactContext& ictx) {
    if (!cand.traces.empty()) {
        const auto& s = cand.traces[cand.traces.size() / 2];
        return {(s.a.x + s.b.x) / 2, (s.a.y + s.b.y) / 2};
    }
    if (!cand.vias.empty()) return cand.vias[cand.vias.size() / 2].pos;
    if (ictx.board) {
        Point a = task_src_point(*ictx.board, cand.task);
        Point b = task_dst_point(*ictx.board, cand.task);
        return {(a.x + b.x) / 2, (a.y + b.y) / 2};
    }
    return cand.gate_a;
}

}  // namespace

double ImpactFeatures::at(int i) const {
    switch (i) {
        case 0: return future_corridor_overlap;
        case 1: return pin_density;
        case 2: return bottleneck_scarcity;
        case 3: return consumed_footprint;
        case 4: return layer_scarcity;
        case 5: return via_site_consumption;
        case 6: return fine_pitch_proximity;
        case 7: return current_width;
        case 8: return voltage_clearance;
        default: return 0.0;
    }
}

void ImpactFeatures::set(int i, double v) {
    v = clamp01(v);
    switch (i) {
        case 0: future_corridor_overlap = v; break;
        case 1: pin_density = v; break;
        case 2: bottleneck_scarcity = v; break;
        case 3: consumed_footprint = v; break;
        case 4: layer_scarcity = v; break;
        case 5: via_site_consumption = v; break;
        case 6: fine_pitch_proximity = v; break;
        case 7: current_width = v; break;
        case 8: voltage_clearance = v; break;
        default: break;
    }
}

JsonValue ImpactFeatures::to_json() const {
    JsonValue o = JsonValue::object();
    for (int i = 0; i < kImpactFeatureCount; ++i) o[kImpactFeatureNames[i]] = at(i);
    return o;
}

double ImpactWeights::sum() const {
    double s = 0;
    for (double v : w) s += v;
    return s;
}

JsonValue ImpactWeights::to_json() const {
    JsonValue o = JsonValue::object();
    for (int i = 0; i < kImpactFeatureCount; ++i) o[kImpactFeatureNames[i]] = w[i];
    return o;
}

MlpImpactConfig MlpImpactConfig::fixed_default() {
    MlpImpactConfig c;
    // Explicit fixed weights: small varied pattern so each feature matters,
    // hidden units mix overlapping subsets deterministically.
    for (int h = 0; h < 4; ++h) {
        for (int i = 0; i < 9; ++i) {
            // pattern in [-0.3, +0.3], deterministic, no training involved.
            double v = 0.05 * ((h * 9 + i) % 7) - 0.15;
            if (((h + i) % 3) == 0) v += 0.10;
            c.w1[h * 9 + i] = v;
        }
        c.b1[h] = -0.1 * h;
        c.w2[h] = 0.25;
    }
    c.b2 = -0.5;
    return c;
}

bool MlpImpactConfig::valid() const {
    for (double v : w1)
        if (!std::isfinite(v)) return false;
    for (double v : b1)
        if (!std::isfinite(v)) return false;
    for (double v : w2)
        if (!std::isfinite(v)) return false;
    return std::isfinite(b2);
}

JsonValue ImpactScore::to_json() const {
    JsonValue o = JsonValue::object();
    o["scorer"] = scorer;
    o["fallback"] = fallback;
    o["legal"] = legal;
    if (!legal_reason.empty()) o["legal_reason"] = legal_reason;
    o["base_cost_nm"] = static_cast<double>(base_cost_nm);
    o["base_cost_mm"] = nm_to_mm(base_cost_nm);
    o["signature"] = signature;
    o["total"] = total;
    o["features"] = features.to_json();
    JsonValue c = JsonValue::object();
    for (int i = 0; i < kImpactFeatureCount; ++i) c[kImpactFeatureNames[i]] = contributions[i];
    o["contributions"] = c;
    o["weights_sum_check"] = total;  // total itself; weights live with options
    return o;
}

ImpactFeatures compute_impact_features(const CandidateRoute& cand,
                                       const ImpactContext& ictx) {
    ImpactFeatures f;
    if (!ictx.board || !ictx.resolver || !ictx.ctx) return f;
    const Board& board = *ictx.board;
    const RuleResolver& resolver = *ictx.resolver;
    const ElectricalContext& ctx = *ictx.ctx;

    Coord wreq = 0, cmax = 0;
    Rect foot = candidate_footprint(cand, ictx, wreq, cmax);
    double foot_area = rect_area_mm2(foot);

    // Per-segment expanded rects (streamed, O(1) scratch): each trace
    // widens by its own half-width + max clearance, each via by half
    // outer-diameter + clearance. Overlap is measured against actual
    // copper, not the candidate bbox (a far detour's bbox can still cover
    // the corridor it avoids).
    std::vector<Rect> segs;
    segs.reserve(cand.traces.size() + cand.vias.size());
    for (const auto& s : cand.traces) {
        Coord half = s.width_nm / 2 + cmax;
        Rect q{std::min(s.a.x, s.b.x) - half, std::min(s.a.y, s.b.y) - half,
               std::max(s.a.x, s.b.x) + half, std::max(s.a.y, s.b.y) + half};
        segs.push_back(q);
    }
    for (const auto& v : cand.vias) {
        Coord half = v.outer_d_nm / 2 + cmax;
        segs.push_back({v.pos.x - half, v.pos.y - half, v.pos.x + half, v.pos.y + half});
    }
    if (segs.empty()) {
        // Empty geometry: score the task endpoints as a point.
        Point p = candidate_mid(cand, ictx);
        segs.push_back({p.x, p.y, p.x, p.y});
        foot_area = 0.0;
    } else {
        double sum = 0.0;
        for (const auto& q : segs) sum += rect_area_mm2(q);
        foot_area = sum;
    }

    // 0. future-corridor overlap: max over (segment, remaining) of
    // overlap/corridorArea. 2. bottleneck scarcity: fraction of remaining
    // corridors touched by any segment.
    double best_frac = 0.0;
    int hit = 0;
    int nrem = 0;
    for (std::size_t i = 0; i < ictx.remaining_corridors.size(); ++i) {
        const Corridor& c = ictx.remaining_corridors[i];
        double ca = rect_area_mm2(c.rect);
        if (ca <= 0) continue;
        ++nrem;
        bool touched = false;
        for (const auto& q : segs) {
            double ov = overlap_area_mm2(q, c.rect);
            if (ov > 0) touched = true;
            best_frac = std::max(best_frac, ov / ca);
        }
        if (touched) ++hit;
    }
    f.future_corridor_overlap = clamp01(best_frac);
    f.bottleneck_scarcity = nrem > 0 ? clamp01(static_cast<double>(hit) / nrem) : 0.0;

    // 1. pin density along the candidate midpoint (pads/mm^2, /12 bound).
    {
        DensityEstimator est;
        double d = est.local_density(board, candidate_mid(cand, ictx));
        f.pin_density = clamp01(d / 12.0);
    }

    // 3. consumed footprint: expanded copper area / 50mm^2.
    f.consumed_footprint = clamp01(foot_area / 50.0);

    // 4. layer scarcity: used-layer share scaled by cost multiplier.
    {
        std::vector<char> used;
        int max_id = 0;
        for (const auto& l : board.layers) max_id = std::max(max_id, l.id);
        used.assign(max_id + 1, 0);
        for (const auto& s : cand.traces) {
            if (s.layer >= 0 && s.layer < (int)used.size()) used[s.layer] = 1;
        }
        for (const auto& v : cand.vias) {
            for (int l = v.top_layer; l <= v.bottom_layer; ++l) {
                if (l >= 0 && l < (int)used.size()) used[l] = 1;
            }
        }
        int nu = 0;
        double mult_max = 1.0;
        for (std::size_t i = 0; i < used.size(); ++i) {
            if (!used[i]) continue;
            ++nu;
            for (const auto& l : board.layers) {
                if (l.id == (int)i) mult_max = std::max(mult_max, l.cost_multiplier);
            }
        }
        double share = board.layers.empty()
                           ? 0.0
                           : static_cast<double>(nu) / board.layers.size();
        double mult_term = clamp01((mult_max - 1.0) / 4.0);
        f.layer_scarcity = clamp01(0.6 * share + 0.4 * mult_term);
    }

    // 5. via-site consumption: via count / 6 (bounded).
    f.via_site_consumption = clamp01(static_cast<double>(cand.vias.size()) / 6.0);

    // 6. fine-pitch proximity: 1 - minDist/4mm over deep pads (depth>0).
    {
        bool any_deep = false;
        Coord best = std::numeric_limits<Coord>::max();
        Point mid = candidate_mid(cand, ictx);
        for (const auto& t : board.terminals) {
            auto it = ictx.centre_depth.find(t.id);
            if (it == ictx.centre_depth.end() || it->second <= 0) continue;
            any_deep = true;
            Coord d = manhattan(mid, t.pos);
            if (d < best) best = d;
        }
        if (!any_deep) {
            f.fine_pitch_proximity = 0.0;
        } else {
            f.fine_pitch_proximity = clamp01(1.0 - nm_to_mm(best) / 4.0);
        }
    }

    // 7. current width burden: required width / 2mm (high-current reflects).
    {
        LayerId lay = cand.traces.empty() ? 0 : cand.traces.front().layer;
        std::string src;
        Coord w = resolver.requiredTraceWidth(cand.task.net, lay, ctx, &src);
        f.current_width = clamp01(nm_to_mm(w) / 2.0);
    }

    // 8. voltage-clearance burden: max pair clearance / 1mm.
    {
        Coord m = 0;
        for (const auto& o : board.nets) {
            if (o.id == cand.task.net) continue;
            std::string cs;
            m = std::max(m, resolver.requiredClearance(cand.task.net, o.id, 0, ctx, &cs));
        }
        f.voltage_clearance = clamp01(nm_to_mm(m) / 1.0);
    }

    return f;
}

double weighted_impact_total(const ImpactFeatures& f, const ImpactWeights& w,
                             std::array<double, kImpactFeatureCount>* contrib) {
    double total = 0.0;
    for (int i = 0; i < kImpactFeatureCount; ++i) {
        double c = w.w[i] * f.at(i);
        if (contrib) (*contrib)[i] = c;
        total += c;
    }
    return clamp01(total);
}

double mlp_impact_total(const ImpactFeatures& f, const MlpImpactConfig& cfg) {
    double h[4];
    for (int j = 0; j < 4; ++j) {
        double z = cfg.b1[j];
        for (int i = 0; i < 9; ++i) z += cfg.w1[j * 9 + i] * f.at(i);
        h[j] = std::tanh(z);
    }
    double logit = cfg.b2;
    for (int j = 0; j < 4; ++j) logit += cfg.w2[j] * h[j];
    double s = 1.0 / (1.0 + std::exp(-logit));
    return clamp01(s);
}

ImpactScore WeightedImpactScorer::score(const CandidateRoute& cand,
                                        const ImpactContext& ictx) const {
    ImpactScore s;
    s.scorer = "weighted";
    s.base_cost_nm = cand.cost_nm;
    s.features = compute_impact_features(cand, ictx);
    s.total = weighted_impact_total(s.features, weights_, &s.contributions);
    if (ictx.board && ictx.resolver && ictx.ctx) {
        std::string why;
        s.legal = candidate_legal_vs_board(cand, *ictx.board, *ictx.resolver, *ictx.ctx, why);
        s.legal_reason = why;
    }
    return s;
}

ImpactScore MlpImpactScorer::score(const CandidateRoute& cand,
                                   const ImpactContext& ictx) const {
    ImpactScore s;
    s.base_cost_nm = cand.cost_nm;
    s.features = compute_impact_features(cand, ictx);
    if (!cfg_.valid()) {
        s.scorer = "mlp(fallback=weighted)";
        s.fallback = true;
        s.total = weighted_impact_total(s.features, fallback_, &s.contributions);
    } else {
        s.scorer = "mlp";
        s.total = mlp_impact_total(s.features, cfg_);
        // Report uniform-equivalent contributions for logging: each feature
        // share of the total proportional to its normalized value.
        double denom = 0;
        for (int i = 0; i < kImpactFeatureCount; ++i) denom += s.features.at(i);
        for (int i = 0; i < kImpactFeatureCount; ++i) {
            s.contributions[i] =
                denom > 0 ? s.total * s.features.at(i) / denom : s.total / kImpactFeatureCount;
        }
    }
    if (ictx.board && ictx.resolver && ictx.ctx) {
        std::string why;
        s.legal = candidate_legal_vs_board(cand, *ictx.board, *ictx.resolver, *ictx.ctx, why);
        s.legal_reason = why;
    }
    return s;
}

std::unique_ptr<RouteImpactScorer> make_impact_scorer(const ImpactOptions& opts) {
    if (opts.use_mlp && opts.mlp.valid())
        return std::make_unique<MlpImpactScorer>(opts.mlp, opts.weights);
    if (opts.use_mlp)
        return std::make_unique<MlpImpactScorer>(opts.mlp, opts.weights);
    return std::make_unique<WeightedImpactScorer>(opts.weights);
}

std::vector<int> rank_candidates_by_impact(const std::vector<CandidateRoute>& candidates,
                                           const std::vector<ImpactScore>& scores) {
    std::vector<int> idx(candidates.size());
    for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = static_cast<int>(i);
    std::sort(idx.begin(), idx.end(), [&](int a, int b) {
        bool la = a < (int)scores.size() ? scores[a].legal : false;
        bool lb = b < (int)scores.size() ? scores[b].legal : false;
        if (la != lb) return la > lb;  // legal first
        double ta = a < (int)scores.size() ? scores[a].total : 0.0;
        double tb = b < (int)scores.size() ? scores[b].total : 0.0;
        if (ta != tb) return ta < tb;  // lower obstruction wins
        if (candidates[a].cost_nm != candidates[b].cost_nm)
            return candidates[a].cost_nm < candidates[b].cost_nm;
        // Stable signature fallback: task identity + cost already fixed, so
        // compare trace/via counts then first-segment geometry.
        if (candidates[a].traces.size() != candidates[b].traces.size())
            return candidates[a].traces.size() < candidates[b].traces.size();
        if (candidates[a].vias.size() != candidates[b].vias.size())
            return candidates[a].vias.size() < candidates[b].vias.size();
        if (a != b) {
            // Deterministic final tie-break on input order (stable sort key).
            return a < b;
        }
        return false;
    });
    return idx;
}

int select_best_candidate(const std::vector<CandidateRoute>& candidates,
                          const std::vector<ImpactScore>& scores,
                          const ImpactOptions& opts) {
    if (candidates.empty()) return -1;
    if (!opts.enabled) {
        // Base-cost order under the identical legality gate.
        int best = -1;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (best < 0) {
                best = static_cast<int>(i);
                continue;
            }
            bool li = i < scores.size() ? scores[i].legal : false;
            bool lb = best < (int)scores.size() ? scores[best].legal : false;
            if (li != lb) {
                if (li) best = static_cast<int>(i);
                continue;
            }
            if (candidates[i].cost_nm < candidates[best].cost_nm)
                best = static_cast<int>(i);
        }
        return best;
    }
    std::vector<int> order = rank_candidates_by_impact(candidates, scores);
    return order.empty() ? -1 : order.front();
}

int score_and_select(const std::vector<CandidateRoute>& candidates,
                     const ImpactContext& ictx, const ImpactOptions& opts,
                     std::vector<ImpactScore>& scores_out) {
    scores_out.clear();
    scores_out.reserve(candidates.size());
    auto scorer = make_impact_scorer(opts);
    for (const auto& c : candidates) scores_out.push_back(scorer->score(c, ictx));
    return select_best_candidate(candidates, scores_out, opts);
}

}  // namespace copperline
