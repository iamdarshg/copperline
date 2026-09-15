// Copperline: future-obstruction scoring for route portfolios (issue #9).
//
// A RouteImpactScorer looks at one already-legal portfolio candidate and
// estimates how much of the board's *future* routing resource it consumes:
// remaining-task corridors it covers, pin-dense / bottleneck / fine-pitch
// ground it occupies, copper footprint (width+clearance), scarce layers,
// via sites, and the electrical width/clearance burden that forced that
// footprint. All nine features are normalized to [0,1]; the deterministic
// weighted baseline sums weight*feature (weights sum to 1, total in [0,1]).
//
// Ranking is lexicographic and stable:
//
//   1. global connectivity legality first (candidate_legal_vs_board),
//   2. then lower obstruction,
//   3. then lower base A* cost,
//   4. then stable signature string.
//
// Disabling the scorer (ImpactOptions.enabled=false) reverts candidate
// selection to base-cost order with the identical legality gate, so removal
// never changes which candidates are legal, only which legal candidate wins.
//
// The optional tiny feed-forward scorer (MlpImpactScorer) sits behind the
// same interface with explicit fixed weights/config only; the heuristic
// weighted scorer stays the default and the fallback when the MLP config is
// invalid. Per-feature contributions and the final score are logged via
// ImpactScore::to_json().
//
// Threading/memory: scoring streams over the remaining-corridor list with
// O(1) extra state per candidate (no dense matrices, no per-thread copies of
// the board); corridor buffers are borrowed from the caller. Effective worker
// threads resolve via resolve_worker_threads (0=auto, explicit wins) and are
// reported in diagnostics alongside the scores. Total router memory stays
// under kRouterMemoryBudgetBytes via the existing K x batch bound.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "router/astar.h"
#include "router/board.h"
#include "router/json.h"
#include "router/parallel.h"
#include "router/rules.h"

namespace copperline {

// Nine normalized obstruction features, each in [0,1]. Ordered for stable
// JSON and for the fixed MLP input order (index matters for w1 layout).
struct ImpactFeatures {
    double future_corridor_overlap = 0.0;  // max remaining-corridor cover frac
    double pin_density = 0.0;              // pads/mm^2 along candidate, /12
    double bottleneck_scarcity = 0.0;      // frac of remaining corridors hit
    double consumed_footprint = 0.0;       // expanded copper area / 50mm^2
    double layer_scarcity = 0.0;           // scarce-layer use in [0,1]
    double via_site_consumption = 0.0;     // via count / 6, capped
    double fine_pitch_proximity = 0.0;     // 1 - minDist/4mm near deep pads
    double current_width = 0.0;            // required width / 2mm
    double voltage_clearance = 0.0;        // max clearance / 1mm

    double at(int i) const;
    void set(int i, double v);
    JsonValue to_json() const;
};

inline constexpr int kImpactFeatureCount = 9;
inline constexpr const char* kImpactFeatureNames[kImpactFeatureCount] = {
    "future_corridor_overlap", "pin_density",           "bottleneck_scarcity",
    "consumed_footprint",      "layer_scarcity",        "via_site_consumption",
    "fine_pitch_proximity",    "current_width",         "voltage_clearance"};

// Deterministic default weights (sum exactly 1.0). Heavier on the future
// (overlap + bottleneck scarcity) so a slightly longer detour that preserves
// the only corridor for a later net outranks the cheap corridor thief.
struct ImpactWeights {
    std::array<double, kImpactFeatureCount> w = {
        {0.22, 0.10, 0.14, 0.12, 0.10, 0.08, 0.08, 0.08, 0.08}};
    static ImpactWeights defaults() { return ImpactWeights{}; }
    double sum() const;
    JsonValue to_json() const;
};

// Tiny feed-forward config: 9 inputs -> 4 hidden (tanh) -> 1 logit (sigmoid).
// All weights are explicit and fixed; there is no training path in the
// router. Layout is row-major: w1[h*9+i], b1[h], w2[h], b2 scalar.
struct MlpImpactConfig {
    std::array<double, 9 * 4> w1;
    std::array<double, 4> b1 = {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> w2 = {{0.25, 0.25, 0.25, 0.25}};
    double b2 = -0.5;
    static MlpImpactConfig fixed_default();
    bool valid() const;  // finite, correctly sized (always true for the
                         // struct, false only for NaN/inf entries)
};

struct ImpactOptions {
    bool enabled = true;   // false = base-cost order, same legality gate
    bool use_mlp = false;  // true = MLP scorer, fallback to weighted
    ImpactWeights weights = ImpactWeights::defaults();
    MlpImpactConfig mlp = MlpImpactConfig::fixed_default();
    std::size_t memory_budget_bytes = kRouterMemoryBudgetBytes;
    int threads_requested = 0;  // 0 = auto (resolve_worker_threads)
};

// Borrowed scoring context. The caller owns every buffer; the scorer copies
// nothing except small per-candidate scalars (streaming, O(1) scratch).
struct ImpactContext {
    const Board* board = nullptr;              // snapshot before candidate
    const RuleResolver* resolver = nullptr;
    const ElectricalContext* ctx = nullptr;
    // Remaining (not-yet-routed) tasks + their probable corridors, parallel.
    // Entries with indices in `exclude_task_pos` are skipped by the caller
    // before filling this (usually the candidate's own task).
    std::vector<ConnectionTask> remaining_tasks;
    std::vector<Corridor> remaining_corridors;
    // Endpoint pin density + centre depth for density/fine-pitch features.
    std::vector<double> terminal_density;  // parallel to board->terminals
    std::map<TermId, int> centre_depth;
    const CongestionMap* congestion = nullptr;  // optional, may be null
};

struct ImpactScore {
    ImpactFeatures features;
    std::array<double, kImpactFeatureCount> contributions = {{0}};
    double total = 0.0;  // in [0,1] for both scorers
    bool legal = false;  // candidate_legal_vs_board vs context board
    std::string legal_reason;
    Coord base_cost_nm = 0;
    std::string signature;
    std::string scorer;  // "weighted" | "mlp" | "mlp(fallback=weighted)"
    bool fallback = false;
    JsonValue to_json() const;
};

// Abstract scorer behind one interface (heuristic + MLP share it).
class RouteImpactScorer {
  public:
    virtual ~RouteImpactScorer() = default;
    virtual ImpactScore score(const CandidateRoute& cand,
                              const ImpactContext& ictx) const = 0;
    virtual const char* name() const = 0;
};

class WeightedImpactScorer : public RouteImpactScorer {
  public:
    explicit WeightedImpactScorer(ImpactWeights w = ImpactWeights::defaults())
        : weights_(w) {}
    ImpactScore score(const CandidateRoute& cand,
                      const ImpactContext& ictx) const override;
    const char* name() const override { return "weighted"; }
    const ImpactWeights& weights() const { return weights_; }

  private:
    ImpactWeights weights_;
};

class MlpImpactScorer : public RouteImpactScorer {
  public:
    explicit MlpImpactScorer(MlpImpactConfig cfg = MlpImpactConfig::fixed_default(),
                             ImpactWeights fb = ImpactWeights::defaults())
        : cfg_(cfg), fallback_(fb) {}
    ImpactScore score(const CandidateRoute& cand,
                      const ImpactContext& ictx) const override;
    const char* name() const override { return "mlp"; }

  private:
    MlpImpactConfig cfg_;
    ImpactWeights fallback_;
};

// Build the scorer selected by opts (weighted default; MLP when requested
// with a valid config, otherwise weighted fallback). Never null.
std::unique_ptr<RouteImpactScorer> make_impact_scorer(const ImpactOptions& opts);

// Pure feature extraction (normalized [0,1]). Deterministic.
ImpactFeatures compute_impact_features(const CandidateRoute& cand,
                                       const ImpactContext& ictx);

// Weighted total in [0,1] + per-feature contributions (w[i]*f[i]).
double weighted_impact_total(const ImpactFeatures& f, const ImpactWeights& w,
                             std::array<double, kImpactFeatureCount>* contrib = nullptr);

// MLP forward pass over normalized features -> (0,1). Pure + deterministic.
double mlp_impact_total(const ImpactFeatures& f, const MlpImpactConfig& cfg);

// Rank candidate indices best-first: legal first, then lower obstruction,
// then lower base cost, then stable signature. Scores[i] parallels
// candidates[i]. Pure + deterministic (stable tie-break on input order).
std::vector<int> rank_candidates_by_impact(const std::vector<CandidateRoute>& candidates,
                                           const std::vector<ImpactScore>& scores);

// Best index or -1 when empty. When opts.enabled==false the scores are
// ignored and the cheapest legal candidate wins (same legality gate:
// legal before illegal, then cost, then signature) -- removal reverts to
// base-cost without changing legality.
int select_best_candidate(const std::vector<CandidateRoute>& candidates,
                          const std::vector<ImpactScore>& scores,
                          const ImpactOptions& opts);

// Convenience: score + rank + pick best in one call. Fills scores_out
// (parallel to candidates) and returns the best index (or -1).
int score_and_select(const std::vector<CandidateRoute>& candidates,
                     const ImpactContext& ictx, const ImpactOptions& opts,
                     std::vector<ImpactScore>& scores_out);

}  // namespace copperline
