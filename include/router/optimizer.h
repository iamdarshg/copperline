// Copperline: transactional cleanup optimizer (Prompt 5).
//
// Runs ONLY after complete legal connectivity (engine gates on COMPLETE +
// BoardVerifier ok). Every transform is transactional: apply, re-verify with
// the independent BoardVerifier, and revert on any harm to connectivity,
// legality, or higher-priority electrical requirements (width floors, via
// current capacity, clearance). Acceptance stays lexicographic — the
// optimizer never trades connectivity/legality for polish.
//
// Transforms (deterministic order, bounded candidates):
//   1. collinear merge  — consecutive same-net/same-layer collinear segments
//   2. bend removal     — replace an A-B-C corner with A-C when shorter
//   3. via elimination  — drop a via whose two sides join on one layer
//   4. preferred-layer  — move a single-layer net onto layer 0 when legal
// Congestion relief falls out of 2-4 (fewer bends/vias = less blockage) and
// is reported via bend/via deltas rather than a separate risky pass.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "router/board.h"
#include "router/json.h"
#include "router/rules.h"
#include "router/verifier.h"

namespace copperline {

struct OptimizerOptions {
    bool enabled = true;
    // Router memory bound share: at most this many candidates per pass.
    std::size_t max_candidates = 256;
    bool collinear_merge = true;
    bool bend_removal = true;
    bool via_elimination = true;
    bool preferred_layer = true;
};

struct OptimizerReport {
    bool enabled = true;
    bool ran = false;  // false when gated off (not COMPLETE / not legal)
    std::string gate_reason;  // why it did not run, when ran == false
    int passes = 0;
    int candidates = 0;
    int applied = 0;
    int reverted = 0;
    int bends_before = 0;
    int bends_after = 0;
    int vias_before = 0;
    int vias_after = 0;
    double length_before_mm = 0.0;
    double length_after_mm = 0.0;
    std::size_t peak_bytes = 0;  // bounded scratch (candidates * per-item)
    JsonValue to_json() const;
};

class CleanupOptimizer {
  public:
    CleanupOptimizer(Board* board, const RuleResolver* resolver,
                     const ElectricalContext& ctx, OptimizerOptions options = {});

    OptimizerReport run();

  private:
    Board* board_;
    const RuleResolver* resolver_;
    ElectricalContext ctx_;
    OptimizerOptions opt_;
};

}  // namespace copperline
