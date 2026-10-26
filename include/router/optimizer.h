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
    // Phase 3 item 19: straighten corner probes (H/V/45) that shorten, gated
    // on verify+revert like every other transform. OFF by default: existing
    // golden geometry is untouched unless a strategy sets "straighten": true.
    bool straighten = false;
    // Phase 3 item 24: Via Miser mode — the optimizer runs ONLY via
    // elimination (a focused via-min pass) and skips the other transforms.
    // OFF by default; enabled by a strategy with "via_miser": true.
    bool via_miser = false;
    // Phase 3 item 20: miter axis-aligned 90-degree corners with a 45-degree
    // chamfer (shorter copper, fewer sharp corners) when a strategy sets
    // "orthogonal": true. OFF by default.
    bool orthogonal = false;
    // Phase 3 item 18: reroute a pad's exit along the pad's longest axis for
    // asymmetric rectangular pads. OFF by default.
    bool clean_pad_entries = false;
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
    bool straighten_ran = false;  // pass 5 ran (strategy-gated)
    bool via_miser_ran = false;   // pass ran in Via Miser mode (item 24)
    bool orthogonal_ran = false;  // pass 6 ran (item 20 miter, strategy-gated)
    bool clean_pad_entries_ran = false;  // pass 7 ran (item 18, strategy-gated)
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
