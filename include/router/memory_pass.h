// Copperline: Memory routing pass (Phase 3 item 16).
//
// The classic "memory" U-pattern router: for each net, find terminal pairs
// in DIFFERENT components that share an X or Y coordinate, and route them
// with a deterministic dogbone/U pattern on one layer:
//
//   aligned on X:  out from A, along X to B's x, in to B   (vertical legs)
//   aligned on Y:  out from A, along Y to B's y, in to B   (horizontal legs)
//
// This mirrors the Situs "Memory"/"Adjacent Memory" passes (see
// docs/reverse/circuitmaker/autorouter_modules.md M7): dense same-net pin
// rows/columns get short legal channels before the global router runs, so
// the global A* never wastes budget on trivially-patternable pairs.
//
// Design constraints:
//   * Deterministic: pairs are sorted (net, terminal-a, terminal-b); the
//     pattern geometry is a pure function of the two pad rects and the
//     board's default trace width.
//   * Legal-first: every candidate leg is clearance-checked against the
//     committed copper + pads + keepouts via the SpatialIndex; an illegal
//     leg aborts the whole pair (no partial commit, no rip-up of others).
//   * Non-invasive: the pass commits through the same owned-route path the
//     escape stage uses (is_memory_stub=true), and the final BoardVerifier
//     gate applies to everything, memory copper included. Gated by the
//     declarative strategy ("memory": true) — OFF by default so existing
//     golden hashes are untouched.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "router/board.h"
#include "router/json.h"
#include "router/rules.h"
#include "router/spatial_index.h"

namespace copperline {

// Per-net memory-pair results for agent observability.
struct MemoryPassReport {
    bool ran = false;          // false when the strategy did not enable it
    int pairs_found = 0;
    int pairs_committed = 0;
    int pairs_skipped_illegal = 0;
    int traces_added = 0;
    int vias_added = 0;
    std::vector<std::string> skipped_nets;  // nets with an illegal pair
    JsonValue to_json() const;
};

// Run the memory pass on `board`, committing legal U-pattern routes.
// `resolver` provides clearance/width rules; `ctx` the electrical context.
// Only same-net pairs in DIFFERENT components that share X or Y and are on
// the same layer are considered. Returns the pass report.
MemoryPassReport run_memory_pass(Board& board, const RuleResolver& resolver,
                                 const ElectricalContext& ctx,
                                 const std::vector<std::string>& via_names);

}  // namespace copperline