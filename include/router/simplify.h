// Copperline: arbitrary-angle minimum-bend trace simplification (issue #17).
//
// Committed traces are exact line segments at arbitrary angles with integer
// nanometre endpoints. The sparse graph + A* search is Manhattan guidance
// only; it must never leak stair-steps, octilinear elbows or 45-45 detours
// into copper. After A* reconstruction, every same-layer waypoint run is
// string-pulled: the simplifier repeatedly connects the farthest later
// waypoint whose exact centerline segment is legal under the current width,
// voltage-clearance, keepout, boundary and via rules.
//
// Pipeline per same-layer run (endpoints fixed, deterministic):
//   1. Prefer the direct source->target segment when exactly legal.
//   2. Otherwise try the shortest legal single-bend path through an expanded
//      obstacle visibility vertex (tangent/corner low-bend detour).
//   3. Otherwise greedy farthest-reach shortcutting over the A* waypoints.
//   4. Merge exactly-collinear runs, drop zero-length/duplicates and tiny
//      jogs (each removal re-checked for exact legality).
//   5. Re-run the full exact legality gate; on any failure retain the prior
//      legal geometry verbatim.
//
// Integer arithmetic throughout (__int128 intermediates); no floating point
// in legality. Length reporting uses integer-rounded Euclidean distance.
//
// Issue #15 hook: tasks flagged `tuning_exempt` (intentional tuning regions
// such as future length-tuning meanders) skip simplification entirely via
// simplify_exempt_task(). The flag exists and is honored; no producer sets
// it yet.
#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "router/board.h"
#include "router/route_tree.h"
#include "router/rules.h"

namespace copperline {

// Integer-rounded Euclidean length: llround(sqrt(dx^2 + dy^2)). Exact inputs,
// deterministic across runs of the same binary.
inline Coord euclid_len_nm(Point a, Point b) {
    __int128 dx = (__int128)a.x - b.x;
    __int128 dy = (__int128)a.y - b.y;
    __int128 d2 = dx * dx + dy * dy;
    long double d = static_cast<long double>(d2);
    // long double sqrt is correctly rounded on x86 (80-bit); llround keeps
    // the result an integer. Only used for reporting/ordering, never for
    // legality (which stays in squared integer space).
    long double r = std::sqrt(d);
    if (r >= static_cast<long double>(std::numeric_limits<Coord>::max()))
        return std::numeric_limits<Coord>::max();
    return static_cast<Coord>(std::llround(r));
}

struct SimplifyOptions {
    Coord tiny_jog_nm = 0;  // 0 = auto: max(width/4, 5um)
    bool use_visibility = true;
    std::size_t max_corners = 256;
};

struct SimplifyStats {
    int segments_before = 0;
    int segments_after = 0;
    int bends_before = 0;
    int bends_after = 0;
    bool direct_used = false;
    bool retained_prior = false;  // legality gate failed: prior kept
    bool skipped_exempt = false;  // tuning-exempt task: untouched
};

// Exact centerline legality of one segment on one layer: board bounds (with
// half width), keepouts at worst-case clearance, foreign pads/traces/vias at
// pair voltage clearance, foreign planes at exact polygon clearance.
// Same-net copper is connectable and never blocks. Mirrors the arbiter's
// trace_legal predicate exactly.
bool simplify_segment_legal(const Board& board, const RuleResolver& resolver, NetId net,
                            LayerId layer, Coord width_nm, const Segment& s,
                            const ElectricalContext& ctx);

// Issue #15: partner-aware variant for pair length tuning. Copper owned by
// `exempt_net` (the pair partner) is skipped by the generic voltage-clearance
// test; the caller enforces the pair gap floor separately with
// pair_gap_legal(). exempt_net < 0 behaves exactly like
// simplify_segment_legal. Deterministic, integer-exact.
bool simplify_segment_legal_except(const Board& board, const RuleResolver& resolver,
                                   NetId net, LayerId layer, Coord width_nm,
                                   const Segment& s, const ElectricalContext& ctx,
                                   NetId exempt_net);

// Deterministic clearance-expanded obstacle corners near the corridor rect:
// keepouts, foreign pads, foreign trace bboxes, foreign via barrels and
// foreign plane bboxes, each expanded by its exact keep distance. Sorted by
// (x, y), unique, capped. Legality of any candidate using them is still
// decided by simplify_segment_legal, never by the expansion itself.
std::vector<Point> simplify_visibility_corners(const Board& board,
                                               const RuleResolver& resolver, NetId net,
                                               LayerId layer, Coord width_nm,
                                               const Rect& corridor,
                                               const ElectricalContext& ctx,
                                               std::size_t max_corners = 256);

// Simplify one same-layer integer waypoint run. Endpoints are fixed. Returns
// the simplified waypoint list; on any legality failure returns the cleaned
// input verbatim (deduplicated). Deterministic.
std::vector<Point> simplify_polyline(const Board& board, const RuleResolver& resolver,
                                     NetId net, LayerId layer, Coord width_nm,
                                     const std::vector<Point>& pts,
                                     const ElectricalContext& ctx,
                                     const SimplifyOptions& opt = SimplifyOptions{});

// Candidate-level simplifier shared by ordinary routes, BGA escape stubs,
// power-plane access (#16) and reroute branches (all flow through the same
// per-layer-run path). `traces` holds route geometry in path order;
// `is_stub[i]` marks via-bundle star stubs that must be preserved verbatim
// (they were planned atomically with their bundle). When tuning_exempt is
// true the geometry is left untouched for issue #15. Never produces illegal
// copper: the exact gate re-runs over the result and any failure restores
// the input.
SimplifyStats simplify_candidate_traces(const Board& snapshot, const RuleResolver& resolver,
                                        const ElectricalContext& ctx, NetId net,
                                        std::vector<TraceSeg>& traces,
                                        const std::vector<char>& is_stub, bool tuning_exempt,
                                        const SimplifyOptions& opt = SimplifyOptions{});

// Escape-level variant: traces carry no stub mask (escape vias are separate);
// every trace is route geometry simplified per layer run.
bool simplify_escape_traces(const Board& work, const RuleResolver& resolver,
                            const ElectricalContext& ctx, NetId net,
                            std::vector<TraceSeg>& traces,
                            const SimplifyOptions& opt = SimplifyOptions{});

// Issue #15 hook: intentional tuning regions are exempt from simplification.
// Currently driven by the per-task flag only; region-based producers land
// with #15.
inline bool simplify_exempt_task(const ConnectionTask& task) { return task.tuning_exempt; }

}  // namespace copperline
