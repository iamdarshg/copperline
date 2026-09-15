// Copperline: hierarchical coarse-to-fine A* guidance (issue #10).
//
// A lightweight multi-resolution guidance grid, separate from the committed
// geometry. Levels run e.g. 0.8 -> 0.4 -> 0.2 -> 0.1mm. The coarsest level
// finds a cheap obstacle/congestion-aware path over (cell x layer) states
// with via transitions; each finer level re-searches only inside a
// refinement window around the best coarser corridor. The final refined
// corridor clips the exact sparse graph to a handoff tube and biases
// (soft pull-to-path cost) the exact sparse A*; exact integer geometry
// remains the sole legality and committed copper source (the simplifier
// of issue #17 stays authoritative, so no grid staircase can leak into
// copper).
//
// Guarantees:
//   - Guidance never removes a valid solution: a clipped miss rebuilds
//     wider, ending in the unrestricted exact A* on the full graph.
//   - Deterministic: grid A* uses fixed neighbor order and lexicographic
//     tie-breaks; all work per task is thread-local. Sharing a
//     HierarchyCache across workers never changes results.
//   - The cache reuses the per-snapshot raw-obstacle list across tasks of an
//     epoch; it self-invalidates when the board signature changes, so it is
//     safe to share across epochs and recovery branches.
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "router/astar.h"
#include "router/board.h"
#include "router/geometry.h"
#include "router/json.h"
#include "router/rules.h"
#include "router/sparse_graph.h"

namespace copperline {

struct HierarchyConfig {
    bool enabled = true;
    // Coarse -> fine cell pitches in integer nm. Default 0.8/0.4/0.2/0.1mm.
    std::vector<Coord> level_pitch_nm = {800000, 400000, 200000, 100000};
    std::int64_t max_coarse_expansions = 200000;  // per level
    int window_margin_cells = 2;   // refinement window padding per level
    int max_window_attempts = 2;   // windowed exact tries before fallback
    std::size_t max_grid_cells = 8000000;  // levels above this are skipped
    // Windowed finer levels whose search area exceeds this are skipped
    // (the coarser window stands). A millimetre-scale handoff tube needs
    // no sub-0.2mm refinement on board-spanning windows; full-board levels
    // (no window yet) always run, so narrow slots are still discovered.
    std::size_t max_window_cells = 2000;
    // Exact-search tube half-width around the finest coarse path (integer
    // nm). The exact sparse graph is built only from base points inside
    // the tube (plus src/dst); later attempts widen it before the
    // unrestricted fallback rebuilds the full graph. The tube must cover
    // sparse-graph node spacing (corner nodes jump 2-5mm along streets),
    // not just the centerline: too tight a tube isolates src and forces
    // wasted rebuilds. Sized for exact node spacing plus local
    // exact-sized detours around the corridor.
    Coord tube_half_nm = 5000000;  // 5mm
    // Pull-to-path bias weight: bias[node] = min(max_bias_nm, dist/4).
    // Weak on purpose: strong bias makes f inconsistent with the
    // Manhattan heuristic and explodes re-expansions. Ordering assist
    // only (never legality); the tube mask does the pruning.
    Coord max_bias_nm = 500000;  // 0.5mm equiv
};

struct HierarchyDiag {
    bool attempted = false;  // guidance ran (enabled and coarse attempted)
    bool guided = false;     // the deciding exact run used a window
    bool fallback = false;   // the deciding exact run was unrestricted
    // "none" | "disabled" | "coarse_fail" | "guided" | "guided_expanded" |
    // "window_miss"
    std::string fallback_reason = "none";
    std::vector<double> levels_used_mm;  // coarse->fine pitches actually searched
    std::int64_t coarse_expansions = 0;  // sum over searched levels
    std::int64_t exact_expansions = 0;   // expansions of the deciding exact run
    int window_attempts = 0;             // windowed exact tries performed
    JsonValue to_json() const;
};

// Per-level coarse path (cell-center polyline, integer nm) for diagnostics
// and tests. Levels are coarse -> fine, parallel to levels_used_mm.
struct GuidanceResult {
    bool found = false;  // at least the coarsest level connected
    std::vector<std::vector<Point>> level_paths;
    std::vector<int> level_layers;  // layer per point of the finest path
    Rect final_window{};  // refinement window around the finest path
    bool has_window = false;
    Coord finest_pitch_nm = 0;  // pitch of the finest successful level
    std::vector<double> levels_used_mm;  // coarse->fine pitches actually searched
    std::int64_t coarse_expansions = 0;
};

struct HierarchyRequest {
    NetId net = -1;
    Point src{};
    LayerId src_layer = 0;
    std::vector<SparseTarget> dsts;
    Coord width_nm = 0;      // routing width (occupied envelope for pairs)
    Coord clearance_nm = 0;  // worst-case clearance burden for this task
    std::vector<double> layer_mult;  // effective (impedance/pair-adjusted) mults
    AStarConfig astar_cfg;
    // Soft planning cost sampler (congestion + reservations). May be empty.
    std::function<Coord(const Segment&)> soft_cost;
};

class HierarchyCache {
  public:
    HierarchyCache() = default;

    // Coarse-to-fine guidance only (no exact search). The caller clips
    // the exact sparse graph to the returned corridor, biases the exact
    // run toward it, and falls back to the full graph on a miss, so
    // guidance never removes a valid solution. Exposed for tests and
    // diagnostics.
    GuidanceResult build_guidance(const Board& board, const RuleResolver& resolver,
                                  const ElectricalContext& ctx, const HierarchyRequest& req,
                                  const HierarchyConfig& cfg) const;

  private:
    struct RawObs {
        Rect raw{};
        LayerId layer = kAllLayers;
        NetId net = -1;  // -1 = keepout (never own copper)
    };
    struct Snapshot {
        std::string key;
        Rect bounds{};
        std::vector<RawObs> obstacles;
    };
    mutable std::mutex mutex_;
    mutable Snapshot cached_;
    mutable std::string cached_key_;

    const Snapshot& snapshot_for(const Board& board) const;
};

}  // namespace copperline
