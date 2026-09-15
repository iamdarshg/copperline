// Copperline: differential-pair corridor planning + post-route materialization
// (issue #12).
//
// Differential pairs participate in global planning WITHOUT expensive
// detailed coupled routing during the main board-fill phase:
//
//   1. Pair metadata (DiffPair in board.h): P/N members, target gap +
//      tolerance, pair width, preferred layers, optional impedance target,
//      max skew, via-pair policy.
//   2. Global routing replaces the two individual net tasks with one
//      PairCorridorTask (ConnectionTask with is_pair_corridor=true) whose
//      occupied width = both traces + pair gap + external clearance.
//      Scheduling, interference, congestion and rip-up treat the corridor
//      atomically; P/N never route independently in the main phase.
//   3. The corridor is routed/reserved with the ordinary global search +
//      #17 arbitrary-angle rules (minimum-bend simplification, exact
//      legality gate).
//   4. After topology closure, DiffPairMaterializer creates the two actual
//      traces inside the reserved corridor: parallel arbitrary-angle
//      centerlines/offsets with paired layer transitions/vias. Bends keep
//      the gap (mitered parallel offsets, no 45-degree doglegs).
//   5. Both members commit atomically only after exact legality; otherwise
//      the pair is marked for corridor re-route/recovery (no
//      one-member-only copper is ever left behind).
//   6. Only after materialization + verification may P/N enter cleanup
//      (there is no optimizer yet, so this is a structural guarantee:
//      the materializer output is never passed through generic bend
//      removal that could break coupling).
//
// All geometry is integer nanometres; legality uses exact integer
// arithmetic. Everything is deterministic across thread counts: corridor
// routing flows through the ordinary deterministic epoch machinery and
// materialization is a single-threaded post-phase in pair-id order.
#pragma once

#include <string>
#include <vector>

#include "router/board.h"
#include "router/json.h"
#include "router/route_tree.h"
#include "router/rules.h"

namespace copperline {

// ---- Pair task construction ----

// True when the pair declaration is structurally usable: both nets exist,
// distinct, disjoint from other pairs (checked by caller), gap sane.
bool diffpair_valid(const Board& board, const DiffPair& pair, std::string& reason_out);

// Resolve the two terminals of each member net. Requires exactly 2
// terminals per member (v1 scope); returns false with a reason otherwise.
// pa/pb are the P terminals (sorted ascending), na/nb the N terminals.
bool diffpair_endpoints(const Board& board, const DiffPair& pair, TermId& pa,
                        TermId& pb, TermId& na, TermId& nb, std::string& reason_out);

// Trace width for one member on one layer: max(pair explicit width,
// member's reconciled required width incl. #11 impedance + #7 ampacity).
// Never drops below the current minimum.
Coord diffpair_member_width(const Board& board, const RuleResolver& resolver,
                            const DiffPair& pair, NetId member, LayerId layer,
                            const ElectricalContext& ctx);

// External clearance for the corridor: max voltage clearance from either
// member to any net outside the pair (the P-N gap itself is governed by
// gap_nm, not by the voltage table).
Coord diffpair_external_clearance(const Board& board, const RuleResolver& resolver,
                                  const DiffPair& pair, const ElectricalContext& ctx);

// Occupied corridor width: w_p + w_n + gap + 2 * external_clearance.
// Sized on the source layer (widths are layer-aware); the candidate uses
// the max across layers for impedance-controlled members.
Coord diffpair_occupied_width(const Board& board, const RuleResolver& resolver,
                              const DiffPair& pair, const ElectricalContext& ctx);

// Build the single atomic corridor task for a valid pair. Returns false
// when endpoints/layers are unusable (caller records a pair failure).
bool make_pair_corridor_task(const Board& board, const DiffPair& pair, int index,
                             ConnectionTask& out, std::string& reason_out);

// Build the full global task list: per-net RouteTrees for non-pair nets
// plus one corridor task per valid pair. Pair member nets never produce
// individual tasks. Invalid pairs are reported via invalid_reasons
// (parallel to the invalid pair ids) and produce no tasks. Deterministic.
std::vector<ConnectionTask> build_global_tasks_with_pairs(
    const Board& board, std::vector<std::string>& invalid_reasons,
    std::vector<int>& invalid_pair_ids);

// ---- Corridor geometry ----

struct PairCorridor {
    int pair_id = -1;
    NetId net_p = -1;
    NetId net_n = -1;
    // Reserved centerline copper as committed (net = P, width = occupied).
    std::vector<TraceSeg> center_traces;
    std::vector<Via> center_vias;
    Coord occupied_width_nm = 0;
};

// ---- Materialization ----

struct MaterializedPair {
    bool ok = false;
    std::string reason;  // "ok" | "pair_layer_mismatch" | "gap_violation" |
                         // "skew_exceeded" | "via_infeasible" | "illegal:..." | ...
    int pair_id = -1;
    NetId net_p = -1;
    NetId net_n = -1;
    std::vector<TraceSeg> traces_p;
    std::vector<TraceSeg> traces_n;
    std::vector<Via> vias_p;
    std::vector<Via> vias_n;
    Coord length_p_nm = 0;
    Coord length_n_nm = 0;
    Coord skew_nm = 0;
    Coord worst_gap_err_nm = 0;  // max |edge_gap - gap| over coupled sections
    Point gap_violation_at{};    // representative location when !ok on gap
    bool has_gap_violation_at = false;
};

// Materialize the two coupled members inside the reserved corridor.
// `board_without_corridor` is the committed board WITH the corridor's
// temporary copper removed (so the pair is checked against real foreign
// copper, not against its own envelope). The corridor centerline comes
// from the accepted corridor candidate (arbitrary-angle, minimum-bend).
// Returns ok=false with a machine-readable reason when the pair cannot be
// realized legally; in that case all output vectors are empty (atomic).
MaterializedPair materialize_pair(const Board& board_without_corridor,
                                  const RuleResolver& resolver,
                                  const ElectricalContext& ctx,
                                  const DiffPair& pair,
                                  const PairCorridor& corridor);

// True when every coupled P/N section on shared layers keeps edge gap
// inside gap +/- tol (exact integer math). The floor (every same-layer
// pair >= gap - tol) rejects pinches; the ceiling (every trunk segment's
// nearest same-layer opposite trace <= gap + tol, issue #26) rejects
// excessive separation. Reports the worst deviation and a location.
// Marked tuning teeth (TraceSeg::tuning_tooth, #15) are intrinsic and
// honored here too; only the pad-incident fanout exemption needs the
// fanout-aware form below (no pad context here).
bool pair_gap_legal(const std::vector<TraceSeg>& traces_p,
                    const std::vector<TraceSeg>& traces_n, Coord gap_nm,
                    Coord tol_nm, Coord& worst_err_out, Point& at_out);

// Fanout-aware variant (issue #26): trace segments with an endpoint exactly
// at one of fanout_pts (pair member pad positions, integer-nm identity) are
// #12 endpoint fanout -- pad-end stitching drops the perpendicular offset at
// pad columns and the fanout-jog inserts short perpendicular jogs, so the
// fan legitimately spans pad pitch rather than the nominal gap. Length-
// tuning teeth (TraceSeg::tuning_tooth, #15 trombones) are likewise
// specified skew-compensation jogs. Both classes are floor-checked like all
// copper but exempt from the ceiling. Every other same-layer section is
// trunk and strictly banded. Deterministic (board order scan, first worst
// wins); integer-nm exact pass/fail.
bool pair_gap_legal_fanout(const std::vector<TraceSeg>& traces_p,
                           const std::vector<TraceSeg>& traces_n, Coord gap_nm,
                           Coord tol_nm, const std::vector<Point>& fanout_pts,
                           Coord& worst_err_out, Point& at_out);

// Total integer-rounded Euclidean length of a trace set.
Coord pair_total_length(const std::vector<TraceSeg>& traces);

// ---- Reporting ----

struct PairReport {
    int pair_id = -1;
    std::string name;
    NetId net_p = -1;
    NetId net_n = -1;
    std::string net_p_name;
    std::string net_n_name;
    // Corridor phase.
    bool corridor_routed = false;
    double occupied_width_mm = 0.0;
    double gap_mm = 0.0;
    double gap_tol_mm = 0.0;
    // Materialization phase.
    bool materialized = false;
    std::string status;  // "MATERIALIZED" | "CORRIDOR_FAILED" |
                         // "MATERIALIZATION_FAILED:<reason>" | "UNATTEMPTED"
    double length_p_mm = 0.0;
    double length_n_mm = 0.0;
    double skew_mm = 0.0;
    double worst_gap_err_mm = 0.0;
    // Issue #13: verifier-measured gap violation location (committed copper,
    // not router metadata). has_gap_location false when no coupled sections
    // were measurable.
    bool has_gap_location = false;
    double gap_x_mm = 0.0;
    double gap_y_mm = 0.0;
    LayerId gap_layer = 0;
    int via_pairs = 0;
    // Issue #13: per-member via counts + symmetry detail ("" when symmetric).
    int vias_p = 0;
    int vias_n = 0;
    std::string via_mismatch;
    JsonValue to_json() const;
};

}  // namespace copperline
