// Copperline: parallel-via bundle planning (issue #5).
//
// A ViaStyle carries a per-via current limit, but route generation used to
// emit exactly one via per layer transition and rejected any style whose
// single via could not carry the net current. This module makes a style
// feasible when N = ceil(required_current / per_via_capacity) vias can be
// placed as one legal parallel bundle around the transition point.
//
// The planner is deterministic: candidate styles are tried in resolver
// preference order (net via_class first, then fewest vias, smallest barrel,
// name) and bundle layouts in fixed order (horizontal row, vertical column,
// compact grid). The first fully legal bundle wins, so equal
// (board, rules, seed) inputs always produce identical copper.
//
// A bundle is legal only when every via disc AND every star stub trace
// (center -> each satellite on both transition layers) clears foreign pads,
// traces, vias and keepouts under the voltage-aware pair clearance, stays
// inside the board outline, and keeps same-net via barrels disjoint.
// Callers commit/revalidate the whole bundle atomically: a transition is
// either fully realized or reported infeasible, never half-built.
#pragma once

#include <string>
#include <vector>

#include "router/board.h"
#include "router/rules.h"

namespace copperline {

struct ClearanceCache;  // router/simplify.h (pass by const-ref; defined there)

// Compact manufacturing pitch between same-net bundle barrels:
// barrel diameter plus one default-clearance step. Deterministic and shared
// by the planner, the sparse-graph gate and the verifier's neighbour radius.
Coord via_bundle_pitch(const ViaStyle& style);

// Conservative neighbour radius around a bundle member inside which the rest
// of its bundle is guaranteed to sit (covers the longest layout: the row).
// The verifier uses it to decide whether a high-current via is backed by a
// full parallel bundle.
Coord via_bundle_radius(const ViaStyle& style, int count);

struct ViaBundle {
    bool feasible = false;
    // "ok" | "no_via_class" | "bundle_blocked"
    std::string reason = "no_via_class";
    ViaStyle style;
    int count = 0;  // N vias required in parallel (>= 1)
    double required_current_a = 0.0;
    std::vector<Point> positions;    // size == count when feasible
    std::vector<TraceSeg> stubs;     // star stubs center->satellite, both layers
    std::string via_class;           // == style.name when feasible
};

class ViaBundlePlanner {
  public:
    // Full selection: pick the best style for this net/transition and lay out
    // its bundle around center. Tries styles in deterministic preference
    // order and returns the first fully legal bundle. route_width_nm sizes
    // the star stubs (caller's electrical trace width).
    static ViaBundle plan(const Board& board, const RuleResolver& resolver, NetId net,
                          Point center, LayerSpan span, Coord route_width_nm,
                          const ElectricalContext& ctx, NetId exempt_net = -1);

    // Fixed-style layout: lay out exactly count vias of style around center.
    // Used by the graph gate/materialization once the style is chosen, and by
    // tests that need a reproducible geometry for one class.
    static ViaBundle plan_with_style(const Board& board, const RuleResolver& resolver,
                                     NetId net, Point center, LayerSpan span,
                                     const ViaStyle& style, int count,
                                     Coord route_width_nm, const ElectricalContext& ctx,
                                     NetId exempt_net = -1);
    // Same layout, but reusing the caller's per-net clearance memo instead of
    // refilling it per call. Exact: the memo is pure in (board, net pair),
    // so shared values are identical to freshly resolved ones. Used by the
    // sparse-graph via gate, which plans hundreds of bundles per build.
    static ViaBundle plan_with_style(const Board& board, const RuleResolver& resolver,
                                     NetId net, Point center, LayerSpan span,
                                     const ViaStyle& style, int count,
                                     Coord route_width_nm, const ElectricalContext& ctx,
                                     const ClearanceCache& cc, NetId exempt_net = -1);

    // Ordered candidate styles for a net (preference first, then fewest vias,
    // smallest barrel, name). Exposed so the sparse graph, the candidate
    // builder and analyze share one ordering.
    static std::vector<ViaStyle> ordered_styles(const RuleResolver& resolver, NetId net,
                                                LayerSpan span);

    static int required_count(const RuleResolver& resolver, const ViaStyle& style,
                              NetId net);
};

}  // namespace copperline
