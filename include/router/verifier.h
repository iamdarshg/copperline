// Copperline: BoardVerifier.
//
// Independently derives connectivity and legality from actual committed
// geometry (pads + traces + vias). It shares the RuleResolver's rule set but
// never the engine's search state, so a green verify means the copper itself
// is legal, not merely that the router believed it was.
#pragma once

#include <string>
#include <vector>

#include "router/board.h"
#include "router/json.h"
#include "router/rules.h"

namespace copperline {

struct Violation {
    // clearance | width | via_current | via_class | connectivity(blocked pad
    // reported as unconnected instead) | off_board | overlap
    std::string type;
    NetId net_a = -1;
    NetId net_b = -1;
    std::string rule;    // rule source, e.g. "voltage_table"
    std::string detail;  // human sentence; machines use the other fields
    double x_mm = 0;     // representative location
    double y_mm = 0;
    LayerId layer = 0;
};

struct Unconnected {
    NetId net = -1;
    std::string net_name;
    TermId terminal = -1;
    std::string component;
    std::string pin;
};

struct PairVerifyDetail {
// Issue #13: independent pair-aware measurement from committed copper.
// The verifier never trusts router metadata (corridor tasks, materializer
// reports). For each declared DiffPair it derives, from traces/vias/pads
// only: total length per member (integer-rounded Euclidean), skew =
// |len(P)-len(N)|, minimum edge gap over coupled sections (exact integer
// math, arbitrary-angle aware), compatible-layer use, paired-via
// count/style/span symmetry and one-member-only detection.
    int pair_id = -1;
    std::string name;
    NetId net_p = -1;
    NetId net_n = -1;
    std::string net_p_name;
    std::string net_n_name;
    double length_p_mm = 0.0;
    double length_n_mm = 0.0;
    double skew_mm = 0.0;
    double worst_gap_err_mm = 0.0;
    bool has_gap_location = false;
    double gap_x_mm = 0.0;
    double gap_y_mm = 0.0;
    LayerId gap_layer = 0;
    int vias_p = 0;
    int vias_n = 0;
    int via_pairs = 0;
    // "" when symmetric; otherwise "count:.." | "span:.." | "style:..".
    std::string via_mismatch;
    // "OK" | "INVALID:<reason>" | combined "+" of
    // ONE_SIDED | LAYER_MISMATCH | GAP_VIOLATION[:too_close|:too_far] |
    // SKEW_EXCEEDED | VIA_MISMATCH:<detail>.
    std::string status = "OK";
    bool ok = true;
    JsonValue to_json() const;
};

struct VerifyResult {
    bool ok = false;  // connected && no violations
    bool connected = false;
    bool legal = false;
    std::vector<Unconnected> unconnected;
    std::vector<Violation> violations;
    std::vector<PairVerifyDetail> pairs;
    JsonValue to_json() const;
};

class BoardVerifier {
  public:
    VerifyResult verify(const Board& board, const RuleResolver& resolver,
                        const ElectricalContext& ctx) const;
    // S3 scoped fast-reject: true means a DEFINITE violation exists among
    // `net`'s copper near `area` (width, off-board, keepout, foreign
    // clearance via the exact full-verify predicates), so the caller may
    // revert without a full verify. False means "unknown" — the caller must
    // still run the FINAL full verify. Never a false positive: every
    // reported hit would also fail verify(). Connectivity, impedance, via
    // current and pair checks stay full-verify-only.
    bool has_local_violation(const Board& board, const RuleResolver& resolver,
                             const ElectricalContext& ctx, NetId net,
                             const Rect& area) const;
};

}  // namespace copperline
