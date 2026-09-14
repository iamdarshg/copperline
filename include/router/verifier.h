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

struct VerifyResult {
    bool ok = false;  // connected && no violations
    bool connected = false;
    bool legal = false;
    std::vector<Unconnected> unconnected;
    std::vector<Violation> violations;
    JsonValue to_json() const;
};

class BoardVerifier {
  public:
    VerifyResult verify(const Board& board, const RuleResolver& resolver,
                        const ElectricalContext& ctx) const;
};

}  // namespace copperline
