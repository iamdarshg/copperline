// Copperline: board analysis for agents (`router analyze --json`).
#pragma once

#include <string>
#include <vector>

#include "router/board.h"
#include "router/json.h"
#include "router/rules.h"

namespace copperline {

struct AnalysisResult {
    JsonValue to_json() const;
    JsonValue data;
};

AnalysisResult analyze_board(const Board& board, const RuleResolver& resolver,
                             const ElectricalContext& ctx,
                             const std::vector<std::string>& import_warnings);

// Phase 5 item 28: pre-route setup report (`router setup-report`). Reports
// what the router will use before it routes: layer directions, drill pairs,
// per-net width/via resolution and setup warnings (layers with no direction
// preference, single-pin nets). Schema `copperline/setup-report/1`.
JsonValue setup_report(const Board& board, const RuleResolver& resolver,
                       const ElectricalContext& ctx,
                       const std::vector<std::string>& import_warnings);

}  // namespace copperline
