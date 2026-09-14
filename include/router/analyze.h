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

}  // namespace copperline
