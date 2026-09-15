// Copperline: post-route meander/length-tuning stage (issue #15).
//
// Length/skew tuning runs as a dedicated stage ONLY after global topology
// closure and #12 pair materialization, immediately before final cleanup.
// It never alters global topology to create tuning room: candidates are
// local trombone (square-wave accordion) meanders inserted into a single
// committed straight segment, using only free space proven legal by exact
// integer clearance checks. Every accepted transaction re-runs the full
// BoardVerifier (voltage, current, impedance, pair) and reverts on failure.
//
// Determinism: targets in (pair-id, net-id) order; base segments longest
// first with coordinate tie-breaks; sides (+normal, -normal); teeth counts
// ascending from the minimum that reaches the window. Integer nanometres
// throughout; candidate generation streams one at a time with reused
// scratch buffers and bounded regions/candidates (memory stays under the
// router 2048 MB budget; tuning itself is a single-threaded post-phase
// like materialization, so geometry is thread-count independent).
//
// #17 hook: committed tuning geometry is flagged tuning_exempt so generic
// simplification/cleanup must not collapse intentional meanders. The
// lead-in/out segments outside the accordion stay arbitrary-angle
// straights at the original segment angle.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "router/board.h"
#include "router/json.h"
#include "router/rules.h"

namespace copperline {

// Global meander configuration. Amplitude/pitch/max-added are lengths;
// style is "trombone" (only v1 style; unknown styles are rejected).
struct TuningConfig {
    bool enabled = true;
    Coord amplitude_nm = mm_to_nm(1.0);
    Coord pitch_nm = mm_to_nm(0.6);
    Coord max_added_nm = mm_to_nm(50.0);
    int max_candidates = 32;
    int max_regions = 8;
    int max_teeth = 16;
    std::string style = "trombone";
    // Global default for pair tuning when the pair declaration does not
    // say otherwise: false = tune shorter member only, true = symmetric.
    bool symmetric_pairs = false;
};

// Parse the optional "tuning" object of --config JSON. Unknown style or
// non-positive amplitude/pitch/max_added is an error (returns false).
bool tuning_config_from_json(const JsonValue& cfg, TuningConfig& out, std::string& err);

// Per-target tuning outcome. kind is "single" or "pair".
struct TuningRecord {
    std::string kind;
    NetId net = -1;  // single: tuned net; pair: shorter-tuned member (-1 if both/symmetric)
    std::string net_name;
    int pair_id = -1;
    std::string pair_name;
    // TUNED | ALREADY_WITHIN_WINDOW | SKIPPED_NO_ROOM(unused: infeasible
    // instead) | INFEASIBLE:<reason> | PARTIAL:<detail>. Reasons:
    // overshoot_cannot_shorten | no_legal_region | exceeds_max_added |
    // no_tooth_count_in_window | skew_conflict | verifier_rejected | not_routed.
    std::string status;
    Coord required_added_nm = 0;
    Coord added_nm = 0;
    double target_length_mm = 0.0;
    double length_tol_mm = 0.0;
    double final_length_mm = 0.0;
    double skew_before_mm = 0.0;
    double skew_after_mm = 0.0;
    double max_skew_mm = 0.0;
    int regions_considered = 0;
    int candidates_evaluated = 0;
    int teeth = 0;
    LayerId layer = 0;
    // Committed tuning copper is exempt from #17 generic simplification.
    bool tuning_exempt = false;
    JsonValue to_json() const;
};

struct TuningSummary {
    // TUNING_COMPLETE | TUNING_PARTIAL | SKIPPED_NOT_CLOSED |
    // SKIPPED_NO_TARGETS | SKIPPED_DISABLED.
    std::string stage = "SKIPPED_NO_TARGETS";
    int threads_used = 1;      // post-phase is single-threaded (deterministic)
    int effective_threads = 1;
    int regions_considered = 0;
    int candidates_evaluated = 0;
    Coord added_total_nm = 0;
    std::size_t peak_scratch_bytes = 0;
    std::vector<TuningRecord> records;
    std::vector<NetId> tuned_nets;  // nets whose copper changed (exempt-marked)
    JsonValue to_json() const;
};

// Total committed copper length of one net (integer-rounded Euclidean,
// same measure the verifier uses for pair members).
Coord tuning_net_length(const Board& board, NetId net);

// LengthTuner mutates *board in place, one transactional target at a time.
// topology_closed must be true (all active tasks routed AND pair
// materialization done); when false run() records SKIPPED_NOT_CLOSED and
// touches nothing. Requires #12/#13 geometry present on the board.
class LengthTuner {
  public:
    LengthTuner(Board* board, const RuleResolver* resolver,
                const ElectricalContext* ctx, TuningConfig cfg);
    TuningSummary run(bool topology_closed);

  private:
    Board* board_;
    const RuleResolver* resolver_;
    const ElectricalContext* ctx_;
    TuningConfig cfg_;
    // Reused scratch (streamed candidates, no per-candidate allocation).
    std::vector<TraceSeg> scratch_new_;
    std::vector<TraceSeg> scratch_gap_probe_;
    std::size_t peak_scratch_bytes_ = 0;
    int candidates_evaluated_ = 0;
};

}  // namespace copperline
