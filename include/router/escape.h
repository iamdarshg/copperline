// Copperline: fine-pitch (BGA/LGA/QFN) escape stage (Prompt 2).
//
// BGA/LGA/dense-array routing is structurally centre-out: deeper pads become
// eligible before shallower pads. This is NOT a centrality bonus added to a
// score; the planner processes terminals in strict eligibility order and a
// shallower terminal is never finally committed while a deeper eligible
// terminal has neither a viable candidate nor an explicit
// temporary-infeasibility record for the current routing state.
//
// Pipeline per fine-pitch footprint:
//   FinePitchDetector  -> which components deserve escape handling
//   CentreDepthAnalyzer -> integer centre depth per pad (perimeter = 0)
//   EscapeBoundary     -> expanded rect + escape portals on its perimeter
//   exit-sector + via-site analysis -> same-ring ordering inputs
//   K-best constrained A* -> genuinely different candidates (portal /
//                           principal direction / first via / layer strategy)
//   commit in eligibility order, recording infeasibility where needed.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "router/board.h"
#include "router/json.h"
#include "router/rules.h"
#include "router/spatial_index.h"

namespace copperline {

// ---- Fine-pitch detection ----

struct FinePitchFootprint {
    std::string component;  // grouping key ("U1"); may be "" for cell clusters
    std::vector<TermId> members;  // sorted terminal ids
    Point centroid{};
    Coord radius_nm = 0;
    Rect bbox{};
    Coord pitch_nm = 0;        // nearest-neighbour pitch (min over members)
    Coord pad_w_nm = 0;        // max pad extent in the group
    Coord pad_h_nm = 0;
    Coord trace_width_nm = 0;  // max required width over member nets
    Coord clearance_nm = 0;    // max pairwise clearance over member nets
    int channel_count = 0;     // tracks fitting between adjacent pads
    int pad_count = 0;
    double pins_per_mm2 = 0.0;
    double local_density = 0.0;  // peak DensityEstimator density
    bool is_fine_pitch = false;
    std::string reason;  // why flagged / not flagged (stable, machine-readable)
};

class FinePitchDetector {
  public:
    struct Options {
        // A group with pitch at or below this is fine-pitch on pitch alone.
        Coord pitch_threshold_nm;
        int min_pins;
        Options() : pitch_threshold_nm(0), min_pins(8) {}
    };
    explicit FinePitchDetector(Options options = Options());

    std::vector<FinePitchFootprint> detect(const Board& board, const RuleResolver& resolver,
                                           const ElectricalContext& ctx) const;

  private:
    Options options_;
};

// ---- Centre depth ----

class CentreDepthAnalyzer {
  public:
    // Integer depth per member terminal: perimeter = 0, increasing inward.
    // Returns map terminal id -> depth. Deterministic.
    std::map<TermId, int> analyze(const Board& board,
                                  const FinePitchFootprint& footprint) const;
};

// ---- Escape boundary + portals ----

struct EscapePortal {
    int id = -1;
    Point pos{};
    int side = 0;  // 0:+x(E) 1:+y(N) 2:-x(W) 3:-y(S)
};

struct EscapeBoundary {
    Rect rect{};
    std::vector<EscapePortal> portals;
};

EscapeBoundary build_escape_boundary(const Board& board, const FinePitchFootprint& footprint,
                                     Coord margin_nm);

// ---- Per-pad escape analysis ----

// 8 exit sectors: 0:E 1:NE 2:N 3:NW 4:W 5:SW 6:S 7:SE
// `foreign` (optional): a spatial index of foreign pads + keepouts and
// `net_max_clear` (optional) the per-net max foreign clearance used to size
// the query box. When supplied, only nearby geometry is tested; the exact
// predicates are unchanged, so the result is identical.
std::vector<int> legal_exit_sectors(const Board& board, const RuleResolver& resolver,
                                    const ElectricalContext& ctx, const FinePitchFootprint& fp,
                                    TermId terminal, Coord route_width_nm,
                                    const SpatialIndex* foreign = nullptr,
                                    const std::map<NetId, Coord>* net_max_clear = nullptr);

// Number of legal nearby via sites (dogbone/via-first opportunities).
int via_site_count(const Board& board, const RuleResolver& resolver, const FinePitchFootprint& fp,
                   TermId terminal, const SpatialIndex* foreign = nullptr);

// Local escape-density map over the footprint bbox (pads per mm^2 per cell).
struct EscapeDensityMap {
    Coord cell_nm = 0;
    int nx = 0;
    int ny = 0;
    std::vector<double> cells;  // row-major ny*nx, pads per mm^2
    double peak = 0.0;
};

EscapeDensityMap build_escape_density_map(const Board& board, const FinePitchFootprint& fp);

// ---- K-best candidates ----

struct EscapeCandidate {
    TermId terminal = -1;
    int portal_id = -1;
    Point portal_pos{};
    int principal_dir = 0;        // 0..7 (E,NE,N,NW,W,SW,S,SE)
    std::string layer_strategy;   // "same-layer" | "dogbone" | "via-first" | "multilayer"
    std::vector<TraceSeg> traces;  // pad -> portal geometry (committable)
    std::vector<Via> vias;
    Coord length_nm = 0;
    int via_count = 0;
    Coord width_nm = 0;
    bool use_neckdown = false;
    std::string via_class;
    std::string signature;  // portal|dir|strategy|via-class|bottleneck
    double cost = 0.0;
};

struct InfeasibilityRecord {
    TermId terminal = -1;
    std::string reason;  // "no_exit_sectors" | "no_via_site" | "no_candidate" |
                         // "width_conflict" | "via_current" | "blocked_channel" | ...
    std::vector<std::string> blockers;
    bool recorded = false;
};

struct PadEscapeResult {
    TermId terminal = -1;
    int centre_depth = 0;
    double density = 0.0;
    int eligibility_index = -1;  // position in centre-out order (0 = first)
    std::vector<int> exit_sectors;
    int via_sites = 0;
    double downstream_difficulty = 0.0;
    std::vector<EscapeCandidate> candidates;  // up to K, distinct signatures
    bool has_viable = false;
    InfeasibilityRecord infeasibility;
};

struct FootprintEscapeResult {
    FinePitchFootprint footprint;
    EscapeBoundary boundary;
    EscapeDensityMap density_map;
    std::vector<PadEscapeResult> pads;  // sorted in eligibility order
    std::vector<TermId> eligibility_order;
    std::vector<TermId> commit_order;  // terminals committed (viable) in order
};

struct EscapeOptions {
    int k_best = 4;
    // Local corridor graphs are small (probe: solvable escapes settle in
    // tens of expansions); the budget only bounds hopeless cases.
    std::int64_t max_expansions = 6000;
    int max_portals_per_pad = 12;
    // A* portal attempts per pad in the fallback phase (same+alt share it).
    int max_fallback_portals = 4;
    // The route command owns one wall-clock budget. Escape preprocessing
    // checks the same deadline between bounded searches so a dense board
    // cannot consume the entire run before the global worker pool starts.
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::time_point::max();
    // Worker threads for escape planning (S1 escape-parallelism): explicit
    // --threads wins via resolve_worker_threads, 0 = auto (hardware
    // concurrency). Footprints plan on this pool and each pad's fallback
    // portal attempts fan out on it; the merge is deterministic
    // (component-ordered footprints, serial-equivalent candidate selection),
    // so --threads never changes the committed geometry.
    int threads = 0;
};

struct EscapeResult {
    std::vector<FootprintEscapeResult> footprints;
    int pads_total = 0;
    int pads_with_candidates = 0;
    int pads_infeasible = 0;
    bool timed_out = false;
    JsonValue to_json() const;
};

class EscapePlanner {
  public:
    explicit EscapePlanner(EscapeOptions options = EscapeOptions()) : options_(options) {}

    EscapeResult plan(const Board& board, const RuleResolver& resolver,
                      const ElectricalContext& ctx) const;

    // Eligibility order for one footprint (centre depth always outranks the
    // same-ring tie-breakers). Public so tests can assert the invariant.
    static std::vector<TermId> eligibility_order(
        const Board& board, const RuleResolver& resolver, const ElectricalContext& ctx,
        const FinePitchFootprint& fp, const std::map<TermId, int>& depth,
        const std::map<TermId, std::vector<int>>& exit_sectors,
        const std::map<TermId, int>& via_sites, const std::map<TermId, double>& density,
        const std::map<TermId, double>& downstream);

    const EscapeOptions& options() const { return options_; }

  private:
    EscapeOptions options_;

    // Plan one footprint against the given scratch board (centre-out,
    // eligibility-ordered, committing viable stubs into work). Pure function
    // of (board, resolver, ctx, fp, work-in, options): the serial path passes
    // the shared accumulated board, the parallel path a per-footprint copy.
    // pad_threads bounds intra-pad fallback fan-out (1 = fully serial).
    // Sets aborted on deadline expiry (pad in flight is discarded, matching
    // the legacy break-without-push semantics).
    FootprintEscapeResult plan_one_footprint(const Board& board, const RuleResolver& resolver,
                                             const ElectricalContext& ctx,
                                             const FinePitchFootprint& fp, Board& work,
                                             std::atomic<bool>& aborted,
                                             int pad_threads,
                                             const SpatialIndex* foreign = nullptr,
                                             const std::map<NetId, Coord>* net_max_clear =
                                                 nullptr) const;
};

int principal_direction(Point from, Point to);

// Candidate signature: portal + principal direction + layer strategy +
// first-via class + bottleneck bucket. Tiny geometric perturbations share a
// signature and are deduplicated.
std::string candidate_signature(int portal_id, int principal_dir,
                                const std::string& layer_strategy,
                                const std::string& via_class, int channel_count);

}  // namespace copperline
