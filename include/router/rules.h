// Copperline: electrical rule engine.
//
// Three separated responsibilities:
//   CurrentCapacityModel  - abstract width/current estimators (explicit,
//                           width-class, IPC-derived). Never buried in A*.
//   VoltageClearanceModel - pair-aware clearance (net-pair rule > class pair
//                           > voltage-difference table > board default).
//   RuleResolver          - answers traceRule / requiredTraceWidth /
//                           requiredClearance / allowedVias for the router,
//                           verifier and analyzer from one rule set.
//
// Explicit design rules always override inferred estimates. When inference
// conflicts with available geometry the caller (engine/verifier) reports
// the conflict; this module never silently narrows a trace.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "router/board.h"
#include "router/json.h"

namespace copperline {

struct ElectricalContext {
    double ambient_c = 25.0;
    double temp_rise_c = 20.0;
    double copper_weight_oz = 1.0;
};

struct TraceRule {
    Coord min_width_nm = 0;
    Coord pref_width_nm = 0;
    bool allow_neckdown = false;
    Coord neck_width_nm = 0;
    Coord neck_max_len_nm = 0;
    std::string width_source;  // "explicit" | "width_class" | "ipc_estimate" | "board_default"
};

struct ViaStyle {
    std::string name = "STD";
    Coord outer_nm = 0;
    Coord hole_nm = 0;
    double max_current_a = 2.0;
};

struct LayerSpan {
    LayerId top = 0;
    LayerId bottom = 0;
};

struct WidthClass {
    std::string name;
    Coord min_width_nm = 0;
    double max_current_a = 1e9;  // class applies up to this current
};

// ---- Current capacity models ----

// Abstract estimator: returns the minimum width this model prescribes, or -1
// when the model does not apply to the net (so composites can fall through).
class CurrentCapacityModel {
  public:
    virtual ~CurrentCapacityModel() = default;
    virtual Coord evaluate(const NetInfo& net, const BoardDefaults& def,
                           const ElectricalContext& ctx) const = 0;
    virtual const char* name() const = 0;
};

// 1. Direct explicit width rules (net min_width wins over everything).
class ExplicitWidthModel : public CurrentCapacityModel {
  public:
    Coord evaluate(const NetInfo& net, const BoardDefaults&, const ElectricalContext&) const override {
        return net.has_min_width ? net.min_width_nm : -1;
    }
    const char* name() const override { return "explicit"; }
};

// 2. User-defined width/current classes.
class WidthClassModel : public CurrentCapacityModel {
  public:
    explicit WidthClassModel(std::map<std::string, WidthClass> classes)
        : classes_(std::move(classes)) {}
    Coord evaluate(const NetInfo& net, const BoardDefaults&,
                   const ElectricalContext& ctx) const override;
    const char* name() const override { return "width_class"; }

  private:
    std::map<std::string, WidthClass> classes_;
};

// 3. Configurable IPC-derived width estimation: width = I * k, clamped.
class IpcEstimateModel : public CurrentCapacityModel {
  public:
    IpcEstimateModel(double mm_per_amp = 0.75, double min_mm = 0.15, double max_mm = 10.0);
    Coord evaluate(const NetInfo& net, const BoardDefaults&,
                   const ElectricalContext& ctx) const override;
    const char* name() const override { return "ipc_estimate"; }
    bool enabled = true;

  private:
    double mm_per_amp_;
    Coord min_nm_;
    Coord max_nm_;
};

// Composite: explicit -> width class -> IPC estimate -> board default.
// Records which tier answered so reports can show inferred-vs-explicit.
class CurrentCapacitySystem {
  public:
    CurrentCapacitySystem();
    void set_classes(std::map<std::string, WidthClass> classes);
    void set_ipc(const IpcEstimateModel& ipc) { ipc_ = ipc; }

    // Effective design current: peak > continuous > board default.
    double effective_current(const NetInfo& net, const BoardDefaults& def,
                             bool& default_used) const;

    Coord required_min_width(const NetInfo& net, const BoardDefaults& def,
                             const ElectricalContext& ctx, std::string& source) const;

    // Neckdown is legal only with explicit permission and a wide-enough neck,
    // unless the neck still meets the full required width (then it is not a
    // neckdown at all and always legal).
    bool neckdown_legal(const NetInfo& net, Coord neck_width_nm, Coord length_nm,
                        const BoardDefaults& def, const ElectricalContext& ctx) const;

    bool via_style_ok(const ViaStyle& style, const NetInfo& net, const BoardDefaults& def,
                      const ElectricalContext&) const;
    // Number of parallel vias needed under this style (>= 1).
    int vias_required(const ViaStyle& style, const NetInfo& net, const BoardDefaults& def,
                      const ElectricalContext&) const;

  private:
    ExplicitWidthModel explicit_;
    WidthClassModel classes_;
    IpcEstimateModel ipc_;
};

// ---- Voltage clearance model ----

struct VoltageTableEntry {
    double delta_v_min = 0.0;
    Coord clearance_nm = 0;
};

// Two-stage clearance resolution (issue #6):
//   Stage 1 (source selection): pair_rule > class_pair > voltage_table >
//     board_default selects a candidate clearance.
//   Stage 2 (hard-floor enforcement): final = max(candidate,
//     netA.min_clearance, netB.min_clearance).
// The resolution carries both the selected candidate source and whether a
// per-net floor raised the result, so diagnostics stay explainable.
struct ClearanceResolution {
    Coord value_nm = 0;              // final enforced clearance
    Coord candidate_nm = 0;          // stage-1 candidate before floors
    std::string candidate_source;    // "pair_rule" | "class_pair" | "voltage_table" | "board_default"
    Coord floor_nm = 0;              // max(netA floor, netB floor), 0 when neither net has one
    bool floor_applied = false;      // true when floor_nm > candidate_nm
    std::string source;              // final source: "net_floor" when floored, else candidate_source
};

class VoltageClearanceModel {
  public:
    VoltageClearanceModel();

    void set_default(Coord nm) { default_nm_ = nm; }
    void set_table(std::vector<VoltageTableEntry> table);
    void add_class_pair(const std::string& a, const std::string& b, Coord clearance_nm);
    void add_net_pair(const std::string& a_net, const std::string& b_net, Coord clearance_nm);

    // Pair-aware clearance from BOTH nets' voltages/classes/names.
    // Two stages: candidate selection, then per-net floor enforcement.
    ClearanceResolution resolve(double v_a, bool has_a, const std::string& class_a,
                                const std::string& name_a, double v_b, bool has_b,
                                const std::string& class_b, const std::string& name_b,
                                const NetInfo* info_a, const NetInfo* info_b) const;
    Coord required(double v_a, bool has_a, const std::string& class_a, const std::string& name_a,
                   double v_b, bool has_b, const std::string& class_b, const std::string& name_b,
                   const NetInfo* info_a, const NetInfo* info_b,
                   std::string& source) const;

  private:
    Coord default_nm_ = mm_to_nm(0.15);
    std::vector<VoltageTableEntry> table_;
    std::map<std::pair<std::string, std::string>, Coord> class_pairs_;
    std::vector<std::pair<std::pair<std::string, std::string>, Coord>> net_pairs_;
};

// ---- Resolver ----

class RuleResolver {
  public:
    RuleResolver(const Board* board, CurrentCapacitySystem current,
                 VoltageClearanceModel voltage, std::vector<ViaStyle> via_styles);

    static RuleResolver defaults_for(const Board& board);
    // Merges a sidecar config (JSON object). Throws BoardError(kRule).
    static RuleResolver from_config(const Board& board, const JsonValue& config);

    // Re-point the resolver at an equivalent board that outlives this call
    // (e.g. after the board was moved into an owning engine). The rule set is
    // unchanged; only the binding moves.
    void rebind(const Board* board) { board_ = board; }

    TraceRule traceRule(NetId net, LayerId layer, RegionId region) const;
    Coord requiredTraceWidth(NetId net, LayerId layer, const ElectricalContext& ctx,
                             std::string* source_out = nullptr) const;
    Coord requiredClearance(NetId a, NetId b, LayerId layer, const ElectricalContext& ctx,
                            std::string* source_out = nullptr) const;
    // Full two-stage resolution: candidate source + floor info alongside the
    // final value. All router/verifier callers share this path.
    ClearanceResolution clearanceResolution(NetId a, NetId b, LayerId layer,
                                            const ElectricalContext& ctx) const;
    std::vector<ViaStyle> allowedVias(NetId net, LayerSpan span) const;
    bool select_via(NetId net, LayerSpan span, ViaStyle& out) const;
    // Named style lookup across ALL configured styles (legal or not), so the
    // verifier can distinguish "unknown class" from "class over current".
    bool lookup_via_style(const std::string& name, ViaStyle& out) const;

    const Board* board() const { return board_; }
    const CurrentCapacitySystem& current() const { return current_; }
    const VoltageClearanceModel& voltage() const { return voltage_; }
    bool default_current_used(NetId net) const;

  private:
    const Board* board_;
    CurrentCapacitySystem current_;
    VoltageClearanceModel voltage_;
    std::vector<ViaStyle> via_styles_;
};

}  // namespace copperline
