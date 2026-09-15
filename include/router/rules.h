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
    // Thermal overrides for the ampacity model (issue #7). Values <= 0 mean
    // "inherit": the resolver falls back to sidecar/board/layer metadata.
    double temp_rise_c = -1.0;
    double copper_weight_oz = -1.0;
};

struct TraceRule {
    Coord min_width_nm = 0;
    Coord pref_width_nm = 0;
    bool allow_neckdown = false;
    Coord neck_width_nm = 0;
    Coord neck_max_len_nm = 0;
    std::string width_source;  // "explicit" | "width_class" | "ampacity" |
                               // "ipc_estimate" (legacy) | "board_default"
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

// 3a. Legacy linear width heuristic (kept for backward compatibility):
// width = I * k, clamped. Superseded by AmpacityModel as the default
// inferred estimator (issue #7); still selected when a config explicitly
// carries an "ipc" block without an "ampacity" block.
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

// 3b. Physically parameterized ampacity model (issue #7, the DEFAULT
// inferred estimator): IPC-2221 section 6.2 external/internal trace
// current-carrying approximation, inverted to solve for width:
//
//   I = k * dT^0.44 * A^0.725,  A = w_mils * t_mils  (A in square mils)
//
// with k = 0.048 for external (outer) layers and k = 0.024 for internal
// layers, dT the allowed temperature rise in C, and copper thickness
// t_mils = weight_oz * 1.37 (1 oz Cu ~= 1.37 mil ~= 34.8 um).
// Inverted: A = (I / (k * dT^0.44))^(1/0.725), w = A / t, clamped to
// [min_mm, max_mm]. This is an IPC-2221-style estimate, NOT an IPC-2152
// qualified rating: no electrothermal simulation, no plane solving, no
// regulatory claim. Like the legacy model it only scales *stated* net
// current and returns -1 when the net carries no current metadata.
class AmpacityModel : public CurrentCapacityModel {
  public:
    AmpacityModel(double min_mm = 0.15, double max_mm = 10.0);
    Coord evaluate(const NetInfo& net, const BoardDefaults&,
                   const ElectricalContext& ctx) const override;
    // Layer-aware entry: resolves thickness/rise with the helpers below.
    Coord evaluate_for(double current_a, double copper_oz, double temp_rise_c,
                       bool internal) const;
    const char* name() const override { return "ampacity"; }
    bool enabled = true;
    // Unit helpers (public for tests/diagnostics).
    static double copper_mils(double weight_oz) { return weight_oz * 1.37; }
    static double width_mm_for(double current_a, double copper_oz, double temp_rise_c,
                               bool internal);

  private:
    Coord min_nm_;
    Coord max_nm_;
};

// Transparent per-net width accounting for analyze/route JSON (issue #7).
struct WidthDetails {
    std::string model;  // "explicit" | "width_class" | "ampacity" |
                        // "ipc_estimate" | "board_default"
    double current_a = 0.0;
    bool default_current_used = false;
    double copper_weight_oz = 1.0;
    double temp_rise_c = 20.0;
    bool internal_layer = false;
    Coord width_nm = 0;
};

// Composite: explicit -> width class -> ampacity (default) or legacy IPC
// estimate -> board default.
// Records which tier answered so reports can show inferred-vs-explicit.
class CurrentCapacitySystem {
  public:
    CurrentCapacitySystem();
    void set_classes(std::map<std::string, WidthClass> classes);
    void set_ipc(const IpcEstimateModel& ipc) {
        ipc_ = ipc;
        has_ipc_config_ = true;
    }
    void set_ampacity(const AmpacityModel& m) {
        ampacity_ = m;
        has_ampacity_config_ = true;
    }
    // Sidecar thermal overrides (<= 0 = inherit board defaults).
    void set_thermal(double temp_rise_c, double copper_weight_oz) {
        thermal_temp_c_ = temp_rise_c;
        thermal_copper_oz_ = copper_weight_oz;
    }

    // Effective design current: peak > continuous > board default.
    double effective_current(const NetInfo& net, const BoardDefaults& def,
                             bool& default_used) const;

    Coord required_min_width(const NetInfo& net, const BoardDefaults& def,
                              const ElectricalContext& ctx, std::string& source) const;
    // Layer-aware entry: resolves copper weight / temperature rise /
    // internal-vs-external from (layer override > ctx > sidecar > board).
    Coord required_min_width(const NetInfo& net, const Board& board, LayerId layer,
                              const ElectricalContext& ctx, std::string& source) const;
    // Full accounting for machine-readable reports.
    WidthDetails width_details(const NetInfo& net, const Board& board, LayerId layer,
                               const ElectricalContext& ctx) const;
    // Merged thermal context: explicit ctx values win, then sidecar, then
    // board defaults. Used by the CLI so --config thermal metadata reaches
    // analysis/routing instead of being silently ignored.
    ElectricalContext default_context(const Board& board) const;

    // Thermal resolution helpers (public for tests).
    double effective_temp_rise(const Board& board, const ElectricalContext& ctx) const;
    double effective_copper_oz(const Board& board, LayerId layer,
                               const ElectricalContext& ctx) const;
    bool effective_internal(const Board& board, LayerId layer) const;
    bool uses_ampacity() const { return ampacity_enabled(); }

    bool ampacity_enabled() const;

  public:
    // Neckdown is legal only with explicit permission and a wide-enough neck,
    // unless the neck still meets the full required width (then it is not a
    // neckdown at all and always legal).
    bool neckdown_legal(const NetInfo& net, Coord neck_width_nm, Coord length_nm,
                        const BoardDefaults& def, const ElectricalContext& ctx) const;
    // Layer-aware entry (issue #24): the required width comes from
    // required_min_width(net, board, layer, ctx), i.e. per-layer copper plus
    // effective_internal(board, layer). Escape stubs must use this overload
    // with the terminal/escape layer so an internal/thin-copper escape is not
    // accepted against the less-conservative external/default width.
    // Explicit-permission and max-length semantics are identical.
    bool neckdown_legal(const NetInfo& net, Coord neck_width_nm, Coord length_nm,
                        const Board& board, LayerId layer,
                        const ElectricalContext& ctx) const;

    bool via_style_ok(const ViaStyle& style, const NetInfo& net, const BoardDefaults& def,
                      const ElectricalContext&) const;
    // Number of parallel vias needed under this style (>= 1).
    int vias_required(const ViaStyle& style, const NetInfo& net, const BoardDefaults& def,
                      const ElectricalContext&) const;

  private:
    ExplicitWidthModel explicit_;
    WidthClassModel classes_;
    IpcEstimateModel ipc_;
    AmpacityModel ampacity_;
    bool has_ipc_config_ = false;
    bool has_ampacity_config_ = false;
    double thermal_temp_c_ = -1.0;    // sidecar temp_rise_c override (<=0 = inherit)
    double thermal_copper_oz_ = -1.0;  // sidecar copper_weight_oz override (<=0 = inherit)
};

// ---- Impedance models (issue #11) ----

// Replaceable single-ended characteristic-impedance estimator. Both built-in
// models are documented closed-form approximations (IPC-2141 /
// Hammerstad-Jensen family), NOT field solvers: they estimate from trace
// width w, dielectric height h, relative permittivity er and foil thickness
// t (all in mm) and are monotonic decreasing in w. Custom models plug in by
// implementing estimate_ohms; the solver, reconciliation and reporting treat
// every model identically.
class ImpedanceModel {
  public:
    virtual ~ImpedanceModel() = default;
    virtual double estimate_ohms(double w_mm, double h_mm, double er,
                                 double t_mm) const = 0;
    virtual const char* name() const = 0;
};

// Outer-layer microstrip:
//   Z0 = 87 / sqrt(er + 1.41) * ln(5.98 * h / (0.8 * w + t))
class MicrostripModel : public ImpedanceModel {
  public:
    double estimate_ohms(double w_mm, double h_mm, double er,
                         double t_mm) const override;
    const char* name() const override { return "microstrip"; }
};

// Symmetric inner-layer stripline with plane spacing b = 2 * h:
//   Z0 = 60 / sqrt(er) * ln(1.9 * b / (0.8 * w + t))
class StriplineModel : public ImpedanceModel {
  public:
    double estimate_ohms(double w_mm, double h_mm, double er,
                         double t_mm) const override;
    const char* name() const override { return "stripline"; }
};

// Per-layer impedance solution, all widths integer nm.
struct ImpedanceOption {
    LayerId layer = -1;
    std::string model;       // "microstrip" | "stripline"
    Coord width_nm = 0;      // reconciled width on this layer (>= current min)
    Coord raw_width_nm = 0;  // unconstrained solver width (target centre)
    double estimated_ohms = 0.0;  // estimate at width_nm
    double target_ohms = 0.0;
    double rel_error = 0.0;  // |est - target| / target at width_nm
    bool in_tolerance = false;
    bool current_conflict = false;  // current minimum exceeds tolerance band
};

// Net-level impedance resolution: per-layer options plus the selected
// layer/width. Conflict means the ampacity (#7) minimum sits above the
// tolerance band on every feasible layer; the caller must report it as an
// explicit constraint conflict and never silently narrow the trace.
struct ImpedanceResolution {
    bool has_target = false;
    NetId net = -1;
    std::string net_name;
    double target_ohms = 0.0;
    double tolerance_frac = 0.10;
    bool feasible = false;  // >= 1 eligible layer solves within tolerance
    bool conflict = false;  // ampacity minimum breaks tolerance everywhere
    std::string conflict_detail;
    LayerId selected_layer = -1;
    Coord selected_width_nm = 0;
    std::string selected_model;
    double estimated_ohms = 0.0;
    double rel_error = 0.0;
    Coord current_min_nm = 0;  // ampacity minimum on the selected layer
    std::string current_source;
    std::vector<ImpedanceOption> options;  // eligible layers, by layer id
    JsonValue to_json() const;
};

// Coupled stackup solver + ampacity reconciliation. Layer eligibility:
// signal layers with a positive dielectric height and permittivity,
// restricted to net.impedance_layers when the net names them. Reference
// planes and the per-layer model override only bias selection (soft cost),
// never hard legality.
class ImpedanceSystem {
  public:
    ImpedanceSystem();

    void set_enabled(bool e) { enabled_ = e; }
    void set_model_override(std::string m) { model_override_ = std::move(m); }
    void set_bounds(Coord min_nm, Coord max_nm) {
        min_nm_ = min_nm;
        max_nm_ = max_nm;
    }
    void set_default_tolerance(double frac) { default_tolerance_frac_ = frac; }
    bool enabled() const { return enabled_; }

    bool has_target(const NetInfo& net) const {
        return enabled_ && net.has_impedance;
    }
    double tolerance_for(const NetInfo& net) const;
    // Model chosen for a layer: explicit override wins, else outer stackup
    // layers solve as microstrip and inner layers as stripline.
    std::string model_name_for(const Board& board, const Layer& layer) const;
    bool layer_eligible(const Board& board, const NetInfo& net,
                        const Layer& layer) const;
    std::vector<LayerId> eligible_layers(const Board& board,
                                         const NetInfo& net) const;
    // Foil thickness in mm: explicit override wins, else weight-derived.
    double copper_mm_for(const Board& board, const Layer& layer,
                         double copper_weight_oz) const;
    // Estimate at an integer-nm width, or -1 when the layer has no stackup.
    double estimate_ohms(const Board& board, const Layer& layer, Coord width_nm,
                         double copper_weight_oz, std::string* model_out = nullptr) const;
    // Unconstrained solver width for a target (target centre), or -1 when
    // the target is unreachable within [min_nm_, max_nm_].
    Coord solve_width(const Board& board, const Layer& layer, double target_ohms,
                      double copper_weight_oz, std::string* model_out = nullptr) const;
    // Tolerance-band edges (width at Z_high / Z_low), -1 when unreachable.
    void tolerance_band(const Board& board, const Layer& layer, double target_ohms,
                        double tol_frac, double copper_weight_oz, Coord& w_lo_nm,
                        Coord& w_hi_nm) const;

    // A* soft bias for one layer: 1.0 when no target; 0.7 on the selected
    // layer, 0.85 on other eligible layers (0.8 when the reference plane
    // also matches the net preference), 2.0 on ineligible layers.
    double layer_multiplier(const Board& board, const NetInfo& net, LayerId layer,
                            LayerId selected_layer) const;

  private:
    const ImpedanceModel& model_for_name(const std::string& name) const;
    MicrostripModel microstrip_;
    StriplineModel stripline_;
    bool enabled_ = true;
    std::string model_override_;  // "" = auto
    Coord min_nm_ = 0;
    Coord max_nm_ = 0;
    double default_tolerance_frac_ = 0.10;
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
                 ImpedanceSystem impedance, VoltageClearanceModel voltage,
                 std::vector<ViaStyle> via_styles);

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
    // Transparent width accounting (model/current/copper/rise/layer).
    WidthDetails widthDetails(NetId net, LayerId layer,
                              const ElectricalContext& ctx) const;
    // ---- Issue #11: impedance-aware routing ----
    // Full per-layer resolution for a net (has_target false when the net
    // names no impedance or the system is disabled). current_min_nm is the
    // ampacity (#7) floor on the selected layer.
    ImpedanceResolution impedanceResolution(NetId net,
                                            const ElectricalContext& ctx) const;
    // Estimate at an explicit width on one layer (-1 when ineligible).
    double impedanceEstimate(NetId net, LayerId layer, Coord width_nm,
                             const ElectricalContext& ctx,
                             std::string* model_out = nullptr) const;
    // Conservative routable width for graph/corridor sizing: max reconciled
    // width across layers when the net has a target, else the source-layer
    // width (identical to requiredTraceWidth, preserving legacy behavior).
    Coord maxRequiredWidth(NetId net, const ElectricalContext& ctx) const;
    // Soft A* bias for (net, layer): 1.0 when impedance is inactive.
    double impedanceLayerMultiplier(NetId net, LayerId layer) const;
    const ImpedanceSystem& impedance() const { return impedance_; }
    // Merged thermal context for this board (ctx override > sidecar > board).
    ElectricalContext defaultContext() const;
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
    const std::vector<ViaStyle>& via_styles() const { return via_styles_; }
    bool default_current_used(NetId net) const;

  private:
    const Board* board_;
    CurrentCapacitySystem current_;
    ImpedanceSystem impedance_;
    VoltageClearanceModel voltage_;
    std::vector<ViaStyle> via_styles_;
};

}  // namespace copperline
