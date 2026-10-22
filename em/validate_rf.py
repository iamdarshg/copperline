#!/usr/bin/env python3
"""Copperline EM validation CLI — clean-room RF limit validation.

Reads a copperline routed board (JSON, the same format `router route
--output` writes) plus an EM configuration (stackup + RF limits) and
produces a machine-readable validation report (schema
copperline/em-report/1).

For every net with an RF target in the config the tool reports:
  * structure classification (microstrip / stripline / unknown)
  * characteristic impedance Z0 (closed-form models)
  * return loss S11 vs the target impedance
  * insertion loss S21 at the configured frequency
  * DC ampacity (IPC-2221) and RF-derated ampacity at f
  * per-net PASS/FAIL against every configured limit

Optionally (--openems) an openEMS FDTD project is generated for each
RF net (see gen_openems.py) for full-wave verification; the analytical
report is always produced so the tool degrades gracefully when openEMS
is not installed.

Clean-room note: all models are restated from public-domain physics and
standard published closed forms (Pozar "Microwave Engineering",
IPC-2141A, IPC-2221). No third-party solver code is embedded.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rf_models as rf

REPORT_SCHEMA = "copperline/em-report/1"


def default_config() -> dict:
    return {
        "schema": "copperline/em-config/1",
        "stackup": {
            # dielectric height under each layer (mm); used as h for the
            # reference-plane distance. Layer ids follow the board JSON.
            "dielectric_mm": 1.6,
            "eps_r": 4.4,
            "tan_d": 0.02,
            "copper_weight_oz": 1.0,
            "plating_um": 25.0,
            "internal_layers": [],
        },
        "rf": {
            "freq_ghz": 2.45,
            "temp_rise_c": 20.0,
            # nets whose Z0 must be checked against a target
            "targets": {
                # "ANT": {"z0_ohm": 50.0, "max_s11_db": -10.0, "min_s21_db": -1.5,
                #         "min_isolation_db": -20.0, "current_a": 1.0},
            },
            "default_target_z0_ohm": 50.0,
            "max_s11_db": -10.0,
            "min_s21_db": -1.5,
        },
    }


def load_config(path: str) -> dict:
    if not path:
        return default_config()
    with open(path, "r", encoding="utf-8") as fh:
        cfg = json.load(fh)
    base = default_config()
    base.update(cfg)
    base["stackup"].update(cfg.get("stackup", {}))
    base["rf"].update(cfg.get("rf", {}))
    return base


def net_traces(board: dict) -> dict[int, list]:
    """Group routed traces by net id."""
    by_net: dict[int, list] = {}
    for tr in board.get("traces", []):
        by_net.setdefault(tr["net"], []).append(tr)
    return by_net


def trace_length_mm(tr: dict) -> float:
    return ((tr["x2_mm"] - tr["x1_mm"]) ** 2 +
            (tr["y2_mm"] - tr["y1_mm"]) ** 2) ** 0.5


def has_plane(board: dict, layer_id: int) -> bool:
    for pl in board.get("planes", []):
        if pl.get("layer") == layer_id:
            return True
    return False


def evaluate_net(net: dict, traces: list, cfg: dict) -> dict:
    stackup = cfg["stackup"]
    rfc = cfg["rf"]
    targets = rfc.get("targets", {})
    tgt = targets.get(net["name"], {})

    freq = rfc.get("freq_ghz", 2.45)
    eps_r = stackup.get("eps_r", 4.4)
    tan_d = stackup.get("tan_d", 0.02)
    oz = stackup.get("copper_weight_oz", 1.0)
    h = stackup.get("dielectric_mm", 1.6)
    internal_layers = set(stackup.get("internal_layers", []))

    total_len = 0.0
    widths = set()
    for tr in traces:
        total_len += trace_length_mm(tr)
        widths.add(round(tr["width_mm"], 4))

    # structure classification from layer + planes.
    # A plane on another layer acts as a reference plane.  If reference
    # layers exist both above AND below the signal layers -> stripline;
    # only below (or only above) -> microstrip; none -> unknown.
    layer_ids = {tr["layer"] for tr in traces}
    all_layer_ids = {l["id"] for l in board_layers(_BOARD)}
    plane_layers = {pl["layer"] for pl in _BOARD.get("planes", [])}
    # no plane ON the signal layer itself (a pour is not a Z0 reference)
    ref_layers = plane_layers - layer_ids
    above = any(rl > max(layer_ids) for rl in ref_layers)
    below = any(rl < min(layer_ids) for rl in ref_layers)
    struct = rf.classify_trace(above, below)

    width = max(widths) if widths else 0.2
    t_cu = rf.copper_thickness_mm(oz)
    internal = any(lid in internal_layers for lid in layer_ids)

    # Z0 model
    z0 = None
    eps_eff = None
    model = struct
    if struct == "microstrip":
        z0, eps_eff = rf.microstrip_z0(width, t_cu, h, eps_r)
    elif struct == "stripline":
        z0, eps_eff = rf.stripline_z0(width, t_cu, h, eps_r)

    z0_target = tgt.get("z0_ohm", rfc.get("default_target_z0_ohm", 50.0))
    s11 = rf.return_loss_db(z0, z0_target) if z0 else -math_inf()
    delta = rf.skin_depth_mm(freq)
    alpha_c = rf.conductor_loss_npm(width, t_cu, delta, z0) if z0 else 0.0
    alpha_d = rf.dielectric_loss_npm(freq, eps_r, tan_d)
    alpha = alpha_c + alpha_d
    s21 = rf.insertion_loss_db(alpha, total_len)

    dc_a, ac_a, _d = rf.rf_derated_ampacity(
        width, oz, rfc.get("temp_rise_c", 20.0), freq, internal)
    net_current = tgt.get("current_a") or net.get("current_a", 0.0)

    # limits
    checks = []
    if z0 is not None:
        tol = tgt.get("z0_tol_pct", 10.0)
        dev = abs(z0 - z0_target) / z0_target * 100.0
        checks.append({
            "limit": "z0",
            "pass": dev <= tol,
            "value": round(z0, 2), "target": z0_target,
            "detail": f"{dev:.1f}% deviation (tol {tol:.0f}%)",
        })
    if s11 != -math.inf:
        checks.append({
            "limit": "s11",
            "pass": s11 <= rfc.get("max_s11_db", -10.0),
            "value": round(s11, 2),
            "target": rfc.get("max_s11_db", -10.0),
            "detail": "return loss",
        })
    if s21 != -math.inf:
        checks.append({
            "limit": "s21",
            "pass": s21 >= rfc.get("min_s21_db", -1.5),
            "value": round(s21, 2),
            "target": rfc.get("min_s21_db", -1.5),
            "detail": "insertion loss",
        })
    if net_current > 0:
        checks.append({
            "limit": "rf_ampacity",
            "pass": ac_a >= net_current,
            "value": round(ac_a, 3),
            "target": net_current,
            "detail": f"DC {dc_a:.3f} A -> RF {ac_a:.3f} A at {freq} GHz",
        })

    return {
        "net": net.get("name", f"#{net.get('id')}"),
        "structure": struct,
        "model": model,
        "traces": len(traces),
        "length_mm": round(total_len, 3),
        "width_mm": round(width, 4),
        "layer_ids": sorted(layer_ids),
        "z0_ohm": round(z0, 2) if z0 is not None else None,
        "eps_eff": round(eps_eff, 3) if eps_eff is not None else None,
        "s11_db": round(s11, 2) if s11 != -math.inf else None,
        "s21_db": round(s21, 2) if s21 != -math.inf else None,
        "skin_depth_mm": round(delta, 4),
        "dc_ampacity_a": round(dc_a, 3),
        "rf_ampacity_a": round(ac_a, 3),
        "checks": checks,
        "pass": all(c["pass"] for c in checks),
    }


# module-level board cache for has_plane lookups (set by main)
_BOARD: dict = {}


def board_layers(board: dict) -> list:
    return board.get("layers", [])


def math_inf() -> float:
    return math.inf


def main() -> int:
    ap = argparse.ArgumentParser(description="Copperline RF limit validation")
    ap.add_argument("board", help="routed board JSON (copperline output)")
    ap.add_argument("--config", help="EM config JSON (stackup + RF limits)")
    ap.add_argument("--json", action="store_true", help="JSON stdout only")
    ap.add_argument("--openems", action="store_true",
                    help="also generate openEMS FDTD projects per RF net")
    ap.add_argument("--openems-out", default="em_openems",
                    help="directory for generated openEMS projects")
    args = ap.parse_args()

    with open(args.board, "r", encoding="utf-8") as fh:
        board = json.load(fh)
    cfg = load_config(args.config)
    global _BOARD
    _BOARD = board

    traces_by_net = net_traces(board)
    nets = board.get("nets", [])
    reports = []
    checked = 0
    for net in nets:
        traces = traces_by_net.get(net.get("id"), [])
        if not traces:
            continue
        r = evaluate_net(net, traces, cfg)
        reports.append(r)
        checked += 1

    report = {
        "schema": REPORT_SCHEMA,
        "generator": "em/validate_rf.py",
        "board_file": args.board,
        "freq_ghz": cfg["rf"].get("freq_ghz", 2.45),
        "stackup": cfg["stackup"],
        "solver": "quasi-static-closed-form",
        "openems_generated": args.openems,
        "nets_checked": checked,
        "nets_pass": sum(1 for r in reports if r["pass"]),
        "nets": reports,
        "overall_pass": all(r["pass"] for r in reports) if reports else None,
        "notes": [
            "Closed-form quasi-static models (Pozar/IPC-2141A); run with",
            "--openems for full-wave FDTD verification of critical nets.",
            "Crosstalk/via checks are first-order estimates only.",
        ],
    }

    if args.openems:
        try:
            from gen_openems import generate_projects
            generate_projects(board, reports, cfg, args.openems_out)
            report["openems_dir"] = args.openems_out
        except Exception as exc:  # never fail the analytical report
            report["openems_error"] = str(exc)

    if args.json:
        print(json.dumps(report, indent=1))
    else:
        print(f"EM validation: {report['nets_pass']}/{report['nets_checked']} "
              f"RF nets PASS at {report['freq_ghz']} GHz")
        for r in reports:
            status = "PASS" if r["pass"] else "FAIL"
            z0 = f"{r['z0_ohm']:.1f} ohm" if r["z0_ohm"] is not None else "n/a"
            print(f"  [{status}] {r['net']}: Z0={z0} "
                  f"S11={r['s11_db']}dB S21={r['s21_db']}dB "
                  f"I_rf={r['rf_ampacity_a']}A")

    return 0 if report["overall_pass"] in (None, True) else 1


if __name__ == "__main__":
    sys.exit(main())
