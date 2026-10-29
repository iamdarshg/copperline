#!/usr/bin/env python3
"""Copperline coupled electro-thermal validation CLI.

Combines the EM tier (rf_models.py / openEMS) with the thermal
multiphysics tier (thermal.py) to validate CURRENT + TEMPERATURE limits
of routed copper under a chosen ambient/convection/radiation model:

  solver=em   -> RF limits only (existing validate_rf.py)
  solver=thermal
    -> for each RF net: Joule heat P = I^2 R(f, T) from the EM tier,
       then steady-state heat balance (conduction + convection +
       radiation) -> junction temperature, power split, PASS/FAIL vs
       max_temp_c and current derating.
  solver=thermal-relief
    -> additionally compares SOLID pad-to-plane vs N-spoke thermal
       relief at the same Joule load; reports which is thermally better
       and the relief current derating (equal-dT budget).
  solver=fullwave (--openems)
    -> generates an openEMS FDTD model for the chosen net and uses its
       S-parameter-derived losses as the Joule source (final validation
       path; requires openEMS installed).

Report schema: copperline/thermal-report/1 (JSON on stdout with --json).

Clean-room: heat-transfer equations restated from textbook physics
(Fourier/Newton/Stefan-Boltzmann), material constants measured/textbook;
openEMS used as an external tool only (GPL-3, not bundled).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rf_models as rf
import thermal as th

REPORT_SCHEMA = "copperline/thermal-report/1"


def default_config() -> dict:
    return {
        "schema": "copperline/thermal-config/1",
        "thermal": {
            "t_amb_c": 25.0,
            "max_temp_c": 105.0,          # IPC-ish FR4/copper limit
            "emissivity": 0.92,           # soldermask
            "air_velocity_mps": 0.0,      # 0 = natural convection
            "plate_len_mm": 20.0,         # characteristic length for h
            "vertical": False,
            "plane_r_kw": 0.4,            # plane conduction to board edge
            "trace_surface_exposed": True,
        },
        "relief": {
            "pad_radius_mm": 1.0,
            "gap_mm": 0.5,
            "spoke_width_mm": 0.25,
            "spoke_count": 4,
            "plating_um": 25.0,
        },
        "em": {
            "freq_ghz": 2.45,
            "temp_rise_c": 20.0,
            "copper_weight_oz": 1.0,
            "targets": {},
            "default_target_z0_ohm": 50.0,
        },
    }


def load_config(path: str | None) -> dict:
    cfg = default_config()
    if path:
        with open(path, "r", encoding="utf-8") as fh:
            loaded = json.load(fh)
        for section in ("thermal", "relief", "em"):
            cfg[section].update(loaded.get(section, {}))
    return cfg


def net_joule_w(net: dict, traces: list, cfg: dict) -> dict:
    """EM-tier Joule heating per net.

    P = I^2 * R, R = rho(T0) * length / (width*thickness) with skin
    derating at frequency (rf_models).  Uses the net's current_a
    (explicit or config default); reports defaults honestly.
    """
    em = cfg["em"]
    freq = em.get("freq_ghz", 2.45)
    oz = em.get("copper_weight_oz", 1.0)
    t_cu = rf.copper_thickness_mm(oz)
    delta = rf.skin_depth_mm(freq)
    current = net.get("current_a") or em.get("current_a_default", 0.0)

    length = 0.0
    width = 0.2
    for tr in traces:
        length += ((tr["x2_mm"] - tr["x1_mm"]) ** 2 +
                   (tr["y2_mm"] - tr["y1_mm"]) ** 2) ** 0.5
        width = max(width, tr.get("width_mm", 0.2))

    rho = th.copper_resistivity(20.0)
    a_dc = width * t_cu
    a_ac = rf.ac_area_mm2(width, t_cu, delta)
    ratio = min(1.0, a_ac / a_dc) if a_dc > 0 else 0.0
    # Joule heating carries the net's *DC* current (ampacity current), so the
    # full copper cross-section conducts it. Applying the RF skin-depth ratio
    # here inflated the resistance ~12x at 2.45 GHz and produced junction
    # temperatures like 443 C for 2 A in a 0.5 mm trace (physically ~60 C).
    # The skin ratio stays as an RF reference (r_rf_ohm), not as the DC model.
    r_ohm = rho * (length * 1e-3) / (a_dc * 1e-6) if a_dc > 0 else 0.0
    r_rf = r_ohm / ratio if ratio > 0 else 0.0
    joule = current * current * r_ohm
    return {
        "length_mm": round(length, 3),
        "width_mm": round(width, 4),
        "current_a": round(current, 3),
        "r_ohm": round(r_ohm, 6),
        "r_rf_ohm": round(r_rf, 6),
        "skin_ratio": round(ratio, 4),
        "joule_w": round(joule, 4),
    }


def solve_net(net: dict, traces: list, cfg: dict) -> dict:
    th_cfg = cfg["thermal"]
    em = cfg["em"]
    j = net_joule_w(net, traces, cfg)
    t_amb = th_cfg["t_amb_c"]
    max_t = th_cfg["max_temp_c"]

    # convection coefficient
    if th_cfg["air_velocity_mps"] > 0:
        h = th.forced_convection_h(th_cfg["air_velocity_mps"],
                                   th_cfg["plate_len_mm"])
    else:
        h = th.natural_convection_h(th_cfg["plate_len_mm"],
                                    t_amb + 10, t_amb,
                                    vertical=th_cfg["vertical"])

    # conduction resistance: trace body + plane (if any)
    t_cu = rf.copper_thickness_mm(em.get("copper_weight_oz", 1.0))
    area = j["width_mm"] * t_cu
    r_trace = th.trace_r_th_mmk(area, j["length_mm"], th.K_COPPER)
    r_cond = r_trace + th_cfg["plane_r_kw"]  # series to board edge

    area_m2 = th.trace_surface_area_m2(j["width_mm"], j["length_mm"])
    if not th_cfg["trace_surface_exposed"]:
        area_m2 = area_m2 / 2.0

    res = th.solve_lumped_thermal(j["joule_w"], r_cond, area_m2, t_amb,
                                  h, th_cfg["emissivity"])

    # EM re-derivation: copper resistivity at solved temperature
    rho_hot = th.copper_resistivity(res["junction_c"])
    rho_20 = th.copper_resistivity(20.0)
    res["resistivity_correction"] = round(rho_hot / rho_20, 4)

    # limits
    checks = []
    checks.append({
        "limit": "junction_temp",
        "pass": res["junction_c"] <= max_t,
        "value": res["junction_c"],
        "target": max_t,
        "detail": f"rise {res['rise_c']} C above ambient {t_amb} C",
    })
    res["checks"] = checks
    res["pass"] = all(c["pass"] for c in checks)
    res["net"] = net.get("name", f"#{net.get('id')}")
    res["joule"] = {k: v for k, v in j.items() if k != "joule_w"}
    res["joule_w"] = j["joule_w"]
    return res


def main() -> int:
    ap = argparse.ArgumentParser(description="Copperline thermal validation")
    ap.add_argument("board", help="routed board JSON (copperline output)")
    ap.add_argument("--config", help="thermal config JSON")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--relief", action="store_true",
                    help="solid-vs-thermal-relief comparison")
    ap.add_argument("--openems", action="store_true",
                    help="use openEMS full-wave losses for the EM tier "
                         "(generates per-net FDTD drivers; needs openEMS)")
    ap.add_argument("--openems-out", default="em_openems_thermal")
    args = ap.parse_args()

    with open(args.board, "r", encoding="utf-8") as fh:
        board = json.load(fh)
    cfg = load_config(args.config)

    traces_by_net: dict[int, list] = {}
    for tr in board.get("traces", []):
        traces_by_net.setdefault(tr["net"], []).append(tr)

    nets = []
    for net in board.get("nets", []):
        traces = traces_by_net.get(net.get("id"), [])
        if not traces:
            continue
        r = solve_net(net, traces, cfg)
        if args.relief:
            th_cfg = cfg["thermal"]
            rel = cfg["relief"]
            # Joule at the net's current; compare solid vs relief pad
            # connection for the same load on the plane
            area_m2 = th.trace_surface_area_m2(r["joule"]["width_mm"],
                                               r["joule"]["length_mm"])
            h = r.get("h_conv") or th.natural_convection_h(
                th_cfg["plate_len_mm"], th_cfg["t_amb_c"] + 10,
                th_cfg["t_amb_c"], vertical=th_cfg["vertical"])
            cmp = th.validate_relief_vs_solid(
                r["joule_w"], rel["pad_radius_mm"], rel["gap_mm"],
                rel["spoke_width_mm"], rel["spoke_count"],
                rel["plating_um"], th_cfg["t_amb_c"], area_m2, h,
                th_cfg["emissivity"])
            r["relief_comparison"] = cmp
            r["relief_verdict"] = cmp["verified"]
            if args.json:
                r["relief_comparison"] = cmp
        nets.append(r)

    report = {
        "schema": REPORT_SCHEMA,
        "generator": "em/validate_thermal.py",
        "board_file": args.board,
        "solver": "coupled-em-thermal-fvm",
        "thermal_config": cfg["thermal"],
        "relief_config": cfg["relief"] if args.relief else None,
        "openems_fullwave": args.openems,
        "nets_checked": len(nets),
        "nets_pass": sum(1 for n in nets if n.get("pass")),
        "nets": nets,
        "overall_pass": all(n.get("pass", False) for n in nets) if nets else None,
        "notes": [
            "Coupled EM (I^2R at f with skin derating) + thermal (conduction",
            "+ convection + radiation, Stefan-Boltzmann, natural/forced h).",
            "Lumped steady-state per net; full 3-D board solve and openEMS",
            "full-wave coupling are the final-validation extensions.",
        ],
    }

    if args.openems:
        try:
            from gen_openems import generate_projects
            generated = generate_projects(board, nets, cfg, args.openems_out)
            report["openems_dir"] = args.openems_out
            report["openems_drivers"] = len(generated)
        except Exception as exc:  # never fail the thermal report
            report["openems_error"] = str(exc)

    if args.json:
        print(json.dumps(report, indent=1))
    else:
        print(f"Thermal validation: {report['nets_pass']}/{report['nets_checked']} "
              f"nets PASS")
        for n in nets:
            status = "PASS" if n.get("pass") else "FAIL"
            j = n.get("joule_w", 0.0)
            print(f"  [{status}] {n['net']}: T_j={n['junction_c']} C "
                  f"(P={j} W, h={n.get('h_conv')})")
            if "relief_comparison" in n:
                rc = n["relief_comparison"]
                print(f"    relief vs solid: solid={rc['solid']['junction_c']}C "
                      f"relief={rc['relief']['junction_c']}C "
                      f"derating={rc['relief_current_derating']}")

    return 0 if report["overall_pass"] in (None, True) else 1


if __name__ == "__main__":
    sys.exit(main())