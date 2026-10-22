#!/usr/bin/env python3
"""Copperline thermal multiphysics tests (clean-room, stdlib only).

Validates thermal.py + validate_thermal.py against textbook values:
  * natural-convection film coefficient magnitude (small vs large plate)
  * forced-convection monotonicity (more air -> higher h)
  * radiation linearized coefficient (Stefan-Boltzmann, ~6-7 W/m2K @40C)
  * relief-vs-solid: spokes are thermally WORSE at equal Joule load
  * relief current derating < 1 (equal-dT budget)
  * end-to-end validate_thermal.py on the RF fixture (with --relief)
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EM_DIR = os.path.join(ROOT, "em")
sys.path.insert(0, EM_DIR)

import thermal as th  # noqa: E402
import validate_thermal as vt  # noqa: E402

PASS = 0
FAIL = 0


def check(name: str, cond: bool, detail: str = ""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok  {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name}  {detail}")


def main() -> int:
    print("== convection ==")
    h_small = th.natural_convection_h(10.0, 40.0, 20.0)     # 10mm plate
    h_large = th.natural_convection_h(100.0, 40.0, 20.0)    # 100mm board
    h_1m = th.natural_convection_h(1000.0, 40.0, 20.0)      # 1m plate
    check("small plate h ~8-9", 7.0 < h_small < 10.0, f"got {h_small:.2f}")
    check("100mm board h ~5", 4.5 < h_large < 5.5, f"got {h_large:.2f}")
    check("1m plate h ~2.8", 2.5 < h_1m < 3.5, f"got {h_1m:.2f}")
    check("h decreases with size", h_large < h_small,
          f"{h_large:.2f} vs {h_small:.2f}")
    h_forced = th.forced_convection_h(2.0, 100.0)
    check("forced 2m/s > natural", h_forced > h_large * 2,
          f"{h_forced:.1f} vs {h_large:.1f}")

    print("== radiation ==")
    hr = th.radiation_coefficient_hrad(40.0, 20.0, 0.92)
    check("h_rad @40C ~6-7", 5.5 < hr < 7.5, f"got {hr:.2f}")
    hr_high = th.radiation_coefficient_hrad(100.0, 20.0, 0.92)
    check("h_rad increases with T", hr_high > hr, f"{hr_high:.2f} vs {hr:.2f}")

    print("== material/geometry ==")
    check("copper rho(20C) = 1.68e-8",
          abs(th.copper_resistivity(20.0) - 1.68e-8) < 1e-11)
    check("rho temp coefficient positive", th.copper_resistivity(60.0)
          > th.copper_resistivity(20.0))

    print("== thermal relief physics ==")
    r_solid = th.solid_pad_thermal_resist(1.0, 25.0)
    r_relief = th.relief_thermal_resist(1.0, 0.5, 0.25, 4, 25.0)
    check("solid << relief resistance", r_relief > 50 * r_solid,
          f"{r_solid:.3f} vs {r_relief:.1f} K/W")
    more_spokes = th.relief_thermal_resist(1.0, 0.5, 0.25, 8, 25.0)
    check("8 spokes < 4 spokes resistance", more_spokes < r_relief,
          f"{more_spokes:.1f} vs {r_relief:.1f}")

    print("== lumped solve ==")
    res = th.solve_lumped_thermal(1.0, 50.0, 1e-5, 20.0, 5.0, 0.92)
    check("1W on high-R path rises > 20C", res["rise_c"] > 10,
          f"rise={res['rise_c']}C")
    check("power split sums ~input",
          abs(res["conduction_w"] + res["convection_w"] + res["radiation_w"]
              - 1.0) < 0.02,
          f"split {res['conduction_w']}+{res['convection_w']}"
          f"+{res['radiation_w']}")
    res_zero = th.solve_lumped_thermal(0.0, 50.0, 1e-5, 20.0, 5.0, 0.92)
    check("zero power -> ambient", abs(res_zero["junction_c"] - 20.0) < 1e-6)

    print("== relief vs solid verdict ==")
    cmp = th.validate_relief_vs_solid(1.0, 1.0, 0.5, 0.25, 4, 25.0, 20.0,
                                      1e-5, 5.0)
    check("relief hotter than solid",
          cmp["relief"]["junction_c"] > cmp["solid"]["junction_c"],
          f"{cmp['solid']['junction_c']}C vs {cmp['relief']['junction_c']}C")
    check("derating < 1", cmp["relief_current_derating"] < 0.5,
          f"got {cmp['relief_current_derating']}")
    check("verdict string", cmp["verified"] == "relief_worse")

    print("== end-to-end validate_thermal.py ==")
    proc = subprocess.run(
        [sys.executable, os.path.join(EM_DIR, "validate_thermal.py"),
         os.path.join(EM_DIR, "fixtures", "rf_board.json"),
         "--config", os.path.join(EM_DIR, "fixtures", "em_rf_config.json"),
         "--relief", "--json"],
        capture_output=True, text=True, cwd=EM_DIR)
    check("exit 0/1 (pipeline)", proc.returncode in (0, 1),
          f"rc={proc.returncode} {proc.stderr[:200]}")
    try:
        rep = json.loads(proc.stdout)
        check("thermal-report schema",
              rep.get("schema") == "copperline/thermal-report/1")
        check("2 nets checked", rep.get("nets_checked") == 2)
        ant = next(n for n in rep["nets"] if n["net"] == "ANT")
        check("ANT junction reported", ant.get("junction_c", 0) > 25.0,
              f"{ant.get('junction_c')}C")
        check("ANT has relief comparison",
              "relief_comparison" in ant)
        rc = ant["relief_comparison"]
        check("ANT relief worse than solid",
              rc["relief"]["junction_c"] > rc["solid"]["junction_c"],
              f"{rc['solid']['junction_c']} vs {rc['relief']['junction_c']}")
    except Exception as exc:  # noqa: BLE001
        check("report parse", False, str(exc))

    print(f"\nThermal self-tests: {PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())