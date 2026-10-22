#!/usr/bin/env python3
"""Copperline EM module self-tests (clean-room, no third-party test deps).

Run:  python tests/test_em.py        (from the repo root)
Or:   python em/../tests/test_em.py

Validates the closed-form RF models against published textbook values:
  * microstrip Z0 known-good (Hammerstad): 50-ohm design checks on 1.6mm FR4
  * stripline sanity bounds
  * skin depth vs published copper values
  * IPC-2221 ampacity vs copperline's AmpacityModel expectations
  * RF derating monotonicity (more frequency -> less capacity)
  * return-loss symmetry and S21 monotonicity
  * via ampacity bounds
  * end-to-end validate_rf.py on the RF fixture
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "em"))
EM_DIR = os.path.join(ROOT, "em")

import rf_models as rf  # noqa: E402

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
    print("== microstrip Z0 ==")
    # 50-ohm design point: w=3.0mm, h=1.6mm, t=0.035mm, er=4.4
    z0, eps_eff = rf.microstrip_z0(3.0, 0.035, 1.6, 4.4)
    check("3mm trace -> ~50 ohm on 1.6mm FR4",
          abs(z0 - 50.0) < 6.0, f"got {z0:.2f}")
    check("eps_eff in (1, 4.4)", 1.0 < eps_eff < 4.4, f"got {eps_eff:.3f}")
    # wider trace -> lower impedance
    z0_wide, _ = rf.microstrip_z0(6.0, 0.035, 1.6, 4.4)
    check("wider trace -> lower Z0", z0_wide < z0, f"{z0_wide:.2f} vs {z0:.2f}")

    print("== stripline ==")
    z0s, _ = rf.stripline_z0(0.2, 0.035, 1.0, 4.4)
    check("thin stripline Z0 plausible", 15 < z0s < 50, f"got {z0s:.2f}")
    z0s_wide, _ = rf.stripline_z0(2.0, 0.035, 1.0, 4.4)
    check("wide stripline lower Z0", z0s_wide < z0s, f"{z0s_wide:.2f} vs {z0s:.2f}")

    print("== skin depth ==")
    d_1g = rf.skin_depth_mm(1.0)
    check("copper skin depth ~0.0021mm at 1GHz",
          abs(d_1g - 0.0021) < 0.0003, f"got {d_1g:.5f}")
    d_10g = rf.skin_depth_mm(10.0)
    check("skin depth scales as 1/sqrt(f)",
          abs(d_10g - d_1g / math.sqrt(10.0)) < 1e-4, f"got {d_10g:.5f}")

    print("== IPC-2221 ampacity vs copperline model ==")
    # copperline AmpacityModel: I = 0.048 * dT^0.44 * A^0.725 (external)
    # A = w_mils * t_mils.  For 1mm/1oz/20C: A = 39.37*1.37 = 53.94 mil^2
    dc = rf.ipc2221_ampacity(1.0, 1.0, 20.0)
    expected = 0.048 * (20.0 ** 0.44) * (53.94 ** 0.725)
    check("matches exact IPC-2221 closed form", abs(dc - expected) < 1e-3,
          f"got {dc:.4f} expected {expected:.4f}")
    check("IPC-2221 1mm/1oz/20C ~3.23A", abs(dc - 3.23) < 0.1, f"got {dc:.4f}")
    dc_int = rf.ipc2221_ampacity(1.0, 1.0, 20.0, internal=True)
    check("internal layer derates by 0.024/0.048",
          abs(dc_int - dc * 0.5) < 1e-9, f"got {dc_int:.4f}")

    print("== RF derating ==")
    d0, a0, _ = rf.rf_derated_ampacity(3.0, 1.0, 20.0, 0.001)
    d1, a1, _ = rf.rf_derated_ampacity(3.0, 1.0, 20.0, 2.45)
    check("DC ampacity > RF ampacity", a0 >= a1, f"{a0:.3f} vs {a1:.3f}")
    check("DC ampacity equals IPC at ~DC", abs(d0 - rf.ipc2221_ampacity(3.0, 1.0, 20.0)) < 1e-6)
    check("RF derating < 1 at 2.45GHz", a1 < d1, f"{a1:.3f} vs {d1:.3f}")

    print("== return / insertion loss ==")
    s11_perfect = rf.return_loss_db(50.0, 50.0)
    check("matched -> -inf dB return loss", s11_perfect == -math.inf)
    s11_off = rf.return_loss_db(55.0, 50.0)
    check("5 ohm mismatch -> ~-26dB", abs(s11_off - (-26.0)) < 2.0, f"got {s11_off:.2f}")
    check("S21 more negative for longer traces",
          rf.insertion_loss_db(0.05, 100.0) < rf.insertion_loss_db(0.05, 10.0))

    print("== via ampacity ==")
    vdc, vac = rf.via_ampacity(0.6, 25.0, 20.0)
    check("0.6mm via/25um plating -> ~4.0A DC", abs(vdc - 4.02) < 0.3,
          f"got {vdc:.3f}")
    vdc2, vac2 = rf.via_ampacity(0.6, 25.0, 20.0, freq_ghz=2.45)
    check("via AC <= DC ampacity", vac2 <= vdc2 + 1e-9, f"{vac2:.3f} vs {vdc2:.3f}")

    print("== structure classification ==")
    check("no planes -> unknown", rf.classify_trace(False, False) == "unknown")
    check("one plane -> microstrip", rf.classify_trace(True, False) == "microstrip")
    check("two planes -> stripline", rf.classify_trace(True, True) == "stripline")

    print("== end-to-end validate_rf.py on RF fixture ==")
    proc = subprocess.run(
        [sys.executable, os.path.join(EM_DIR, "validate_rf.py"),
         os.path.join(EM_DIR, "fixtures", "rf_board.json"),
         "--config", os.path.join(EM_DIR, "fixtures", "em_rf_config.json"),
         "--json"],
        capture_output=True, text=True, cwd=EM_DIR)
    check("validate_rf exit 0 (pipeline)", proc.returncode in (0, 1),
          f"rc={proc.returncode} stderr={proc.stderr[:200]}")
    try:
        rep = json.loads(proc.stdout)
        check("report schema", rep.get("schema") == "copperline/em-report/1")
        check("2 nets checked", rep.get("nets_checked") == 2, str(rep.get("nets_checked")))
        names = {n["net"] for n in rep["nets"]}
        check("ANT + GND present", names == {"ANT", "GND"}, str(names))
        ant = next(n for n in rep["nets"] if n["net"] == "ANT")
        check("ANT classified microstrip", ant["structure"] == "microstrip",
              ant["structure"])
        check("ANT Z0 ~50 ohm", abs(ant["z0_ohm"] - 50.0) < 6.0,
              f"got {ant['z0_ohm']}")
    except Exception as exc:  # noqa: BLE001
        check("report parse", False, str(exc))

    print("== gen_openems.py generation ==")
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        proc = subprocess.run(
            [sys.executable, os.path.join(EM_DIR, "gen_openems.py"),
             os.path.join(EM_DIR, "fixtures", "rf_board.json"),
             "--config", os.path.join(EM_DIR, "fixtures", "em_rf_config.json"),
             "--out", td],
            capture_output=True, text=True, cwd=EM_DIR)
        ok = proc.returncode == 0 and os.path.exists(os.path.join(td, "ANT_sim.py"))
        check("gen_openems writes ANT_sim.py", ok, proc.stderr[:200])
        if ok:
            import py_compile
            try:
                py_compile.compile(os.path.join(td, "ANT_sim.py"), doraise=True)
                check("ANT_sim.py compiles", True)
            except Exception as exc:  # noqa: BLE001
                check("ANT_sim.py compiles", False, str(exc))

    print(f"\n{'-' * 40}\nEM self-tests: {PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())