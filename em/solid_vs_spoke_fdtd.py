#!/usr/bin/env python3
"""openEMS full-wave solid-vs-thermal-relief comparison (clean-room).

Model: a 2-arm microstrip through-line on FR4 over a ground plane. The
middle junction connects the two arms either as SOLID copper or as a
thermal-relief (annular gap + N spokes). S21 insertion-loss difference
directly measures "what does cutting the copper into spokes cost".

Targets the installed openEMS v0.37 Python API (verified signatures):
  FDTD.AddLumpedPort(port_nr, R, start, stop, p_dir, excite=0)
  csx.GetGrid().AddLine(axis, list)          # axis: 0=x,1=y,2=z
  material.SetMaterialProperty(**kw)         # epsilon, mue, kappa
  port.CalcPort(sim_path, freq, ref_impedance)   # then .uf_inc/.uf_ref/.uf
  FDTD.SetGaussExcite(f0, fc); FDTD.Run(sim_path, cleanup=False)

Bounded + machine-safe: small board, coarse 0.5 mm mesh, 1-3 GHz sweep,
~2-4 min per run. The metal uses a lossy-equivalent conductivity so the
FDTD mesh can resolve the skin layer (documented modelling choice; both
variants use identical material, so the RELATIVE loss difference is the
quantity of interest).

Usage: python solid_vs_spoke_fdtd.py [solid|spoke] [--out DIR]
"""

from __future__ import annotations

import argparse
import math
import os
import sys

OPENEMS_DLL_DIRS = [
    r"D:\mcp\eng\openems\openEMS",
    r"D:\mcp\eng\openems\openEMS\lib",
]
for _d in OPENEMS_DLL_DIRS:
    if os.path.isdir(_d):
        os.add_dll_directory(_d)

# ---- geometry (mm) ---------------------------------------------------------
LEN_MM = 24.0            # total line length (arms + junction)
W_MM = 2.0              # arm width
GAP_MM = 0.5            # thermal-relief annular gap
N_SPOKES = 4
SPOKE_W_MM = 0.25
PAD_R_MM = 1.0          # centre pad radius
H_MM = 1.6              # substrate height
T_CU_MM = 0.035
EPS_R = 4.4
# lossy-equivalent copper so FDTD mesh (0.5mm) resolves the skin layer
# (real kappa 5.8e7 -> skin depth 0.7um at 1GHz, unresolvable; 1e5 -> ~0.5mm)
KAPPA_EQUIV = 1.0e5
# domain margin around the board so ports sit fully inside the field box
MARGIN = 2.0
X_MIN, X_MAX = -LEN_MM / 2 - MARGIN, LEN_MM / 2 + MARGIN
Y_MIN, Y_MAX = -4 - MARGIN, 4 + MARGIN
Z_MIN, Z_MAX = -H_MM - MARGIN, T_CU_MM + 0.5


def build_csx(relief: bool) -> tuple:
    from openEMS import openEMS
    from CSXCAD import ContinuousStructure

    FDTD = openEMS(NrTS=0)
    csx = ContinuousStructure()
    FDTD.SetCSX(csx)

    # substrate
    sub = csx.AddMaterial(name="FR4")
    sub.SetMaterialProperty(epsilon=EPS_R, mue=1.0)
    sub.AddBox(start=[-LEN_MM, -4, -H_MM], stop=[LEN_MM, 4, 0], priority=0)

    # ground plane
    gnd = csx.AddMetal(name="GND")
    gnd.AddBox(start=[-LEN_MM, -4, -H_MM - 1], stop=[LEN_MM, 4, -H_MM],
               priority=1)

    # lossy trace metal
    trace = csx.AddMaterial(name="trace")
    trace.SetMaterialProperty(epsilon=1.0, mue=1.0, kappa=KAPPA_EQUIV)

    if relief:
        # junction = centre pad + annular gap + 4 spokes
        j_hlf = PAD_R_MM + GAP_MM + 0.2
        # left arm
        trace.AddBox(start=[-LEN_MM / 2, -W_MM / 2, 0],
                     stop=[-j_hlf, W_MM / 2, T_CU_MM], priority=2)
        # right arm
        trace.AddBox(start=[j_hlf, -W_MM / 2, 0],
                     stop=[LEN_MM / 2, W_MM / 2, T_CU_MM], priority=2)
        # centre pad (square approximation of round pad)
        trace.AddBox(start=[-PAD_R_MM * 0.7, -PAD_R_MM * 0.7, 0],
                     stop=[PAD_R_MM * 0.7, PAD_R_MM * 0.7, T_CU_MM], priority=3)
        # 4 spokes: horizontal + vertical bars crossing the gap
        trace.AddBox(start=[j_hlf - SPOKE_W_MM, -SPOKE_W_MM / 2, 0],
                     stop=[-j_hlf + SPOKE_W_MM, SPOKE_W_MM / 2, T_CU_MM],
                     priority=3)
        trace.AddBox(start=[-SPOKE_W_MM / 2, j_hlf - SPOKE_W_MM, 0],
                     stop=[SPOKE_W_MM / 2, -j_hlf + SPOKE_W_MM, T_CU_MM],
                     priority=3)
    else:
        # solid junction: continuous strip
        trace.AddBox(start=[-LEN_MM / 2, -W_MM / 2, 0],
                     stop=[LEN_MM / 2, W_MM / 2, T_CU_MM], priority=2)

    # mesh: 0.5 mm over x/y, graded z through the stackup, covering the
    # full field domain so no port probe falls outside
    g = csx.GetGrid()
    g.SetDeltaUnit(1e-3)  # mm -> m
    g.AddLine(0, [round(x, 4) for x in
                  [X_MIN] + [-LEN_MM / 2 + i * 0.5
                             for i in range(int(LEN_MM / 0.5) + 1)] + [X_MAX]])
    g.AddLine(1, [round(y, 4) for y in
                  [Y_MIN] + [-4 + i * 0.5 for i in range(17)] + [Y_MAX]])
    g.AddLine(2, [round(z, 4) for z in sorted(set([
        Z_MIN, -H_MM - 1, -H_MM, -H_MM + 0.2, -0.05, 0.0,
        T_CU_MM / 2, T_CU_MM, 0.5, Z_MAX]))])

    FDTD.SetBoundaryCond(["MUR", "MUR", "MUR", "MUR", "MUR", "MUR"])

    # lumped ports at arm ends (fully inside the domain); capture the
    # returned port objects for v0.37 wave-decomposition S-parameters
    p1 = FDTD.AddLumpedPort(1, 50.0,
                            [-LEN_MM / 2 + 0.5, -W_MM / 2, -H_MM],
                            [-LEN_MM / 2 + 1.5, W_MM / 2, T_CU_MM], "x",
                            excite=1)
    p2 = FDTD.AddLumpedPort(2, 50.0,
                            [LEN_MM / 2 - 1.5, -W_MM / 2, -H_MM],
                            [LEN_MM / 2 - 0.5, W_MM / 2, T_CU_MM], "x")

    # excitation: Gaussian covering 1-3 GHz
    FDTD.SetGaussExcite(2.0e9, 1.0e9)
    FDTD.SetEndCriteria(1e-4)
    # bounded timestep budget (this build runs 0 steps with NrTS=0);
    # 40k steps ≈ 40 ns of simulated time — enough for a 2 GHz Gaussian
    FDTD.SetNumberOfTimeSteps(40000)
    return FDTD, csx, p1, p2


def main() -> int:
    ap = argparse.ArgumentParser(description="solid vs thermal-relief FDTD")
    ap.add_argument("variant", choices=["solid", "spoke"])
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "fdtd_results"))
    ap.add_argument("--port1", type=int, default=1)
    ap.add_argument("--port2", type=int, default=2)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    sim_path = os.path.join(args.out, f"{args.variant}_sim")
    os.makedirs(sim_path, exist_ok=True)

    FDTD, csx, p1, p2 = build_csx(relief=(args.variant == "spoke"))
    FDTD.Run(sim_path, cleanup=False, verbose=2)

    # S-parameters via v0.37 port wave decomposition.
    freq = 2.0e9  # single design frequency (mid of Gaussian band)
    for label, p in (("port1", p1), ("port2", p2)):
        try:
            p.CalcPort(sim_path, freq, ref_impedance=50.0)
        except Exception as exc:  # noqa: BLE001
            print(f"  {label}: CalcPort error: {str(exc)[:140]}")
            continue

    def _c(x):
        try:
            return complex(x.item()) if hasattr(x, "item") else complex(x)
        except Exception:  # noqa: BLE001
            return complex(x[0]) if hasattr(x, "__getitem__") else complex(x)

    def _f(x):
        try:
            return float(x.item()) if hasattr(x, "item") else float(x)
        except Exception:  # noqa: BLE001
            return float(x[0]) if hasattr(x, "__getitem__") else float(x)

    u1_inc = _c(p1.uf_inc)
    u1_tot = _c(p1.uf_tot)
    u2_tot = _c(p2.uf_tot)
    s11 = abs(_c(p1.uf_ref) / u1_inc) if abs(u1_inc) > 1e-12 else 0.0
    # transmission: port2 total wave referenced to port1 incident
    s21 = abs(u2_tot / max(abs(u1_inc), 1e-12)) if abs(u1_inc) > 1e-12 else 0.0
    p1_inc = _f(p1.P_inc)
    p1_ref = _f(p1.P_ref)
    p2_acc = _f(p2.P_acc) if hasattr(p2, "P_acc") else 0.0
    p_abs = max(0.0, p1_inc - p1_ref - p2_acc)  # absorbed by the junction
    print(f"  {args.variant}: |S11| = {20*math.log10(s11 + 1e-12):.2f} dB "
          f"|S21| = {20*math.log10(s21 + 1e-12):.2f} dB "
          f"P1_inc={p1_inc:.3e} W P_absorbed={p_abs:.3e} W")
    # persist for cross-variant comparison
    with open(os.path.join(args.out, f"{args.variant}_sp.txt"), "w",
              encoding="utf-8") as fh:
        fh.write(f"variant={args.variant} s11_db={20*math.log10(s11+1e-12):.2f} "
                 f"s21_db={20*math.log10(s21+1e-12):.2f} "
                 f"p1_inc={p1_inc:.4e} p1_ref={p1_ref:.4e} "
                 f"p2_acc={p2_acc:.4e} p_abs={p_abs:.4e}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())