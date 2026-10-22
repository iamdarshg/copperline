#!/usr/bin/env python3
"""Clean-room parametric power-pad thermal-relief study (stdlib only).

Electrical connection losses use R=rho*L/A and P=I^2R. At AC, a simple
skin-depth area cap is used (delta=sqrt(2*rho/(omega*mu0))); not a substitute
for a converged full-wave current distribution. Thermal balance delegates to
thermal.py (Fourier conduction, Newton cooling, Stefan-Boltzmann radiation).
"""
from __future__ import annotations
import csv, itertools, json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import thermal as th

MU0 = 4e-7 * math.pi
FREQUENCIES = (0, 1e6, 100e6, 2.45e9)

def connection_resistance(variant, radius_mm, gap_mm, width_mm, n, plating_um, freq):
    rho = th.copper_resistivity(20.0)
    thick = plating_um * 1e-6
    if variant == "solid":
        # Annular circumference times plating cross section; radial path ~ pad radius.
        length = radius_mm * 1e-3
        area = 2 * math.pi * radius_mm * 1e-3 * thick
    else:
        length = gap_mm * 1e-3
        skin = math.sqrt(2*rho/(2*math.pi*freq*MU0)) if freq else math.inf
        area = n * width_mm*1e-3 * min(thick, 2*skin)
    return rho * length / area

def run_case(current_a, freq, n, width, gap, radius, plating_um=25.0):
    # Fixed representative board-scale heat rejection area; same for variants.
    area = 1e-4
    h = th.natural_convection_h(20, 35, 25)
    out = {"current_a": current_a, "frequency_hz": freq, "spoke_count": n,
           "spoke_width_mm": width, "gap_mm": gap, "pad_radius_mm": radius}
    for variant in ("solid", "relief"):
        resistance = connection_resistance(variant, radius, gap, width, n, plating_um, freq)
        # Include the corresponding thermal path as heat sink. Current loss in the connection.
        rth = th.solid_pad_thermal_resist(radius, plating_um) if variant == "solid" else th.relief_thermal_resist(radius, gap, width, n, plating_um)
        result = th.solve_lumped_thermal(current_a**2*resistance, rth, area, 25, h, th.EPS_SOLDERMASK)
        out[variant] = {**result, "connection_ohm": resistance,
                        "conduction_pct": round(100*result["conduction_w"]/max(result["power_w"],1e-12),2),
                        "convection_pct": round(100*result["convection_w"]/max(result["power_w"],1e-12),2),
                        "radiation_pct": round(100*result["radiation_w"]/max(result["power_w"],1e-12),2)}
    out["relief_current_derating"] = th.relief_current_derating(
        th.solid_pad_thermal_resist(radius, plating_um),
        th.relief_thermal_resist(radius, gap, width, n, plating_um))
    out["rise_ratio"] = out["relief"]["rise_c"] / max(out["solid"]["rise_c"], 1e-9)
    out["unacceptable"] = out["rise_ratio"] > 2 or out["relief_current_derating"] < .70
    return out

def sweep():
    return [run_case(i,f,n,w,g,r) for i,f,n,w,g,r in itertools.product(
        (1,2,5,10), FREQUENCIES, (2,4,6,8), (.2,.3,.4,.5), (.3,.5,1.0), (.8,1.,1.5))]

if __name__ == "__main__":
    data=sweep()
    if len(sys.argv)>1:
        with open(sys.argv[1],"w",newline="",encoding="utf-8") as fp:
            w=csv.DictWriter(fp,fieldnames=data[0].keys()); w.writeheader(); w.writerows(data)
    else: print(json.dumps(data,indent=2))
