# Solid vs. thermal-relief power-pad research

## Result and scope

In the clean-room lumped model, solid attachment is the thermal winner: the plane's conductive path is much lower resistance than copper spokes crossing a dielectric gap. Relief's engineering benefit is solderability/rework, not heat removal. The independent legacy helper reports equal-temperature current derating using thermal-resistance ratios; do not interpret that as a validated PCB ampacity rating. In the representative 1 mm radius, 0.5 mm gap, 4 x 0.25 mm, 25 um plating case its derating is **8.9%** (2 A/100 MHz); the thermal model predicts junction 25.00 C solid vs 25.13 C relief.

`research_relief.py` executes 2,304 combinations: current {1,2,5,10} A, DC/1 MHz/100 MHz/2.45 GHz, N={2,4,6,8}, widths {.2,.3,.4,.5} mm, gaps {.3,.5,1} mm, radii {.8,1,1.5} mm. Every combination triggers the configured unacceptable criterion (derating <70%); the candidate 2x-rise criterion is numerically unsuitable here because temperatures are rounded to 0.01 C and solid rises are near zero. Example derating by spoke count (averaged over all other sweep dimensions):

| N spokes | Equal-delta-T current derating |
|---:|---:|
| 2 | 7.2% |
| 4 | 10.2% |
| 6 | 12.5% |
| 8 | 14.4% |

Wider spokes and narrower gaps improve derating. Average derating by width .2/.3/.4/.5 mm is 8.5/10.4/12.0/13.4%; by gap .3/.5/1 mm is 14.3/11.1/7.8%. For the representative geometry, 2/4/6/8 spokes give modeled relief junctions 25.48/25.13/25.06/25.03 C and deratings 6.3/8.9/10.9/12.6%. Current increases Joule input quadratically; frequency also increases modeled loss above skin depth becomes relevant. Temperatures must be understood as the output of this small lumped model, not board qualification.

The power split is available per result as `conduction_pct`, `convection_pct`, `radiation_pct`; broad sweep summaries are omitted because very low connection losses round displayed powers to zero. In the chosen connection-only load, conduction into plane dominates. Geometry is first-order: the electrical solid connection assumes a radial path of pad radius and circumference-times-plating area; relief uses gap/(N*width*plating), with a skin-depth area cap. Existing thermal `relief_thermal_resist` similarly treats spokes in parallel. Neither model resolves actual pour spreading or pad/barrel topology.

## Full-wave status

openEMS v0.37 now runs **both variants full-wave** (`em/solid_vs_spoke_fdtd.py`):
solid and spoke models each execute 40,000 FDTD timesteps (~21 s each, 9.7k cells)
with valid S-parameter extraction (v0.37 `CalcPort` wave decomposition →
`uf_inc/uf_ref/uf_tot`, `P_inc/P_ref`, persisted to `solid_sp.txt`/`spoke_sp.txt`).

**Current limitation (honest):** the probe shows port1 S11 = −0.12 dB — the
lumped feed is badly mismatched to the line, so ~99% of the incident power
reflects at the port before reaching the junction. Absorbed power is
dominated by the lossy-equivalent trace body (kappa=1e5 to keep the skin
layer mesh-resolvable), so the solid-vs-spoke junction difference
(4.0422e-25 vs 4.0423e-25 W) is within feed/mesh noise, not yet a resolvable
signal. To make the measurement meaningful the model needs: (a) a matched
50 Ω microstrip feed (AddMSLPort or a tapered launcher) so power enters the
line, (b) short low-loss arms so the junction is the dominant loss path,
(c) more timesteps to settle. This is a bounded follow-up, not a blocker:
**no full-wave S-parameter validation of the relief verdict yet** — the
analytical coupled-thermal tier remains the validated result.

## Router recommendation

For CONNECTSTYLE relief/solid import, retain user's explicit style, but emit a review warning on thermal-relief plane connections for power nets. Use configured net current and spoke geometry; warn conservatively when estimated equal-delta-T derating is below 70% (all current sweep cases under the present helper), or when a qualified thermal calculation exceeds the net temperature limit. Prefer solid for high-current/low-temperature-rise requirements; use wider/more spokes and smaller gap when relief is required. Do not call this a certified current threshold: calibrate with plane geometry and validated EM/thermal analysis before hard rejection.

## Verification

- `python tests/test_relief_research.py` — 7 assertions pass.
- `python tests/test_thermal.py` — 23 assertions pass.
- `python tests/test_em.py` — 29 assertions pass.
