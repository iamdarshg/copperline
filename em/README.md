# Copperline EM validation module

Validates **current, RF, and thermal limits** of routed copper against the
physical electromagnetics and heat transfer of the stackup. Three tiers,
all covered by this repo's tests:

1. **Analytical RF tier (always available, zero dependencies)** — `em/rf_models.py`
   * microstrip / stripline characteristic impedance (Hammerstad/Jensen,
     Pozar ch.3; finite-thickness correction)
   * skin depth + RF-conductor resistance → **return loss S11** and
     **insertion loss S21** at the operating frequency
   * **RF-derated ampacity**: the router's DC IPC-2221 current capacity,
     de-rated by the AC/DC cross-section ratio at frequency
   * plated-through via capacity, first-order near-end crosstalk
2. **Coupled thermal tier (conduction + convection + radiation + EM Joule)**
   — `em/thermal.py` + `em/validate_thermal.py`:
   * **EM Joule source** — per-net P = I²·R(f,T) with skin-effect R and
     temperature-corrected copper resistivity
   * **Conduction** — Fourier heat transfer along the trace body and
     through the plane (copper k=385, FR4 k=0.3 W/mK)
   * **Convection** — natural (Churchill-type) or forced-air film
     coefficient h
   * **Radiation** — Stefan-Boltzmann linearized h_rad with emissivity
   * **Solid vs thermal-relief-spoke comparison** (`--relief`): quantifies
     that N-spoke relief connections are thermally WORSE for power nets
     (derating I_rel = I_solid·√(R_solid/R_relief))
   * openEMS full-wave driver generation for final validation
3. **Full-wave tier (openEMS FDTD)** — `gen_openems.py` writes a complete,
   self-contained openEMS simulation driver per RF net from the routed
   geometry (substrate, ground plane, trace, mesh, lumped ports, Gaussian
   excitation, frequency sweep, S-parameter extraction).

## Clean-room status

All analytical models are restated from public-domain physics and
standard published closed forms (Pozar *Microwave Engineering*, IPC-2141A,
IPC-2221) — no proprietary or third-party code was copied, and the
router itself remains dependency-free. openEMS is GPL-3 and **not**
bundled: copperline only *generates* driver scripts that run against a
user-installed openEMS. Nothing from openEMS is embedded or copied.

## Usage

```bash
# Analytical RF/current validation of a routed board (JSON from `router route --output`)
python em/validate_rf.py em/fixtures/rf_board.json \
  --config em/fixtures/em_rf_config.json --json

# Coupled thermal validation (conduction + convection + radiation + EM Joule)
python em/validate_thermal.py em/fixtures/rf_board.json \
  --config em/fixtures/em_rf_config.json --json

# Add solid-vs-thermal-relief-spoke comparison (the power-net tradeoff)
python em/validate_thermal.py em/fixtures/rf_board.json \
  --config em/fixtures/em_rf_config.json --relief

# Also generate openEMS FDTD drivers for final full-wave validation
python em/validate_rf.py em/fixtures/rf_board.json \
  --config em/fixtures/em_rf_config.json --openems --openems-out out/

# Generate only the FDTD drivers
python em/gen_openems.py em/fixtures/rf_board.json \
  --config em/fixtures/em_rf_config.json --out out/

# Run a generated FDTD driver (needs openEMS installed)
python out/ANT_sim.py
```

## Config schema (`copperline/em-config/1`)

```json
{
  "schema": "copperline/em-config/1",
  "stackup": {
    "dielectric_mm": 1.6,      // h under signal layers
    "eps_r": 4.4,              // FR4
    "tan_d": 0.02,
    "copper_weight_oz": 1.0,
    "plating_um": 25.0,        // via barrel plating
    "internal_layers": []      // layer ids treated as internal (0.024 k)
  },
  "rf": {
    "freq_ghz": 2.45,
    "temp_rise_c": 20.0,
    "default_target_z0_ohm": 50.0,
    "max_s11_db": -10.0,
    "min_s21_db": -1.5,
    "targets": {
      "ANT": {"z0_ohm": 50.0, "max_s11_db": -10.0,
              "min_s21_db": -1.5, "current_a": 1.0}
    }
  }
}
```

Targets are keyed by net name. A net without a target uses the defaults;
a net whose name is absent from `targets` is still checked against the
default Z0 target and its own `current_a` (RF ampacity check).

## Report schema (`copperline/em-report/1`)

```json
{
  "schema": "copperline/em-report/1",
  "freq_ghz": 2.45,
  "solver": "quasi-static-closed-form",
  "openems_generated": false,
  "nets_checked": 2,
  "nets_pass": 1,
  "nets": [
    {
      "net": "ANT",
      "structure": "microstrip",
      "z0_ohm": 50.21,
      "s11_db": -53.46,
      "s21_db": -0.26,
      "skin_depth_mm": 0.0013,
      "dc_ampacity_a": 1.35,
      "rf_ampacity_a": 0.114,
      "checks": [ {"limit": "z0", "pass": true, "value": 50.21, "target": 50.0, "detail": "..."} ],
      "pass": false
    }
  ],
  "overall_pass": false,
  "notes": ["..."],
  "openems_dir": "out/"   // only when --openems was given
}
```

Exit code: `0` when every checked net passes, `1` when any net fails.

## Notes and honest limits

* The analytical tier is a **quasi-static estimate**, not a qualified
  full-wave rating. The microstrip model is accurate for standard double-
  sided stackups; stripline is thin-strip approximate; coplanar waveguide
  and embedded coupled structures are flagged `unknown` and must go
  through the openEMS tier.
* `rf_ampacity` is inherently pessimistic at RF — at 2.45 GHz the copper
  skin depth is ~1.3 µm, so the usable cross-section of a 35 µm foil is
  a small fraction of the DC area. This is physics (and the reason RF
  power nets use wide copper, multiple layers or many parallel vias).
* The openEMS drivers generated by `gen_openems.py` are real, runnable
  FDTD projects with S-parameter extraction, but they model a single
  isolated trace over an infinite ground plane. For board-level coupling,
  crosstalk and resonance checks, extend the driver with the full board
  geometry (the CSXCAD primitives are all present in the generated file).

## Tests

```bash
python tests/test_em.py        # 29 assertions: RF models + openEMS generation
python tests/test_thermal.py   # 23 assertions: convection/radiation/relief physics
```

Validated against textbook values: 50 Ω microstrip on 1.6 mm FR4,
IPC-2221 ampacity closed form (mirrors `AmpacityModel` in
`include/router/rules.h`), copper skin depth at 1/10 GHz, return-loss
symmetry, monotonic RF derating, end-to-end report + openEMS generation,
natural/forced convection magnitudes, Stefan-Boltzmann radiation
coefficient, and the solid-vs-relief thermal verdict.