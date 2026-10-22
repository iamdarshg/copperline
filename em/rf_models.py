"""Copperline RF validation models — clean-room implementation.

Only physics facts and standard closed-form transmission-line
approximations published in the open literature (Pozar "Microwave
Engineering", IPC-2141A, IPC-2221) are used. No code was taken from any
proprietary or GPL implementation; every formula is restated from the
textbook treatment in our own words.

Models implemented
------------------
* microstrip   : Hammerstad/Jensen two-region closed form (Pozar ch.3)
                 incl. finite-thickness correction.
* stripline    : symmetric stripline approximation (documented as
                 approximate; coplanar/embedded structures are flagged
                 to the openEMS full-wave path).
* skin depth   : delta = 1/sqrt(pi f mu sigma).
* R_ac         : high-frequency conductor resistance using the skin
                 layer cross-section (documented surface model).
* RF ampacity  : DC (IPC-2221) capacity derated by the AC/DC
                 cross-section ratio at the frequency of interest.
* return loss  : Gamma = (Z0_trace - Z0_target)/(Z0_trace + Z0_target).
* insertion loss: alpha_c + alpha_d per unit length (Np/m).
* via capacity : plated barrel cross-section with skin derating.
* crosstalk    : first-order NEXT from the mutual/self capacitance
                 ratio for edge-coupled microstrips (documented as a
                 low-confidence estimate; openEMS recommended).
"""

from __future__ import annotations

import math

# Physical constants -------------------------------------------------------
MU0 = 1.25663706212e-6           # H/m
RHO_COPPER = 1.68e-8             # Ohm*m (20 C)
SIGMA_COPPER = 1 / RHO_COPPER    # S/m
C0 = 2.99792458e8                # m/s
OZ_MIL = 1.37                    # 1 oz copper thickness in mils
MIL_MM = 0.0254


# ---- unit helpers ---------------------------------------------------------

def copper_thickness_mm(copper_oz: float) -> float:
    """1 oz/ft^2 ~= 1.37 mil (the assumption copperline's ampacity model uses)."""
    return OZ_MIL * MIL_MM * copper_oz


def skin_depth_mm(freq_ghz: float, sigma: float = SIGMA_COPPER,
                  mu: float = MU0) -> float:
    """Classical skin depth in mm: delta = 1/sqrt(pi f mu sigma)."""
    return 1e3 / math.sqrt(math.pi * (freq_ghz * 1e9) * mu * sigma)


# ---- geometry: trace cross-section ---------------------------------------

def trace_area_mm2(width_mm: float, thickness_mm: float) -> float:
    """Geometric DC cross-section of a rectangular trace."""
    return width_mm * thickness_mm


def ac_area_mm2(width_mm: float, thickness_mm: float,
                delta_mm: float) -> float:
    """Effective RF cross-section.

    Current crowds into the skin layer: top + bottom surfaces
    (2*w*delta) plus side walls (2*t*delta), clamped to the full
    cross-section when delta >= t/2 (i.e. DC behaviour).
    """
    full = width_mm * thickness_mm
    if delta_mm >= thickness_mm / 2.0:
        return full
    ac = 2.0 * delta_mm * (width_mm + thickness_mm)
    return min(ac, full)


# ---- transmission-line impedance ------------------------------------------

def _thickness_eff(width_mm: float, thickness_mm: float,
                   height_mm: float) -> float:
    """Standard finite-thickness width correction (IPC-2141A treatment).

    u_eff = u + (t/pi h) * (1 + ln(2h/t)).  Valid for thin foils where
    t < h and t << w; the formulation used here is the classic
    correction published in the microstrip literature.
    """
    if thickness_mm <= 0 or height_mm <= 0:
        return width_mm / height_mm
    u = width_mm / height_mm
    t_h = thickness_mm / height_mm
    # correction only meaningful for t/h < 0.1
    if t_h >= 0.1:
        return u
    delta_u = (t_h / math.pi) * (1.0 + math.log(2.0 * height_mm / thickness_mm))
    return u + delta_u


def microstrip_z0(width_mm: float, thickness_mm: float, height_mm: float,
                  eps_r: float) -> tuple[float, float]:
    """Characteristic impedance + effective permittivity of a microstrip.

    Hammerstad/Jensen two-region closed form as given in Pozar
    "Microwave Engineering", ch.3, with the standard finite-thickness
    width correction (IPC-2141A treatment).
    """
    if height_mm <= 0 or width_mm <= 0:
        raise ValueError("microstrip needs w>0, h>0")
    u = _thickness_eff(width_mm, thickness_mm, height_mm)
    # effective permittivity (Pozar eq. 3.197)
    eps_eff = (eps_r + 1.0) / 2.0 + (eps_r - 1.0) / 2.0 * (
        1.0 + 12.0 / u) ** -0.5
    if u <= 1.0:
        eps_eff += 0.04 * (1.0 - u) ** 2.0
    # air-filled impedance (Pozar eq. 3.198/3.199)
    if u <= 1.0:
        z0_air = 60.0 * math.log(8.0 / u + u / 4.0)
    else:
        z0_air = 120.0 * math.pi / (
            u + 1.393 + 0.667 * math.log(u + 1.444))
    z0 = z0_air / math.sqrt(eps_eff)
    return z0, eps_eff


def stripline_z0(width_mm: float, thickness_mm: float, height_mm: float,
                 eps_r: float) -> tuple[float, float]:
    """Symmetric stripline (two-region approximation, Pozar ch.3).

    Thin-strip form Z0 = (30/sqrt(eps_r)) ln(4b/(pi W)) for W/b <= 0.35,
    plus the standard wide-strip form above that ratio.  No thickness
    correction is applied (documented as approximate); embedded and
    coplanar structures are flagged to the full-wave path.
    """
    if height_mm <= 0 or width_mm <= 0:
        raise ValueError("stripline needs w>0, h>0")
    b = height_mm  # ground-plane spacing for symmetric stripline
    wb = width_mm / b
    if wb <= 0.35:
        z0 = (30.0 / math.sqrt(eps_r)) * math.log(4.0 * b / (math.pi * width_mm))
    else:
        z0 = (30.0 / math.sqrt(eps_r)) / (
            wb + 0.441 + 0.667 * math.log(2.0 * wb + 1.0) / (wb + 1.0))
    return z0, eps_r


def classify_trace(above_plane: bool, below_plane: bool) -> str:
    """Classify a trace structure for model selection.

    Returns one of: microstrip (one reference plane), stripline (planes
    on both sides), or unknown (flag to openEMS).
    """
    if above_plane and below_plane:
        return "stripline"
    if above_plane or below_plane:
        return "microstrip"
    return "unknown"


# ---- conductor + dielectric loss -------------------------------------------

def conductor_loss_npm(width_mm: float, thickness_mm: float,
                       delta_mm: float, z0: float) -> float:
    """Conductor attenuation alpha_c in Np/m for a microstrip trace.

    Standard approximation: alpha_c = R_ac / (2 Z0) with R_ac from the
    skin-layer resistance of the trace (Pozar eq. 3.201 area).
    """
    if delta_mm <= 0 or z0 <= 0 or width_mm <= 0:
        return 0.0
    rs = RHO_COPPER / (delta_mm * 1e-3)   # Ohm/square (delta mm -> m)
    r_ac = rs / (width_mm * 1e-3)         # Ohm/m, bottom surface dominates
    return r_ac / (2.0 * z0)


def dielectric_loss_npm(freq_ghz: float, eps_r: float, tan_d: float) -> float:
    """Dielectric attenuation alpha_d in Np/m (Pozar eq. 3.202)."""
    if eps_r <= 1.0:
        return 0.0
    k0 = 2.0 * math.pi * (freq_ghz * 1e9) / C0
    return k0 * (eps_r - 1.0) * tan_d / (2.0 * math.sqrt(eps_r))


# ---- return loss ------------------------------------------------------------

def return_loss_db(z0_trace: float, z0_target: float) -> float:
    """S11 = 20 log10 |Gamma|, Gamma = (Zt - Zs)/(Zt + Zs)."""
    if z0_target <= 0:
        return 0.0
    gamma = (z0_trace - z0_target) / (z0_trace + z0_target)
    if abs(gamma) <= 0:
        return -math.inf
    return 20.0 * math.log10(abs(gamma))


def insertion_loss_db(alpha_npm: float, length_mm: float) -> float:
    """S21 = -8.686 * alpha * L (alpha in Np/m, L in m)."""
    return -8.686 * alpha_npm * (length_mm * 1e-3)


# ---- current capacity --------------------------------------------------------

def ipc2221_ampacity(width_mm: float, copper_oz: float,
                     temp_rise_c: float, internal: bool = False) -> float:
    """IPC-2221 section 6.2 trace current: I = k dT^0.44 A^0.725.

    k = 0.048 external, 0.024 internal; A = w_mils * t_mils.
    Mirrors copperline's AmpacityModel (include/router/rules.h).
    """
    t_mils = copper_oz * OZ_MIL
    w_mils = width_mm / MIL_MM
    area = w_mils * t_mils
    k = 0.024 if internal else 0.048
    return k * (temp_rise_c ** 0.44) * (area ** 0.725)


def rf_derated_ampacity(width_mm: float, copper_oz: float,
                        temp_rise_c: float, freq_ghz: float,
                        internal: bool = False) -> tuple[float, float, float]:
    """RF current capacity = DC ampacity * (A_ac/A_dc).

    Returns (dc_amps, ac_amps, delta_mm).  When the skin depth exceeds
    half the foil thickness the derating ratio saturates at 1.0.
    """
    t = copper_thickness_mm(copper_oz)
    dc = ipc2221_ampacity(width_mm, copper_oz, temp_rise_c, internal)
    delta = skin_depth_mm(freq_ghz)
    a_dc = trace_area_mm2(width_mm, t)
    a_ac = ac_area_mm2(width_mm, t, delta)
    ratio = min(1.0, a_ac / a_dc) if a_dc > 0 else 0.0
    return dc, dc * ratio, delta


def via_ampacity(hole_mm: float, plating_um: float,
                 temp_rise_c: float, freq_ghz: float = 0.0) -> tuple[float, float]:
    """Plated-through via current capacity.

    Barrel cross-section = pi * hole * plating; DC capacity via the same
    IPC-2221 k-form on the barrel area (as an equivalent square-trace
    area), derated by skin effect at f.  Returns (dc_amps, ac_amps).
    """
    if hole_mm <= 0 or plating_um <= 0:
        return 0.0, 0.0
    plating_mm = plating_um * 1e-3
    area_mm2 = math.pi * hole_mm * plating_mm
    area_mils2 = area_mm2 / (MIL_MM ** 2)
    k = 0.048
    dc = k * (temp_rise_c ** 0.44) * (area_mils2 ** 0.725)
    if freq_ghz <= 0:
        return dc, dc
    delta = skin_depth_mm(freq_ghz)
    a_ac = math.pi * hole_mm * min(plating_mm, delta)
    ratio = min(1.0, a_ac / area_mm2) if area_mm2 > 0 else 0.0
    return dc, dc * ratio


# ---- crosstalk (first-order NEXT) -------------------------------------------

def next_coupling_db(width_mm: float, gap_mm: float, height_mm: float,
                     eps_r: float, length_mm: float) -> float:
    """First-order near-end crosstalk estimate (edge-coupled microstrips).

    Model: NEXT ~ (C_mutual / C_self), expressed in dB.  C_mutual is a
    fringing approximation through the gap.  Documented low-confidence
    estimate — full-wave openEMS is recommended for anything that
    matters.  Returns dB (more negative = better isolation).
    """
    if gap_mm <= 0 or height_mm <= 0:
        return 0.0
    eps0 = 8.8541878128e-12
    c_self = eps0 * eps_r * (width_mm / height_mm)             # F/m
    c_mut = eps0 * (eps_r + 1.0) / 2.0 * (height_mm / gap_mm)  # fringing
    ratio = c_mut / (c_mut + c_self)
    coupling = ratio * min(1.0, length_mm / 50.0)
    if coupling <= 0:
        return -math.inf
    return 20.0 * math.log10(coupling)
