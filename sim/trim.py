"""Static aerodynamic coefficient helpers, pitch trim solver and trim tables.

The trim tables (alpha x Mach x speedbrake x gear) feed the reduced 3-DOF model used
by guidance (SCP/MPC later), the spawn generator and the placeholder guidance.
"""
from __future__ import annotations

import hashlib
import json
import math
import os

import numpy as np

from .aero import AeroModel
from .atmosphere import atmosphere
from .geometry import CH_BF, CH_LI, CH_LO, CH_RI, CH_RO, CH_SB, GEOM_PATH, ROOT

DATA_DIR = os.path.join(ROOT, "sim", "data")
TABLE_PATH = os.path.join(DATA_DIR, "trim_tables.npz")

ALPHA_GRID = np.arange(-4.0, 40.01, 1.0)           # deg
MACH_GRID = np.array([0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.2,
                      1.4, 1.6, 1.8, 2.0, 2.3, 2.6, 3.0])
SB_GRID = np.array([0.0, 25.0, 50.0, 75.0, 98.6])  # deg
GEAR_GRID = np.array([0.0, 1.0])
ELEVON_TARGET = math.radians(-1.0)                 # body-flap auto-trim target elevon
ELEVON_TARGET_LDG = math.radians(3.0)              # landing configuration (gear down): more lift


def elevon_target(gear=0.0, mach=0.5):
    """Body-flap auto-trim target for the symmetric elevon."""
    return ELEVON_TARGET_LDG if (gear > 0.5 and mach < 0.95) else ELEVON_TARGET


def flight_condition(M, qbar=10000.0):
    """Altitude giving dynamic pressure ``qbar`` at Mach M (clamped to sea level)."""
    h = 10000.0
    for _ in range(30):
        T, p, rho, a, mu = atmosphere(h)
        rho_req = 2.0 * qbar / (M * a) ** 2
        h_new = h + 7000.0 * math.log(rho / rho_req)
        h_new = min(max(h_new, 0.0), 40000.0)
        if abs(h_new - h) < 1.0:
            h = h_new
            break
        h = h_new
    T, p, rho, a, mu = atmosphere(h)
    return M * a, rho, a, mu, h


def coeffs(am: AeroModel, M, alpha, beta=0.0, ctrl=None, qbar=10000.0, rates=(0.0, 0.0, 0.0),
           gear=0.0, h_agl=1e4, cond=None):
    """Body-axis/wind-axis coefficients for a batch of control vectors (angles in rad)."""
    V, rho, a, mu, h = cond if cond is not None else flight_condition(M, qbar)
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    v = np.array([V * ca * cb, V * sb, V * sa * cb])
    up = np.array([-sa, 0.0, -ca])        # wings level, pitch = alpha (gamma = 0)
    if ctrl is None:
        ctrl = np.zeros((1, 7))
    ctrl = np.atleast_2d(ctrl)
    r = am.evaluate(v, np.array(rates), rho, a, mu, h_agl, up, ctrl, gear_pos=gear)
    qS = 0.5 * rho * V * V * am.Sref
    F, Mo = r.F, r.M
    D = -(F[:, 0] * ca * cb + F[:, 1] * sb + F[:, 2] * sa * cb)
    Y = -F[:, 0] * ca * sb + F[:, 1] * cb - F[:, 2] * sa * sb
    Lf = F[:, 0] * sa - F[:, 2] * ca
    return dict(CL=Lf / qS, CD=D / qS, CY=Y / qS, Cl=Mo[:, 0] / (qS * am.bref),
                Cm=Mo[:, 1] / (qS * am.cref), Cn=Mo[:, 2] / (qS * am.bref),
                CX=F[:, 0] / qS, CZ=F[:, 2] / qS, V=V, rho=rho, h=h)


def sym_ctrl(de, bf=0.0, sb=0.0, rud=0.0):
    de = np.atleast_1d(de)
    c = np.zeros((len(de), 7))
    c[:, CH_LI] = c[:, CH_LO] = c[:, CH_RI] = c[:, CH_RO] = de
    c[:, CH_BF] = bf
    c[:, 5] = rud
    c[:, CH_SB] = sb
    return c


_DE_GRID = np.radians(np.arange(-33.0, 18.01, 0.75))
_BF_GRID = np.radians(np.arange(-11.7, 22.51, 0.6))


def _zero_cross(x, y):
    s = np.sign(y)
    idx = np.where(s[:-1] * s[1:] <= 0)[0]
    if len(idx) == 0:
        return None
    # choose the crossing with negative slope (stable trim) closest to zero deflection
    best = None
    for i in idx:
        x0, x1, y0, y1 = x[i], x[i + 1], y[i], y[i + 1]
        xc = x0 if y1 == y0 else x0 - y0 * (x1 - x0) / (y1 - y0)
        if best is None or abs(xc) < abs(best):
            best = xc
    return best


def trim_elevon(am, M, alpha, bf=0.0, sb=0.0, gear=0.0, cond=None):
    """Symmetric elevon (rad) giving Cm = 0 for a fixed body flap; None if impossible."""
    c = coeffs(am, M, alpha, ctrl=sym_ctrl(_DE_GRID, bf, sb), gear=gear, cond=cond)
    de = _zero_cross(_DE_GRID, c["Cm"])
    return de


def trim_scheduled(am, M, alpha, sb=0.0, gear=0.0, cond=None):
    """Trim the way the FCS does: body flap moves so the elevon sits at ELEVON_TARGET;
    if the body flap saturates the elevon takes the remainder.
    Returns (de, bf, coeff dict at trim) or None."""
    tgt = elevon_target(gear, M)
    ctrl = sym_ctrl(np.full(len(_BF_GRID), tgt), 0.0, sb)
    ctrl[:, CH_BF] = _BF_GRID
    c = coeffs(am, M, alpha, ctrl=ctrl, gear=gear, cond=cond)
    bf = _zero_cross(_BF_GRID, c["Cm"])
    if bf is not None:
        de = tgt
    else:
        # body flap saturated: pick the limit that helps, trim with elevon
        # Cm decreases with body flap: all negative -> flap fully up helps most, else fully down
        bf = _BF_GRID[0] if np.mean(c["Cm"]) < 0 else _BF_GRID[-1]
        de = trim_elevon(am, M, alpha, bf, sb, gear, cond)
        if de is None:
            return None
    ct = coeffs(am, M, alpha, ctrl=sym_ctrl(de, bf, sb), gear=gear, cond=cond)
    return de, bf, {k: (float(v[0]) if isinstance(v, np.ndarray) else v) for k, v in ct.items()}


# ----------------------------------------------------------------------------- tables
def _signature(am: AeroModel):
    h = hashlib.sha1()
    with open(GEOM_PATH, "rb") as f:
        h.update(f.read())
    h.update(json.dumps(am.P.__dict__, sort_keys=True, default=str).encode())
    h.update(json.dumps([ALPHA_GRID.tolist(), MACH_GRID.tolist(), SB_GRID.tolist(), GEAR_GRID.tolist()]).encode())
    for mod in ("aero.py", "trim.py", "geometry.py"):
        with open(os.path.join(ROOT, "sim", mod), "rb") as f:
            h.update(f.read())
    return h.hexdigest()[:16]


def build_trim_tables(am: AeroModel, verbose=False):
    nA, nM, nS, nG = len(ALPHA_GRID), len(MACH_GRID), len(SB_GRID), len(GEAR_GRID)
    CL = np.full((nA, nM, nS, nG), np.nan)
    CD = np.full_like(CL, np.nan)
    DE = np.full_like(CL, np.nan)
    BF = np.full_like(CL, np.nan)
    for j, M in enumerate(MACH_GRID):
        cond = flight_condition(M)
        if verbose:
            print(f"  trim tables: Mach {M:.2f}", flush=True)
        for k, sbd in enumerate(SB_GRID):
            for g, gr in enumerate(GEAR_GRID):
                if gr > 0 and M > 0.95:
                    # gear never deployed supersonic: copy gear-up values + increment later
                    continue
                for i, ad in enumerate(ALPHA_GRID):
                    t = trim_scheduled(am, M, math.radians(ad), math.radians(sbd), gr, cond)
                    if t is None:
                        # untrimmable: store untrimmed (zero controls) values, flag via NaN DE
                        c = coeffs(am, M, math.radians(ad), ctrl=sym_ctrl(0.0, 0.0, math.radians(sbd)),
                                   gear=gr, cond=cond)
                        CL[i, j, k, g], CD[i, j, k, g] = c["CL"][0], c["CD"][0]
                        continue
                    de, bf, c = t
                    CL[i, j, k, g], CD[i, j, k, g] = c["CL"], c["CD"]
                    DE[i, j, k, g], BF[i, j, k, g] = de, bf
            # supersonic gear-down = gear-up (never used, keeps the table rectangular)
            if M > 0.95:
                CL[:, j, k, 1], CD[:, j, k, 1] = CL[:, j, k, 0], CD[:, j, k, 0]
                DE[:, j, k, 1], BF[:, j, k, 1] = DE[:, j, k, 0], BF[:, j, k, 0]
    return dict(alpha=ALPHA_GRID, mach=MACH_GRID, sb=SB_GRID, gear=GEAR_GRID,
                CL=CL, CD=CD, DE=np.degrees(DE), BF=np.degrees(BF))


def load_or_build_trim_tables(am: AeroModel, verbose=True, force=False):
    sig = _signature(am)
    if not force and os.path.exists(TABLE_PATH):
        try:
            d = np.load(TABLE_PATH, allow_pickle=False)
            if str(d["signature"]) == sig:
                return {k: d[k] for k in d.files if k != "signature"}
        except Exception:
            pass
    if verbose:
        print("[sim] building trim tables from the full aero model (cached afterwards) ...", flush=True)
    t = build_trim_tables(am, verbose=False)
    os.makedirs(DATA_DIR, exist_ok=True)
    tmp = TABLE_PATH + f".{os.getpid()}.tmp.npz"
    np.savez_compressed(tmp, signature=np.array(sig), **t)
    try:
        os.replace(tmp, TABLE_PATH)        # atomic: parallel processes never read a partial file
    except OSError:
        try:
            os.remove(tmp)
        except OSError:
            pass
    return t
