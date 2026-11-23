"""U.S. Standard Atmosphere 1976 (0 .. 86 km geometric).

Scalar, allocation-free implementation: it is called once per aerodynamic
evaluation (i.e. every RK4 stage), so it must be cheap.
"""
from __future__ import annotations

import math

G0 = 9.80665              # m/s^2
R_AIR = 287.05287         # J/(kg K)  (R* / M0 of US76)
GAMMA = 1.4
R_EARTH_GEOPOT = 6356766.0
RHO0 = 1.225              # sea-level density kg/m^3
T0 = 288.15
P0 = 101325.0
A0 = math.sqrt(GAMMA * R_AIR * T0)

# Layer table: geopotential base altitude [m], base temperature [K], lapse [K/m]
_LAYERS = [
    (0.0, 288.15, -0.0065),
    (11000.0, 216.65, 0.0),
    (20000.0, 216.65, 0.0010),
    (32000.0, 228.65, 0.0028),
    (47000.0, 270.65, 0.0),
    (51000.0, 270.65, -0.0028),
    (71000.0, 214.65, -0.0020),
    (84852.0, 186.946, 0.0),
]


def _build_base_pressures():
    p = [P0]
    for i in range(len(_LAYERS) - 1):
        hb, tb, lr = _LAYERS[i]
        ht = _LAYERS[i + 1][0]
        dh = ht - hb
        if lr == 0.0:
            p.append(p[-1] * math.exp(-G0 * dh / (R_AIR * tb)))
        else:
            tt = tb + lr * dh
            p.append(p[-1] * (tt / tb) ** (-G0 / (R_AIR * lr)))
    return p


_PBASE = _build_base_pressures()


def geopotential(h: float) -> float:
    return R_EARTH_GEOPOT * h / (R_EARTH_GEOPOT + h)


def atmosphere(h: float):
    """Return (T [K], p [Pa], rho [kg/m^3], a [m/s], mu [Pa s]) at geometric altitude h [m]."""
    if h > 86000.0:
        # exponential continuation above the model top (rarely used)
        T, p, rho, a, mu = atmosphere(86000.0)
        f = math.exp(-(h - 86000.0) / 6000.0)
        return T, p * f, rho * f, a, mu
    H = geopotential(h)
    i = 0
    if H >= 11000.0:
        i = 1
        while i < len(_LAYERS) - 1 and H >= _LAYERS[i + 1][0]:
            i += 1
    hb, tb, lr = _LAYERS[i]
    pb = _PBASE[i]
    dh = H - hb
    if lr == 0.0:
        T = tb
        p = pb * math.exp(-G0 * dh / (R_AIR * tb))
    else:
        T = tb + lr * dh
        p = pb * (T / tb) ** (-G0 / (R_AIR * lr))
    rho = p / (R_AIR * T)
    a = math.sqrt(GAMMA * R_AIR * T)
    mu = 1.458e-6 * T ** 1.5 / (T + 110.4)
    return T, p, rho, a, mu


def density(h: float) -> float:
    return atmosphere(h)[2]


def speed_of_sound(h: float) -> float:
    return atmosphere(h)[3]


def gravity(h: float) -> float:
    re = 6371000.0
    return G0 * (re / (re + max(h, -1000.0))) ** 2


if __name__ == "__main__":
    for hh in (0, 1000, 5000, 11000, 20000, 25000, 32000, 50000, 80000):
        T, p, rho, a, mu = atmosphere(hh)
        print(f"{hh:6d} m  T={T:7.2f} K  p={p:10.2f} Pa  rho={rho:.5e}  a={a:6.1f}  mu={mu:.3e}")
