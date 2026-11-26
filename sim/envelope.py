"""Flight-envelope warnings for the PFD: overspeed and aerodynamic stall.

The two are independent measurements, so both can be active at once (a hard pull-up or a
storm updraft at high dynamic pressure: an accelerated stall while overspeeding).

Overspeed
    Equivalent airspeed, i.e. dynamic pressure = structural load, above the limit:
    VMO = 350 KEAS clean (q = 19.9 kPa, 10 kt above the fastest outer glide slope the
    guidance flies), VLE = 300 KEAS while the gear is not up (cockpit placard
    "GEAR DN NOT ABOVE 300 KEAS").

Stall
    Not an airspeed threshold.  A wing is stalled when it has passed its maximum lift: more
    angle of attack no longer gives more lift (dL/dalpha <= 0) because the flow has
    separated from it.  It is measured on the strip model of :mod:`sim.aero` in the
    current local flow - body rates (the down-going wing in a roll sees a larger angle),
    sideslip (a swept wing's advancing half), elevon deflections (trailing edge down
    raises the section angle), gusts and drafts, ground effect - by evaluating the lift of
    each wing half at alpha and alpha +- 1 deg:

        slope  dL/dalpha    = (L+ - L-) / 2d
        curvature           = (L+ - 2 L0 + L-) / d^2
        stall margin        = -slope / curvature   (Newton step to the lift maximum)

    * STALL (warning): either wing half has dL/dalpha <= 0 (past its lift maximum; one
      half only = asymmetric stall, wing drop)
    * stall caution: the lift maximum of either half is less than 3 deg of alpha away
    * lift loss (information): normal force the separated flow costs, compared with the
      attached-flow value, per wing half

    The low-speed band of the speed tape is the matching speed: the EAS at which the
    present load factor needs the vehicle's maximum lift coefficient,
    Vs(n) = sqrt(2 n W / (rho0 S CLmax(M))).  It rises with the load factor and can reach
    into the overspeed band ("coffin corner").
"""
from __future__ import annotations

import math

import numpy as np

from .aero import _ss
from .atmosphere import RHO0

KT = 1.0 / 0.514444
D2R = math.pi / 180.0
R2D = 180.0 / math.pi

VMO_KEAS = 350.0          # clean
VLE_KEAS = 300.0          # gear in transit or down
OVERSPEED_HYST = 3.0      # kt below the limit to clear the warning
STALL_DA = 1.0 * D2R      # alpha step of the lift-curve evaluation
CAUTION_ON = 3.0          # deg: stall caution when the lift maximum is closer than this
CAUTION_OFF = 4.0
STALL_CLEAR = 0.5         # deg of margin (both halves) to clear the stall warning
STALL_HOLD = 1.0          # s the warning stays on at least (no flicker in buffeting flow)
QBAR_MIN = 300.0          # Pa: below this there is no meaningful air data / lift
MARGIN_CAP = 30.0         # deg: 'far from the stall'
REFINE_BELOW = 12.0       # deg: refine the stall margin when the Newton estimate is below this


class EnvelopeMonitor:
    """Evaluated by the simulator at 25 Hz (the GPC flight-control cycle)."""

    def __init__(self, aero, tables, mass, S):
        self.aero = aero
        self.W = mass * 9.80665
        self.S = S
        wing = aero.is_wing
        self.left = wing & (aero.qc[1] < 0.0)
        self.right = wing & (aero.qc[1] > 0.0)
        # maximum lift coefficient per Mach (clean, untrimmed table maximum over alpha <= 40 deg)
        self.cl_mach = np.asarray(tables.mach, float)
        self.cl_max = np.nanmax(np.asarray(tables.CL)[:, :, 0, 0], axis=0)
        self.reset()

    def reset(self):
        self.stall = False
        self.stall_t = -1e9
        self.caution = False
        self.overspeed = False
        self.n_f = 1.0
        self.out = self._blank()

    @staticmethod
    def _blank():
        return {"valid": False, "stall": False, "stall_l": False, "stall_r": False, "stall_caution": False,
                "margin_l": None, "margin_r": None, "alpha_stall": None, "cla_l": None, "cla_r": None,
                "loss_l": None, "loss_r": None, "overspeed": False, "vmo": VMO_KEAS, "vmo_clean": VMO_KEAS,
                "vle": VLE_KEAS, "vs": None, "vs1g": None, "n": None}

    # ------------------------------------------------------------------ physics
    def lift_curve(self, v_air, omega, rho, a_snd, mu, h_cg, up_b, ctrl):
        """Lift-curve state of the left / right wing half in the current local flow.

        Returns dict(slope (2,) [N/rad], margin (2,) [deg of alpha to the lift maximum, < 0 past
        it, MARGIN_CAP when far away], loss (2,) [fraction of the attached-flow normal force
        lost to separation], qS) or None without airflow."""
        vx, vy, vz = (float(v) for v in v_air)
        V = math.sqrt(vx * vx + vy * vy + vz * vz)
        if V < 1.0 or rho <= 0.0:
            return None
        A = self.aero
        M = V / a_snd
        wsup = _ss(0.85, 1.30, M)
        p, q, r = (float(w) for w in omega)
        c = np.atleast_2d(np.asarray(ctrl, float))[:1]
        alpha = math.atan2(vz, vx)
        sides = (self.left, self.right)

        def lift(d, want_loss=False):
            """lift of each wing half with the air velocity rotated to alpha + d (rates, controls kept)"""
            cd, sd = math.cos(d), math.sin(d)
            ad = alpha + d
            s = A._strips(vx * cd - vz * sd, vy, vx * sd + vz * cd, p, q, r, rho, a_snd, mu, h_cg, up_b, c,
                          M, ad, wsup)
            lf = s["Fx"][0] * math.sin(ad) - s["Fz"][0] * math.cos(ad)
            L = np.array([lf[m].sum() for m in sides])
            if not want_loss:
                return L
            qS, cn, cna = s["qS"], s["cN"][0], s["cN_att"][0]
            loss = np.array([max(float((qS[m] * (cna[m] - cn[m])).sum()), 0.0)
                             / max(float((qS[m] * np.abs(cna[m])).sum()), 1e-6) for m in sides])
            return L, loss

        Lm, (L0, loss), Lp = lift(-STALL_DA), lift(0.0, True), lift(STALL_DA)
        slope = (Lp - Lm) / (2.0 * STALL_DA)
        curv = (Lp - 2.0 * L0 + Lm) / (STALL_DA * STALL_DA)
        margin = np.full(2, MARGIN_CAP)
        for i in range(2):
            if curv[i] < 0.0:
                margin[i] = -slope[i] / curv[i] * R2D           # Newton step to dL/dalpha = 0
            elif slope[i] <= 0.0:
                margin[i] = -STALL_DA * R2D
        # the Newton step overestimates the distance (the lift curve bends more and more toward
        # its maximum): refine with the slope a little short of the predicted maximum (secant)
        near = (margin > 0.0) & (margin < REFINE_BELOW)
        if near.any():
            h = 0.8 * float(margin[near].min()) * D2R
            sh = (lift(h + STALL_DA) - lift(h - STALL_DA)) / (2.0 * STALL_DA)
            for i in np.flatnonzero(near):
                if sh[i] < slope[i]:
                    margin[i] = min(margin[i], h * slope[i] / (slope[i] - sh[i]) * R2D)
        return {"slope": slope, "margin": np.clip(margin, -MARGIN_CAP, MARGIN_CAP), "loss": loss,
                "qS": 0.5 * rho * V * V * self.S}

    def cl_max_at(self, mach):
        return float(np.interp(mach, self.cl_mach, self.cl_max))

    # ------------------------------------------------------------------ warnings
    def update(self, t, dt, o, omega, ctrl, gear_pos, on_ground, lift):
        """o: Simulator.observe() dict.  Returns the list of (level, text) events to log."""
        events = []
        keas = o["eas"] * KT
        n = lift / self.W
        self.n_f += (n - self.n_f) * min(1.0, dt / 0.3)
        if on_ground or o["qbar"] < QBAR_MIN:
            # inhibited with weight on wheels (as on real aircraft) and without usable air data
            self.stall = self.caution = self.overspeed = False
            self.out = self._blank()
            return events
        out = self._blank()
        out["valid"] = True
        # ---- overspeed
        vlim = VLE_KEAS if gear_pos > 0.02 else VMO_KEAS
        if not self.overspeed and keas > vlim:
            self.overspeed = True
            what = f"VLE {VLE_KEAS:.0f} KEAS (gear not up)" if vlim == VLE_KEAS else f"VMO {VMO_KEAS:.0f} KEAS"
            events.append(("warn", f"OVERSPEED: {keas:.0f} KEAS above {what}, qbar {o['qbar'] / 1000:.1f} kPa"))
        elif self.overspeed and keas < vlim - OVERSPEED_HYST:
            self.overspeed = False
            events.append(("info", f"overspeed cleared: {keas:.0f} KEAS"))
        out.update(overspeed=self.overspeed, vmo=vlim)
        # ---- stall (flow state of the wing)
        lc = self.lift_curve(o["v_air"], omega, o["rho"], o["a"], o["mu_visc"], o["agl"], o["R"][2, :], ctrl)
        if lc is not None:
            mg, sl = lc["margin"], lc["slope"]
            past = sl <= 0.0
            if not self.stall and past.any():
                self.stall = True
                self.stall_t = t
                side = "both wings" if past.all() else ("left wing" if past[0] else "right wing")
                events.append(("warn", f"STALL: {side} past maximum lift - alpha {o['alpha'] * R2D:.1f} deg, "
                                       f"{keas:.0f} KEAS, nz {self.n_f:.1f} g, lift lost L {lc['loss'][0] * 100:.0f}% "
                                       f"R {lc['loss'][1] * 100:.0f}%"))
            elif self.stall and not past.any() and mg.min() > STALL_CLEAR and t - self.stall_t >= STALL_HOLD:
                self.stall = False
                events.append(("info", f"stall recovered: alpha {o['alpha'] * R2D:.1f} deg, {keas:.0f} KEAS"))
            m = float(mg.min())
            if self.caution:
                self.caution = m < CAUTION_OFF
            else:
                self.caution = m < CAUTION_ON
            cla = sl / max(0.5 * lc["qS"], 1e-6)                # per wing half, based on S/2
            out.update(stall=self.stall, stall_l=bool(past[0]), stall_r=bool(past[1]),
                       stall_caution=bool(self.caution and not self.stall),
                       margin_l=float(mg[0]), margin_r=float(mg[1]),
                       alpha_stall=float(o["alpha"] * R2D + m) if m < REFINE_BELOW else None,
                       cla_l=float(cla[0]), cla_r=float(cla[1]),
                       loss_l=float(lc["loss"][0]), loss_r=float(lc["loss"][1]))
        # ---- stall speed at the present load factor (speed tape low-speed band)
        clm = self.cl_max_at(o["mach"])
        vs1g = math.sqrt(2.0 * self.W / (RHO0 * self.S * clm)) * KT
        out.update(vs1g=vs1g, vs=vs1g * math.sqrt(max(self.n_f, 0.0)), n=self.n_f)
        self.out = out
        return events
