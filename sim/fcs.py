"""Fly-by-wire flight control system (inner loop), 50 Hz.

Structure (time-scale separated nonlinear dynamic inversion):

1. Outer attitude loop on angle of attack alpha, sideslip beta (-> beta_cmd) and the
   wind-axis bank angle mu.  Desired attitude rates are converted to body-rate commands
   with the exact flight-path kinematics (lift / side force / gravity feed-forward,
   turn coordination):
       p_s = cos(b) [mu_dot - L/(mV) (tan(g) sin(mu) + tan(b)) + g/V cos(g) cos(mu) tan(b)]
       r_s = (Y + m g cos(g) sin(mu)) / (mV) - beta_dot
       q   = alpha_dot + tan(b) p_s + (L - m g cos(g) cos(mu)) / (mV cos(b))
2. Inner incremental NDI on body rates.  The required moment
       M_req = J omega_dot_des + omega x J omega - M_gear
   is met by moving the effectors (elevon li, lo, ri, ro and rudder) by
       du = d + W B^T (B W B^T + lam I)^-1 (dM - B d)
   where B = dM/du is obtained from the full aero model by batched finite differences
   at the current flight condition and d pulls the solution toward a preferred
   distribution (inboard = outboard elevon per side, rudder neutral).  Saturated
   effectors are removed and the remainder redistributed.
3. Body-flap auto-trim slowly unloads the symmetric elevon toward ELEVON_TARGET.
4. Ground modes: main-gear-only (derotation, pitch-rate controlled including gear
   moments), all gear (elevon load relief, roll levelling, rudder + nose-wheel steering).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import CH_BF, CH_LI, CH_LO, CH_RI, CH_RO, CH_RUD, CH_SB
from .trim import elevon_target

D2R = math.pi / 180.0
EFF = [CH_LI, CH_LO, CH_RI, CH_RO, CH_RUD]


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class FcsInput:
    """Flight state snapshot assembled by the simulator for the FCS."""
    alpha: float
    beta: float
    mu: float          # wind-axis bank
    gamma: float       # air-relative flight-path angle
    V: float
    qbar: float
    mach: float
    omega: np.ndarray
    phi: float
    theta: float
    lift: float        # aerodynamic lift (wind axes) [N]
    side: float        # aerodynamic side force (wind axes) [N]
    g: float
    u_act: np.ndarray  # current actuator positions (7,) rad
    M_now: np.ndarray  # aero moment at the current actuator positions
    M_gear: np.ndarray
    wow_main: bool
    wow_nose: bool
    gear_pos: float = 0.0


class FlightControl:
    def __init__(self, geo, body):
        self.geo = geo
        self.body = body
        self.m = geo.mass
        self.J = geo.J
        lim = geo.limits
        self.lo = np.array([lim["elevon"][0]] * 4 + [lim["bodyflap"][0], lim["rudder"][0], lim["speedbrake"][0]])
        self.hi = np.array([lim["elevon"][1]] * 4 + [lim["bodyflap"][1], lim["rudder"][1], lim["speedbrake"][1]])
        # gains
        self.K_alpha = 0.9
        self.Ki_alpha = 0.12
        self.K_mu = 0.9
        self.K_beta = 1.6
        self.Ki_beta = 0.3
        self.K_w = np.array([2.6, 2.4, 2.2])
        self.rate_lim = np.array([20.0, 8.0, 10.0]) * D2R
        self.alpha_dot_lim = 6.0 * D2R
        self.tau_aref = 0.7           # alpha reference-model time constant [s]
        self.aref_rate = 4.0 * D2R    # alpha reference-model rate limit
        self.mu_dot_lim = 12.0 * D2R
        self.K_bf = 1.2
        self.wheelbase = abs(geo.gear_pts["nose"][0] - geo.gear_pts["main_l"][0])
        self.reset()

    def reset(self):
        self.i_alpha = 0.0
        self.i_beta = 0.0
        self.cmd = np.zeros(7)
        self.omega_cmd = np.zeros(3)
        self.saturated = False
        self.B = np.zeros((3, 5))
        self.amax = np.zeros(3)
        self._pre = None
        self.F0 = np.zeros(3)
        self._apos = self._aneg = np.zeros(3)
        self._rl_pos = self._rl_neg = self.rate_lim.copy()
        self.rate_lim_eff = self.rate_lim.copy()
        self.mu_dot_lim_eff = self.mu_dot_lim
        self._a_ref = None
        self._wc_prev = None
        self._wc_dot = np.zeros(3)
        self._was_ground = False

    # ------------------------------------------------------------------ allocation
    def allocate(self, B, dM, u_act, rows, pref, qbar):
        """Weighted, regularised pseudo-inverse with saturation redistribution."""
        S = self.geo.S
        ms = qbar * S * np.array([self.geo.b, self.geo.cbar, self.geo.b])
        ms = np.maximum(ms, 1.0)
        Bt = (B / ms[:, None])[rows]
        dm = (dM / ms)[rows]
        ua = u_act[EFF]
        lo = self.lo[EFF]
        hi = self.hi[EFF]
        w = np.array([1.0, 1.0, 1.0, 1.0, 0.8])
        d = pref - ua
        free = np.ones(5, dtype=bool)
        du = np.zeros(5)
        lam = 1e-5
        for _ in range(3):
            f = free
            Bf = Bt[:, f]
            Wf = w[f]
            fixed_contrib = Bt[:, ~f] @ du[~f] if (~f).any() else 0.0
            rhs = dm - fixed_contrib - Bf @ d[f]
            G = (Bf * Wf) @ Bf.T + lam * np.eye(Bf.shape[0])
            try:
                sol = np.linalg.solve(G, rhs)
            except np.linalg.LinAlgError:
                sol = np.zeros(Bf.shape[0])
            du[f] = d[f] + Wf * (Bf.T @ sol)
            u = ua + du
            sat = (u < lo) | (u > hi)
            if not (sat & f).any():
                break
            du = np.clip(u, lo, hi) - ua
            free = free & ~sat
            if not free.any():
                break
        u = np.clip(ua + du, lo, hi)
        self.saturated = bool(((u <= lo + 1e-6) | (u >= hi - 1e-6)).any())
        return u

    def attainable(self, B, u_act):
        """Max moment per axis and DIRECTION with the other two axes held at zero (from u_act).
        Returns (pos (3,), neg (3,)) magnitudes."""
        ua = u_act[EFF]
        lo, hi = self.lo[EFF], self.hi[EFF]
        try:
            Bp = np.linalg.pinv(B)
        except np.linalg.LinAlgError:
            return np.zeros(3), np.zeros(3)
        out = np.zeros((2, 3))
        for i in range(3):
            d = Bp[:, i]
            for k, sgn in enumerate((1.0, -1.0)):
                dd = sgn * d
                trav = np.where(dd > 0, hi - ua, ua - lo)
                with np.errstate(divide="ignore", invalid="ignore"):
                    s = np.where(np.abs(dd) > 1e-12, np.maximum(trav, 0.0) / np.abs(dd), np.inf)
                v = float(np.min(s))
                out[k, i] = v if np.isfinite(v) else 0.0
        return out[0], out[1]

    def precompute(self, eval_batch, u_act):
        """Run the effectiveness batch once; returns the base aero force/moment (row 0) and
        caches B for the next update() (saves a separate base evaluation)."""
        B, M0 = self.effectiveness(eval_batch, u_act)
        self._pre = (B, M0)
        return self.F0, M0

    def effectiveness(self, eval_batch, u_act):
        delta = 1.0 * D2R
        ctrls = np.repeat(u_act[None, :], 6, axis=0)
        for k, ch in enumerate(EFF):
            # perturb away from the nearer limit
            s = 1.0 if u_act[ch] + delta <= self.hi[ch] else -1.0
            ctrls[k + 1, ch] += s * delta
        F, M = eval_batch(ctrls)
        self.F0 = F[0].copy()
        B = np.zeros((3, 5))
        for k, ch in enumerate(EFF):
            s = 1.0 if u_act[ch] + delta <= self.hi[ch] else -1.0
            B[:, k] = (M[k + 1] - M[0]) / (s * delta)
        self.B = B
        return B, M[0]

    @staticmethod
    def _relief(s, V):
        """Rollout elevon load relief (TE down): keeps the nose gear loaded against the
        drag-chute / speedbrake nose-up moments; grows with any pitch-up attitude."""
        if V <= 25.0:
            return 0.0
        th = max(s.theta * 180.0 / math.pi - 0.3, 0.0)
        return min(5.0 + 10.0 * th + 15.0 * max(s.omega[1], 0.0) * 180.0 / math.pi / 2.0, 18.0) * D2R

    # ------------------------------------------------------------------ main update
    def update(self, dt, s: FcsInput, cmd, eval_batch):
        """cmd: dict(alpha, bank, beta, sb (rad), steer (rad), pitch_rate (rad/s or None),
        hold_attitude flags).  Returns the actuator command vector (7,) [rad]."""
        out = s.u_act.copy()
        m, V = self.m, max(s.V, 1.0)
        J = self.J
        w = s.omega
        u_act = s.u_act
        ground = s.wow_main or s.wow_nose
        if s.qbar < 150.0:
            # no aerodynamic authority: neutral surfaces
            out[:4] = 0.0
            out[CH_RUD] = 0.0
            out[CH_SB] = cmd.get("sb", 0.0)
            self.cmd = np.clip(out, self.lo, self.hi)
            return self.cmd

        if self._pre is not None:
            B, M0 = self._pre
            self._pre = None
        else:
            B, M0 = self.effectiveness(eval_batch, u_act)
        # control authority: attainable PURE-axis angular acceleration from the current
        # deflections (pseudo-inverse direction, limited by the remaining travel)
        apos, aneg = self.attainable(B, u_act)
        apos, aneg = apos / np.diag(J), aneg / np.diag(J)
        amax = np.minimum(apos, aneg)
        self.amax = amax
        # a rate change of rate_lim must be achievable within ~1.5 s using <= 35 % authority
        # (direction specific: e.g. nose-down recovery stays available when nose-up is saturated)
        floor = np.array([2.0, 1.0, 0.6]) * D2R
        rl_pos = np.maximum(np.minimum(self.rate_lim, 0.35 * apos * 1.5), floor)
        rl_neg = np.maximum(np.minimum(self.rate_lim, 0.35 * aneg * 1.5), floor)
        rl = np.minimum(rl_pos, rl_neg)
        sa_ = abs(math.sin(s.alpha))
        mu_dot_lim = min(self.mu_dot_lim, 0.9 * rl[0], 0.8 * rl[2] / max(sa_, 0.05))
        self._apos, self._aneg = apos, aneg
        self._rl_pos, self._rl_neg = rl_pos, rl_neg
        self.rate_lim_eff = rl
        self.mu_dot_lim_eff = mu_dot_lim
        de_sym = float(np.mean(u_act[:4]))
        el_l = 0.5 * (u_act[CH_LI] + u_act[CH_LO])
        el_r = 0.5 * (u_act[CH_RI] + u_act[CH_RO])

        if not ground:
            # ---------------------------------------------------------- outer loop
            # alpha reference model (first order, rate limited) -> smooth model following
            if self._a_ref is None:
                self._a_ref = s.alpha
            a_ref_dot = float(np.clip((cmd["alpha"] - self._a_ref) / self.tau_aref,
                                      -self.aref_rate, self.aref_rate))
            self._a_ref += a_ref_dot * dt
            a_err = self._a_ref - s.alpha
            # conditional integration (only near the reference, never while saturated)
            if not self.saturated and abs(a_err) < 0.5 * D2R:
                self.i_alpha = float(np.clip(self.i_alpha + a_err * dt, -0.08, 0.08))
            a_dot = np.clip(a_ref_dot + self.K_alpha * a_err + self.Ki_alpha * self.i_alpha,
                            -self.alpha_dot_lim, self.alpha_dot_lim)
            mu_err = wrap(cmd["bank"] - s.mu)
            mu_dot = float(np.clip(self.K_mu * mu_err, -mu_dot_lim, mu_dot_lim))
            b_err = cmd.get("beta", 0.0) - s.beta
            if not self.saturated and abs(b_err) < 1.0 * D2R:
                self.i_beta = float(np.clip(self.i_beta + b_err * dt, -0.05, 0.05))
            b_dot = self.K_beta * b_err + self.Ki_beta * self.i_beta
            ca, sa = math.cos(s.alpha), math.sin(s.alpha)
            cb, tb = math.cos(s.beta), math.tan(s.beta)
            cg, tg = math.cos(s.gamma), math.tan(max(min(s.gamma, 1.4), -1.4))
            cm, sm = math.cos(s.mu), math.sin(s.mu)
            L, Y, g = s.lift, s.side, s.g
            p_s = cb * (mu_dot - L / (m * V) * (tg * sm + tb) + g / V * cg * cm * tb)
            r_s = (Y + m * g * cg * sm) / (m * V) - b_dot
            q_c = a_dot + tb * p_s + (L - m * g * cg * cm) / (m * V * cb)
            p_c = p_s * ca - r_s * sa
            r_c = p_s * sa + r_s * ca
            w_cmd = np.clip(np.array([p_c, q_c, r_c]), -rl_neg, rl_pos)
            rows = [0, 1, 2]
            pref = np.array([el_l, el_l, el_r, el_r, 0.0])
        else:
            # ---------------------------------------------------------- ground modes
            self.i_alpha = 0.0
            self.i_beta = 0.0
            self._a_ref = None
            steer = cmd.get("steer", 0.0)
            r_c = V * math.tan(steer) / self.wheelbase if V > 3.0 else 0.0
            p_c = -1.2 * s.phi
            if not s.wow_nose:
                qr = cmd.get("pitch_rate")
                q_c = qr if qr is not None else -1.5 * D2R
                if s.theta < 0.3 * D2R and qr is None:
                    q_c = 0.0
                w_cmd = np.array([p_c, q_c, r_c])
                rows = [0, 1, 2]
                pref = np.array([el_l, el_l, el_r, el_r, 0.0])
            else:
                w_cmd = np.array([p_c, 0.0, r_c])
                rows = [0, 2]
                relief = self._relief(s, V)
                asym = 0.5 * (el_l - el_r)
                pref = np.array([relief + asym, relief + asym, relief - asym, relief - asym, 0.0])
        self.omega_cmd = w_cmd
        # reference-derivative feed-forward (filtered finite difference of the rate command)
        if self._wc_prev is None or ground != self._was_ground:
            self._wc_prev = w_cmd.copy()
            self._wc_dot[:] = 0.0
        raw = (w_cmd - self._wc_prev) / dt
        self._wc_prev = w_cmd.copy()
        self._was_ground = ground
        self._wc_dot += (np.clip(raw, -0.5, 0.5) - self._wc_dot) * min(1.0, dt / 0.08)
        # (rate-command derivative feed-forward disabled: the alpha reference model already gives a
        #  smooth q command, and differentiating an authority-clipped command caused chatter)
        ffw = np.zeros(3)
        w_dot = self.K_w * (w_cmd - w) + ffw
        if not ground:
            # pseudo-control limit: never ask for more than ~40 % of the available authority
            fl = np.array([2.0, 1.0, 0.5]) * D2R
            w_dot = np.clip(w_dot, -np.maximum(0.4 * self._aneg, fl), np.maximum(0.4 * self._apos, fl))
        M_req = J @ w_dot + np.cross(w, J @ w) - s.M_gear
        dM = M_req - M0
        # prioritised allocation: pitch first; scale the roll/yaw demand back until the
        # solution stays inside the effector limits (prevents roll-induced pitch departures)
        u_eff = None
        for k_ry in (1.0, 0.6, 0.3, 0.0):
            dMk = dM.copy()
            dMk[0] *= k_ry
            dMk[2] *= k_ry
            u_try = self.allocate(B, dMk, u_act, rows, pref, s.qbar)
            if u_eff is None:
                u_eff = u_try
            if not self.saturated:
                u_eff = u_try
                break
            u_eff = u_try
        if ground and s.wow_nose:
            # pitch channel not controlled: impose the symmetric load-relief position
            asym_l = 0.5 * (u_eff[0] + u_eff[1]) - 0.5 * (u_eff[2] + u_eff[3])
            relief = self._relief(s, V)
            u_eff[0] = u_eff[1] = relief + 0.5 * asym_l
            u_eff[2] = u_eff[3] = relief - 0.5 * asym_l
        for k, ch in enumerate(EFF):
            out[ch] = u_eff[k]
        # body flap auto-trim (holds once both gears are on the ground)
        if not (ground and s.wow_nose):
            out[CH_BF] = u_act[CH_BF] + self.K_bf * (de_sym - elevon_target(s.gear_pos, s.mach))
        else:
            out[CH_BF] = u_act[CH_BF]
        out[CH_SB] = cmd.get("sb", 0.0)
        self.cmd = np.clip(out, self.lo, self.hi)
        return self.cmd


class Actuators:
    """First-order lag + rate + position limits for the 7 surfaces."""

    def __init__(self, geo):
        r = geo.rates
        lim = geo.limits
        self.rate = np.array([r["elevon"]] * 4 + [r["bodyflap"], r["rudder"], r["speedbrake"]])
        self.lo = np.array([lim["elevon"][0]] * 4 + [lim["bodyflap"][0], lim["rudder"][0], lim["speedbrake"][0]])
        self.hi = np.array([lim["elevon"][1]] * 4 + [lim["bodyflap"][1], lim["rudder"][1], lim["speedbrake"][1]])
        self.tau = np.array([0.05] * 4 + [0.2, 0.05, 0.1])
        self.pos = np.zeros(7)
        self.cmd = np.zeros(7)

    def reset(self, pos=None):
        self.pos = np.zeros(7) if pos is None else np.clip(np.asarray(pos, float), self.lo, self.hi)
        self.cmd = self.pos.copy()

    def step(self, dt):
        c = np.clip(self.cmd, self.lo, self.hi)
        v = (c - self.pos) / self.tau
        v = np.clip(v, -self.rate, self.rate)
        self.pos = np.clip(self.pos + v * dt, self.lo, self.hi)
        return self.pos
