"""Real-time component build-up aerodynamics for the Orbiter.

Every evaluation uses the LOCAL flow at each element (body air-relative velocity +
omega x r, gusts already folded into the air-relative velocity):

Lifting strips (wing L/R incl. glove + carry-through, body flap, vertical tail)
    * simple-sweep decomposition: forces from the velocity components perpendicular
      to the strip span axis ``es`` (normal ``n`` and chordwise ``ect``)
    * local flow collocated at 3/4 chord (subsonic) -> 1/2 chord (supersonic),
      force applied at the local aerodynamic centre 0.25c -> 0.5c
    * surface lift slope from DATCOM (subsonic) / linear delta-wing theory (subsonic or
      supersonic leading edge, elliptic integral) with a smooth transonic blend
    * normal/axial-force formulation valid at all angles: potential term with partial
      leading-edge suction + Polhamus vortex lift, vortex breakdown / stall decay and
      flat-plate behaviour toward 90 deg
    * flaps (4 elevons, body flap, rudder): thin-airfoil flap effectiveness tau(cf)
      and c_m increment subsonic, Ackeret load-on-flap supersonic, effectiveness loss at
      large deflections, deflection drag, hinge moments
    * profile drag: turbulent skin friction (local Reynolds, compressibility, tile
      roughness) with form factor, transonic drag rise / supersonic wave drag
    * ground effect (Wieselsberger induced-drag reduction + lift-slope increase + aft
      load shift) from each strip's height above ground
    * vertical tail sidewash and fuselage blanketing at high alpha / high Mach;
      split rudder/speedbrake panels as two outward-opening plates
Fuselage + OMS pods: slender-body (Munk) apparent-mass forces with separate vertical /
lateral apparent masses, Allen-Perkins viscous crossflow, skin friction, forebody wave
drag and Mach-dependent base drag.
Gear drag (scaled by gear position) and drag chute.

``evaluate`` accepts a batch of control vectors (K,7) so the flight-control system can
obtain control effectiveness by finite differences with a single call.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .geometry import (CH_BF, CH_LI, CH_LO, CH_RI, CH_RO, CH_RUD, CH_SB, G, GROUPS,
                       SURF_BF, SURF_VT, SURF_WING, Geometry)


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _ss(e0, e1, x):
    t = min(max((x - e0) / (e1 - e0), 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def ellipe_hastings(m):
    """Complete elliptic integral of the second kind E(m), m = k^2 (Hastings approx.)."""
    m1 = max(1.0 - m, 1e-12)
    return (1.0 + m1 * (0.4630151 + 0.1077812 * m1)
            + m1 * (0.2452727 + 0.0412496 * m1) * math.log(1.0 / m1))


@dataclass
class AeroParams:
    """Calibration constants (tuned with tools/aero_report.py against Orbiter-like data)."""
    k_wing: float = 0.84         # wing lift-slope multiplier
    k_vt: float = 1.00
    k_bf: float = 1.00
    k_sup: float = 0.68          # thickness / bluntness reduction of linear supersonic theory
    kappa: float = 0.95          # section lift-slope ratio cl_a / 2pi
    k_munk: float = 1.00
    k_upwash: float = 1.90       # forebody in the wing upwash field (subsonic, vertical plane)
    cdc_z: float = 1.60          # crossflow drag coefficient, vertical (flat bottom)
    cdc_y: float = 1.10          # crossflow drag coefficient, lateral
    eta_cf: float = 0.65         # finite-length crossflow factor
    rough: float = 1.35          # tile-roughness skin-friction factor
    k_sb: float = 1.70           # speedbrake panel pressure-drag factor
    k_fd: float = 1.00           # flap deflection drag factor
    ge_k: float = 0.60           # ground-effect lift-slope increase (low-AR law, exp(-2.5 h/b))
    ge_shift: float = 0.06       # ground-effect aft load shift (fraction of chord)
    sidewash: float = 0.12       # d(sigma)/d(beta) at the vertical tail (subsonic)
    eta_bf: float = 0.75         # dynamic-pressure ratio at the body flap
    deda_sub: float = 0.50       # flow-angle reduction at the body flap (downwash + flat-bottom flow)
    deda_sup: float = 0.12
    eta_visc_sub: float = 0.97   # viscous flap-effectiveness reduction
    eta_visc_sup: float = 0.90
    cm0_wing: float = 0.020      # section zero-lift pitching moment (TE reflex), wing strips
    alpha0_wing: float = -0.026  # zero-lift angle of wing strips (rad, negative = camber, -1.5 deg)
    base_scale: float = 1.00
    nose_cd: float = 0.25        # forebody wave-drag coefficient (based on max area)
    misc_cd0: float = 0.010      # protuberances, gaps, RCS ports, windows, tile steps (dCD)
    vt_blank_a0: float = math.radians(12.0)
    vt_blank_a1: float = math.radians(32.0)
    vt_blank_min: float = 0.35


@dataclass
class AeroResult:
    F: np.ndarray            # (K,3) body-axis force [N]
    M: np.ndarray            # (K,3) body-axis moment about CG [N m]
    mach: float = 0.0
    qbar: float = 0.0
    alpha: float = 0.0
    beta: float = 0.0
    # detail (only when requested, for the first control vector)
    elem_f: np.ndarray | None = None      # (n_elements, 3)
    comp: dict | None = None              # group -> [Fx,Fy,Fz,Mx,My,Mz]
    coef: dict | None = None
    hinge: dict | None = None
    xcp: float = 0.0
    lift: float = 0.0
    drag: float = 0.0
    side: float = 0.0


class AeroModel:
    def __init__(self, geo: Geometry, params: AeroParams | None = None):
        self.geo = geo
        self.P = params or AeroParams()
        L = geo.lift
        self.N = L.n
        # component arrays -------------------------------------------------
        self.qc = L.qc.T.copy()      # (3,N)
        self.hc = L.hc.T.copy()
        self.c34 = L.c34.T.copy()
        self.nrm = L.nrm.T.copy()
        self.ect = L.ect.T.copy()
        self.ep = L.ep.T.copy()
        self.flap_pt = L.flap_pt.T.copy()
        self.S = L.S
        self.chord = L.chord
        self.tc = L.tc
        self.surf = L.surf
        self.is_wing = L.surf == SURF_WING
        self.is_vt = L.surf == SURF_VT
        self.is_bf = L.surf == SURF_BF
        lam_e = np.where(self.is_wing, np.minimum(L.sweep, math.radians(50.0)), L.sweep)
        self.inv_cos_lam = 1.0 / np.cos(lam_e)
        glove = self.is_wing & (L.sweep > math.radians(60.0))
        self.xle = self.qc[0] + 0.25 * L.chord                  # strip leading-edge x
        # aerodynamic centre (chord fraction): long glove/root strips are front-loaded
        self.xac_sub = np.where(glove, 0.16, 0.25)
        self.xac_sup = np.where(glove, 0.20, 0.29)
        self.xcol_sub = np.where(glove, 0.60, 0.75)                # collocation (pitch damping)
        self.xcol_sup = np.full(L.n, 0.60)
        self.yz_col = self.qc[1:].copy()
        self.eff = L.eff
        self.Kv = L.Kv
        self.suction = L.suction
        self.a_stall = L.a_stall
        self.cf = L.cf
        self.fmapT = L.fmap.T.copy()          # (6,N)
        self.ge_mask = L.ground_effect
        self.rud_idx = np.where(L.rud_area > 0)[0]
        self.rud_area = L.rud_area[self.rud_idx]
        self.xn = L.xnose_dist
        exposed = self.eff > 0.5
        self.swet = np.where(exposed | self.is_bf | self.is_vt, 2.04 * self.S, 0.0)
        self.swet = np.where(self.is_bf, 1.0 * self.S, self.swet)   # body flap: only lower face in free stream
        self.ff = (1.0 + 1.71 * self.tc + 100.0 * self.tc ** 4) * np.cos(L.sweep) ** 0.28 * 1.1
        self.mcr = 0.87 - self.tc - 0.12      # critical (perpendicular) Mach for drag rise
        self.cdw_peak = 1.5 * self.tc ** (5.0 / 3.0)   # 3-D relieved transonic peak

        # thin-airfoil flap data (subsonic) --------------------------------
        cf = np.clip(self.cf, 1e-6, 1.0)
        th = np.arccos(np.clip(2.0 * cf - 1.0, -1.0, 1.0))
        tau = 1.0 - (th - np.sin(th)) / math.pi
        dcl = 2.0 * ((math.pi - th) + np.sin(th))           # per rad
        dcm = -0.5 * np.sin(th) * (1.0 - np.cos(th))       # about c/4 per rad
        xcp = 0.25 - np.where(dcl > 1e-9, dcm / np.maximum(dcl, 1e-9), 0.0)
        self.tau_sub = np.where(self.cf > 0, tau, 0.0)
        self.tau_sub = np.where(self.cf >= 0.999, 1.0, self.tau_sub)
        self.xcpf_sub = np.where(self.cf >= 0.999, 0.25, xcp)
        self.all_moving = self.cf >= 0.999

        # body -------------------------------------------------------------
        B = geo.body
        self.B = B
        self.bpts = np.stack([B.bx, B.by, B.bz])            # (3,nb)
        self.spos = B.pos.T.copy()                          # (3,ns)
        self.nseg = len(B.fore)

        # gear / chute
        self.gear_pts = geo.gear_drag_pts.T.copy()          # (3,3)
        self.gear_cda = geo.gear_cda
        self.chute_pt = geo.chute_attach.copy()
        self.chute_cds = geo.chute_cds

        # surface lift-slope data: (A, tan(Lambda_c/2), tan(Lambda_LE), kind)
        self.surf_data = {
            SURF_WING: (geo.b ** 2 / geo.S, 0.411, 1.0, "delta"),
            SURF_VT: (2.30, math.tan(math.radians(37.0)), 1.0, "delta"),
            SURF_BF: (2.96, 0.0, 0.0, "rect"),
        }
        self.Sref, self.bref, self.cref = geo.S, geo.b, geo.cbar
        self.flap_idx = geo.lift_flap_idx
        self.flap_group = L.flap_group[self.flap_idx]
        self.group = L.group
        self.sl = geo.element_slices
        self.n_elements = len(geo.elements)

    # ------------------------------------------------------------------ helpers
    def lift_slope(self, M: float, surf: int) -> float:
        A, tc2, tle, kind = self.surf_data[surf]
        k = self.P.kappa

        def sub(m):
            b2 = max(1.0 - m * m, 0.0)
            return 2.0 * math.pi * A / (2.0 + math.sqrt(A * A / (k * k) * (b2 + tc2 * tc2) + 4.0))

        def sup(m):
            beta = math.sqrt(max(m * m - 1.0, 1e-6))
            if kind == "rect":
                return 4.0 / beta * max(1.0 - 1.0 / (2.0 * A * beta), 0.3)
            mm = beta / tle
            if mm < 1.0:
                return 2.0 * math.pi / tle / ellipe_hastings(1.0 - mm * mm)
            return 4.0 / beta

        if M <= 0.9:
            v = sub(M)
        elif M >= 1.2:
            v = self.P.k_sup * sup(M)
        else:
            w = _ss(0.9, 1.2, M)
            v = (1.0 - w) * sub(M) + w * self.P.k_sup * sup(1.2)
        mult = {SURF_WING: self.P.k_wing, SURF_VT: self.P.k_vt, SURF_BF: self.P.k_bf}[surf]
        return v * mult

    @staticmethod
    def cf_turb(Re, M):
        Re = np.maximum(Re, 1e5)
        return 0.455 / np.log10(Re) ** 2.58 / (1.0 + 0.144 * M * M) ** 0.65

    @staticmethod
    def base_cp(M):
        # blunt base with engine bells, no tail cone: subsonic -0.22, transonic peak
        if M < 1.05:
            return -0.22 - 0.23 * _ss(0.6, 1.05, M)
        return -0.45 * (1.05 / M) ** 1.8

    def nose_cd(self, M):
        if M < 1.15:
            return self.P.nose_cd * _ss(0.75, 1.15, M)
        return self.P.nose_cd * (1.15 / M) ** 0.3

    # ------------------------------------------------------------------ strips
    def _strips(self, vx, vy, vz, p, q, r, rho, a_snd, mu, h_cg, up_b, ctrl, M, alpha_b, wsup):
        """Lifting-strip kernel for a batch of control vectors ctrl (K,7)."""
        P = self.P
        rcx = self.xle - (self.xcol_sub + (self.xcol_sup - self.xcol_sub) * wsup) * self.chord
        rcy, rcz = self.yz_col
        ux = -(vx + (q * rcz - r * rcy))                                  # air seen by the strip
        uy = -(vy + (r * rcx - p * rcz))
        uz = -(vz + (p * rcy - q * rcx))
        nx, ny, nz = self.nrm
        ex, ey, ez = self.ect
        un = ux * nx + uy * ny + uz * nz
        uct = ux * ex + uy * ey + uz * ez
        # vertical-tail sidewash
        un = np.where(self.is_vt, un * (1.0 + P.sidewash * (1.0 - wsup)), un)
        Vp2 = un * un + uct * uct
        a_perp = np.arctan2(un, uct)
        # body-flap downwash, wing camber
        deda = P.deda_sub * (1.0 - wsup) + P.deda_sup * wsup
        a_perp = np.where(self.is_bf, a_perp - deda * alpha_b, a_perp)
        a_perp = np.where(self.is_wing, a_perp - P.alpha0_wing, a_perp)
        # fuselage blanketing of the vertical tail (high alpha) and high-Mach loss
        eta_v = 1.0 - (1.0 - P.vt_blank_min) * _ss(P.vt_blank_a0, P.vt_blank_a1, abs(alpha_b))
        eta_v *= 1.0 - 0.30 * _ss(1.0, 2.6, M)
        etaq = np.where(self.is_vt, eta_v, np.where(self.is_bf, P.eta_bf, 1.0))
        qp = 0.5 * rho * Vp2 * etaq
        Mp = np.sqrt(Vp2) / a_snd
        aw = self.lift_slope(M, SURF_WING)
        av = self.lift_slope(M, SURF_VT)
        ab = self.lift_slope(M, SURF_BF)
        a_s = np.where(self.is_wing, aw, np.where(self.is_vt, av, ab)) * self.eff * self.inv_cos_lam
        suction = self.suction * (1.0 - wsup)
        app_shift = 0.0
        if h_cg < 3.0 * self.bref:                                        # ground effect
            hi = h_cg + up_b[0] * self.qc[0] + up_b[1] * self.qc[1] + up_b[2] * self.qc[2]
            hh = np.maximum(hi, 0.3) / self.bref
            hb = 16.0 * hh
            phi = np.where(self.ge_mask, hb * hb / (1.0 + hb * hb), 1.0)  # Wieselsberger
            gl = np.where(self.ge_mask, np.exp(-2.5 * hh), 0.0)          # low-AR lift increase
            kge = 1.0 + P.ge_k * gl
            a_s = a_s * kge
            # induced drag of the free-air lift reduced by phi, GE lift increment drag-free
            suction = np.where(self.ge_mask, 1.0 - (1.0 - suction) * phi / kge, suction)
            app_shift = P.ge_shift * gl
        xapp = self.xac_sub + (self.xac_sup - self.xac_sub) * wsup + app_shift   # chord fraction
        # ---- control-dependent part (K,N)
        dl = ctrl[:, :6] @ self.fmapT
        tau = self.tau_sub * (1.0 - wsup) + np.where(self.all_moving, 1.0, self.cf) * wsup
        etav = np.where(self.all_moving, 1.0, P.eta_visc_sub * (1.0 - wsup) + P.eta_visc_sup * wsup)
        # large-deflection effectiveness loss: the local slope falls from 1 to 0.55 between
        # 12 and 30 deg; the effective deflection is its integral (strictly monotonic)
        dabs = np.abs(dl)
        e0, e1 = 0.21, 0.52
        tt = np.clip((dabs - e0) / (e1 - e0), 0.0, 1.0)
        Sx = np.where(dabs < e1, (e1 - e0) * (tt ** 3 - 0.5 * tt ** 4), 0.5 * (e1 - e0) + (dabs - e1))
        dalpha = tau * etav * np.sign(dl) * (dabs - 0.45 * Sx)
        ae = a_perp + dalpha
        sa, ca = np.sin(ae), np.cos(ae)
        sg = np.sin(a_perp)
        fst = 1.0 / (1.0 + np.exp((np.abs(ae) - self.a_stall) / 0.055))
        cN_att = a_s * sa * ca + self.Kv * (1.0 - 0.6 * wsup) * sg * np.abs(sg)
        cN = fst * cN_att + (1.0 - fst) * 1.25 * sa
        cdw = np.where(Mp < 1.0, self.cdw_peak * smoothstep(self.mcr, 1.0, Mp),
                       np.minimum(self.cdw_peak, (16.0 / 3.0) * self.tc ** 2
                                  / np.sqrt(np.maximum(Mp * Mp - 1.0, 1e-6))))
        sdl = np.sin(dl)
        cA = -suction * a_s * sa * sa * fst + cdw + P.k_fd * self.cf * sdl * sdl
        xcpf = self.xcpf_sub * (1.0 - wsup) + (1.0 - 0.5 * self.cf) * wsup
        dcm = -(a_s * dalpha * fst) * (xcpf - xapp) + np.where(self.is_wing, P.cm0_wing, 0.0)
        qS = qp * self.S
        Fx = qS * (cN * nx + cA * ex)
        Fy = qS * (cN * ny + cA * ey)
        Fz = qS * (cN * nz + cA * ez)
        cpl = qS * self.chord * dcm
        ax = self.xle - xapp * self.chord
        # ---- state-only: skin friction / form drag along the local flow (at mid chord)
        u2 = ux * ux + uy * uy + uz * uz
        um = np.sqrt(u2) + 1e-9
        Re = rho * um * self.xn / mu
        cfr = self.cf_turb(Re, um / a_snd) * P.rough * self.ff
        ffm = 0.5 * rho * u2 * cfr * self.swet * etaq / um
        # ---- split rudder / speedbrake panels (K, nr)
        ri = self.rud_idx
        drud = ctrl[:, CH_RUD]
        dsb = ctrl[:, CH_SB]
        csb = P.k_sb * (np.sin(drud + 0.5 * dsb) ** 2 + np.sin(drud - 0.5 * dsb) ** 2
                        - 2.0 * np.sin(drud) ** 2)
        qloc = 0.5 * rho * u2[ri] * eta_v
        sbm = (qloc * self.rud_area / um[ri])[None, :] * csb[:, None]
        return dict(Fx=Fx, Fy=Fy, Fz=Fz, cpl=cpl, ax=ax, ay=self.qc[1], az=self.qc[2],
                    fx0=ffm * ux, fy0=ffm * uy, fz0=ffm * uz,
                    sbx=sbm * ux[ri], sby=sbm * uy[ri], sbz=sbm * uz[ri],
                    a_perp=a_perp, qp=qp, cN=cN, xapp=xapp, eta_v=eta_v)

    # ------------------------------------------------------------------ evaluate
    def evaluate(self, v_air, omega, rho, a_snd, mu, h_cg, up_b, ctrl,
                 gear_pos=0.0, chute=0.0, detail=False) -> AeroResult:
        """Aerodynamic force/moment in body axes.

        v_air : (3,) body-axis velocity of the CG relative to the air mass [m/s]
        omega : (3,) body rates p,q,r [rad/s]
        h_cg  : CG height above ground [m] (ground effect)
        up_b  : (3,) world 'up' unit vector expressed in body axes
        ctrl  : (7,) or (K,7) control vector(s) [rad]: li, lo, ri, ro, bf, rudder, speedbrake
        gear_pos : 0..1 gear extension, chute : 0..1 inflated drag-chute area fraction
        """
        P = self.P
        ctrl = np.atleast_2d(np.asarray(ctrl, dtype=float))
        K = ctrl.shape[0]
        vx, vy, vz = float(v_air[0]), float(v_air[1]), float(v_air[2])
        p, q, r = float(omega[0]), float(omega[1]), float(omega[2])
        V = math.sqrt(vx * vx + vy * vy + vz * vz)
        if V < 0.5 or rho <= 0.0:
            res = AeroResult(F=np.zeros((K, 3)), M=np.zeros((K, 3)))
            if detail:
                self._empty_detail(res)
            return res
        M = V / a_snd
        qinf = 0.5 * rho * V * V
        alpha_b = math.atan2(vz, vx)
        beta_b = math.asin(max(-1.0, min(1.0, vy / V)))
        wsup = _ss(0.85, 1.30, M)

        s = self._strips(vx, vy, vz, p, q, r, rho, a_snd, mu, h_cg, up_b, ctrl, M, alpha_b, wsup)
        Fx, Fy, Fz, cpl = s["Fx"], s["Fy"], s["Fz"], s["cpl"]
        ax, ay, az = s["ax"], s["ay"], s["az"]
        fx0, fy0, fz0 = s["fx0"], s["fy0"], s["fz0"]
        sbx, sby, sbz = s["sbx"], s["sby"], s["sbz"]
        hx, hy, hz = self.hc
        ri = self.rud_idx
        rpx, rpy, rpz = self.flap_pt[0][ri], self.flap_pt[1][ri], self.flap_pt[2][ri]
        epx, epy, epz = self.ep
        FX = Fx.sum(1) + fx0.sum() + sbx.sum(1)
        FY = Fy.sum(1) + fy0.sum() + sby.sum(1)
        FZ = Fz.sum(1) + fz0.sum() + sbz.sum(1)
        MX = Fz @ ay - Fy @ az + cpl @ epx + (hy @ fz0 - hz @ fy0) + (sbz @ rpy - sby @ rpz)
        MY = Fx @ az - Fz @ ax + cpl @ epy + (hz @ fx0 - hx @ fz0) + (sbx @ rpz - sbz @ rpx)
        MZ = Fy @ ax - Fx @ ay + cpl @ epz + (hx @ fy0 - hy @ fx0) + (sby @ rpx - sbx @ rpy)

        bF, bM, bparts = self._body(vx, vy, vz, p, q, r, rho, a_snd, mu, M, qinf)
        gF = np.zeros(3)
        gM = np.zeros(3)
        gear_f = np.zeros((3, 3))
        chute_f = np.zeros(3)
        if gear_pos > 0.0:
            gpx, gpy, gpz = self.gear_pts
            gvx = vx + (q * gpz - r * gpy)
            gvy = vy + (r * gpx - p * gpz)
            gvz = vz + (p * gpy - q * gpx)
            gv = np.sqrt(gvx * gvx + gvy * gvy + gvz * gvz) + 1e-9
            k = -0.5 * rho * gv * self.gear_cda * gear_pos
            gear_f = np.stack([k * gvx, k * gvy, k * gvz], axis=1)
            gF = gear_f.sum(0)
            gM = np.cross(self.gear_pts.T, gear_f).sum(0)
        if chute > 0.0:
            cpt = self.chute_pt
            cvx = vx + (q * cpt[2] - r * cpt[1])
            cvy = vy + (r * cpt[0] - p * cpt[2])
            cvz = vz + (p * cpt[1] - q * cpt[0])
            cv = math.sqrt(cvx * cvx + cvy * cvy + cvz * cvz) + 1e-9
            k = -0.5 * rho * cv * self.chute_cds * chute
            chute_f = np.array([k * cvx, k * cvy, k * cvz])
            gF = gF + chute_f
            gM = gM + np.cross(cpt, chute_f)
        kmisc = -qinf * P.misc_cd0 * self.Sref / V                          # protuberances
        misc_f = np.array([kmisc * vx, kmisc * vy, kmisc * vz])
        F = np.stack([FX, FY, FZ], axis=1) + (bF + gF + misc_f)
        Mo = np.stack([MX, MY, MZ], axis=1) + (bM + gM)
        res = AeroResult(F=F, M=Mo, mach=M, qbar=qinf, alpha=alpha_b, beta=beta_b)
        if detail:
            cz = ctrl[0].copy()
            cz[:CH_SB + 1] = 0.0
            z = self._strips(vx, vy, vz, p, q, r, rho, a_snd, mu, h_cg, up_b, cz[None, :], M, alpha_b, wsup)
            self._detail(res, ctrl[0], s, z, bparts, gear_f, chute_f, misc_f, rho, V, M, wsup)
        return res

    # ------------------------------------------------------------------ body
    def _body(self, vx, vy, vz, p, q, r, rho, a_snd, mu, M, qinf):
        P = self.P
        B = self.B
        bp = self.bpts
        # boundary point velocities (body moving through air)
        bvx = vx + (q * bp[2] - r * bp[1])
        bvy = vy + (r * bp[0] - p * bp[2])
        bvz = vz + (p * bp[1] - q * bp[0])
        f, a = B.fore, B.aft
        U = 0.5 * (bvx[f] + bvx[a])
        wsup = _ss(0.85, 1.30, M)
        kup = 1.0 + (P.k_upwash - 1.0) * (1.0 - wsup)
        munk_y = P.k_munk * B.contract
        munk_z = P.k_munk * np.where(B.contract < 1.0, B.contract, kup * B.is_fus + (1.0 - B.is_fus))
        Fz = -rho * U * (B.Az[a] * bvz[a] - B.Az[f] * bvz[f]) * munk_z
        Fy = -rho * U * (B.Ay[a] * bvy[a] - B.Ay[f] * bvy[f]) * munk_y
        # viscous crossflow at segment centres
        sp = self.spos
        svx = vx + (q * sp[2] - r * sp[1])
        svy = vy + (r * sp[0] - p * sp[2])
        svz = vz + (p * sp[1] - q * sp[0])
        vc = np.sqrt(svy * svy + svz * svz)
        mc = vc / a_snd
        cdf = 1.0 + 0.6 * smoothstep(0.4, 1.0, mc) - 0.1 * smoothstep(1.0, 2.0, mc)
        kk = -0.5 * rho * P.eta_cf * cdf * vc * B.dx
        Fy = Fy + kk * P.cdc_y * B.height * svy
        Fz = Fz + kk * P.cdc_z * B.width * svz
        # skin friction (axial)
        aU = np.abs(svx) + 1e-6
        Re = rho * aU * B.xdist / mu
        cfr = self.cf_turb(Re, aU / a_snd) * P.rough
        Fx = -0.5 * rho * svx * aU * cfr * B.perim * B.dx
        # forebody wave drag
        Fx = Fx - qinf * self.nose_cd(M) * B.amax * B.nose_w * np.sign(vx)
        # base drag (fuselage + pods)
        cpb = self.base_cp(M) * P.base_scale
        fbase = cpb * qinf * (B.base_area + B.pod_base_area) * (1.0 if vx >= 0 else -1.0)
        bpos = B.base_pos
        Fsum = np.array([Fx.sum() + fbase, Fy.sum(), Fz.sum()])
        Mx = (sp[1] * Fz - sp[2] * Fy).sum()
        My = (sp[2] * Fx - sp[0] * Fz).sum() + bpos[2] * fbase
        Mz = (sp[0] * Fy - sp[1] * Fx).sum() - bpos[1] * fbase
        return Fsum, np.array([Mx, My, Mz]), (Fx, Fy, Fz, fbase)

    # ------------------------------------------------------------------ detail
    def _empty_detail(self, res):
        res.elem_f = np.zeros((self.n_elements, 3))
        res.comp = {g: [0.0] * 6 for g in GROUPS}
        res.coef = {k: 0.0 for k in ("CL", "CD", "CY", "Cl", "Cm", "Cn")}
        res.hinge = {k: 0.0 for k in ("elevon_li", "elevon_lo", "elevon_ri", "elevon_ro",
                                       "bodyflap", "rudder", "speedbrake")}

    def _detail(self, res, c0, s, z, bparts, gear_f, chute_f, misc_f, rho, V, M, wsup):
        """Per-element forces (flap increments split out), component breakdown, coefficients,
        hinge moments. ``s`` = strip arrays for the actual controls (row 0), ``z`` = same with
        all flaps at zero (basic wing)."""
        ef = np.zeros((self.n_elements, 3))
        s0, s1 = self.sl["lift"]
        zx, zy, zz = z["Fx"][0], z["Fy"][0], z["Fz"][0]
        ef[s0:s1, 0] = zx + s["fx0"]
        ef[s0:s1, 1] = zy + s["fy0"]
        ef[s0:s1, 2] = zz + s["fz0"]
        f0, f1 = self.sl["flap"]
        fi = self.flap_idx
        ef[f0:f1, 0] = (s["Fx"][0] - zx)[fi]
        ef[f0:f1, 1] = (s["Fy"][0] - zy)[fi]
        ef[f0:f1, 2] = (s["Fz"][0] - zz)[fi]
        rpos = {int(i): k for k, i in enumerate(self.rud_idx)}
        for k, i in enumerate(fi):
            j = rpos.get(int(i))
            if j is not None:
                ef[f0 + k, 0] += s["sbx"][0, j]
                ef[f0 + k, 1] += s["sby"][0, j]
                ef[f0 + k, 2] += s["sbz"][0, j]
        b0, b1 = self.sl["body"]
        bFx, bFy, bFz, fbase = bparts
        ef[b0:b1, 0] = bFx
        ef[b0:b1, 1] = bFy
        ef[b0:b1, 2] = bFz
        ba = self.sl["base"][0]
        ef[ba] = misc_f
        ef[ba, 0] += fbase
        g0 = self.sl["gear"][0]
        ef[g0:g0 + 3] = gear_f
        ef[self.sl["chute"][0]] = chute_f
        res.elem_f = ef

        # component breakdown (forces + moments about CG)
        if not hasattr(self, "_el_pos"):
            self._el_pos = np.array([e["pos"] for e in self.geo.elements], dtype=float)
            self._el_grp = np.array([G[e["group"]] for e in self.geo.elements])
        pos = self._el_pos.copy()
        pos[s0:s1, 0] = s["ax"]
        mom = np.cross(pos, ef)
        fr = np.stack([s["fx0"], s["fy0"], s["fz0"]], 1)
        mom[s0:s1] += np.cross(self.hc.T - pos[s0:s1], fr)      # friction acts at mid chord
        ep = self.ep.T
        cpl, cplz = s["cpl"][0], z["cpl"][0]
        comp = np.zeros((len(GROUPS), 6))
        np.add.at(comp, self._el_grp, np.concatenate([ef, mom], axis=1))
        np.add.at(comp[:, 3:], self.group, cplz[:, None] * ep)
        fgrp = self.geo.lift.flap_group
        tgt = np.where(fgrp >= 0, fgrp, self.group)
        np.add.at(comp[:, 3:], tgt, (cpl - cplz)[:, None] * ep)
        res.comp = {GROUPS[i]: [float(x) for x in comp[i]] for i in range(len(GROUPS))}

        # coefficients
        F, Mo = res.F[0], res.M[0]
        qS = res.qbar * self.Sref
        al, be = res.alpha, res.beta
        ca, sa, cb, sb = math.cos(al), math.sin(al), math.cos(be), math.sin(be)
        drag = -(F[0] * ca * cb + F[1] * sb + F[2] * sa * cb)
        side = -F[0] * ca * sb + F[1] * cb - F[2] * sa * sb
        lift = F[0] * sa - F[2] * ca
        res.lift, res.drag, res.side = lift, drag, side
        res.coef = {"CL": lift / qS, "CD": drag / qS, "CY": side / qS,
                    "Cl": Mo[0] / (qS * self.bref), "Cm": Mo[1] / (qS * self.cref),
                    "Cn": Mo[2] / (qS * self.bref)}
        res.xcp = float(-Mo[1] / F[2]) if abs(F[2]) > 1e-3 * qS else 0.0

        # hinge moments [N m], + = trailing edge down (elevons, body flap) / left (rudder)
        beff = max(math.sqrt(max(M * M - 1.0, 0.0)), 0.5)
        ap, qp = s["a_perp"], s["qp"]
        Lf = self.geo.lift
        hinge = {}
        for ch, key in ((CH_LI, "elevon_li"), (CH_LO, "elevon_lo"), (CH_RI, "elevon_ri"),
                        (CH_RO, "elevon_ro"), (CH_RUD, "rudder")):
            cov = Lf.fmap[:, ch]
            m = cov > 0
            dl = c0[ch]
            chh = (-0.25 * ap[m] - 0.55 * dl) * (1.0 - wsup) + (-2.0 / beff * (ap[m] + dl)) * wsup
            Sf = self.cf[m] * self.S[m] * cov[m]
            hinge[key] = float((qp[m] * Sf * self.cf[m] * self.chord[m] * chh).sum())
        bfm = self.is_bf
        Nbf = qp[bfm] * self.S[bfm] * s["cN"][0][bfm]
        xapp = s["xapp"] if np.ndim(s["xapp"]) == 0 else s["xapp"][bfm]
        hinge["bodyflap"] = float(-(Nbf * self.chord[bfm] * xapp).sum())
        ri = self.rud_idx
        qloc = 0.5 * rho * V * V * s["eta_v"]
        hinge["speedbrake"] = float(-(qloc * self.rud_area * 1.2 * math.sin(0.5 * c0[CH_SB])
                                      * 0.5 * self.cf[ri] * self.chord[ri]).sum())
        res.hinge = hinge
