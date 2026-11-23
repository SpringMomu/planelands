"""6-DOF rigid-body dynamics (flat earth, ENU world, FRD body) with RK4 integration.

State vector x (13): [E, N, U, vE, vN, vU, qw, qx, qy, qz, p, q, r]
q rotates body-FRD vectors into ENU:  v_ENU = R(q) v_FRD.
"""
from __future__ import annotations

import math

import numpy as np

from .atmosphere import atmosphere, gravity

# ENU <-> NED permutation (symmetric, its own inverse)
M_ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    if q[0] < 0:
        q = -q
    return q / np.linalg.norm(q)


def euler_ned_to_R(psi, theta, phi):
    """Body FRD -> ENU rotation from aviation Euler angles (3-2-1, relative to NED)."""
    cps, sps = math.cos(psi), math.sin(psi)
    cth, sth = math.cos(theta), math.sin(theta)
    cph, sph = math.cos(phi), math.sin(phi)
    R_nb = np.array([
        [cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps],
        [cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps],
        [-sth, sph * cth, cph * cth],
    ])
    return M_ENU_NED @ R_nb


def R_to_euler_ned(R):
    R_nb = M_ENU_NED @ R
    theta = -math.asin(max(-1.0, min(1.0, R_nb[2, 0])))
    phi = math.atan2(R_nb[2, 1], R_nb[2, 2])
    psi = math.atan2(R_nb[1, 0], R_nb[0, 0])
    return psi, theta, phi


class Context:
    """Quantities held constant over one integration step."""
    __slots__ = ("ctrl", "gear_pos", "chute", "wind_w", "gust_b", "brakes", "steer",
                 "frozen_aero", "gear_active")

    def __init__(self):
        self.ctrl = np.zeros(7)
        self.gear_pos = 0.0
        self.chute = 0.0
        self.wind_w = np.zeros(3)
        self.gust_b = np.zeros(3)
        self.brakes = 0.0
        self.steer = 0.0
        self.frozen_aero = None      # (F, M) to reuse in all stages (ground sub-steps)
        self.gear_active = True


class RigidBody:
    def __init__(self, geo, aero, gear, terrain):
        self.geo = geo
        self.aero = aero
        self.gear = gear
        self.terrain = terrain
        self.m = geo.mass
        self.J = geo.J
        self.Jinv = geo.Jinv

    def air_data(self, x, ctx):
        """Body air-relative velocity, atmosphere and AGL for state x."""
        R = quat_to_R(x[6:10])
        T, p, rho, a, mu = atmosphere(x[2])
        hg = self.terrain.height(x[0], x[1])
        wind = ctx.wind_w + R @ ctx.gust_b
        v_air = R.T @ (x[3:6] - wind)
        return R, rho, a, mu, x[2] - hg, v_air

    def aero_eval(self, x, ctx, detail=False, ctrl=None):
        R, rho, a, mu, agl, v_air = self.air_data(x, ctx)
        return self.aero.evaluate(v_air, x[10:13], rho, a, mu, agl, R[2, :],
                                  ctx.ctrl if ctrl is None else ctrl,
                                  gear_pos=ctx.gear_pos, chute=ctx.chute, detail=detail), R, rho, a, agl, v_air

    def derivs(self, x, ctx, record=False):
        q = x[6:10]
        R = quat_to_R(q)
        w = x[10:13]
        if ctx.frozen_aero is not None:
            Fa, Ma = ctx.frozen_aero
        else:
            T, p, rho, a, mu = atmosphere(x[2])
            hg = self.terrain.height(x[0], x[1])
            wind = ctx.wind_w + R @ ctx.gust_b
            v_air = R.T @ (x[3:6] - wind)
            res = self.aero.evaluate(v_air, w, rho, a, mu, x[2] - hg, R[2, :], ctx.ctrl,
                                     gear_pos=ctx.gear_pos, chute=ctx.chute)
            Fa, Ma = res.F[0], res.M[0]
        if ctx.gear_active:
            Fg, Mg = self.gear.forces(x[0:3], x[3:6], R, w, self.terrain.height,
                                      brakes=ctx.brakes, steer=ctx.steer, record=record)
        else:
            Fg = Mg = np.zeros(3)
        Fb = Fa + Fg
        Mb = Ma + Mg
        acc = (R @ Fb) / self.m
        acc[2] -= gravity(x[2])
        qw, qx, qy, qz = q
        p_, q_, r_ = w
        qdot = 0.5 * np.array([-qx * p_ - qy * q_ - qz * r_,
                               qw * p_ + qy * r_ - qz * q_,
                               qw * q_ + qz * p_ - qx * r_,
                               qw * r_ + qx * q_ - qy * p_])
        Jw = self.J @ w
        wxJw = np.array([w[1] * Jw[2] - w[2] * Jw[1], w[2] * Jw[0] - w[0] * Jw[2], w[0] * Jw[1] - w[1] * Jw[0]])
        wdot = self.Jinv @ (Mb - wxJw)
        dx = np.empty(13)
        dx[0:3] = x[3:6]
        dx[3:6] = acc
        dx[6:10] = qdot
        dx[10:13] = wdot
        return dx, Fb, Mb

    def rk4(self, x, dt, ctx):
        k1, Fb, Mb = self.derivs(x, ctx, record=True)
        k2, _, _ = self.derivs(x + 0.5 * dt * k1, ctx)
        k3, _, _ = self.derivs(x + 0.5 * dt * k2, ctx)
        k4, _, _ = self.derivs(x + dt * k3, ctx)
        xn = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        qn = xn[6:10]
        xn[6:10] = qn / np.linalg.norm(qn)
        return xn, Fb, Mb
