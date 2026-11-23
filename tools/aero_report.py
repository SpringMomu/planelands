"""Aerodynamic calibration report for the Orbiter component build-up model.

    python tools/aero_report.py            # full report
    python tools/aero_report.py --quick    # subsonic + 2 supersonic Mach numbers only
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.aero import AeroModel  # noqa: E402
from sim.geometry import CH_LI, CH_LO, CH_RI, CH_RO, CH_RUD, CH_SB, build_geometry  # noqa: E402
from sim.trim import coeffs, flight_condition, sym_ctrl, trim_elevon, trim_scheduled  # noqa: E402

R = math.radians


def report(am: AeroModel, machs, gear=0.0):
    cbar = am.cref
    print("=" * 112)
    print(" Orbiter aero report (gear %s, speedbrake closed, CG x=0)" % ("down" if gear else "up"))
    print("=" * 112)
    summary = []
    for M in machs:
        cond = flight_condition(M)
        V, rho, a, mu, h = cond
        print(f"\nMach {M:.2f}   (h={h/1000:5.1f} km, V={V:6.1f} m/s, qbar=10 kPa unless clamped)")
        print("  alpha   CL      CD      L/D    Cm(d=0)   de_trim(bf=0)  [de,bf]_sched  CL_tr   CD_tr  (L/D)_tr")
        best = (0, None)
        alphas = np.arange(-2, 31, 2.0)
        cls, cms = [], []
        for ad in alphas:
            c = coeffs(am, M, R(ad), gear=gear, cond=cond)
            de0 = trim_elevon(am, M, R(ad), 0.0, 0.0, gear, cond)
            ts = trim_scheduled(am, M, R(ad), 0.0, gear, cond)
            cls.append(c["CL"][0])
            cms.append(c["Cm"][0])
            if ts:
                de, bf, ct = ts
                ld = ct["CL"] / ct["CD"]
                if ld > best[0]:
                    best = (ld, ad)
                sched = f"[{math.degrees(de):5.1f},{math.degrees(bf):5.1f}]"
                trs = f"{ct['CL']:6.3f} {ct['CD']:7.4f} {ld:6.2f}"
            else:
                sched, trs = "   untrimmable", ""
            des = f"{math.degrees(de0):6.1f}" if de0 is not None else "   n/a"
            print(f"  {ad:5.1f} {c['CL'][0]:7.3f} {c['CD'][0]:7.4f} {c['CL'][0]/max(c['CD'][0],1e-6):6.2f}"
                  f" {c['Cm'][0]:8.4f}      {des}       {sched}  {trs}")
        # derivatives at alpha=8 (subsonic) / alpha=10
        a0 = R(8.0 if M < 1 else 10.0)
        d = R(1.0)
        c = coeffs(am, M, a0, gear=gear, cond=cond)
        cp = coeffs(am, M, a0 + d, gear=gear, cond=cond)
        CLa = (cp["CL"][0] - c["CL"][0]) / 1.0
        Cma = (cp["Cm"][0] - c["Cm"][0]) / 1.0
        sm = -Cma / CLa * 100.0
        ce = coeffs(am, M, a0, ctrl=sym_ctrl(np.array([0.0, d])), gear=gear, cond=cond)
        Cmde = ce["Cm"][1] - ce["Cm"][0]
        CLde = ce["CL"][1] - ce["CL"][0]
        cb = coeffs(am, M, a0, beta=R(1.0), gear=gear, cond=cond)
        Cnb = cb["Cn"][0] - c["Cn"][0]
        Clb = cb["Cl"][0] - c["Cl"][0]
        CYb = cb["CY"][0] - c["CY"][0]
        ctrl = np.zeros((4, 7))
        ctrl[1, CH_LI] = ctrl[1, CH_LO] = d
        ctrl[1, CH_RI] = ctrl[1, CH_RO] = -d           # +aileron: right roll
        ctrl[2, CH_RUD] = d
        ctrl[3, CH_SB] = R(98.6)
        cc = coeffs(am, M, a0, ctrl=ctrl, gear=gear, cond=cond)
        Clda = cc["Cl"][1] - cc["Cl"][0]
        Cnda = cc["Cn"][1] - cc["Cn"][0]
        Cndr = cc["Cn"][2] - cc["Cn"][0]
        Cldr = cc["Cl"][2] - cc["Cl"][0]
        dCDsb = cc["CD"][3] - cc["CD"][0]
        dCmsb = cc["Cm"][3] - cc["Cm"][0]
        # pitch damping
        qh = 0.02
        cq = coeffs(am, M, a0, rates=(0.0, qh * 2 * V / cbar, 0.0), gear=gear, cond=cond)
        Cmq = (cq["Cm"][0] - c["Cm"][0]) / qh
        cpr = coeffs(am, M, a0, rates=(qh * 2 * V / am.bref, 0.0, 0.0), gear=gear, cond=cond)
        Clp = (cpr["Cl"][0] - c["Cl"][0]) / qh
        crr = coeffs(am, M, a0, rates=(0.0, 0.0, qh * 2 * V / am.bref), gear=gear, cond=cond)
        Cnr = (crr["Cn"][0] - c["Cn"][0]) / qh
        print(f"  @alpha={math.degrees(a0):.0f}: CLa={CLa:.4f}/deg  Cma={Cma:+.5f}/deg  static margin={sm:+.1f}% cbar"
              f"  Cm_de={Cmde:+.5f}/deg  CL_de={CLde:+.5f}/deg")
        print(f"            Cn_beta={Cnb:+.5f}/deg  Cl_beta={Clb:+.5f}/deg  CY_beta={CYb:+.5f}/deg"
              f"  Cl_da={Clda:+.5f}  Cn_da={Cnda:+.5f}  Cn_dr={Cndr:+.5f}  Cl_dr={Cldr:+.5f} /deg")
        print(f"            Cmq={Cmq:+.3f}  Clp={Clp:+.3f}  Cnr={Cnr:+.3f}   dCD(sb 98.6)={dCDsb:+.4f}  dCm(sb)={dCmsb:+.4f}"
              f"   (L/D)max trimmed = {best[0]:.2f} @ alpha {best[1]}")
        summary.append((M, CLa, sm, best[0], best[1], Cmde, Cnb, Clb, dCDsb))
    if gear == 0.0:
        M = 0.4
        cond = flight_condition(M)
        cu = coeffs(am, M, R(8.0), cond=cond)
        cg = coeffs(am, M, R(8.0), gear=1.0, cond=cond)
        print(f"\n gear-down increment @M0.4 a8: dCD = {cg['CD'][0]-cu['CD'][0]:+.4f}   dCm = {cg['Cm'][0]-cu['Cm'][0]:+.4f}")
        for hh in (2.0, 5.0, 10.0, 30.0):
            v = flight_condition(0.3)
            V, rho, a, mu, h = v
            cge = coeffs(am, 0.3, R(8.0), cond=(V, 1.2, 340.0, 1.8e-5, 0.0), h_agl=hh + 4.9)
            cfa = coeffs(am, 0.3, R(8.0), cond=(V, 1.2, 340.0, 1.8e-5, 0.0))
            print(f" ground effect h_gear={hh:4.1f} m: dCL={cge['CL'][0]-cfa['CL'][0]:+.4f}"
                  f" dCD={cge['CD'][0]-cfa['CD'][0]:+.4f} dCm={cge['Cm'][0]-cfa['Cm'][0]:+.4f}")
    print("\n SUMMARY  Mach  CLa/deg  SM%cbar  (L/D)max@alpha   Cm_de/deg  Cn_b/deg  Cl_b/deg  dCD_sb")
    for s in summary:
        print(f"         {s[0]:4.2f}  {s[1]:.4f}  {s[2]:+6.1f}   {s[3]:5.2f} @ {s[4]}     {s[5]:+.5f}  {s[6]:+.5f}  {s[7]:+.5f}  {s[8]:+.4f}")


if __name__ == "__main__":
    geo = build_geometry()
    am = AeroModel(geo)
    quick = "--quick" in sys.argv
    machs = [0.3, 0.6, 1.5, 2.5] if quick else [0.3, 0.6, 0.9, 1.2, 1.5, 2.0, 2.5]
    report(am, machs)
