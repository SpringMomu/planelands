"""Autonomous GNC for the orbiter: SCP trajectory planning + LTV-MPC tracking + A/L.

Modules
    model      vectorised 3-DOF planning model (trimmed aero tables, US76, mean wind)
    geometry   runway ends, HAC / Dubins initial guess, reachability, runway selection
    approach   approach & landing reference (OGS, preflare, IGS, flare) and A/L laws
    scp        free-final-time successive convexification (PTR) to the A/L interface
    reference  dense reference path with spatial projection
    mpc        LTV model-predictive tracking controller (OSQP)
    planner    synchronous / worker-process planner front-end
    guidance_scpmpc  the "scp_mpc" guidance law (phase state machine)
"""
