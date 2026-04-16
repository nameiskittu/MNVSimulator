# maneuvers/pullout.py
"""
Pull-Out Maneuver — with optional PID course-keeping.

Protocol (Pakkan 2007, pp.120-122 / ITTC 7.5-04-02-01 §4.3):
    1. Straight cruise (PID holds psi=0 or open-loop zero rudder).
    2. Apply fixed rudder_deg — open-loop, same in both modes
       (the 20° rudder IS the test stimulus; a PID would mask it).
    3. Return rudder to zero. Observe r decay.
       Open-loop:   zero rudder, r decays naturally.
       Closed-loop: PID tries to steer back to the original heading.
                    Tests whether the autopilot can recover course —
                    meaningful operational scenario.

Reference:
    Pakkan (2007) pp.120-122; ITTC 7.5-04-02-01 §4.3.
"""

import numpy as np
from scipy.integrate import solve_ivp
from control.pid import PIDController


def run_pullout(ship, init_state, rudder_deg=20.0,
                t_straight=70.0, t_turn=200.0, t_coast=150.0, n_cmd=95.0,
                dt=0.05, use_pid=False,
                Kp=20.0, Ki=0.3, Kd=3.0, delta_max=35.0):
    """
    Simulate a pull-out maneuver.

    Args:
        ship          ShipModel instance
        init_state    14-element initial state
        rudder_deg    [deg]  Rudder during turn phase (thesis: 20°)
        t_straight    [s]    Straight-line buildup (thesis: 70 s)
        t_turn        [s]    Turn duration (thesis: 200 s)
        t_coast       [s]    Coast/recovery observation time
        n_cmd         [RPM]  Propeller command
        dt            [s]    Integration step
        use_pid       bool   If True: PID course-keeps in straight + coast phases
        Kp, Ki, Kd    PID gains
        delta_max     [deg]  Rudder saturation limit for PID

    Returns:
        t, y, delta_arr, results
    """
    pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd, delta_max=delta_max) if use_pid else None
    if pid: pid.reset()

    state = np.array(init_state, dtype=float)
    t     = 0.0
    t_end = t_straight + t_turn + t_coast
    t_rudder_in  = t_straight
    t_rudder_out = t_straight + t_turn

    # PID targets
    psi_straight = 0.0                       # hold psi=0 during buildup
    psi_recovery = None                      # set after pullout (current heading)

    t_arr=[t]; y_arr=[state.copy()]; d_arr=[0.0]

    while t < t_end:
        # ── Phase logic ────────────────────────────────────────────────
        if t < t_rudder_in:
            # Straight buildup
            if use_pid:
                psi_w = (state[11]+np.pi)%(2*np.pi)-np.pi
                delta_cmd = pid.compute(t, psi_w, psi_straight)
            else:
                delta_cmd = 0.0

        elif t < t_rudder_out:
            # Turn: always open-loop fixed rudder (the test stimulus)
            delta_cmd = rudder_deg
            if use_pid and t == t_rudder_in:
                pid.reset()   # clear integral accumulated during straight

        else:
            # Coast / recovery
            if psi_recovery is None:
                psi_recovery = (state[11]+np.pi)%(2*np.pi)-np.pi
                if use_pid: pid.reset()
            if use_pid:
                # PID tries to recover to the heading at pullout moment
                psi_w = (state[11]+np.pi)%(2*np.pi)-np.pi
                delta_cmd = pid.compute(t, psi_w, psi_recovery)
            else:
                delta_cmd = 0.0

        sol   = solve_ivp(ship.dynamics, (t, t+dt), state,
                          args=(n_cmd, delta_cmd), method='RK45', max_step=dt)
        state = sol.y[:, -1]
        t    += dt
        t_arr.append(t); y_arr.append(state.copy()); d_arr.append(delta_cmd)

    t_arr     = np.array(t_arr)
    y_arr     = np.array(y_arr).T
    d_arr     = np.array(d_arr)

    # Metrics
    idx_ss_start = np.searchsorted(t_arr, t_rudder_out - 20.0)
    idx_ss_end   = np.searchsorted(t_arr, t_rudder_out)
    r_steady     = np.mean(np.rad2deg(y_arr[5, idx_ss_start:idx_ss_end]))
    r_final      = np.rad2deg(y_arr[5, -1])
    stable       = abs(r_final) < 0.05

    results = {
        't_rudder_in'  : t_rudder_in,
        't_rudder_out' : t_rudder_out,
        'r_steady_ss'  : r_steady,
        'r_final'      : r_final,
        'stable'       : stable,
        'verdict'      : 'DIRECTIONALLY STABLE' if stable else 'DIRECTIONALLY UNSTABLE',
        'mode'         : 'closed-loop PID' if use_pid else 'open-loop',
    }
    return t_arr, y_arr, d_arr, results