# maneuvers/stopping_trial.py
"""
Stopping Trial — with optional PID heading-hold during deceleration.

Protocol (Pakkan 2007, p.119 / IMO MSC.137(76) para 3.3):
    Full ahead for t_buildup seconds, then full astern until stop.

Open-loop:   zero rudder throughout (thesis protocol).
Closed-loop: PID holds heading = psi_at_reversal throughout, preventing
             the ship from veering off course during the crash-stop.
             This is a realistic operational scenario (helmsman holds course).

Reference:
    Pakkan (2007) p.119; IMO MSC.137(76) para 3.3; ITTC 7.5-04-02-01.
"""

import numpy as np
from scipy.integrate import solve_ivp
from control.pid import PIDController


def run_stopping_trial(ship, init_state, n_full=160.0, t_buildup=100.0,
                       t_end=300.0, dt=0.1,
                       use_pid=False, Kp=20.0, Ki=0.3, Kd=3.0, delta_max=35.0):
    """
    Simulate a full-ahead to full-astern stopping trial.

    Args:
        ship         ShipModel instance
        init_state   14-element initial state
        n_full       [RPM]  Full-ahead shaft speed (thesis: n_max = 160 RPM)
        t_buildup    [s]    Time at full-ahead before reversal (thesis: 100 s)
        t_end        [s]    Total simulation time
        dt           [s]    Integration step (thesis: ≤ 0.3 s for shaft stability)
        use_pid      bool   If True, PID holds heading during crash-stop
        Kp, Ki, Kd   PID gains
        delta_max    [deg]  Rudder saturation limit for PID

    Returns:
        t, y, n_cmd_arr, results
    """
    pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd, delta_max=delta_max) if use_pid else None
    if pid: pid.reset()

    state = np.array(init_state, dtype=float)
    t     = 0.0
    psi_hold = None     # heading to hold (set at reversal point)

    t_arr=[t]; y_arr=[state.copy()]; n_cmd_arr=[n_full]; d_arr=[0.0]
    t_reversal=None; x_reversal=None; t_stop=None; reversal_done=False

    while t < t_end:
        n_cmd = n_full if t < t_buildup else -n_full

        if not reversal_done and t >= t_buildup:
            t_reversal   = t
            x_reversal   = state[6]
            reversal_done = True
            if use_pid:
                psi_hold = (state[11] + np.pi) % (2 * np.pi) - np.pi

        # Rudder: PID holds heading after reversal, else zero
        if use_pid and psi_hold is not None:
            psi_w     = (state[11] + np.pi) % (2 * np.pi) - np.pi
            delta_cmd = pid.compute(t, psi_w, psi_hold)
        else:
            delta_cmd = 0.0

        sol   = solve_ivp(ship.dynamics, (t, t + dt), state,
                          args=(n_cmd, delta_cmd), method='RK45', max_step=dt)
        state = sol.y[:, -1]
        t    += dt

        if reversal_done and t_stop is None and state[0] <= 0.0:
            t_stop = t

        t_arr.append(t); y_arr.append(state.copy())
        n_cmd_arr.append(n_cmd); d_arr.append(delta_cmd)

        if t_stop is not None and t > t_stop + 20:
            break

    t_arr     = np.array(t_arr)
    y_arr     = np.array(y_arr).T
    n_cmd_arr = np.array(n_cmd_arr)
    d_arr     = np.array(d_arr)

    L = ship.m.L
    head_reach = time_to_stop = None
    if t_stop is not None and x_reversal is not None:
        idx_stop  = np.searchsorted(t_arr, t_stop)
        head_reach   = abs(y_arr[6, idx_stop] - x_reversal)
        time_to_stop = t_stop - t_reversal

    imo_limit = 15.0 * L
    results = {
        't_reversal'   : t_reversal,
        'x_reversal'   : x_reversal,
        't_stop'       : t_stop,
        'head_reach'   : head_reach,
        'head_reach_L' : head_reach / L if head_reach else None,
        'time_to_stop' : time_to_stop,
        'imo_limit_m'  : imo_limit,
        'imo_pass'     : (head_reach is not None) and (head_reach <= imo_limit),
        'mode'         : 'closed-loop PID' if use_pid else 'open-loop',
    }
    return t_arr, y_arr, n_cmd_arr, d_arr, results