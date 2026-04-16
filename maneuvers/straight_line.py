# maneuvers/straight_line.py
"""
Straight-Line Run — with optional PID course-keeping autopilot.

Open-loop:   zero rudder command throughout, ship follows its natural path.
Closed-loop: PID holds heading at psi=0 (or any desired heading), correcting
             any coupling-induced drift with continuous rudder modulation.
"""

import numpy as np
from scipy.integrate import solve_ivp
from control.pid import PIDController


def run_straight_line(ship, init_state, n_cmd=95.0, t_end=300,
                      use_pid=False, psi_desired=0.0,
                      Kp=20.0, Ki=0.3, Kd=3.0, delta_max=35.0):
    """
    Run a straight-line acceleration.

    Args:
        ship          ShipModel instance
        init_state    14-element initial state
        n_cmd         [RPM]  Propeller command
        t_end         [s]    Simulation duration
        use_pid       bool   If True, PID holds psi_desired heading
        psi_desired   [deg]  Target heading for PID (default: 0°)
        Kp, Ki, Kd    PID gains  (see control/pid.py)
        delta_max     [deg]  Rudder saturation limit for PID

    Returns:
        sol           scipy OdeSolution  (open-loop)
        OR
        (t, y, d)     arrays             (closed-loop — t [s], y [14×N], d [deg])
    """
    if not use_pid:
        t_span = (0, t_end)
        t_eval = np.linspace(0, t_end, max(2000, int(t_end * 10)))
        return solve_ivp(ship.dynamics, t_span, init_state,
                         args=(n_cmd, 0.0), t_eval=t_eval)

    # ── Closed-loop: PID course-keeping ──────────────────────────────────
    pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd, delta_max=delta_max)
    pid.reset()
    psi_ref = np.deg2rad(psi_desired)

    state = np.array(init_state, dtype=float)
    t     = 0.0
    dt    = 0.05

    t_arr = [t]; y_arr = [state.copy()]; d_arr = [0.0]

    while t < t_end:
        psi_w     = (state[11] + np.pi) % (2 * np.pi) - np.pi
        delta_cmd = pid.compute(t, psi_w, psi_ref)

        sol = solve_ivp(ship.dynamics, (t, t + dt), state,
                        args=(n_cmd, delta_cmd), method='RK45', max_step=dt)
        state = sol.y[:, -1]
        t    += dt

        t_arr.append(t); y_arr.append(state.copy()); d_arr.append(delta_cmd)

    return np.array(t_arr), np.array(y_arr).T, np.array(d_arr)