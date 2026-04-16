# maneuvers/spiral.py
"""
Spiral Maneuver (Dieudonné) — with optional closed-loop PID variant.

Protocol (Pakkan 2007, pp.122-126 / ITTC 7.5-04-02-01 §4.4):
    1. Optional straight pre-run to cruise speed (like thesis: ship travels
       ~200 s at full ahead before spiral starts).
    2. Apply starting rudder angle delta_start.
    3. Hold until steady yaw rate achieved (t_settle seconds).
    4. Step rudder in increments to delta_end, then back to delta_start.
    5. Plot r(δ) — hysteresis loop = unstable, single-valued = stable.

Trajectory note (matching thesis Figure 36):
    Use t_settle ≈ 83 s (~1 full circle at 20° rudder) for a trajectory that
    resembles the thesis. Longer t_settle creates more circles per cluster.
    t_settle = 150 s is needed for accurate r_ss measurement.
    Use t_settle = 83 s if the trajectory appearance is the priority.

Reference:
    Pakkan (2007) pp.122-126; ITTC 7.5-04-02-01 §4.4;
    Dieudonné, J. (1953). ATMA Bulletin.
"""

import numpy as np
from scipy.integrate import solve_ivp
from control.pid import PIDController


def run_spiral(ship, init_state, n_cmd=95.0,
               delta_start=20.0, delta_end=-20.0, delta_step=5.0,
               t_settle=150.0, dt=0.1,
               t_straight=0.0,
               use_pid=False, Kp=20.0, Ki=0.3, Kd=3.0, delta_max=35.0):
    """
    Simulate a Dieudonné spiral maneuver.

    Args:
        ship          ShipModel instance
        init_state    14-element initial state (ideally at cruise speed)
        n_cmd         [RPM]   Propeller command (held constant)
        delta_start   [deg]   Starting rudder/heading (+20° = starboard)
        delta_end     [deg]   End rudder/heading (-20° = port)
        delta_step    [deg]   Step size (positive; direction inferred)
        t_settle      [s]     Settle time per step.
                              150 s: accurate r_ss measurement.
                              83 s: ~1 circle per step, matches thesis Figure 36.
        dt            [s]     Integration step
        t_straight    [s]     Optional straight pre-run before spiral starts.
                              Set > 0 to include the initial straight path in
                              the returned trajectory (matches thesis Fig.36 which
                              shows the ship travelling ~200 s before spiral begins).
        use_pid       bool    If True: PID heading-step variant
        Kp, Ki, Kd    PID gains
        delta_max     [deg]   Rudder saturation for PID

    Returns:
        t_full, y_full, delta_full  — full time history (incl. pre-run if t_straight>0)
        spiral_delta, spiral_r      — (step value, settled yaw rate) pairs
        results                     — stability verdict dict
    """
    pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd, delta_max=delta_max) if use_pid else None
    if pid: pid.reset()

    state = np.array(init_state, dtype=float)
    t     = 0.0

    t_full=[t]; y_full=[state.copy()]; delta_full=[0.0]

    # ── Optional straight pre-run ─────────────────────────────────────────
    if t_straight > 0:
        t_pre_end = t_straight
        while t < t_pre_end:
            sol = solve_ivp(ship.dynamics, (t, t + dt), state,
                            args=(n_cmd, 0.0), method='RK45', max_step=dt)
            state = sol.y[:, -1]; t += dt
            t_full.append(t); y_full.append(state.copy()); delta_full.append(0.0)

    # ── Build step sequence (descending then ascending) ───────────────────
    seq_down = list(np.arange(delta_start,  delta_end - 1e-9, -abs(delta_step)))
    seq_up   = list(np.arange(delta_end,    delta_start + 1e-9,  abs(delta_step)))
    step_seq = seq_down + seq_up

    delta_full[-1] = step_seq[0]   # update first rudder value

    spiral_delta = []
    spiral_r     = []

    for step_val in step_seq:
        t_step_start = t
        if use_pid:
            pid.reset()
            psi_target = np.deg2rad(step_val)

        while t < t_step_start + t_settle:
            if use_pid:
                psi_w     = (state[11] + np.pi) % (2 * np.pi) - np.pi
                delta_cmd = pid.compute(t, psi_w, psi_target)
            else:
                delta_cmd = float(step_val)

            sol   = solve_ivp(ship.dynamics, (t, t + dt), state,
                              args=(n_cmd, delta_cmd), method='RK45', max_step=dt)
            state = sol.y[:, -1]; t += dt
            t_full.append(t); y_full.append(state.copy()); delta_full.append(delta_cmd)

        # Record settled yaw rate (average of last 20 s)
        n_pts    = int(20.0 / dt)
        r_settled = np.mean(
            np.rad2deg([y_full[-i][5] for i in range(1, min(n_pts + 1, len(y_full)))])
        )
        spiral_delta.append(step_val)
        spiral_r.append(r_settled)

    t_full     = np.array(t_full)
    y_full     = np.array(y_full).T
    delta_full = np.array(delta_full)
    spiral_delta = np.array(spiral_delta)
    spiral_r     = np.array(spiral_r)

    # ── Stability analysis ────────────────────────────────────────────────
    n_half    = len(seq_down)
    r_descend = spiral_r[:n_half]
    r_ascend  = spiral_r[n_half:]
    d_descend = spiral_delta[:n_half]
    d_ascend  = spiral_delta[n_half:]

    d_common       = set(np.round(d_descend, 1)) & set(np.round(d_ascend, 1))
    max_hysteresis = 0.0
    for dc in d_common:
        idx_d = np.argmin(np.abs(d_descend - dc))
        idx_a = np.argmin(np.abs(d_ascend  - dc))
        max_hysteresis = max(max_hysteresis, abs(r_descend[idx_d] - r_ascend[idx_a]))

    zc = np.where(np.diff(np.sign(r_descend)))[0]
    if len(zc):
        d0, d1 = d_descend[zc[0]], d_descend[zc[0] + 1]
        r0, r1 = r_descend[zc[0]], r_descend[zc[0] + 1]
        delta_at_r0 = d0 - r0 * (d1 - d0) / (r1 - r0)
    else:
        delta_at_r0 = None

    stable = max_hysteresis < 0.05
    results = {
        'spiral_delta'   : spiral_delta,
        'spiral_r'       : spiral_r,
        'r_descend'      : r_descend,
        'r_ascend'       : r_ascend,
        'd_descend'      : d_descend,
        'd_ascend'       : d_ascend,
        'max_hysteresis' : max_hysteresis,
        'delta_at_r0'    : delta_at_r0,
        'stable'         : stable,
        'verdict'        : ('DIRECTIONALLY STABLE' if stable
                            else 'DIRECTIONALLY UNSTABLE — hysteresis loop present'),
        'mode'           : 'closed-loop PID heading steps' if use_pid
                           else 'open-loop rudder steps',
    }
    return t_full, y_full, delta_full, spiral_delta, spiral_r, results