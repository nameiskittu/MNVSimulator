import numpy as np
from scipy.integrate import solve_ivp


def run_turning_circle(ship, init_state, n_cmd=95.0, rudder_deg=18.0):
    """
    Simulates a turning circle maneuver (2-phase):

    Phase 1: Straight run until 200 m
    Phase 2: Apply constant rudder

    Returns:
        dict with time histories and metrics
    """

    # -------------------------------
    # PHASE 1: Straight run
    # -------------------------------
    t_straight = 300.0
    dt = 0.05
    t_eval = np.arange(0, t_straight, dt)

    sol_pre = solve_ivp(
        ship.dynamics,
        (0, t_straight),
        init_state,
        args=(n_cmd, 0.0),
        t_eval=t_eval,
        method='RK45'
    )

    x_arr = sol_pre.y[6]
    idx_200 = np.argmax(x_arr >= 200.0)

    if idx_200 == 0:
        idx_200 = len(x_arr) - 1

    t_rudder = sol_pre.t[idx_200]
    state_at_200 = sol_pre.y[:, idx_200]

    # -------------------------------
    # PHASE 2: Turning
    # -------------------------------
    t_turn = 300.0

    sol_turn = solve_ivp(
        ship.dynamics,
        (0, t_turn),
        state_at_200,
        args=(n_cmd, rudder_deg),
        t_eval=np.linspace(0, t_turn, 6000),
        method='RK45'
    )

    # -------------------------------
    # Combine results
    # -------------------------------
    t_full = np.concatenate([
        sol_pre.t[:idx_200 + 1],
        sol_pre.t[idx_200] + sol_turn.t[1:]
    ])

    y_full = np.hstack([
        sol_pre.y[:, :idx_200 + 1],
        sol_turn.y[:, 1:]
    ])

    # -------------------------------
    # Metrics
    # -------------------------------
    x_turn = sol_turn.y[6]
    y_turn = sol_turn.y[7]

    max_advance = np.max(x_turn - x_turn[0])
    max_transfer = np.max(np.abs(y_turn - y_turn[0]))

    mid = len(x_turn) // 2
    dx = np.gradient(x_turn[mid-50:mid+50])
    dy = np.gradient(y_turn[mid-50:mid+50])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-9)**1.5
    turning_radius = 1.0 / (np.mean(curvature) + 1e-9)

    return {
        "t": t_full,
        "y": y_full,
        "t_rudder": t_rudder,
        "advance": max_advance,
        "transfer": max_transfer,
        "radius": turning_radius,
        "sol_pre": sol_pre,
        "sol_turn": sol_turn
    }