import numpy as np
from scipy.integrate import solve_ivp


def run_zigzag_openloop(ship, init_speed, delta_zz_deg=10.0,
                        t_end=300.0, dt=0.05, n_rpm=95.0):
    """
    ITTC Open Loop ZigZag Maneuver.

    Sign convention (important for trigger logic):
      +delta (starboard rudder) -> ship turns to PORT -> psi decreases.
      -delta (port rudder)      -> ship turns to STARBOARD -> psi increases.

    Switching sequence:
      phase= 1: rudder=+delta, waiting for psi <= -psi_trigger (heading gone -delta deg).
      phase=-1: rudder=-delta, waiting for psi >= +psi_trigger (heading gone +delta deg).

    Heading is evaluated wrapped to [-180, 180] deg to avoid false triggers
    from accumulated drift.

    Returns: t, y, X, Y, K, N, delta histories
    """
    psi_trigger = np.deg2rad(delta_zz_deg)

    rudder = delta_zz_deg   # start with +delta (starboard)
    phase  = 1              # expect psi to go negative

    state = np.array([
        init_speed, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        n_rpm, 0
    ], dtype=float)

    t = 0.0

    t_arr     = [t]
    y_arr     = [state.copy()]
    X_arr, Y_arr, K_arr, N_arr = [0.0], [0.0], [0.0], [0.0]
    delta_arr = [rudder]

    while t < t_end:
        # Wrap heading to [-pi, pi] to avoid accumulated drift triggering false switches
        psi = (state[11] + np.pi) % (2 * np.pi) - np.pi

        # phase= 1: rudder=+delta (starboard) -> psi goes negative -> flip at -psi_trigger
        # phase=-1: rudder=-delta (port)       -> psi goes positive -> flip at +psi_trigger
        if phase == 1 and psi <= -psi_trigger:
            rudder = -delta_zz_deg
            phase  = -1
        elif phase == -1 and psi >= psi_trigger:
            rudder = delta_zz_deg
            phase  = 1

        sol = solve_ivp(
            ship.dynamics,
            (t, t + dt),
            state,
            args=(n_rpm, rudder),
            method='RK45',
            max_step=dt
        )

        state = sol.y[:, -1]
        t    += dt

        X, Y, K, N = ship.get_forces(state, rudder)

        t_arr.append(t)
        y_arr.append(state.copy())
        X_arr.append(X)
        Y_arr.append(Y)
        K_arr.append(K)
        N_arr.append(N)
        delta_arr.append(rudder)

    return (
        np.array(t_arr),
        np.array(y_arr).T,
        np.array(X_arr),
        np.array(Y_arr),
        np.array(K_arr),
        np.array(N_arr),
        np.array(delta_arr)
    )


def compute_overshoot(y, delta_zz_deg):
    """
    Computes zigzag overshoot angles (first overshoot after each rudder reversal).

    Heading psi is wrapped to [-180, 180] before analysis.
    """
    psi_deg = (np.rad2deg(y[11]) + 180.0) % 360.0 - 180.0

    # Starboard overshoot: max positive heading beyond +delta_zz
    # Port overshoot: max negative heading below -delta_zz
    overshoot_starboard = max(0, np.max(psi_deg)  - delta_zz_deg)
    overshoot_port      = max(0, np.max(-psi_deg) - delta_zz_deg)

    return overshoot_starboard, overshoot_port