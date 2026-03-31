import numpy as np
from scipy.integrate import solve_ivp


def run_zigzag_openloop(ship, init_speed, delta_zz_deg=10.0,
                        t_end=300.0, dt=0.05, n_rpm=95.0):
    """
    ITTC Open Loop ZigZag

    Returns:
        t, y, forces, rudder history
    """

    psi_trigger = np.deg2rad(delta_zz_deg)

    rudder = delta_zz_deg
    phase = 1

    # Initial state
    state = np.array([
        init_speed, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        n_rpm, 0
    ], dtype=float)

    t = 0.0

    # Storage
    t_arr = [t]
    y_arr = [state.copy()]

    X_arr, Y_arr, K_arr, N_arr = [0.0], [0.0], [0.0], [0.0]
    delta_arr = [rudder]

    while t < t_end:
        psi = state[11]

        # Switching logic
        if phase == 1 and psi <= -psi_trigger:
            rudder = -delta_zz_deg
            phase = -1
        elif phase == -1 and psi >= psi_trigger:
            rudder = delta_zz_deg
            phase = 1

        sol = solve_ivp(
            ship.dynamics,
            (t, t + dt),
            state,
            args=(n_rpm, rudder),
            method='RK45',
            max_step=dt
        )

        state = sol.y[:, -1]
        t += dt

        # Forces
        X, Y, K, N = ship.get_forces(state, rudder)

        # Store
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
    Computes zigzag overshoot angles
    """
    psi_deg = np.rad2deg(y[11])

    overshoot_starboard = max(0, np.max(psi_deg) - delta_zz_deg)
    overshoot_port = max(0, np.max(-psi_deg) - delta_zz_deg)

    return overshoot_starboard, overshoot_port