# main.py

import matplotlib.pyplot as plt

# Vessel + Model
from vessel.params import VesselParams
from vessel.model import ShipModel

# Maneuvers
from maneuvers.straight_line import run_straight_line
from maneuvers.turning_circle import run_turning_circle
from maneuvers.zigzag import run_zigzag_openloop, compute_overshoot

# Plotters
from utils.plotters import (
    plot_straight_line,
    plot_trajectory,
    plot_turning_circle,
    plot_zigzag_states,
    plot_zigzag_forces,
    plot_zigzag_standard,
    plot_zigzag_trajectory
)


# ==========================================
# USER CONTROLS (TOGGLES)
# ==========================================
RUN_STRAIGHT_LINE  = False
RUN_TURNING_CIRCLE = False
RUN_ZIGZAG         = True


# ==========================================
# SETUP SHIP
# ==========================================
params = VesselParams()
ship   = ShipModel(params)

print("\nShip initialized")


# ==========================================
# INITIAL STATE
# ==========================================
# [u, v, w, p, q, r, x, y, z, phi, theta, psi, n, delta]
init_state = [
    0.1, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    95.0,
    0
]


# ==========================================
# STRAIGHT LINE
# ==========================================
if RUN_STRAIGHT_LINE:
    print("\nRunning Straight Line (300 s)...")

    sol_sl = run_straight_line(
        ship,
        init_state,
        n_cmd=95.0,
        t_end=300
    )

    print(f"Final speed u = {sol_sl.y[0, -1]:.2f} m/s")
    print(f"Final position x = {sol_sl.y[6, -1]:.2f} m")

    fig_sl1 = plot_straight_line(sol_sl)
    fig_sl2 = plot_trajectory(sol_sl.y[6], sol_sl.y[7])


# ==========================================
# TURNING CIRCLE
# ==========================================
if RUN_TURNING_CIRCLE:
    print("\nRunning Turning Circle...")

    tc_results = run_turning_circle(
        ship,
        init_state,
        n_cmd=95.0,
        rudder_deg=18.0
    )

    print(f"Turning Radius  = {tc_results['radius']:.1f} m")
    print(f"Advance         = {tc_results['advance']:.1f} m")
    print(f"Transfer        = {tc_results['transfer']:.1f} m")

    fig_tc = plot_turning_circle(tc_results)


# ==========================================
# ZIGZAG
# ==========================================
if RUN_ZIGZAG:
    print("\nRunning ZigZag Maneuvers...")

    for delta in [10.0, 20.0]:
        print(f"\n--- {int(delta)}/{int(delta)} ZigZag ---")

        t, y, X, Y, K, N, d = run_zigzag_openloop(
            ship,
            init_speed=init_state[0],
            delta_zz_deg=delta,
            t_end=300,
            n_rpm=95.0
        )

        os_stbd, os_port = compute_overshoot(y, delta)

        print(f"Overshoot Starboard = {os_stbd:.2f} deg")
        print(f"Overshoot Port      = {os_port:.2f} deg")

        fig1 = plot_zigzag_states(t, y, delta)
        fig2 = plot_zigzag_forces(t, X, Y, K, N)
        fig3 = plot_zigzag_standard(t, y, d, delta)
        fig4 = plot_zigzag_trajectory(y)


# ==========================================
# SHOW ALL PLOTS
# ==========================================
print("\nSimulation complete. Showing plots...")
plt.show()