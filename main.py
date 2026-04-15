# main.py

import matplotlib.pyplot as plt

# Vessel + Model
from vessel.params import VesselParams
from vessel.model import ShipModel

# Maneuvers
from maneuvers.straight_line import run_straight_line
from maneuvers.turning_circle import run_turning_circle
from maneuvers.zigzag import run_zigzag_openloop, compute_overshoot
from maneuvers.zigzag_closedloop import run_zigzag_closedloop, compute_overshoot_cl

# Plotters
from utils.plotters import (
    plot_straight_line,
    plot_trajectory,
    plot_turning_circle,
    plot_zigzag_dashboard,
)


# ==========================================
# USER CONTROLS (TOGGLES)
# ==========================================
RUN_STRAIGHT_LINE  = False
RUN_TURNING_CIRCLE = False
RUN_ZIGZAG         = True

# ZigZag mode: 'open-loop', 'closed-loop', or 'both'
ZIGZAG_MODE = 'both'

# Closed-loop PID gains  (see control/pid.py for tuning guide)
PID_KP        = 20.0   # [deg/rad]    proportional
PID_KI        = 0.3    # [deg/rad·s]  integral
PID_KD        = 3.0    # [deg·s/rad]  derivative
PID_DELTA_MAX = 35.0   # [deg]        rudder saturation


# ==========================================
# SETUP SHIP
# ==========================================
params = VesselParams()
ship   = ShipModel(params)

print("\nShip initialized")


# ==========================================
# INITIAL STATE
# ==========================================
# [u, v, w, p, q, r, x, y, z, phi, theta, psi, n_act, delta_act]
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
    print(f"\nRunning ZigZag Maneuvers  [mode: {ZIGZAG_MODE}]...")

    run_ol = ZIGZAG_MODE in ('open-loop', 'both')
    run_cl = ZIGZAG_MODE in ('closed-loop', 'both')

    for delta in [20.0]:
        print(f"\n--- {int(delta)}/{int(delta)} ZigZag ---")

        # ── OPEN-LOOP ────────────────────────────────────────────────
        if run_ol:
            t, y, X, Y, K, N, d = run_zigzag_openloop(
                ship,
                init_speed=init_state[0],
                delta_zz_deg=delta,
                t_end=300,
                n_rpm=95.0
            )

            os_stbd, os_port = compute_overshoot(y, delta)
            print(f"  [Open-Loop]  Overshoot Stbd = {os_stbd:.2f} deg  "
                  f"Port = {os_port:.2f} deg")

            fig_ol = plot_zigzag_dashboard(
                t=t, y=y, delta_cmd=d, delta_zz=delta,
                X=X, Y=Y, K=K, N=N,
                mode='open-loop'
            )

        # ── CLOSED-LOOP ──────────────────────────────────────────────
        if run_cl:
            t_cl, y_cl, X_cl, Y_cl, K_cl, N_cl, d_cl, setpt = run_zigzag_closedloop(
                ship,
                init_speed=init_state[0],
                delta_zz_deg=delta,
                t_end=300,
                n_rpm=95.0,
                Kp=PID_KP,
                Ki=PID_KI,
                Kd=PID_KD,
                delta_max=PID_DELTA_MAX
            )

            os_stbd_cl, os_port_cl = compute_overshoot_cl(y_cl, delta)
            print(f"  [Closed-Loop] Overshoot Stbd = {os_stbd_cl:.2f} deg  "
                  f"Port = {os_port_cl:.2f} deg  "
                  f"(Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD})")

            fig_cl = plot_zigzag_dashboard(
                t=t_cl, y=y_cl, delta_cmd=d_cl, delta_zz=delta,
                X=X_cl, Y=Y_cl, K=K_cl, N=N_cl,
                setpoint=setpt,
                mode='closed-loop'
            )


# ==========================================
# SHOW ALL PLOTS
# ==========================================
print("\nSimulation complete. Showing plots...")
plt.show()