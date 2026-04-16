# main.py
"""
MNV Simulator — main entry point.

Each maneuver can be toggled on/off independently.
Every maneuver that accepts a PID option also has a USE_PID_* flag.
Set RUN_VALIDATION = True for the full IMO/ITTC compliance suite.
"""

import matplotlib.pyplot as plt
import numpy as np

# ── Vessel ──────────────────────────────────────────────────────────────────
from vessel.params import VesselParams
from vessel.model  import ShipModel

# ── Maneuvers ───────────────────────────────────────────────────────────────
from maneuvers.straight_line     import run_straight_line
from maneuvers.turning_circle    import run_turning_circle
from maneuvers.zigzag            import run_zigzag_openloop, compute_overshoot
from maneuvers.zigzag_closedloop import run_zigzag_closedloop, compute_overshoot_cl
from maneuvers.stopping_trial    import run_stopping_trial
from maneuvers.pullout           import run_pullout
from maneuvers.spiral            import run_spiral
from maneuvers.validation        import run_validation

# ── Plotters ────────────────────────────────────────────────────────────────
from utils.plotters           import (plot_straight_line, plot_trajectory,
                                      plot_turning_circle, plot_zigzag_dashboard,
                                      plot_stopping_trial, plot_pullout, plot_spiral)
from utils.validation_plotter import plot_validation_report


# ============================================================================
# ── MANEUVER TOGGLES ────────────────────────────────────────────────────────
# ============================================================================

RUN_STRAIGHT_LINE   = False
RUN_TURNING_CIRCLE  = False
RUN_ZIGZAG          = False
RUN_STOPPING        = False   # Stopping trial  (thesis p.119, IMO para 3.3)
RUN_PULLOUT         = False   # Pull-out        (thesis p.120-122, ITTC §4.3)
RUN_SPIRAL          = True   # Spiral          (thesis p.122-126, ITTC §4.4)
RUN_VALIDATION      = False    # Full IMO/ITTC suite (runs all tests above)

# ── PID options — one flag per maneuver ─────────────────────────────────────
# Set True to run that maneuver with closed-loop PID heading control.
# Turning circle is always open-loop (rudder command IS the test).
USE_PID_STRAIGHT    = False   # course-keeping autopilot on straight run
USE_PID_ZIGZAG      = False   # 'open-loop' | 'closed-loop' | 'both' (see below)
USE_PID_STOPPING    = False   # PID holds heading during crash-stop
USE_PID_PULLOUT     = False   # PID course-keeps in straight & coast phases
USE_PID_SPIRAL      = False   # PID heading-step spiral (vs fixed rudder steps)
USE_PID_VALIDATION  = False   # run the full validation suite in closed-loop PID

# ZigZag mode (only used when RUN_ZIGZAG = True)
ZIGZAG_MODE = 'both'          # 'open-loop' | 'closed-loop' | 'both'

# ── Global PID gains ─────────────────────────────────────────────────────────
# Used wherever use_pid=True.  See control/pid.py for tuning guidance.
PID_KP        = 20.0    # [deg/rad]    proportional
PID_KI        = 0.3     # [deg/rad·s]  integral  ← primary overshoot lever
PID_KD        = 3.0     # [deg·s/rad]  derivative
PID_DELTA_MAX = 35.0    # [deg]        rudder saturation limit

# convenience dict passed to all maneuver functions
_pid = dict(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD, delta_max=PID_DELTA_MAX)


# ============================================================================
# ── SETUP ───────────────────────────────────────────────────────────────────
# ============================================================================

params = VesselParams()
ship   = ShipModel(params)
print("\nShip initialised")

# 14-element initial state:
# [u, v, w, p, q, r, x, y, z, phi, theta, psi, n_act, delta_act]
init_state = [
    0.1, 0, 0,
    0,   0, 0,
    0,   0, 0,
    0,   0, 0,
    95.0, 0
]


# ============================================================================
# ── STRAIGHT LINE ────────────────────────────────────────────────────────────
# ============================================================================
if RUN_STRAIGHT_LINE:
    mode_sl = 'Closed-Loop PID' if USE_PID_STRAIGHT else 'Open-Loop'
    print(f"\nRunning Straight Line  [{mode_sl}]...")

    result_sl = run_straight_line(
        ship, init_state, n_cmd=95.0, t_end=300,
        use_pid=USE_PID_STRAIGHT, psi_desired=0.0, **_pid
    )

    if USE_PID_STRAIGHT:
        t_sl, y_sl, d_sl = result_sl
        print(f"  Final speed u  = {y_sl[0, -1]:.2f} m/s")
        print(f"  Final position = {y_sl[6, -1]:.2f} m North, {y_sl[7, -1]:.2f} m East")
        print(f"  Heading drift  = {np.rad2deg(y_sl[11, -1]):.4f} deg (PID held psi=0)")
        # Build a simple OdeSolution-like object for the plotter
        class _Sol:
            pass
        sol_sl = _Sol(); sol_sl.t = t_sl; sol_sl.y = y_sl
        plot_straight_line(sol_sl)
        plot_trajectory(y_sl[6], y_sl[7])
    else:
        sol_sl = result_sl
        print(f"  Final speed u = {sol_sl.y[0, -1]:.2f} m/s")
        print(f"  Final position x = {sol_sl.y[6, -1]:.2f} m")
        plot_straight_line(sol_sl)
        plot_trajectory(sol_sl.y[6], sol_sl.y[7])


# ============================================================================
# ── TURNING CIRCLE ───────────────────────────────────────────────────────────
# (Always open-loop — fixed rudder is the test input by definition)
# ============================================================================
if RUN_TURNING_CIRCLE:
    print("\nRunning Turning Circle  [Open-Loop — fixed rudder]...")
    tc = run_turning_circle(ship, init_state, n_cmd=95.0, rudder_deg=18.0)
    print(f"  Turning Radius = {tc['radius']:.1f} m")
    print(f"  Advance        = {tc['advance']:.1f} m")
    print(f"  Transfer       = {tc['transfer']:.1f} m")
    print(f"  phi_ss (roll)  = {np.rad2deg(tc['y'][9,-1]):.1f} deg")
    plot_turning_circle(tc)


# ============================================================================
# ── ZIGZAG ───────────────────────────────────────────────────────────────────
# ============================================================================
if RUN_ZIGZAG:
    run_ol = ZIGZAG_MODE in ('open-loop',  'both')
    run_cl = ZIGZAG_MODE in ('closed-loop','both')
    print(f"\nRunning ZigZag  [mode: {ZIGZAG_MODE}]...")

    for delta in [10.0, 20.0]:
        print(f"\n  --- {int(delta)}/{int(delta)} ZigZag ---")

        if run_ol:
            t, y, X, Y, K, N, d = run_zigzag_openloop(
                ship, init_speed=init_state[0],
                delta_zz_deg=delta, t_end=300, n_rpm=95.0)
            os_s, os_p = compute_overshoot(y, delta)
            print(f"  [Open-Loop]   Stbd={os_s:.2f}°  Port={os_p:.2f}°")
            plot_zigzag_dashboard(t=t, y=y, delta_cmd=d, delta_zz=delta,
                                  X=X, Y=Y, K=K, N=N, mode='open-loop')

        if run_cl:
            t_cl,y_cl,X_cl,Y_cl,K_cl,N_cl,d_cl,setpt = run_zigzag_closedloop(
                ship, init_speed=init_state[0],
                delta_zz_deg=delta, t_end=300, n_rpm=95.0, **_pid)
            os_s_cl, os_p_cl = compute_overshoot_cl(y_cl, delta)
            print(f"  [Closed-Loop] Stbd={os_s_cl:.2f}°  Port={os_p_cl:.2f}°"
                  f"  (Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD})")
            plot_zigzag_dashboard(t=t_cl, y=y_cl, delta_cmd=d_cl, delta_zz=delta,
                                  X=X_cl, Y=Y_cl, K=K_cl, N=N_cl,
                                  setpoint=setpt, mode='closed-loop')


# ============================================================================
# ── STOPPING TRIAL ───────────────────────────────────────────────────────────
# Thesis p.119 / IMO MSC.137(76) para 3.3
# Open-loop:   zero rudder (thesis protocol)
# Closed-loop: PID holds heading at the moment of engine reversal
# ============================================================================
if RUN_STOPPING:
    mode_st = 'Closed-Loop PID' if USE_PID_STOPPING else 'Open-Loop'
    print(f"\nRunning Stopping Trial  [{mode_st}]"
          "  (thesis p.119, IMO MSC.137(76) para 3.3)...")

    t_st, y_st, n_arr, d_arr, res_st = run_stopping_trial(
        ship, init_state,
        n_full=ship.m.nmax, t_buildup=100.0, t_end=300.0,
        use_pid=USE_PID_STOPPING, **_pid
    )

    hr = res_st['head_reach']; hrL = res_st['head_reach_L']
    t2s = res_st['time_to_stop']
    status = 'PASS' if res_st['imo_pass'] else 'FAIL'
    print(f"  Head reach     = {hr:.0f} m  ({hrL:.2f} L)  [IMO limit: 15 L] → {status}")
    print(f"  Time to stop   = {t2s:.0f} s")

    plot_stopping_trial(t_st, y_st, n_arr, d_arr, res_st)

# ============================================================================
# ── PULL-OUT MANEUVER ────────────────────────────────────────────────────────
# Thesis p.120-122 / ITTC 7.5-04-02-01 §4.3
# Open-loop:   zero rudder in straight & coast phases (thesis protocol)
# Closed-loop: PID course-keeps in straight phase; PID recovers in coast phase
#              (turn phase always uses fixed open-loop rudder)
# ============================================================================
if RUN_PULLOUT:
    mode_po = 'Closed-Loop PID' if USE_PID_PULLOUT else 'Open-Loop'
    print(f"\nRunning Pull-Out Maneuver  [{mode_po}]"
          "  (thesis p.120-122, ITTC §4.3)...")

    t_po, y_po, d_po, res_po = run_pullout(
        ship, init_state,
        rudder_deg=20.0, t_straight=70.0, t_turn=200.0, t_coast=150.0,
        n_cmd=95.0, use_pid=USE_PID_PULLOUT, **_pid
    )

    print(f"  Steady yaw rate (in turn)  = {res_po['r_steady_ss']:.3f} deg/s")
    print(f"  Residual yaw rate (final)  = {res_po['r_final']:.5f} deg/s")
    print(f"  Verdict: {res_po['verdict']}")

    plot_pullout(t_po, y_po, d_po, res_po)

# ============================================================================
# ── SPIRAL MANEUVER (Dieudonné) ──────────────────────────────────────────────
# Thesis p.122-126 / ITTC 7.5-04-02-01 §4.4
# Open-loop:   fixed rudder steps (+20°→-20°→+20° in 5° increments)
# Closed-loop: PID drives ship to equivalent heading steps
# ============================================================================
if RUN_SPIRAL:
    mode_sp = 'Closed-Loop PID (heading steps)' if USE_PID_SPIRAL else 'Open-Loop (rudder steps)'
    print(f"\nRunning Spiral Maneuver  [{mode_sp}]"
          "  (thesis p.122-126, ITTC §4.4)...")

    t_sp, y_sp, d_sp, sp_d, sp_r, res_sp = run_spiral(
        ship, init_state,
        n_cmd=95.0, delta_start=20.0, delta_end=-20.0,
        delta_step=5.0, t_settle=83.0,   # ~1 circle/step → matches thesis Fig.36 trajectory
        t_straight=200.0,                 # straight pre-run before spiral (matches thesis)
        use_pid=USE_PID_SPIRAL, **_pid
    )

    print(f"  Max hysteresis = {res_sp['max_hysteresis']:.4f} deg/s")
    print(f"  Verdict: {res_sp['verdict']}")

    plot_spiral(t_sp, y_sp, d_sp, sp_d, sp_r, res_sp)

# ============================================================================
# ── FULL VALIDATION SUITE ────────────────────────────────────────────────────
# Runs all 7 IMO/ITTC checks and renders the compliance report.
# Toggle USE_PID_VALIDATION to switch between open-loop and PID modes.
# ============================================================================
if RUN_VALIDATION:
    mode_val = 'Closed-Loop PID' if USE_PID_VALIDATION else 'Open-Loop'
    print(f"\nRunning Full IMO/ITTC Validation Suite  [{mode_val}]...")
    print("Standards referenced:")
    print("  [1] IMO MSC.137(76) (2002) — Standards for Ship Maneuverability")
    print("  [2] ITTC 7.5-04-02-01 (2002) — Maneuvering Trial Code")
    print("  [3] IMO MSC/Circ.1053 (2002) — Explanatory Notes to MSC.137(76)")
    print("  [4] Pakkan (2007) — Modeling and Simulation of a Maneuvering Ship")
    print()

    scorecard, raw = run_validation(
        ship, init_state, n_cmd=95.0,
        use_pid=USE_PID_VALIDATION, verbose=True, **_pid
    )
    fig_val = plot_validation_report(scorecard, raw, ship)


# ============================================================================
# ── SHOW ALL PLOTS ───────────────────────────────────────────────────────────
# ============================================================================
print("\nSimulation complete.  Showing plots...")
plt.show()


# ============================================================================
# ── OPEN-LOOP vs CLOSED-LOOP COMPARISON ─────────────────────────────────────
# Runs every maneuver in BOTH modes and renders side-by-side comparison plots.
# Individual plots per maneuver + one master dashboard figure.
# ============================================================================
RUN_COMPARISON = False   # ← set True to enable

if RUN_COMPARISON:
    from utils.comparison_plotter import (
        plot_comparison_straightline, plot_comparison_zigzag,
        plot_comparison_stopping, plot_comparison_pullout,
        plot_comparison_spiral, plot_comparison_dashboard,
    )

    print(f"\nRunning Open-Loop vs Closed-Loop Comparison Suite...")
    print(f"  PID gains: Kp={PID_KP}  Ki={PID_KI}  Kd={PID_KD}")
    print()

    # ── 1. Straight line ─────────────────────────────────────────────────
    print("  [1/6] Straight line...")
    _sol_ol = run_straight_line(ship, init_state, n_cmd=95., t_end=300)
    _t_cl_sl, _y_cl_sl, _d_cl_sl = run_straight_line(
        ship, init_state, n_cmd=95., t_end=300, use_pid=True, **_pid)

    # ── 2. ZigZag 10/10 ──────────────────────────────────────────────────
    print("  [2/6] ZigZag 10/10...")
    _t_ol10,_y_ol10,_,_,_,_,_d_ol10 = run_zigzag_openloop(
        ship, init_state[0], 10., 300, n_rpm=95.)
    _t_cl10,_y_cl10,_,_,_,_,_d_cl10,_sp10 = run_zigzag_closedloop(
        ship, init_state[0], 10., 300, n_rpm=95., **_pid)
    _os_ol10 = compute_overshoot(_y_ol10, 10.)
    _os_cl10 = compute_overshoot_cl(_y_cl10, 10.)
    print(f"       OL: {_os_ol10[0]:.2f}°/{_os_ol10[1]:.2f}°  "
          f"CL: {_os_cl10[0]:.2f}°/{_os_cl10[1]:.2f}°")

    # ── 3. ZigZag 20/20 ──────────────────────────────────────────────────
    print("  [3/6] ZigZag 20/20...")
    _t_ol20,_y_ol20,_,_,_,_,_d_ol20 = run_zigzag_openloop(
        ship, init_state[0], 20., 300, n_rpm=95.)
    _t_cl20,_y_cl20,_,_,_,_,_d_cl20,_sp20 = run_zigzag_closedloop(
        ship, init_state[0], 20., 300, n_rpm=95., **_pid)
    _os_ol20 = compute_overshoot(_y_ol20, 20.)
    _os_cl20 = compute_overshoot_cl(_y_cl20, 20.)
    print(f"       OL: {_os_ol20[0]:.2f}°/{_os_ol20[1]:.2f}°  "
          f"CL: {_os_cl20[0]:.2f}°/{_os_cl20[1]:.2f}°")

    # ── 4. Stopping trial ─────────────────────────────────────────────────
    print("  [4/6] Stopping trial...")
    _t_st_ol,_y_st_ol,_n_ol,_d_ol_st,_r_ol = run_stopping_trial(
        ship, init_state, n_full=ship.m.nmax, t_buildup=100., t_end=250.)
    _t_st_cl,_y_st_cl,_n_cl,_d_cl_st,_r_cl = run_stopping_trial(
        ship, init_state, n_full=ship.m.nmax, t_buildup=100., t_end=250.,
        use_pid=True, **_pid)
    print(f"       OL reach: {_r_ol['head_reach']:.0f}m  "
          f"CL reach: {_r_cl['head_reach']:.0f}m")

    # ── 5. Pull-out ───────────────────────────────────────────────────────
    print("  [5/6] Pull-out...")
    _t_po_ol,_y_po_ol,_d_po_ol,_res_po_ol = run_pullout(
        ship, init_state, rudder_deg=20., t_straight=70., t_turn=200., t_coast=150.)
    _t_po_cl,_y_po_cl,_d_po_cl,_res_po_cl = run_pullout(
        ship, init_state, rudder_deg=20., t_straight=70., t_turn=200., t_coast=150.,
        use_pid=True, **_pid)
    print(f"       OL r_final: {_res_po_ol['r_final']:.5f}°/s  "
          f"CL r_final: {_res_po_cl['r_final']:.5f}°/s")

    # ── 6. Spiral ─────────────────────────────────────────────────────────
    print("  [6/6] Spiral (longest — ~30 min)...")
    _t_sp_ol,_y_sp_ol,_d_sp_ol,_spd_ol,_spr_ol,_res_sp_ol = run_spiral(
        ship, init_state, t_settle=150., delta_step=5.)
    _t_sp_cl,_y_sp_cl,_d_sp_cl,_spd_cl,_spr_cl,_res_sp_cl = run_spiral(
        ship, init_state, t_settle=150., delta_step=5., use_pid=True, **_pid)
    print(f"       OL hyst: {_res_sp_ol['max_hysteresis']:.5f}  "
          f"CL hyst: {_res_sp_cl['max_hysteresis']:.5f}")

    # ── Generate individual comparison figures ─────────────────────────────
    print("\n  Generating comparison figures...")

    plot_comparison_straightline(_sol_ol, _t_cl_sl, _y_cl_sl, _d_cl_sl)
    print("    Straight-line comparison done")

    plot_comparison_zigzag(_t_ol10,_y_ol10,_d_ol10,_os_ol10,
                           _t_cl10,_y_cl10,_d_cl10,_os_cl10, 10., _sp10)
    print("    ZZ 10/10 comparison done")

    plot_comparison_zigzag(_t_ol20,_y_ol20,_d_ol20,_os_ol20,
                           _t_cl20,_y_cl20,_d_cl20,_os_cl20, 20., _sp20)
    print("    ZZ 20/20 comparison done")

    plot_comparison_stopping(_t_st_ol,_y_st_ol,_n_ol,_d_ol_st,_r_ol,
                             _t_st_cl,_y_st_cl,_n_cl,_d_cl_st,_r_cl)
    print("    Stopping comparison done")

    plot_comparison_pullout(_t_po_ol,_y_po_ol,_d_po_ol,_res_po_ol,
                            _t_po_cl,_y_po_cl,_d_po_cl,_res_po_cl)
    print("    Pull-out comparison done")

    plot_comparison_spiral(_t_sp_ol,_y_sp_ol,_d_sp_ol,_spd_ol,_spr_ol,_res_sp_ol,
                           _t_sp_cl,_y_sp_cl,_d_sp_cl,_spd_cl,_spr_cl,_res_sp_cl)
    print("    Spiral comparison done")

    # ── Master dashboard (all maneuvers in one figure) ─────────────────────
    _cmp_data = {
        'sl'      : (_sol_ol, (_t_cl_sl, _y_cl_sl, _d_cl_sl)),
        'zz10'    : ((_t_ol10,_y_ol10,_d_ol10,_os_ol10),
                     (_t_cl10,_y_cl10,_d_cl10,_os_cl10,_sp10)),
        'zz20'    : ((_t_ol20,_y_ol20,_d_ol20,_os_ol20),
                     (_t_cl20,_y_cl20,_d_cl20,_os_cl20,_sp20)),
        'stopping': ((_t_st_ol,_y_st_ol,_n_ol,_d_ol_st,_r_ol),
                     (_t_st_cl,_y_st_cl,_n_cl,_d_cl_st,_r_cl)),
        'pullout' : ((_t_po_ol,_y_po_ol,_d_po_ol,_res_po_ol),
                     (_t_po_cl,_y_po_cl,_d_po_cl,_res_po_cl)),
        'spiral'  : ((_t_sp_ol,_y_sp_ol,_d_sp_ol,_spd_ol,_spr_ol,_res_sp_ol),
                     (_t_sp_cl,_y_sp_cl,_d_sp_cl,_spd_cl,_spr_cl,_res_sp_cl)),
    }
    plot_comparison_dashboard(_cmp_data, _pid)
    print("    Master dashboard done")