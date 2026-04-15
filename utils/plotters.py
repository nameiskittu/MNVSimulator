# utils/plotters.py — polished publication-quality plots

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.ticker import AutoMinorLocator
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection

# ──────────────────────────────────────────────
# DESIGN SYSTEM
# ──────────────────────────────────────────────
DARK_BG   = "#0d1117"
PANEL_BG  = "#161b22"
GRID_CLR  = "#21262d"
BORDER    = "#30363d"
TEXT_PRI  = "#e6edf3"
TEXT_SEC  = "#8b949e"

# Named palette
C_SURGE   = "#58a6ff"   # u  – electric blue
C_SWAY    = "#f78166"   # v  – coral
C_HEAVE   = "#3fb950"   # w  – green
C_ROLL    = "#d2a8ff"   # φ  – lavender
C_PITCH   = "#ffa657"   # θ  – amber
C_YAW     = "#79c0ff"   # ψ  – sky
C_X       = "#58a6ff"   # X force
C_Y       = "#f78166"   # Y force
C_K       = "#d2a8ff"   # K moment
C_N       = "#3fb950"   # N moment
C_RUDDER  = "#ffa657"   # δ  – amber
C_PSI     = "#58a6ff"   # ψ  – blue
C_PSI2    = "#79c0ff"   # ψ second ZZ
C_TRIG    = "#f78166"   # trigger lines
C_MARK    = "#3fb950"   # markers / annotations
C_TRAJ    = "#58a6ff"   # trajectory line
C_VLINE   = "#8b949e"   # vertical event lines
C_NORTH   = "#3fb950"   # north position
C_EAST    = "#f78166"   # east position


def _apply_style(fig, axes_flat):
    """Apply dark theme to a figure and all its axes."""
    fig.patch.set_facecolor(DARK_BG)
    for ax in axes_flat:
        ax.set_facecolor(PANEL_BG)
        ax.tick_params(colors=TEXT_SEC, labelsize=8, length=3)
        ax.xaxis.label.set_color(TEXT_SEC)
        ax.yaxis.label.set_color(TEXT_SEC)
        ax.title.set_color(TEXT_PRI)
        for spine in ax.spines.values():
            spine.set_edgecolor(BORDER)
        ax.grid(True, color=GRID_CLR, linewidth=0.6, linestyle="-")
        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.grid(True, which='minor', color=GRID_CLR, linewidth=0.3, alpha=0.5)
        ax.set_xlabel("t  [s]", fontsize=8)


def _title_style(fig, title):
    fig.suptitle(title, color=TEXT_PRI, fontsize=13, fontweight='bold',
                 x=0.5, y=0.98, ha='center')


def _line(ax, x, y, color, label=None, lw=1.6, alpha=1.0, ls='-'):
    ax.plot(x, y, color=color, linewidth=lw, alpha=alpha,
            linestyle=ls, label=label)


def _hline(ax, y, color=C_TRIG, lw=1.0, ls='--', alpha=0.75):
    ax.axhline(y, color=color, linewidth=lw, linestyle=ls, alpha=alpha)


def _vline(ax, x, color=C_VLINE, lw=1.0, ls='--', alpha=0.75):
    ax.axvline(x, color=color, linewidth=lw, linestyle=ls, alpha=alpha)


def _subtitle(ax, label, unit):
    ax.set_title(f"{label}  [{unit}]", color=TEXT_PRI, fontsize=9, pad=4)


def _legend(ax, **kw):
    leg = ax.legend(fontsize=7.5, facecolor=DARK_BG, edgecolor=BORDER,
                    labelcolor=TEXT_PRI, **kw)
    return leg


# ──────────────────────────────────────────────
# STRAIGHT LINE
# ──────────────────────────────────────────────
def plot_straight_line(sol):
    t = sol.t
    y = sol.y

    fig = plt.figure(figsize=(16, 10))
    _title_style(fig, "Straight-Line Run — State Variables")
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.38,
                           left=0.07, right=0.97, top=0.93, bottom=0.07)

    specs = [
        (0, 0, y[0],           C_SURGE,  "u",   "m/s"),
        (0, 1, y[1],           C_SWAY,   "v",   "m/s"),
        (0, 2, y[2],           C_HEAVE,  "w",   "m/s"),
        (1, 0, np.rad2deg(y[9]),  C_ROLL,  "φ (roll)",    "deg"),
        (1, 1, np.rad2deg(y[10]), C_PITCH, "θ (pitch)",   "deg"),
        (1, 2, np.rad2deg(y[11]), C_YAW,   "ψ (heading)", "deg"),
        (2, 0, y[6],           C_NORTH,  "x (North)",  "m"),
        (2, 1, y[7],           C_EAST,   "y (East)",   "m"),
        (2, 2, y[12],          C_RUDDER, "n_act",  "RPM"),
    ]

    axes = []
    for row, col, data, color, label, unit in specs:
        ax = fig.add_subplot(gs[row, col])
        _line(ax, t, data, color)
        _subtitle(ax, label, unit)
        axes.append(ax)

    _apply_style(fig, axes)
    return fig


def plot_trajectory(x, y_pos):
    fig, ax = plt.subplots(figsize=(8, 6))
    fig.patch.set_facecolor(DARK_BG)
    ax.set_facecolor(PANEL_BG)

    # Colour trajectory by time (gradient)
    points  = np.array([y_pos, x]).T.reshape(-1, 1, 2)
    segs    = np.concatenate([points[:-1], points[1:]], axis=1)
    lc      = LineCollection(segs, cmap='cool', linewidth=2)
    lc.set_array(np.linspace(0, 1, len(x)))
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, label="Progress (t=0 → t_end)")
    cb.ax.yaxis.label.set_color(TEXT_SEC)
    cb.ax.tick_params(colors=TEXT_SEC)

    ax.scatter([y_pos[0]],  [x[0]],  color=C_MARK,   s=60, zorder=5, label="Start")
    ax.scatter([y_pos[-1]], [x[-1]], color=C_RUDDER,  s=60, zorder=5, label="End",
               marker='s')

    ax.set_xlim(y_pos.min() - 20, y_pos.max() + 20)
    ax.set_ylim(x.min()   - 20, x.max()   + 20)
    ax.set_xlabel("East  [m]",  color=TEXT_SEC, fontsize=9)
    ax.set_ylabel("North  [m]", color=TEXT_SEC, fontsize=9)
    ax.set_title("Straight-Line Trajectory", color=TEXT_PRI, fontsize=11, fontweight='bold')
    ax.tick_params(colors=TEXT_SEC)
    ax.grid(True, color=GRID_CLR, linewidth=0.6)
    for sp in ax.spines.values(): sp.set_edgecolor(BORDER)
    _legend(ax, loc='upper left')
    plt.tight_layout()
    return fig


# ──────────────────────────────────────────────
# TURNING CIRCLE
# ──────────────────────────────────────────────
def plot_turning_circle(results):
    t        = results["t"]
    y        = results["y"]
    t_rudder = results["t_rudder"]

    phi  = np.rad2deg(y[9])
    r    = np.rad2deg(y[5])
    V    = np.sqrt(y[0]**2 + y[1]**2 + y[2]**2)
    x_   = y[6]
    y_   = y[7]

    fig = plt.figure(figsize=(16, 9))
    _title_style(fig, "Turning Circle Maneuver")
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.55, wspace=0.32,
                           left=0.07, right=0.97, top=0.93, bottom=0.07)

    # --- Trajectory (spans full left column) ---
    ax_traj = fig.add_subplot(gs[:, 0])
    points = np.array([y_, x_]).T.reshape(-1, 1, 2)
    segs   = np.concatenate([points[:-1], points[1:]], axis=1)
    lc     = LineCollection(segs, cmap='plasma', linewidth=2)
    lc.set_array(np.linspace(0, 1, len(x_)))
    ax_traj.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax_traj, label="Progress")
    cb.ax.yaxis.label.set_color(TEXT_SEC); cb.ax.tick_params(colors=TEXT_SEC)
    ax_traj.autoscale()
    ax_traj.set_xlabel("East  [m]",  color=TEXT_SEC, fontsize=9)
    ax_traj.set_ylabel("North  [m]", color=TEXT_SEC, fontsize=9)
    ax_traj.set_title("Trajectory", color=TEXT_PRI, fontsize=9, pad=4)
    ax_traj.set_aspect('equal', 'box')
    ax_traj.set_facecolor(PANEL_BG)
    ax_traj.tick_params(colors=TEXT_SEC)
    ax_traj.grid(True, color=GRID_CLR, linewidth=0.6)
    for sp in ax_traj.spines.values(): sp.set_edgecolor(BORDER)

    # --- Right column: Roll, Yaw Rate, Speed ---
    panels = [
        (gs[0, 1], phi, C_ROLL,   "φ  (Roll)",        "deg"),
        (gs[1, 1], r,   C_YAW,    "r  (Yaw Rate)",    "deg/s"),
        (gs[2, 1], V,   C_SURGE,  "V  (Speed)",       "m/s"),
    ]
    axes_right = []
    for gspec, data, color, label, unit in panels:
        ax = fig.add_subplot(gspec)
        _line(ax, t, data, color)
        _vline(ax, t_rudder)
        ax.annotate("rudder in", xy=(t_rudder, data[np.searchsorted(t, t_rudder)]),
                    xytext=(t_rudder + 5, ax.get_ylim()[0]),
                    color=C_VLINE, fontsize=6.5,
                    arrowprops=dict(arrowstyle='->', color=C_VLINE, lw=0.8))
        _subtitle(ax, label, unit)
        axes_right.append(ax)

    _apply_style(fig, axes_right)
    fig.patch.set_facecolor(DARK_BG)
    return fig


# ──────────────────────────────────────────────
# ZIGZAG — COMPREHENSIVE DASHBOARD
# ──────────────────────────────────────────────
def plot_zigzag_dashboard(t, y, delta_cmd, delta_zz, X, Y, K, N,
                          setpoint=None, mode='open-loop'):
    """
    Combines Trajectory, ITTC Standard, Forces, and Key States into a single window.

    Args:
        t, y, delta_cmd, delta_zz, X, Y, K, N  — standard zigzag outputs
        setpoint   [array | None]  — heading setpoint history [deg], for closed-loop only
        mode       str             — 'open-loop' or 'closed-loop' (used in title)
    """
    psi_wrapped = (np.rad2deg(y[11]) + 180) % 360 - 180
    x_ = y[6]
    y_pos = y[7]
    u = y[0]
    v = y[1]
    r = np.rad2deg(y[5])
    phi = np.rad2deg(y[9])

    mode_label = ' — Closed-Loop PID' if mode == 'closed-loop' else ' — Open-Loop'
    fig = plt.figure(figsize=(18, 12))
    _title_style(fig, f"{int(delta_zz)}/{int(delta_zz)} ZigZag Maneuver{mode_label}")
    
    # 4 rows, 4 columns layout
    gs = gridspec.GridSpec(4, 4, figure=fig, hspace=0.6, wspace=0.4,
                           left=0.06, right=0.97, top=0.92, bottom=0.06)

    axes_to_style = []

    # 1. TRAJECTORY (Spans top left: 2 rows, 2 columns)
    ax_traj = fig.add_subplot(gs[0:2, 0:2])
    points = np.array([y_pos, x_]).T.reshape(-1, 1, 2)
    segs = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segs, cmap='cool', linewidth=2.2)
    lc.set_array(np.linspace(0, 1, len(x_)))
    ax_traj.add_collection(lc)
    
    cb = fig.colorbar(lc, ax=ax_traj, pad=0.02)
    cb.set_label("Progress (t=0 → t_end)", color=TEXT_SEC, fontsize=9)
    cb.ax.yaxis.label.set_color(TEXT_SEC)
    cb.ax.tick_params(colors=TEXT_SEC)

    ax_traj.scatter([y_pos[0]], [x_[0]], color=C_MARK, s=70, zorder=5, label="Start", marker='o')
    ax_traj.scatter([y_pos[-1]], [x_[-1]], color=C_RUDDER, s=70, zorder=5, label="End", marker='s')

    ax_traj.autoscale()
    ax_traj.set_aspect('equal', 'datalim')
    ax_traj.set_xlabel("East  [m]", color=TEXT_SEC, fontsize=9)
    ax_traj.set_ylabel("North  [m]", color=TEXT_SEC, fontsize=9)
    ax_traj.set_title("Trajectory", color=TEXT_PRI, fontsize=11, fontweight='bold')
    _legend(ax_traj, loc='upper left')
    axes_to_style.append(ax_traj)

    # 2. ITTC STANDARD (Spans top right: 2 rows, 2 columns)
    # Heading (Row 0, Cols 2-3)
    ax_psi = fig.add_subplot(gs[0, 2:4])
    if setpoint is not None:
        ax_psi.step(t, setpoint, color=C_PSI2, lw=1.0, ls='--', alpha=0.75,
                    where='post', label='ψ_desired (setpoint)')
    _line(ax_psi, t, psi_wrapped, C_PSI, label="ψ  (heading)")
    _hline(ax_psi, delta_zz, C_TRIG)
    _hline(ax_psi, -delta_zz, C_TRIG)
    ax_psi.fill_between(t, -delta_zz, delta_zz, color=C_TRIG, alpha=0.07)
    ax_psi.fill_between(t, delta_zz, psi_wrapped, where=(psi_wrapped > delta_zz),
                        color=C_MARK, alpha=0.18, label="Overshoot (stbd)")
    ax_psi.fill_between(t, psi_wrapped, -delta_zz, where=(psi_wrapped < -delta_zz),
                        color=C_RUDDER, alpha=0.18, label="Overshoot (port)")
    _subtitle(ax_psi, "Heading", "deg")
    _legend(ax_psi, loc='upper right')
    axes_to_style.append(ax_psi)

    # Rudder (Row 1, Cols 2-3)
    ax_delta = fig.add_subplot(gs[1, 2:4], sharex=ax_psi)
    _line(ax_delta, t, delta_cmd, C_RUDDER, label="δ  (rudder cmd)")
    _hline(ax_delta, 0, color=BORDER, lw=0.8, ls='-')
    ax_delta.fill_between(t, 0, delta_cmd, color=C_RUDDER, alpha=0.15)
    _subtitle(ax_delta, "Rudder Angle", "deg")
    _legend(ax_delta, loc='upper right')
    axes_to_style.append(ax_delta)

    # 3. FORCES & MOMENTS (Row 2, all 4 columns)
    panels_forces = [
        (gs[2, 0], X/1e3, C_X, "X (Surge Force)", "kN"),
        (gs[2, 1], Y/1e3, C_Y, "Y (Sway Force)", "kN"),
        (gs[2, 2], K/1e3, C_K, "K (Roll Moment)", "kN·m"),
        (gs[2, 3], N/1e3, C_N, "N (Yaw Moment)", "kN·m"),
    ]
    for position, data, color, label, unit in panels_forces:
        ax = fig.add_subplot(position)
        _line(ax, t, data, color)
        _hline(ax, 0, color=BORDER, lw=0.8, alpha=0.9, ls='-')
        ax.fill_between(t, 0, data, color=color, alpha=0.12)
        _subtitle(ax, label, unit)
        axes_to_style.append(ax)

    # 4. KEY STATE VARIABLES (Row 3, all 4 columns)
    panels_states = [
        (gs[3, 0], u, C_SURGE, "u (Surge Velocity)", "m/s"),
        (gs[3, 1], v, C_SWAY, "v (Sway Velocity)", "m/s"),
        (gs[3, 2], r, C_YAW, "r (Yaw Rate)", "deg/s"),
        (gs[3, 3], phi, C_ROLL, "φ (Roll Angle)", "deg"),
    ]
    for position, data, color, label, unit in panels_states:
        ax = fig.add_subplot(position, sharex=ax_psi)
        _line(ax, t, data, color)
        _subtitle(ax, label, unit)
        axes_to_style.append(ax)

    _apply_style(fig, axes_to_style)
    
    return fig


# ──────────────────────────────────────────────
# ZIGZAG — INDIVIDUAL PLOTS (Kept for backwards compatibility)
# ──────────────────────────────────────────────
def plot_zigzag_states(t, y, delta_zz):
    fig = plt.figure(figsize=(16, 11))
    _title_style(fig, f"{int(delta_zz)}/{int(delta_zz)} ZigZag — State Variables")
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.38,
                           left=0.07, right=0.97, top=0.93, bottom=0.07)

    psi_wrapped = (np.rad2deg(y[11]) + 180) % 360 - 180

    specs = [
        (0, 0, y[0],              C_SURGE,  "u",            "m/s"),
        (0, 1, y[1],              C_SWAY,   "v",            "m/s"),
        (0, 2, np.rad2deg(y[3]), C_ROLL,   "p (roll rate)",  "deg/s"),
        (1, 0, np.rad2deg(y[4]), C_PITCH,  "q (pitch rate)", "deg/s"),
        (1, 1, np.rad2deg(y[5]), C_YAW,    "r (yaw rate)",   "deg/s"),
        (2, 0, np.rad2deg(y[9]),  C_ROLL,   "φ (roll)",       "deg"),
        (2, 1, np.rad2deg(y[10]), C_PITCH,  "θ (pitch)",      "deg"),
    ]

    axes = []
    for row, col, data, color, label, unit in specs:
        ax = fig.add_subplot(gs[row, col])
        _line(ax, t, data, color)
        _subtitle(ax, label, unit)
        axes.append(ax)

    ax_hide = fig.add_subplot(gs[1, 2])
    ax_hide.set_visible(False)

    ax_psi = fig.add_subplot(gs[2, 2])
    _line(ax_psi, t, psi_wrapped, C_PSI, label="ψ")
    _hline(ax_psi,  delta_zz, C_TRIG)
    _hline(ax_psi, -delta_zz, C_TRIG)
    ax_psi.fill_between(t, -delta_zz, delta_zz, color=C_TRIG, alpha=0.06)
    _subtitle(ax_psi, "ψ (heading)", "deg")
    _legend(ax_psi, loc='upper right')
    axes.append(ax_psi)

    _apply_style(fig, axes)
    return fig


def plot_zigzag_forces(t, X, Y, K, N):
    fig = plt.figure(figsize=(14, 9))
    _title_style(fig, "ZigZag — Hydrodynamic Forces & Moments")
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.35,
                           left=0.08, right=0.97, top=0.93, bottom=0.08)

    panels = [
        (0, 0, X/1e3, C_X,  "X  (Surge Force)", "kN"),
        (0, 1, Y/1e3, C_Y,  "Y  (Sway Force)",  "kN"),
        (1, 0, K/1e3, C_K,  "K  (Roll Moment)", "kN·m"),
        (1, 1, N/1e3, C_N,  "N  (Yaw Moment)",  "kN·m"),
    ]

    axes = []
    for row, col, data, color, label, unit in panels:
        ax = fig.add_subplot(gs[row, col])
        _line(ax, t, data, color)
        _hline(ax, 0, color=BORDER, lw=0.8, alpha=0.9, ls='-')
        ax.fill_between(t, 0, data, color=color, alpha=0.12)
        _subtitle(ax, label, unit)
        axes.append(ax)

    _apply_style(fig, axes)
    return fig


def plot_zigzag_standard(t, y, delta_cmd, delta_zz):
    psi_wrapped = (np.rad2deg(y[11]) + 180) % 360 - 180

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    _title_style(fig, f"{int(delta_zz)}/{int(delta_zz)} ZigZag — ITTC Standard Plot")
    fig.subplots_adjust(hspace=0.12, left=0.08, right=0.97, top=0.92, bottom=0.09)

    ax0 = axes[0]
    _line(ax0, t, psi_wrapped, C_PSI, label="ψ  (heading)")
    _hline(ax0,  delta_zz, C_TRIG)
    _hline(ax0, -delta_zz, C_TRIG)
    ax0.fill_between(t, -delta_zz, delta_zz, color=C_TRIG, alpha=0.07)
    ax0.fill_between(t, delta_zz, psi_wrapped,
                     where=(psi_wrapped > delta_zz),
                     color=C_MARK, alpha=0.18, label="Overshoot (stbd)")
    ax0.fill_between(t, psi_wrapped, -delta_zz,
                     where=(psi_wrapped < -delta_zz),
                     color=C_RUDDER, alpha=0.18, label="Overshoot (port)")
    ax0.set_ylabel("ψ  [deg]", color=TEXT_SEC, fontsize=9)
    _legend(ax0, loc='upper right')

    ax1 = axes[1]
    _line(ax1, t, delta_cmd, C_RUDDER, label="δ  (rudder command)")
    _hline(ax1, 0, color=BORDER, lw=0.8, ls='-')
    ax1.fill_between(t, 0, delta_cmd, color=C_RUDDER, alpha=0.15)
    ax1.set_ylabel("δ  [deg]", color=TEXT_SEC, fontsize=9)
    ax1.set_xlabel("t  [s]",   color=TEXT_SEC, fontsize=9)
    _legend(ax1, loc='upper right')

    _apply_style(fig, list(axes))
    return fig


def plot_zigzag_trajectory(y):
    x_    = y[6]
    y_pos = y[7]

    fig, ax = plt.subplots(figsize=(9, 7))
    fig.patch.set_facecolor(DARK_BG)
    ax.set_facecolor(PANEL_BG)

    points = np.array([y_pos, x_]).T.reshape(-1, 1, 2)
    segs   = np.concatenate([points[:-1], points[1:]], axis=1)
    lc     = LineCollection(segs, cmap='cool', linewidth=2.2)
    lc.set_array(np.linspace(0, 1, len(x_)))
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, label="Progress (t=0 → t_end)")
    cb.ax.yaxis.label.set_color(TEXT_SEC); cb.ax.tick_params(colors=TEXT_SEC)

    ax.scatter([y_pos[0]],  [x_[0]],   color=C_MARK,   s=70, zorder=5, label="Start", marker='o')
    ax.scatter([y_pos[-1]], [x_[-1]],  color=C_RUDDER, s=70, zorder=5, label="End",   marker='s')

    ax.autoscale()
    ax.set_aspect('equal', 'datalim')
    ax.set_xlabel("East  [m]",  color=TEXT_SEC, fontsize=9)
    ax.set_ylabel("North  [m]", color=TEXT_SEC, fontsize=9)
    ax.set_title("ZigZag Trajectory", color=TEXT_PRI, fontsize=11, fontweight='bold')
    ax.tick_params(colors=TEXT_SEC)
    ax.grid(True, color=GRID_CLR, linewidth=0.6)
    for sp in ax.spines.values(): sp.set_edgecolor(BORDER)
    _legend(ax, loc='upper left')
    plt.tight_layout()
    return fig