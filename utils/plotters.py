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
C_PSI2    = "#79c0ff"   # setpoint
C_TRIG    = "#f78166"   # trigger lines
C_MARK    = "#3fb950"   # markers / annotations
C_VLINE   = "#8b949e"   # vertical event lines
C_NORTH   = "#3fb950"   # north position
C_EAST    = "#f78166"   # east position


# ──────────────────────────────────────────────
# SHARED HELPERS
# ──────────────────────────────────────────────
def _apply_style(fig, axes_flat):
    fig.patch.set_facecolor(DARK_BG)
    for ax in axes_flat:
        ax.set_facecolor(PANEL_BG)
        ax.tick_params(colors=TEXT_SEC, labelsize=8, length=3)
        ax.xaxis.label.set_color(TEXT_SEC)
        ax.yaxis.label.set_color(TEXT_SEC)
        ax.title.set_color(TEXT_PRI)
        for spine in ax.spines.values():
            spine.set_edgecolor(BORDER)
        ax.grid(True, color=GRID_CLR, linewidth=0.6)
        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.grid(True, which='minor', color=GRID_CLR, linewidth=0.3, alpha=0.5)
        ax.set_xlabel("t  [s]", fontsize=8)


def _title_style(fig, title):
    fig.suptitle(title, color=TEXT_PRI, fontsize=12, fontweight='bold',
                 x=0.5, y=0.98, ha='center')


def _line(ax, x, y, color, label=None, lw=1.6, alpha=1.0, ls='-'):
    ax.plot(x, y, color=color, linewidth=lw, alpha=alpha, linestyle=ls, label=label)


def _hline(ax, y, color=C_TRIG, lw=1.0, ls='--', alpha=0.75):
    ax.axhline(y, color=color, linewidth=lw, linestyle=ls, alpha=alpha)


def _vline(ax, x, color=C_VLINE, lw=1.0, ls='--', alpha=0.75):
    ax.axvline(x, color=color, linewidth=lw, linestyle=ls, alpha=alpha)


def _subtitle(ax, label, unit):
    ax.set_title(f"{label}  [{unit}]", color=TEXT_PRI, fontsize=9, pad=4)


def _legend(ax, **kw):
    return ax.legend(fontsize=7.5, facecolor=DARK_BG, edgecolor=BORDER,
                     labelcolor=TEXT_PRI, **kw)


def _style_traj_ax(ax):
    """Apply dark theme to a spatial (non-time) trajectory axis."""
    ax.set_facecolor(PANEL_BG)
    ax.tick_params(colors=TEXT_SEC, labelsize=8, length=3)
    ax.xaxis.label.set_color(TEXT_SEC)
    ax.yaxis.label.set_color(TEXT_SEC)
    ax.title.set_color(TEXT_PRI)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)
    ax.grid(True, color=GRID_CLR, linewidth=0.5)
    ax.set_xlabel("East  [m]",  fontsize=9)
    ax.set_ylabel("North  [m]", fontsize=9)


def _traj_panel(ax, x_north, y_east, cmap='cool', title='2D Trajectory',
                start_label='Start', end_label='End',
                waypoints=None, waypoint_labels=None, waypoint_colors=None,
                force_equal=True, pad_frac=0.08):
    """
    Draw a gradient-coloured 2D NE-plane trajectory on an existing Axes.

    Parameters
    ----------
    ax              Matplotlib Axes (already created by caller)
    x_north         North position array [m]
    y_east          East  position array [m]
    cmap            Colourmap name for progress gradient
    title           Panel title string
    start_label     Legend label for start marker
    end_label       Legend label for end marker
    waypoints       List of (north, east) tuples for event markers
    waypoint_labels List of strings matching waypoints
    waypoint_colors List of colour strings matching waypoints
    force_equal     If True, enforce equal axis scaling (circular paths)
                    If False, auto-scale each axis independently (straight paths)
    pad_frac        Fractional padding added around the data range
    """
    _style_traj_ax(ax)

    # Gradient line via LineCollection
    pts  = np.array([y_east, x_north]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc   = LineCollection(segs, cmap=cmap, linewidth=2.2, zorder=3)
    lc.set_array(np.linspace(0, 1, len(x_north)))
    ax.add_collection(lc)

    # Compact colourbar — keep it thin and labelled simply
    cb = plt.colorbar(lc, ax=ax, pad=0.02, fraction=0.035, aspect=25)
    cb.set_label("t →", color=TEXT_SEC, fontsize=7, rotation=0, labelpad=4)
    cb.ax.yaxis.label.set_color(TEXT_SEC)
    cb.ax.tick_params(colors=TEXT_SEC, labelsize=6)
    cb.set_ticks([0, 0.5, 1])
    cb.set_ticklabels(['t₀', '½', 'tₑ'])

    # Explicit axis limits — prevents extreme width when East range ≈ 0
    e_range = max(abs(y_east).max() - y_east.min(), 20.0)  # min 20m east range
    n_range = max(x_north.max() - x_north.min(), 1.0)

    e_pad = e_range * pad_frac
    n_pad = n_range * pad_frac

    if force_equal:
        # Equal scaling: use the larger range for both axes
        half = max(e_range, n_range) * (0.5 + pad_frac)
        e_ctr = (y_east.max()  + y_east.min())  / 2
        n_ctr = (x_north.max() + x_north.min()) / 2
        ax.set_xlim(e_ctr - half, e_ctr + half)
        ax.set_ylim(n_ctr - half, n_ctr + half)
    else:
        # Independent scaling: each axis spans its own data range + padding
        e_ctr = (y_east.max() + y_east.min()) / 2
        e_half = max(e_range / 2 + e_pad, 15.0)   # minimum half-span of 15m
        ax.set_xlim(e_ctr - e_half, e_ctr + e_half)
        ax.set_ylim(x_north.min() - n_pad, x_north.max() + n_pad)

    # Start / End markers
    ax.scatter([y_east[0]],  [x_north[0]],  color=C_MARK,   s=70, zorder=6,
               marker='o', label=start_label)
    ax.scatter([y_east[-1]], [x_north[-1]], color=C_RUDDER, s=70, zorder=6,
               marker='s', label=end_label)

    # Optional event waypoints
    if waypoints:
        colors  = waypoint_colors or ['#ffa657'] * len(waypoints)
        labels_ = waypoint_labels or [f'Event {i+1}' for i in range(len(waypoints))]
        for (xn, ye), lbl, col in zip(waypoints, labels_, colors):
            ax.scatter([ye], [xn], color=col, s=90, zorder=7, marker='^', label=lbl)

    ax.set_title(title, color=TEXT_PRI, fontsize=9, pad=4, fontweight='bold')
    ax.legend(fontsize=7, facecolor=DARK_BG, edgecolor=BORDER,
              labelcolor=TEXT_PRI, loc='best', markerscale=0.9,
              borderpad=0.4, handlelength=1.2)
    return lc


# ──────────────────────────────────────────────
# STRAIGHT LINE
# ──────────────────────────────────────────────
def plot_straight_line(sol):
    """
    3×3 state grid on the left, 2D trajectory panel on the right column.
    The trajectory uses independent axis scaling (ship goes mostly North).
    """
    t = sol.t
    y = sol.y

    fig = plt.figure(figsize=(18, 11))
    _title_style(fig, "Straight-Line Run — State Variables & 2D Trajectory")
    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.58, wspace=0.42,
                           left=0.06, right=0.96, top=0.93, bottom=0.07)

    # ── State panels (left 3 cols × 3 rows) ───────────────────────────────
    specs = [
        (0, 0, y[0],               C_SURGE,  "u",           "m/s"),
        (0, 1, y[1],               C_SWAY,   "v",           "m/s"),
        (0, 2, y[2],               C_HEAVE,  "w",           "m/s"),
        (1, 0, np.rad2deg(y[9]),  C_ROLL,   "φ (roll)",    "deg"),
        (1, 1, np.rad2deg(y[10]), C_PITCH,  "θ (pitch)",   "deg"),
        (1, 2, np.rad2deg(y[11]), C_YAW,    "ψ (heading)", "deg"),
        (2, 0, y[6],               C_NORTH,  "x (North)",   "m"),
        (2, 1, y[7],               C_EAST,   "y (East)",    "m"),
        (2, 2, y[12],              C_RUDDER, "n_act",       "RPM"),
    ]
    axes = []
    for row, col, data, color, label, unit in specs:
        ax = fig.add_subplot(gs[row, col])
        _line(ax, t, data, color)
        _subtitle(ax, label, unit)
        axes.append(ax)
    _apply_style(fig, axes)

    # ── 2D Trajectory — right column, all rows ────────────────────────────
    ax_traj = fig.add_subplot(gs[:, 3])
    _traj_panel(ax_traj, y[6], y[7], cmap='cool',
                title='2D Trajectory\n(North–East)',
                force_equal=False)   # straight run — independent axes

    return fig


def plot_trajectory(x, y_pos):
    """Standalone trajectory figure (backward compatibility)."""
    fig, ax = plt.subplots(figsize=(8, 7))
    fig.patch.set_facecolor(DARK_BG)
    _traj_panel(ax, x, y_pos, cmap='cool', title='Trajectory (North–East)',
                force_equal=False)
    plt.tight_layout()
    return fig


# ──────────────────────────────────────────────
# TURNING CIRCLE
# ──────────────────────────────────────────────
def plot_turning_circle(results):
    """
    Left column: 2D trajectory with equal-aspect circle.
    Right column: Roll, Yaw rate, Speed vs time — all marked with rudder-in event.
    """
    t        = results["t"]
    y        = results["y"]
    t_rudder = results["t_rudder"]
    x_  = y[6];  y_  = y[7]
    phi = np.rad2deg(y[9])
    r   = np.rad2deg(y[5])
    V   = np.sqrt(y[0]**2 + y[1]**2 + y[2]**2)

    idx_rud = np.searchsorted(t, t_rudder)

    fig = plt.figure(figsize=(16, 9))
    _title_style(fig, "Turning Circle Maneuver — 2D Trajectory & State History")
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.55, wspace=0.35,
                           left=0.07, right=0.96, top=0.93, bottom=0.07)

    # Trajectory — full left column, equal aspect
    ax_traj = fig.add_subplot(gs[:, 0])
    _traj_panel(ax_traj, x_, y_, cmap='plasma',
                title='2D Trajectory (North–East)',
                waypoints=[(x_[idx_rud], y_[idx_rud])],
                waypoint_labels=['Rudder applied'],
                waypoint_colors=['#ffa657'],
                force_equal=True)

    # Right column: state time-series
    panels = [
        (gs[0, 1], phi, C_ROLL,  "φ  (Roll)",     "deg"),
        (gs[1, 1], r,   C_YAW,   "r  (Yaw Rate)", "deg/s"),
        (gs[2, 1], V,   C_SURGE, "V  (Speed)",    "m/s"),
    ]
    axes_right = []
    for gspec, data, color, label, unit in panels:
        ax = fig.add_subplot(gspec)
        _line(ax, t, data, color)
        _vline(ax, t_rudder)
        _subtitle(ax, label, unit)
        axes_right.append(ax)
    _apply_style(fig, axes_right)
    fig.patch.set_facecolor(DARK_BG)
    return fig


# ──────────────────────────────────────────────
# ZIGZAG — DASHBOARD
# ──────────────────────────────────────────────
def plot_zigzag_dashboard(t, y, delta_cmd, delta_zz, X, Y, K, N,
                          setpoint=None, mode='open-loop'):
    """
    4-row dashboard:
      Row 0-1 left:  compact 2D trajectory
      Row 0 right:   Heading (ITTC standard)
      Row 1 right:   Rudder command
      Row 2:         Forces × 4
      Row 3:         Key states × 4
    """
    psi_wrapped = (np.rad2deg(y[11]) + 180) % 360 - 180
    x_   = y[6];  y_pos = y[7]
    u    = y[0];  v = y[1]
    r    = np.rad2deg(y[5])
    phi  = np.rad2deg(y[9])

    mode_label = ' — Closed-Loop PID' if mode == 'closed-loop' else ' — Open-Loop'
    fig = plt.figure(figsize=(18, 14))
    _title_style(fig, f"{int(delta_zz)}/{int(delta_zz)} ZigZag Maneuver{mode_label}")
    gs = gridspec.GridSpec(4, 4, figure=fig, hspace=0.65, wspace=0.42,
                           left=0.06, right=0.96, top=0.93, bottom=0.06)

    axes_to_style = []

    # ── Compact trajectory (rows 0-1, cols 0-1) ───────────────────────────
    ax_traj = fig.add_subplot(gs[0:2, 0:2])
    _traj_panel(ax_traj, x_, y_pos, cmap='cool',
                title='2D Trajectory (North–East)',
                force_equal=False)
    ax_traj.set_xlabel("East  [m]", fontsize=8)
    axes_to_style.append(ax_traj)

    # ── Heading — row 0, cols 2-3 ─────────────────────────────────────────
    ax_psi = fig.add_subplot(gs[0, 2:4])
    if setpoint is not None:
        ax_psi.step(t, setpoint, color=C_PSI2, lw=1.0, ls='--', alpha=0.75,
                    where='post', label='ψ_desired')
    _line(ax_psi, t, psi_wrapped, C_PSI, label="ψ  (heading)")
    _hline(ax_psi,  delta_zz, C_TRIG)
    _hline(ax_psi, -delta_zz, C_TRIG)
    ax_psi.fill_between(t, -delta_zz, delta_zz, color=C_TRIG, alpha=0.07)
    ax_psi.fill_between(t,  delta_zz, psi_wrapped,
                         where=(psi_wrapped > delta_zz),
                         color=C_MARK,   alpha=0.18, label="Overshoot (stbd)")
    ax_psi.fill_between(t, psi_wrapped, -delta_zz,
                         where=(psi_wrapped < -delta_zz),
                         color=C_RUDDER, alpha=0.18, label="Overshoot (port)")
    _subtitle(ax_psi, "Heading", "deg")
    _legend(ax_psi, loc='upper right')
    axes_to_style.append(ax_psi)

    # ── Rudder — row 1, cols 2-3 ──────────────────────────────────────────
    ax_delta = fig.add_subplot(gs[1, 2:4], sharex=ax_psi)
    _line(ax_delta, t, delta_cmd, C_RUDDER, label="δ  (rudder cmd)")
    _hline(ax_delta, 0, color=BORDER, lw=0.8, ls='-')
    ax_delta.fill_between(t, 0, delta_cmd, color=C_RUDDER, alpha=0.15)
    _subtitle(ax_delta, "Rudder Angle", "deg")
    _legend(ax_delta, loc='upper right')
    axes_to_style.append(ax_delta)

    # ── Forces — row 2 ────────────────────────────────────────────────────
    for col, (data, color, label, unit) in enumerate([
        (X/1e3, C_X, "X (Surge Force)", "kN"),
        (Y/1e3, C_Y, "Y (Sway Force)",  "kN"),
        (K/1e3, C_K, "K (Roll Moment)", "kN·m"),
        (N/1e3, C_N, "N (Yaw Moment)",  "kN·m"),
    ]):
        ax = fig.add_subplot(gs[2, col])
        _line(ax, t, data, color)
        _hline(ax, 0, color=BORDER, lw=0.8, ls='-')
        ax.fill_between(t, 0, data, color=color, alpha=0.12)
        _subtitle(ax, label, unit)
        axes_to_style.append(ax)

    # ── Key States — row 3 ────────────────────────────────────────────────
    for col, (data, color, label, unit) in enumerate([
        (u,   C_SURGE, "u (Surge)",    "m/s"),
        (v,   C_SWAY,  "v (Sway)",     "m/s"),
        (r,   C_YAW,   "r (Yaw Rate)", "deg/s"),
        (phi, C_ROLL,  "φ (Roll)",     "deg"),
    ]):
        ax = fig.add_subplot(gs[3, col], sharex=ax_psi)
        _line(ax, t, data, color)
        _subtitle(ax, label, unit)
        axes_to_style.append(ax)

    _apply_style(fig, axes_to_style)

    # Fix spatial axes (trajectory panels must not have 't [s]' label)
    ax_traj.set_xlabel("East  [m]", fontsize=8, color=TEXT_SEC)

    return fig


# ──────────────────────────────────────────────
# STOPPING TRIAL
# ──────────────────────────────────────────────
def plot_stopping_trial(t, y, n_cmd_arr, d_arr, results):
    """
    Left (2/3 width): 3 state time-series stacked (surge, shaft, rudder).
    Right (1/3 width): 2D trajectory — straight crash-stop path with event markers.
    """
    u   = y[0];  x_ = y[6];  y_ = y[7]

    idx_rev  = np.searchsorted(t, results['t_reversal']) if results['t_reversal'] else None
    idx_stop = np.searchsorted(t, results['t_stop'])     if results['t_stop']     else None

    pass_str = '✓ PASS' if results['imo_pass'] else '✗ FAIL'
    fig = plt.figure(figsize=(16, 9))
    _title_style(fig,
        f"Stopping Trial  [{results['mode']}]\n"
        f"Head reach: {results['head_reach']:.0f} m  "
        f"({results['head_reach_L']:.2f} L)  "
        f"[IMO ≤ 15 L]   {pass_str}")
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.52, wspace=0.38,
                           left=0.07, right=0.96, top=0.90, bottom=0.07)

    # ── State time-series (left 2 cols, 3 rows) ───────────────────────────
    axes = []
    for row, (data, color, label, unit) in enumerate([
        (u,         C_SURGE,  "u  (surge velocity)", "m/s"),
        (n_cmd_arr, C_RUDDER, "n_cmd  (shaft speed)", "RPM"),
        (d_arr,     C_PITCH,  "δ  (rudder)",          "deg"),
    ]):
        ax = fig.add_subplot(gs[row, 0:2])
        _line(ax, t, data, color)
        _hline(ax, 0, color=BORDER, lw=0.8, ls='-')
        _subtitle(ax, label, unit)
        if results['t_reversal']:
            _vline(ax, results['t_reversal'], color='#f78166')
            ax.text(results['t_reversal'] + 0.5,
                    data.max() * 0.85,
                    'Engine reversed', color='#f78166', fontsize=7)
        if results['t_stop']:
            _vline(ax, results['t_stop'], color=C_MARK)
            ax.text(results['t_stop'] + 0.5,
                    data.max() * 0.65,
                    'Full stop', color=C_MARK, fontsize=7)
        axes.append(ax)
    _apply_style(fig, axes)

    # ── 2D Trajectory (right col, all rows) ───────────────────────────────
    ax_traj = fig.add_subplot(gs[:, 2])

    wps, wp_lbl, wp_col = [], [], []
    if idx_rev is not None:
        wps.append((x_[idx_rev], y_[idx_rev]))
        wp_lbl.append('Engine reversed')
        wp_col.append('#f78166')
    if idx_stop is not None:
        wps.append((x_[idx_stop], y_[idx_stop]))
        wp_lbl.append('Full stop')
        wp_col.append(C_MARK)

    _traj_panel(ax_traj, x_, y_, cmap='inferno',
                title='2D Trajectory\n(North–East)',
                waypoints=wps or None,
                waypoint_labels=wp_lbl or None,
                waypoint_colors=wp_col or None,
                force_equal=False)    # crash-stop is nearly straight

    # Annotate head reach on trajectory
    if idx_rev is not None and idx_stop is not None:
        n_mid = (x_[idx_rev] + x_[idx_stop]) / 2
        e_mid = (y_[idx_rev] + y_[idx_stop]) / 2 + max(abs(y_).max() * 0.15, 5)
        ax_traj.annotate(
            f"Head reach\n{results['head_reach']:.0f} m",
            xy=(y_[idx_stop], x_[idx_stop]),
            xytext=(e_mid, n_mid),
            color=TEXT_PRI, fontsize=7,
            ha='center',
            arrowprops=dict(arrowstyle='->', color=TEXT_SEC, lw=0.9))

    ax_traj.set_xlabel("East  [m]", fontsize=9, color=TEXT_SEC)
    return fig


# ──────────────────────────────────────────────
# PULL-OUT MANEUVER
# ──────────────────────────────────────────────
def plot_pullout(t, y, d_arr, results):
    """
    Left (2/3 width): Yaw rate, Rudder, Heading vs time.
    Right (1/3 width): 2D trajectory — straight + arc + coast path.
    """
    r   = np.rad2deg(y[5]);  psi = np.rad2deg(y[11])
    x_  = y[6];              y_  = y[7]
    t_in  = results['t_rudder_in']
    t_out = results['t_rudder_out']
    idx_in  = np.searchsorted(t, t_in)
    idx_out = np.searchsorted(t, t_out)

    verdict_col = C_MARK if results['stable'] else '#f78166'
    fig = plt.figure(figsize=(16, 9))
    _title_style(fig,
        f"Pull-Out Maneuver  [{results['mode']}]  —  {results['verdict']}\n"
        f"r_steady = {results['r_steady_ss']:.3f} °/s   "
        f"r_final = {results['r_final']:.5f} °/s")
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.52, wspace=0.38,
                           left=0.07, right=0.96, top=0.90, bottom=0.07)

    # ── State time-series (left 2 cols) ───────────────────────────────────
    axes = []
    for row, (data, color, label, unit) in enumerate([
        (r,     C_YAW,    "r  (yaw rate)",  "deg/s"),
        (d_arr, C_RUDDER, "δ  (rudder)",    "deg"),
        (psi,   C_PSI,    "ψ  (heading)",   "deg"),
    ]):
        ax = fig.add_subplot(gs[row, 0:2])
        _line(ax, t, data, color)
        _hline(ax, 0, color=BORDER, lw=0.8, ls='-')
        _vline(ax, t_in,  color='#ffa657')
        _vline(ax, t_out, color='#f78166')
        if row == 0:
            y_top = data.max() if data.max() != 0 else 0.1
            ax.text(t_in  + 2, y_top * 0.85, 'Rudder in',  color='#ffa657', fontsize=7)
            ax.text(t_out + 2, y_top * 0.85, 'Rudder out', color='#f78166', fontsize=7)
            ax.text(0.97, 0.08, results['verdict'],
                    transform=ax.transAxes, color=verdict_col,
                    fontsize=7.5, ha='right', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor=DARK_BG,
                              edgecolor=verdict_col, alpha=0.85))
        _subtitle(ax, label, unit)
        axes.append(ax)
    _apply_style(fig, axes)

    # ── 2D Trajectory (right col) ─────────────────────────────────────────
    ax_traj = fig.add_subplot(gs[:, 2])
    _traj_panel(ax_traj, x_, y_, cmap='cool',
                title='2D Trajectory\n(North–East)',
                waypoints=[
                    (x_[idx_in],  y_[idx_in]),
                    (x_[idx_out], y_[idx_out]),
                ],
                waypoint_labels=['Rudder in', 'Rudder out'],
                waypoint_colors=['#ffa657', '#f78166'],
                force_equal=True)    # arc path — keep proportional
    ax_traj.set_xlabel("East  [m]", fontsize=9, color=TEXT_SEC)
    return fig


# ──────────────────────────────────────────────
# SPIRAL MANEUVER
# ──────────────────────────────────────────────
def plot_spiral(t, y, d_arr, spiral_delta, spiral_r, results):
    """
    2×2 layout:
      Top-left:     r(δ) hysteresis curve
      Top-right:    Yaw rate + Heading time history
      Bottom-left:  Rudder command time history
      Bottom-right: 2D trajectory (the ship spirals back to start)
    """
    r   = np.rad2deg(y[5])
    psi = (np.rad2deg(y[11]) + 180) % 360 - 180
    x_  = y[6];  y_  = y[7]

    verdict_col = C_MARK if results['stable'] else '#f78166'
    x_label = 'ψ_target  [deg]' if 'heading' in results['mode'] else 'δ  [deg]'

    fig = plt.figure(figsize=(16, 10))
    _title_style(fig,
        f"Spiral Maneuver (Dieudonné)  [{results['mode']}]  —  {results['verdict']}\n"
        f"Max hysteresis: {results['max_hysteresis']:.4f} °/s")
    gs = gridspec.GridSpec(2, 4, figure=fig, hspace=0.52, wspace=0.45,
                           left=0.07, right=0.96, top=0.90, bottom=0.07)

    # ── r(δ) curve — top-left 2 cols ──────────────────────────────────────
    ax_rd = fig.add_subplot(gs[0, 0:2])
    ax_rd.plot(results['d_descend'], results['r_descend'],
               color=C_SURGE, lw=2.0, marker='o', ms=5,
               label=f'Descend (+→−)')
    ax_rd.plot(results['d_ascend'],  results['r_ascend'],
               color=C_MARK, lw=2.0, marker='s', ms=5, ls='--',
               label=f'Ascend  (−→+)')
    ax_rd.axhline(0, color=BORDER, lw=0.8)
    ax_rd.axvline(0, color=BORDER, lw=0.8)
    ax_rd.set_xlabel(x_label, fontsize=9, color=TEXT_SEC)
    ax_rd.set_ylabel('r_ss  [deg/s]', fontsize=9, color=TEXT_SEC)
    ax_rd.set_facecolor(PANEL_BG)
    ax_rd.tick_params(colors=TEXT_SEC, labelsize=8)
    for sp in ax_rd.spines.values(): sp.set_edgecolor(BORDER)
    ax_rd.grid(True, color=GRID_CLR, lw=0.5)
    ax_rd.set_title('Steady Yaw Rate vs Rudder/Heading  [r(δ) curve]',
                    color=TEXT_PRI, fontsize=9, pad=4, fontweight='bold')
    _legend(ax_rd)
    ax_rd.text(0.03, 0.07, results['verdict'].replace(' — ', '\n'),
               transform=ax_rd.transAxes, color=verdict_col, fontsize=7.5,
               fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.3', facecolor=DARK_BG,
                         edgecolor=verdict_col, alpha=0.85))

    # ── Yaw rate + Heading — top-right 2 cols ─────────────────────────────
    ax_r = fig.add_subplot(gs[0, 2:4])
    _line(ax_r, t, r,   C_YAW,  label='r  (yaw rate)')
    _line(ax_r, t, psi, C_PSI,  label='ψ  (heading)', ls='--', alpha=0.7)
    _hline(ax_r, 0, color=BORDER, lw=0.8, ls='-')
    _subtitle(ax_r, 'Yaw Rate & Heading', 'deg/s  /  deg')
    _legend(ax_r)
    time_axes = [ax_r]

    # ── Rudder cmd — bottom-left 2 cols ───────────────────────────────────
    ax_d = fig.add_subplot(gs[1, 0:2])
    _line(ax_d, t, d_arr, C_RUDDER, label='δ  (rudder cmd)')
    _hline(ax_d, 0, color=BORDER, lw=0.8, ls='-')
    _subtitle(ax_d, 'Rudder Command', 'deg')
    _legend(ax_d)
    time_axes.append(ax_d)
    _apply_style(fig, time_axes)

    # ── 2D Trajectory — bottom-right 2 cols ───────────────────────────────
    ax_traj = fig.add_subplot(gs[1, 2:4])
    _traj_panel(ax_traj, x_, y_, cmap='viridis',
                title='2D Trajectory (North–East)',
                force_equal=True)
    ax_traj.set_xlabel("East  [m]", fontsize=9, color=TEXT_SEC)
    return fig


# ──────────────────────────────────────────────
# ZIGZAG — INDIVIDUAL (backward compatibility)
# ──────────────────────────────────────────────
def plot_zigzag_states(t, y, delta_zz):
    fig = plt.figure(figsize=(16, 11))
    _title_style(fig, f"{int(delta_zz)}/{int(delta_zz)} ZigZag — State Variables")
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.38,
                           left=0.07, right=0.97, top=0.93, bottom=0.07)
    psi_wrapped = (np.rad2deg(y[11]) + 180) % 360 - 180
    specs = [
        (0, 0, y[0],               C_SURGE, "u",             "m/s"),
        (0, 1, y[1],               C_SWAY,  "v",             "m/s"),
        (0, 2, np.rad2deg(y[3]),  C_ROLL,  "p (roll rate)", "deg/s"),
        (1, 0, np.rad2deg(y[4]),  C_PITCH, "q (pitch rate)","deg/s"),
        (1, 1, np.rad2deg(y[5]),  C_YAW,   "r (yaw rate)",  "deg/s"),
        (2, 0, np.rad2deg(y[9]),  C_ROLL,  "φ (roll)",      "deg"),
        (2, 1, np.rad2deg(y[10]), C_PITCH, "θ (pitch)",     "deg"),
    ]
    axes = []
    for row, col, data, color, label, unit in specs:
        ax = fig.add_subplot(gs[row, col])
        _line(ax, t, data, color)
        _subtitle(ax, label, unit)
        axes.append(ax)
    fig.add_subplot(gs[1, 2]).set_visible(False)
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
        (0, 0, X/1e3, C_X, "X  (Surge Force)", "kN"),
        (0, 1, Y/1e3, C_Y, "Y  (Sway Force)",  "kN"),
        (1, 0, K/1e3, C_K, "K  (Roll Moment)", "kN·m"),
        (1, 1, N/1e3, C_N, "N  (Yaw Moment)",  "kN·m"),
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
    ax0.fill_between(t,  delta_zz, psi_wrapped,
                     where=(psi_wrapped >  delta_zz),
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
    """Standalone zigzag trajectory (backward compatibility)."""
    fig, ax = plt.subplots(figsize=(9, 7))
    fig.patch.set_facecolor(DARK_BG)
    _traj_panel(ax, y[6], y[7], cmap='cool',
                title='ZigZag Trajectory (North–East)',
                force_equal=False)
    ax.set_xlabel("East  [m]", fontsize=9, color=TEXT_SEC)
    plt.tight_layout()
    return fig