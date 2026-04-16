# utils/comparison_plotter.py
"""
Side-by-side Open-Loop vs Closed-Loop PID comparison plots.

Each function takes pre-run OL and CL data and produces a comparison figure.
All NE-plane trajectories use _traj_panel from plotters.py.

Colour convention
-----------------
  Open-loop  :  coral   #f78166
  Closed-loop:  green   #3fb950
  Trigger / limit lines : grey  #8b949e
  Setpoint trace        : lavender #d2a8ff
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from matplotlib.ticker import AutoMinorLocator

# Re-use the shared trajectory helper from plotters
from utils.plotters import _traj_panel

# ── Design tokens ─────────────────────────────────────────────────────────
DARK   = '#0d1117'; PANEL  = '#161b22'; GRID   = '#21262d'
BORDER = '#30363d'; PRI    = '#e6edf3'; SEC    = '#8b949e'
C_OL   = '#f78166'   # open-loop  — coral
C_CL   = '#3fb950'   # closed-loop — green
C_TRIG = '#8b949e'   # limit / trigger lines
C_SETP = '#d2a8ff'   # PID setpoint trace — lavender
C_BLUE = '#58a6ff'   # general accent
C_WARN = '#ffa657'   # amber


def _sty(ax, ylabel=None, title=None, xlabel='t  [s]'):
    ax.set_facecolor(PANEL)
    ax.tick_params(colors=SEC, labelsize=8, length=3)
    ax.xaxis.label.set_color(SEC); ax.yaxis.label.set_color(SEC)
    ax.title.set_color(PRI)
    for sp in ax.spines.values(): sp.set_edgecolor(BORDER)
    ax.grid(True, color=GRID, lw=0.5)
    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.yaxis.set_minor_locator(AutoMinorLocator())
    ax.grid(True, which='minor', color=GRID, lw=0.25, alpha=0.4)
    if ylabel: ax.set_ylabel(ylabel, fontsize=8.5, color=SEC)
    if title:  ax.set_title(title,   fontsize=9,   color=PRI, pad=4, fontweight='bold')
    if xlabel: ax.set_xlabel(xlabel, fontsize=8,   color=SEC)


def _leg(ax, **kw):
    ax.legend(fontsize=7.5, facecolor=DARK, edgecolor=BORDER, labelcolor=PRI, **kw)


def _metric_box(ax, text, color=C_CL, loc='lower right'):
    x  = 0.97 if 'right' in loc else 0.03
    y  = 0.05 if 'lower' in loc else 0.92
    ha = 'right' if 'right' in loc else 'left'
    va = 'bottom' if 'lower' in loc else 'top'
    ax.text(x, y, text, transform=ax.transAxes,
            color=color, fontsize=7, ha=ha, va=va, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.35', facecolor=DARK,
                      edgecolor=color, alpha=0.88))


OL_LEG = Line2D([0],[0], color=C_OL, lw=2, label='Open-Loop')
CL_LEG = Line2D([0],[0], color=C_CL, lw=2, ls='--', label='Closed-Loop PID')


# ─────────────────────────────────────────────────────────────────────────────
# 1.  STRAIGHT LINE
# ─────────────────────────────────────────────────────────────────────────────
def plot_comparison_straightline(sol_ol, t_cl, y_cl, d_cl):
    """
    Compare straight-line run: zero-rudder vs PID course-keeping.
    Adds an NE-plane trajectory panel showing how well each mode tracks North.
    """
    t_ol = sol_ol.t;  y_ol = sol_ol.y
    psi_ol = np.rad2deg(y_ol[11]);  psi_cl = np.rad2deg(y_cl[11])

    fig = plt.figure(figsize=(18, 11))
    fig.patch.set_facecolor(DARK)
    fig.suptitle('Straight-Line Run — Open-Loop vs Closed-Loop PID Course-Keeping',
                 color=PRI, fontsize=12, fontweight='bold', y=0.99)
    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.58, wspace=0.40,
                           left=0.06, right=0.96, top=0.95, bottom=0.06)

    panels = [
        (0, 0, y_ol[0],  y_cl[0],  'u  [m/s]',      'Surge Velocity'),
        (0, 1, y_ol[1],  y_cl[1],  'v  [m/s]',      'Sway Velocity'),
        (0, 2, psi_ol,   psi_cl,   'ψ  [deg]',      'Heading'),
        (1, 0, d_cl*0,   d_cl,     'δ  [deg]',      'Rudder Command'),
        (1, 1, y_ol[7],  y_cl[7],  'y  [m East]',   'East Drift'),
        (2, 0, y_ol[0],  y_cl[0],  'u  [m/s]',      'Surge (overlay)'),
    ]

    axs = []
    for row, col, ol, cl, ylabel, title in [
        (0, 0, y_ol[0],  y_cl[0],  'u  [m/s]',    'Surge Velocity'),
        (0, 1, y_ol[1],  y_cl[1],  'v  [m/s]',    'Sway Velocity'),
        (0, 2, psi_ol,   psi_cl,   'ψ  [deg]',    'Heading'),
        (1, 0, t_ol*0,   d_cl,     'δ  [deg]',    'Rudder (OL=0, CL=PID)'),
        (1, 1, y_ol[7],  y_cl[7],  'y  [m East]', 'East Drift (target: 0)'),
        (2, 0, y_ol[6],  y_cl[6],  'x  [m North]','North Position'),
    ]:
        ax = fig.add_subplot(gs[row, col])
        if row == 1 and col == 0:
            ax.axhline(0, color=C_OL, lw=1.8, label='Open-Loop δ = 0°')
            ax.plot(t_cl, cl, color=C_CL, lw=1.8, label='Closed-Loop PID δ')
        else:
            ax.plot(t_ol, ol, color=C_OL, lw=1.8, label='Open-Loop')
            ax.plot(t_cl, cl, color=C_CL, lw=1.8, ls='--', label='Closed-Loop PID')
        ax.axhline(0, color=BORDER, lw=0.6)
        _sty(ax, ylabel=ylabel, title=title)
        _leg(ax)
        axs.append(ax)

    # Metric annotation on East drift
    drift_ol = abs(y_ol[7, -1]); drift_cl = abs(y_cl[7, -1])
    _metric_box(axs[4], f'OL drift: {drift_ol:.3f} m\nCL drift: {drift_cl:.4f} m')

    # NE-plane trajectory — right 2 cols, rows 1-2
    ax_traj = fig.add_subplot(gs[1:3, 2:4])
    # Plot OL and CL trajectories on the same panel in their respective colours
    ax_traj.set_facecolor(PANEL)
    ax_traj.plot(y_ol[7], y_ol[6], color=C_OL, lw=2, label='Open-Loop')
    ax_traj.plot(y_cl[7], y_cl[6], color=C_CL, lw=2, ls='--', label='Closed-Loop PID')
    ax_traj.scatter([y_ol[7, 0]], [y_ol[6, 0]], color=C_OL, s=60, zorder=5, marker='o')
    ax_traj.scatter([y_ol[7,-1]], [y_ol[6,-1]], color=C_OL, s=60, zorder=5, marker='s')
    ax_traj.scatter([y_cl[7, 0]], [y_cl[6, 0]], color=C_CL, s=60, zorder=5, marker='o')
    ax_traj.scatter([y_cl[7,-1]], [y_cl[6,-1]], color=C_CL, s=60, zorder=5, marker='s')
    ax_traj.axvline(0, color=BORDER, lw=0.8, ls='--', alpha=0.6)
    ax_traj.tick_params(colors=SEC, labelsize=8)
    for sp in ax_traj.spines.values(): sp.set_edgecolor(BORDER)
    ax_traj.grid(True, color=GRID, lw=0.5)
    ax_traj.set_xlabel('East  [m]', fontsize=9, color=SEC)
    ax_traj.set_ylabel('North  [m]', fontsize=9, color=SEC)
    ax_traj.set_title('2D Trajectory Comparison (North–East)',
                      color=PRI, fontsize=9, pad=4, fontweight='bold')
    e_max = max(abs(y_ol[7]).max(), abs(y_cl[7]).max(), 15.0)
    ax_traj.set_xlim(-e_max * 1.3, e_max * 1.3)
    _leg(ax_traj)
    _metric_box(ax_traj, f'CL eliminates heading drift\nPID holds ψ = 0°', color=C_CL, loc='upper left')

    return fig


# ─────────────────────────────────────────────────────────────────────────────
# 2.  ZIGZAG  (10/10 or 20/20)
# ─────────────────────────────────────────────────────────────────────────────
def plot_comparison_zigzag(t_ol, y_ol, d_ol, os_ol,
                            t_cl, y_cl, d_cl, os_cl,
                            delta_zz, setpoint=None):
    """
    Compare zigzag: bang-bang vs PID.
    Includes NE-plane trajectory overlay showing the snake-like lateral path.
    """
    psi_ol = (np.rad2deg(y_ol[11]) + 180) % 360 - 180
    psi_cl = (np.rad2deg(y_cl[11]) + 180) % 360 - 180
    phi_ol = np.rad2deg(y_ol[9]);  phi_cl = np.rad2deg(y_cl[9])
    r_ol   = np.rad2deg(y_ol[5]);  r_cl   = np.rad2deg(y_cl[5])

    fig = plt.figure(figsize=(18, 14))
    fig.patch.set_facecolor(DARK)
    fig.suptitle(f'{int(delta_zz)}/{int(delta_zz)} ZigZag — Open-Loop vs Closed-Loop PID',
                 color=PRI, fontsize=12, fontweight='bold', y=0.99)
    gs = gridspec.GridSpec(4, 4, figure=fig, hspace=0.58, wspace=0.40,
                           left=0.06, right=0.96, top=0.95, bottom=0.05)

    # ── Row 0: Heading (full width) ────────────────────────────────────────
    ax_psi = fig.add_subplot(gs[0, :])
    ax_psi.axhline( delta_zz, color=C_TRIG, lw=0.9, ls='--', alpha=0.7)
    ax_psi.axhline(-delta_zz, color=C_TRIG, lw=0.9, ls='--', alpha=0.7)
    ax_psi.fill_between(t_ol, -delta_zz, delta_zz, color=C_TRIG, alpha=0.05)
    ax_psi.fill_between(t_ol,  delta_zz, psi_ol, where=(psi_ol >  delta_zz), color=C_OL, alpha=0.20)
    ax_psi.fill_between(t_ol, psi_ol, -delta_zz, where=(psi_ol < -delta_zz), color=C_OL, alpha=0.20)
    ax_psi.fill_between(t_cl,  delta_zz, psi_cl, where=(psi_cl >  delta_zz), color=C_CL, alpha=0.20)
    ax_psi.fill_between(t_cl, psi_cl, -delta_zz, where=(psi_cl < -delta_zz), color=C_CL, alpha=0.20)
    if setpoint is not None:
        ax_psi.step(t_cl, setpoint, color=C_SETP, lw=0.9, ls=':', alpha=0.7,
                    where='post', label='Setpoint ψ_desired')
    ax_psi.plot(t_ol, psi_ol, color=C_OL, lw=2.0,
                label=f'Open-Loop   os={os_ol[0]:.2f}° / {os_ol[1]:.2f}°')
    ax_psi.plot(t_cl, psi_cl, color=C_CL, lw=2.0,
                label=f'Closed-Loop os={os_cl[0]:.2f}° / {os_cl[1]:.2f}°')
    _sty(ax_psi, ylabel='ψ  [deg]', title='Heading Response')
    _leg(ax_psi, loc='lower right', ncols=3)
    reduction = (1 - os_cl[0] / os_ol[0]) * 100 if os_ol[0] > 0 else 0
    _metric_box(ax_psi, f'Overshoot reduction: {reduction:.0f}%\n'
                         f'OL: {os_ol[0]:.2f}° / {os_ol[1]:.2f}°\n'
                         f'CL: {os_cl[0]:.2f}° / {os_cl[1]:.2f}°', loc='lower left')

    # ── Row 1: Rudder + Yaw rate ───────────────────────────────────────────
    ax_d = fig.add_subplot(gs[1, 0:2])
    ax_d.step(t_ol, d_ol, color=C_OL, lw=1.6, where='post', label='Open-Loop (bang-bang)')
    ax_d.plot(t_cl, d_cl, color=C_CL, lw=1.5, label='Closed-Loop PID')
    ax_d.axhline(0, color=BORDER, lw=0.7)
    _sty(ax_d, ylabel='δ  [deg]', title='Rudder Command')
    _leg(ax_d)
    _metric_box(ax_d, f'OL: ±{abs(d_ol).max():.1f}°  CL: ±{abs(d_cl).max():.1f}°')

    ax_r = fig.add_subplot(gs[1, 2:4])
    ax_r.plot(t_ol, r_ol, color=C_OL, lw=1.6, label='Open-Loop r')
    ax_r.plot(t_cl, r_cl, color=C_CL, lw=1.6, ls='--', label='Closed-Loop r')
    ax_r.axhline(0, color=BORDER, lw=0.7)
    _sty(ax_r, ylabel='r  [deg/s]', title='Yaw Rate')
    _leg(ax_r)

    # ── Row 2: Roll + Sway ────────────────────────────────────────────────
    ax_phi = fig.add_subplot(gs[2, 0:2])
    ax_phi.plot(t_ol, phi_ol, color=C_OL, lw=1.6, label=f'OL  pk={abs(phi_ol).max():.1f}°')
    ax_phi.plot(t_cl, phi_cl, color=C_CL, lw=1.6, ls='--', label=f'CL  pk={abs(phi_cl).max():.1f}°')
    ax_phi.axhline(0, color=BORDER, lw=0.7)
    _sty(ax_phi, ylabel='φ  [deg]', title='Roll Angle')
    _leg(ax_phi)

    ax_v = fig.add_subplot(gs[2, 2:4])
    ax_v.plot(t_ol, y_ol[1], color=C_OL, lw=1.6, label='OL sway v')
    ax_v.plot(t_cl, y_cl[1], color=C_CL, lw=1.6, ls='--', label='CL sway v')
    ax_v.axhline(0, color=BORDER, lw=0.7)
    _sty(ax_v, ylabel='v  [m/s]', title='Sway Velocity')
    _leg(ax_v)

    # ── Row 3: NE-plane trajectories side by side ─────────────────────────
    ax_tol = fig.add_subplot(gs[3, 0:2])
    _traj_panel(ax_tol, y_ol[6], y_ol[7], cmap='cool',
                title=f'OL Trajectory  (os={os_ol[0]:.2f}°)',
                force_equal=False)
    ax_tol.set_xlabel('East  [m]', fontsize=8, color=SEC)

    ax_tcl = fig.add_subplot(gs[3, 2:4])
    _traj_panel(ax_tcl, y_cl[6], y_cl[7], cmap='YlGn',
                title=f'CL Trajectory  (os={os_cl[0]:.2f}°)',
                force_equal=False)
    ax_tcl.set_xlabel('East  [m]', fontsize=8, color=SEC)

    return fig


# ─────────────────────────────────────────────────────────────────────────────
# 3.  STOPPING TRIAL
# ─────────────────────────────────────────────────────────────────────────────
def plot_comparison_stopping(t_ol, y_ol, n_ol, d_ol, res_ol,
                              t_cl, y_cl, n_cl, d_cl, res_cl):
    """
    Compare crash-stop: zero rudder vs PID heading-hold.
    Trajectory panel shows how well PID keeps the ship on course.
    """
    fig = plt.figure(figsize=(18, 12))
    fig.patch.set_facecolor(DARK)
    fig.suptitle('Stopping Trial — Open-Loop vs Closed-Loop PID Heading-Hold\n'
                 'Ref: Pakkan (2007) p.119 · IMO MSC.137(76) para 3.3',
                 color=PRI, fontsize=12, fontweight='bold', y=0.99)
    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.58, wspace=0.40,
                           left=0.06, right=0.96, top=0.93, bottom=0.06)

    panels = [
        (0, 0, y_ol[0],               y_cl[0],               'u  [m/s]',   'Surge Velocity'),
        (0, 1, n_ol,                   n_cl,                   'n  [RPM]',   'Shaft Speed'),
        (1, 0, d_ol,                   d_cl,                   'δ  [deg]',   'Rudder Command'),
        (1, 1, np.rad2deg(y_ol[11]),  np.rad2deg(y_cl[11]),  'ψ  [deg]',   'Heading'),
        (2, 0, y_ol[1],               y_cl[1],               'v  [m/s]',   'Sway Velocity'),
        (2, 1, y_ol[7],               y_cl[7],               'y  [m East]', 'East Drift'),
    ]

    axs = []
    for row, col, ol, cl, ylabel, title in panels:
        ax = fig.add_subplot(gs[row, col])
        ax.plot(t_ol, ol, color=C_OL, lw=1.8, label='Open-Loop')
        ax.plot(t_cl, cl, color=C_CL, lw=1.8, ls='--', label='Closed-Loop PID')
        ax.axhline(0, color=BORDER, lw=0.7)
        if res_ol['t_reversal']:
            ax.axvline(res_ol['t_reversal'], color=C_WARN, lw=0.9, ls='--', alpha=0.7)
        _sty(ax, ylabel=ylabel, title=title)
        _leg(ax)
        axs.append(ax)

    # Metric boxes
    psi_drift_ol = abs(np.rad2deg(y_ol[11, -1]))
    psi_drift_cl = abs(np.rad2deg(y_cl[11, -1]))
    _metric_box(axs[3], f'OL ψ drift: {psi_drift_ol:.3f}°\n'
                         f'CL ψ drift: {psi_drift_cl:.4f}°  (PID held course)')

    hr_ol = res_ol['head_reach']; hrL_ol = res_ol['head_reach_L']
    hr_cl = res_cl['head_reach']; hrL_cl = res_cl['head_reach_L']
    _metric_box(axs[5], f'OL reach: {hr_ol:.0f} m ({hrL_ol:.2f} L)\n'
                         f'CL reach: {hr_cl:.0f} m ({hrL_cl:.2f} L)\n'
                         f'IMO limit: 15 L')

    # NE-plane trajectories — right 2 cols, all rows
    ax_tol = fig.add_subplot(gs[0:2, 2:4])
    ax_tol.set_facecolor(PANEL)
    ax_tol.plot(y_ol[7], y_ol[6], color=C_OL, lw=2.0, label='Open-Loop path')
    ax_tol.plot(y_cl[7], y_cl[6], color=C_CL, lw=2.0, ls='--', label='Closed-Loop PID path')
    ax_tol.axvline(0, color=BORDER, lw=0.8, ls='--', alpha=0.5)
    ax_tol.tick_params(colors=SEC, labelsize=8)
    for sp in ax_tol.spines.values(): sp.set_edgecolor(BORDER)
    ax_tol.grid(True, color=GRID, lw=0.5)
    e_span = max(abs(y_ol[7]).max(), abs(y_cl[7]).max(), 15.0)
    ax_tol.set_xlim(-e_span * 1.4, e_span * 1.4)
    ax_tol.set_xlabel('East  [m]', fontsize=9, color=SEC)
    ax_tol.set_ylabel('North  [m]', fontsize=9, color=SEC)
    ax_tol.set_title('2D Trajectory — OL vs CL', color=PRI, fontsize=9,
                     pad=4, fontweight='bold')
    _leg(ax_tol)
    _metric_box(ax_tol, 'CL PID prevents\nheading veering', color=C_CL, loc='lower right')

    # East drift comparison at bottom-right
    ax_e = fig.add_subplot(gs[2, 2:4])
    ax_e.plot(t_ol, y_ol[7], color=C_OL, lw=1.8, label='OL East drift')
    ax_e.plot(t_cl, y_cl[7], color=C_CL, lw=1.8, ls='--', label='CL East drift')
    ax_e.axhline(0, color=BORDER, lw=0.7)
    _sty(ax_e, ylabel='East  [m]', title='East Position (OL vs CL)')
    _leg(ax_e)

    return fig


# ─────────────────────────────────────────────────────────────────────────────
# 4.  PULL-OUT MANEUVER
# ─────────────────────────────────────────────────────────────────────────────
def plot_comparison_pullout(t_ol, y_ol, d_ol, res_ol,
                             t_cl, y_cl, d_cl, res_cl):
    """
    Compare pull-out: zero-rudder coast vs PID course recovery.
    Trajectory panel shows the arc and coast path for each mode.
    """
    t_in  = res_ol['t_rudder_in']
    t_out = res_ol['t_rudder_out']

    fig = plt.figure(figsize=(18, 13))
    fig.patch.set_facecolor(DARK)
    fig.suptitle('Pull-Out Maneuver — Open-Loop vs Closed-Loop PID\n'
                 'Ref: Pakkan (2007) pp.120-122 · ITTC 7.5-04-02-01 §4.3',
                 color=PRI, fontsize=12, fontweight='bold', y=0.99)
    gs = gridspec.GridSpec(4, 4, figure=fig, hspace=0.58, wspace=0.40,
                           left=0.06, right=0.96, top=0.94, bottom=0.05)

    panels = [
        (0, 0, np.rad2deg(y_ol[5]),  np.rad2deg(y_cl[5]),  'r  [deg/s]', 'Yaw Rate'),
        (0, 1, d_ol,                  d_cl,                  'δ  [deg]',   'Rudder Command'),
        (1, 0, np.rad2deg(y_ol[11]), np.rad2deg(y_cl[11]), 'ψ  [deg]',   'Heading'),
        (1, 1, np.rad2deg(y_ol[9]),  np.rad2deg(y_cl[9]),  'φ  [deg]',   'Roll Angle'),
        (2, 0, y_ol[1],               y_cl[1],               'v  [m/s]',   'Sway Velocity'),
        (2, 1, y_ol[0],               y_cl[0],               'u  [m/s]',   'Surge Speed'),
    ]

    axs = []
    for row, col, ol, cl, ylabel, title in panels:
        ax = fig.add_subplot(gs[row, col])
        ax.plot(t_ol, ol, color=C_OL, lw=1.8, label='Open-Loop')
        ax.plot(t_cl, cl, color=C_CL, lw=1.8, ls='--', label='Closed-Loop PID')
        ax.axhline(0, color=BORDER, lw=0.7)
        ax.axvline(t_in,  color=C_WARN, lw=0.9, ls='--', alpha=0.7)
        ax.axvline(t_out, color='#f78166', lw=0.9, ls='--', alpha=0.7)
        _sty(ax, ylabel=ylabel, title=title)
        _leg(ax)
        axs.append(ax)

    _metric_box(axs[0], f'OL r_final: {res_ol["r_final"]:.5f} °/s\n'
                          f'CL r_final: {res_cl["r_final"]:.5f} °/s\n'
                          f'→ {res_ol["verdict"]}')

    # NE trajectories — right half, rows 0-2
    ax_tol = fig.add_subplot(gs[0:2, 2:4])
    _traj_panel(ax_tol, y_ol[6], y_ol[7], cmap='cool',
                title='OL Trajectory (straight → turn → coast)',
                force_equal=True)
    ax_tol.set_xlabel('East  [m]', fontsize=8, color=SEC)

    ax_tcl = fig.add_subplot(gs[2:4, 2:4])
    _traj_panel(ax_tcl, y_cl[6], y_cl[7], cmap='YlGn',
                title='CL Trajectory (PID course recovery in coast)',
                force_equal=True)
    ax_tcl.set_xlabel('East  [m]', fontsize=8, color=SEC)

    # Overlay on a single panel (row 3, left)
    ax_ov = fig.add_subplot(gs[3, 0:2])
    ax_ov.set_facecolor(PANEL)
    ax_ov.plot(y_ol[7], y_ol[6], color=C_OL, lw=2, label='Open-Loop')
    ax_ov.plot(y_cl[7], y_cl[6], color=C_CL, lw=2, ls='--', label='Closed-Loop PID')
    ax_ov.tick_params(colors=SEC, labelsize=8)
    for sp in ax_ov.spines.values(): sp.set_edgecolor(BORDER)
    ax_ov.grid(True, color=GRID, lw=0.5)
    ax_ov.set_aspect('equal', 'datalim')
    ax_ov.set_xlabel('East  [m]', fontsize=9, color=SEC)
    ax_ov.set_ylabel('North  [m]', fontsize=9, color=SEC)
    ax_ov.set_title('Trajectory Overlay (OL vs CL)', color=PRI, fontsize=9,
                    pad=4, fontweight='bold')
    _leg(ax_ov)

    return fig


# ─────────────────────────────────────────────────────────────────────────────
# 5.  SPIRAL MANEUVER
# ─────────────────────────────────────────────────────────────────────────────
def plot_comparison_spiral(t_ol, y_ol, d_ol, spd_ol, spr_ol, res_ol,
                            t_cl, y_cl, d_cl, spd_cl, spr_cl, res_cl):
    """
    Compare spiral: fixed rudder steps vs PID heading steps.
    Includes overlaid r(δ) curves and NE-plane trajectory panels.
    """
    fig = plt.figure(figsize=(18, 12))
    fig.patch.set_facecolor(DARK)
    fig.suptitle('Spiral Maneuver (Dieudonné) — Open-Loop vs Closed-Loop PID\n'
                 'Ref: Pakkan (2007) pp.122-126 · ITTC 7.5-04-02-01 §4.4',
                 color=PRI, fontsize=12, fontweight='bold', y=0.99)
    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.58, wspace=0.40,
                           left=0.06, right=0.96, top=0.93, bottom=0.06)

    # r(δ) overlay — top 2 cols
    ax_rd = fig.add_subplot(gs[0:2, 0:2])
    ax_rd.set_facecolor(PANEL)
    ax_rd.plot(res_ol['d_descend'], res_ol['r_descend'],
               color=C_OL, lw=2, marker='o', ms=4, label='OL ↓')
    ax_rd.plot(res_ol['d_ascend'],  res_ol['r_ascend'],
               color=C_OL, lw=2, marker='s', ms=4, ls='--', alpha=0.7, label='OL ↑')
    ax_rd.plot(res_cl['d_descend'], res_cl['r_descend'],
               color=C_CL, lw=2, marker='^', ms=4, label='CL ↓')
    ax_rd.plot(res_cl['d_ascend'],  res_cl['r_ascend'],
               color=C_CL, lw=2, marker='D', ms=4, ls='--', alpha=0.7, label='CL ↑')
    ax_rd.axhline(0, color=BORDER, lw=0.8); ax_rd.axvline(0, color=BORDER, lw=0.8)
    ax_rd.tick_params(colors=SEC, labelsize=8)
    for sp in ax_rd.spines.values(): sp.set_edgecolor(BORDER)
    ax_rd.grid(True, color=GRID, lw=0.5)
    ax_rd.set_xlabel('δ / ψ_target  [deg]', fontsize=9, color=SEC)
    ax_rd.set_ylabel('r_ss  [deg/s]', fontsize=9, color=SEC)
    ax_rd.set_title('r(δ) Curve — OL vs CL', color=PRI, fontsize=9,
                    pad=4, fontweight='bold')
    _leg(ax_rd, ncols=2)
    _metric_box(ax_rd,
                f'OL: {res_ol["max_hysteresis"]:.4f} °/s  →  {res_ol["verdict"][:6]}\n'
                f'CL: {res_cl["max_hysteresis"]:.4f} °/s  →  {res_cl["verdict"][:6]}')

    # Yaw rate time history — top right
    ax_rt = fig.add_subplot(gs[0, 2:4])
    ax_rt.plot(t_ol, np.rad2deg(y_ol[5]), color=C_OL, lw=1.5, label='OL r')
    ax_rt.plot(t_cl, np.rad2deg(y_cl[5]), color=C_CL, lw=1.5, ls='--', label='CL r')
    ax_rt.axhline(0, color=BORDER, lw=0.8)
    _sty(ax_rt, ylabel='r  [deg/s]', title='Yaw Rate Time History')
    _leg(ax_rt)

    # Rudder history — middle right
    ax_d = fig.add_subplot(gs[1, 2:4])
    ax_d.plot(t_ol, d_ol, color=C_OL, lw=1.5, label='OL δ')
    ax_d.plot(t_cl, d_cl, color=C_CL, lw=1.5, ls='--', label='CL δ')
    ax_d.axhline(0, color=BORDER, lw=0.8)
    _sty(ax_d, ylabel='δ  [deg]', title='Rudder Command')
    _leg(ax_d)

    # NE trajectories — bottom row
    ax_tol = fig.add_subplot(gs[2, 0:2])
    _traj_panel(ax_tol, y_ol[6], y_ol[7], cmap='cool',
                title='OL Trajectory (fixed rudder steps)',
                force_equal=True)
    ax_tol.set_xlabel('East  [m]', fontsize=8, color=SEC)

    ax_tcl = fig.add_subplot(gs[2, 2:4])
    _traj_panel(ax_tcl, y_cl[6], y_cl[7], cmap='YlGn',
                title='CL Trajectory (PID heading steps)',
                force_equal=True)
    ax_tcl.set_xlabel('East  [m]', fontsize=8, color=SEC)

    return fig


# ─────────────────────────────────────────────────────────────────────────────
# 6.  MASTER COMPARISON DASHBOARD
# ─────────────────────────────────────────────────────────────────────────────
def plot_comparison_dashboard(data, pid_gains):
    """
    Single-figure summary: key state + trajectory side-by-side for each maneuver.

    data keys: 'sl', 'zz10', 'zz20', 'stopping', 'pullout', 'spiral'
    """
    fig = plt.figure(figsize=(20, 24))
    fig.patch.set_facecolor(DARK)
    fig.suptitle(
        f'Open-Loop vs Closed-Loop PID — Full Maneuver Comparison\n'
        f'PID gains: Kp={pid_gains["Kp"]}, Ki={pid_gains["Ki"]}, Kd={pid_gains["Kd"]}',
        color=PRI, fontsize=13, fontweight='bold', y=0.995)

    # 7 rows × 4 cols: each row = (OL state, CL state, OL traj, CL traj)
    gs = gridspec.GridSpec(7, 4, figure=fig, hspace=0.62, wspace=0.38,
                           left=0.06, right=0.96, top=0.975, bottom=0.02)

    def _ax(row, col, ylabel, title):
        ax = fig.add_subplot(gs[row, col])
        _sty(ax, ylabel=ylabel, title=title)
        return ax

    def _mini_traj(row, col, x_n, y_e, cmap, title, eq=False):
        ax = fig.add_subplot(gs[row, col])
        _traj_panel(ax, x_n, y_e, cmap=cmap, title=title,
                    force_equal=eq, pad_frac=0.12)
        ax.set_xlabel('East  [m]', fontsize=7.5, color=SEC)
        return ax

    legend_h = [OL_LEG, CL_LEG]

    row = 0

    # ── Straight line ─────────────────────────────────────────────────────
    if 'sl' in data:
        sol_ol, (t_cl, y_cl, d_cl) = data['sl']
        psi_ol = np.rad2deg(sol_ol.y[11])
        psi_cl = np.rad2deg(y_cl[11])

        ax = _ax(row, 0, 'ψ  [deg]', 'SL — Heading Drift')
        ax.plot(sol_ol.t, psi_ol, color=C_OL, lw=1.8)
        ax.plot(t_cl,     psi_cl, color=C_CL, lw=1.8, ls='--')
        ax.axhline(0, color=BORDER, lw=0.7)
        ax.legend(handles=legend_h, fontsize=7, facecolor=DARK,
                  edgecolor=BORDER, labelcolor=PRI)

        ax2 = _ax(row, 1, 'y  [m East]', 'SL — East Drift')
        ax2.plot(sol_ol.t, sol_ol.y[7], color=C_OL, lw=1.8)
        ax2.plot(t_cl,     y_cl[7],     color=C_CL, lw=1.8, ls='--')
        ax2.axhline(0, color=BORDER, lw=0.7)

        _mini_traj(row, 2, sol_ol.y[6], sol_ol.y[7], 'cool', 'OL Trajectory', eq=False)
        _mini_traj(row, 3, y_cl[6],     y_cl[7],     'YlGn', 'CL Trajectory', eq=False)
        row += 1

    # ── ZZ 10/10 ──────────────────────────────────────────────────────────
    if 'zz10' in data:
        (t_ol,y_ol,d_ol,os_ol), (t_cl,y_cl,d_cl,os_cl,sp) = data['zz10']
        psi_ol = (np.rad2deg(y_ol[11])+180)%360-180
        psi_cl = (np.rad2deg(y_cl[11])+180)%360-180

        ax = _ax(row, 0, 'ψ  [deg]', f'ZZ 10/10 — Heading  OL={os_ol[0]:.1f}° CL={os_cl[0]:.2f}°')
        ax.axhline(10, color=C_TRIG, lw=0.8, ls='--', alpha=0.7)
        ax.axhline(-10, color=C_TRIG, lw=0.8, ls='--', alpha=0.7)
        ax.plot(t_ol, psi_ol, color=C_OL, lw=1.8)
        ax.plot(t_cl, psi_cl, color=C_CL, lw=1.8, ls='--')

        ax2 = _ax(row, 1, 'δ  [deg]', 'ZZ 10/10 — Rudder')
        ax2.step(t_ol, d_ol, color=C_OL, lw=1.5, where='post')
        ax2.plot(t_cl, d_cl, color=C_CL, lw=1.5, ls='--')
        ax2.axhline(0, color=BORDER, lw=0.7)

        _mini_traj(row, 2, y_ol[6], y_ol[7], 'cool', f'OL Traj (snake)', eq=False)
        _mini_traj(row, 3, y_cl[6], y_cl[7], 'YlGn', f'CL Traj (straighter)', eq=False)
        row += 1

    # ── ZZ 20/20 ──────────────────────────────────────────────────────────
    if 'zz20' in data:
        (t_ol,y_ol,d_ol,os_ol), (t_cl,y_cl,d_cl,os_cl,sp) = data['zz20']
        psi_ol = (np.rad2deg(y_ol[11])+180)%360-180
        psi_cl = (np.rad2deg(y_cl[11])+180)%360-180

        ax = _ax(row, 0, 'ψ  [deg]', f'ZZ 20/20 — Heading  OL={os_ol[0]:.1f}° CL={os_cl[0]:.2f}°')
        ax.axhline(20, color=C_TRIG, lw=0.8, ls='--', alpha=0.7)
        ax.axhline(-20, color=C_TRIG, lw=0.8, ls='--', alpha=0.7)
        ax.plot(t_ol, psi_ol, color=C_OL, lw=1.8)
        ax.plot(t_cl, psi_cl, color=C_CL, lw=1.8, ls='--')

        ax2 = _ax(row, 1, 'δ  [deg]', 'ZZ 20/20 — Rudder')
        ax2.step(t_ol, d_ol, color=C_OL, lw=1.5, where='post')
        ax2.plot(t_cl, d_cl, color=C_CL, lw=1.5, ls='--')
        ax2.axhline(0, color=BORDER, lw=0.7)

        _mini_traj(row, 2, y_ol[6], y_ol[7], 'cool', 'OL Traj', eq=False)
        _mini_traj(row, 3, y_cl[6], y_cl[7], 'YlGn', 'CL Traj', eq=False)
        row += 1

    # ── Stopping ──────────────────────────────────────────────────────────
    if 'stopping' in data:
        (t_ol,y_ol,n_ol,d_ol,r_ol), (t_cl,y_cl,n_cl,d_cl,r_cl) = data['stopping']

        ax = _ax(row, 0, 'u  [m/s]', 'Stopping — Surge Velocity')
        ax.plot(t_ol, y_ol[0], color=C_OL, lw=1.8)
        ax.plot(t_cl, y_cl[0], color=C_CL, lw=1.8, ls='--')
        ax.axhline(0, color=BORDER, lw=0.7)
        if r_ol['t_reversal']:
            ax.axvline(r_ol['t_reversal'], color=C_WARN, lw=0.9, ls='--', alpha=0.7)

        ax2 = _ax(row, 1, 'ψ  [deg]', 'Stopping — Heading (PID holds course)')
        ax2.plot(t_ol, np.rad2deg(y_ol[11]), color=C_OL, lw=1.8, label='Open-Loop')
        ax2.plot(t_cl, np.rad2deg(y_cl[11]), color=C_CL, lw=1.8, ls='--', label='Closed-Loop PID')
        ax2.axhline(0, color=BORDER, lw=0.7)
        ax2.legend(handles=legend_h, fontsize=7, facecolor=DARK,
                   edgecolor=BORDER, labelcolor=PRI)

        _mini_traj(row, 2, y_ol[6], y_ol[7], 'inferno', 'OL Stop Traj', eq=False)
        _mini_traj(row, 3, y_cl[6], y_cl[7], 'YlGn',    'CL Stop Traj', eq=False)
        row += 1

    # ── Pull-out ──────────────────────────────────────────────────────────
    if 'pullout' in data:
        (t_ol,y_ol,d_ol,r_ol), (t_cl,y_cl,d_cl,r_cl) = data['pullout']
        t_out = r_ol['t_rudder_out']

        ax = _ax(row, 0, 'r  [deg/s]', 'Pull-Out — Yaw Rate')
        ax.plot(t_ol, np.rad2deg(y_ol[5]), color=C_OL, lw=1.8)
        ax.plot(t_cl, np.rad2deg(y_cl[5]), color=C_CL, lw=1.8, ls='--')
        ax.axhline(0, color=BORDER, lw=0.7)
        ax.axvline(t_out, color=C_WARN, lw=0.9, ls='--', alpha=0.7)
        _metric_box(ax, f'OL r_final: {r_ol["r_final"]:.5f}°/s\n'
                         f'CL r_final: {r_cl["r_final"]:.5f}°/s')

        ax2 = _ax(row, 1, 'ψ  [deg]', 'Pull-Out — Heading')
        ax2.plot(t_ol, np.rad2deg(y_ol[11]), color=C_OL, lw=1.8)
        ax2.plot(t_cl, np.rad2deg(y_cl[11]), color=C_CL, lw=1.8, ls='--')
        ax2.axhline(0, color=BORDER, lw=0.7)
        ax2.axvline(t_out, color=C_WARN, lw=0.9, ls='--', alpha=0.7)

        _mini_traj(row, 2, y_ol[6], y_ol[7], 'cool', 'OL Pullout Traj', eq=True)
        _mini_traj(row, 3, y_cl[6], y_cl[7], 'YlGn', 'CL Pullout Traj', eq=True)
        row += 1

    # ── Spiral ────────────────────────────────────────────────────────────
    if 'spiral' in data:
        (t_ol,y_ol,d_ol,_,_,res_ol_sp), (t_cl,y_cl,d_cl,_,_,res_cl_sp) = data['spiral']

        ax = _ax(row, 0, 'r_ss  [deg/s]', 'Spiral — r(δ) Curve')
        ax.plot(res_ol_sp['d_descend'], res_ol_sp['r_descend'],
                color=C_OL, lw=2, marker='o', ms=3, label='OL ↓')
        ax.plot(res_ol_sp['d_ascend'],  res_ol_sp['r_ascend'],
                color=C_OL, lw=2, marker='s', ms=3, ls='--', alpha=0.7, label='OL ↑')
        ax.plot(res_cl_sp['d_descend'], res_cl_sp['r_descend'],
                color=C_CL, lw=2, marker='^', ms=3, label='CL ↓')
        ax.plot(res_cl_sp['d_ascend'],  res_cl_sp['r_ascend'],
                color=C_CL, lw=2, marker='D', ms=3, ls='--', alpha=0.7, label='CL ↑')
        ax.axhline(0, color=BORDER, lw=0.8); ax.axvline(0, color=BORDER, lw=0.8)
        ax.set_xlabel('δ  [deg]', fontsize=7.5, color=SEC)
        _leg(ax, ncols=2)
        _metric_box(ax,
                    f'OL: {res_ol_sp["max_hysteresis"]:.4f}°/s → {res_ol_sp["verdict"][:6]}\n'
                    f'CL: {res_cl_sp["max_hysteresis"]:.4f}°/s → {res_cl_sp["verdict"][:6]}')

        ax2 = _ax(row, 1, 'r  [deg/s]', 'Spiral — Yaw Rate')
        ax2.plot(t_ol, np.rad2deg(y_ol[5]), color=C_OL, lw=1.5, label='OL r')
        ax2.plot(t_cl, np.rad2deg(y_cl[5]), color=C_CL, lw=1.5, ls='--', label='CL r')
        ax2.axhline(0, color=BORDER, lw=0.8)
        _leg(ax2)

        _mini_traj(row, 2, y_ol[6], y_ol[7], 'viridis', 'OL Spiral Traj', eq=True)
        _mini_traj(row, 3, y_cl[6], y_cl[7], 'YlGn',    'CL Spiral Traj', eq=True)

    return fig