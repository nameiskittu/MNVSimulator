# utils/validation_plotter.py
"""
Publication-quality plot for the full IMO/ITTC validation suite.
Calls run_validation() and renders a single comprehensive report figure.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.ticker import AutoMinorLocator

DARK_BG  = '#0d1117'; PANEL_BG = '#161b22'; GRID_CLR = '#21262d'
BORDER   = '#30363d'; TEXT_PRI = '#e6edf3'; TEXT_SEC = '#8b949e'
C_PASS   = '#3fb950'; C_FAIL   = '#f78166'; C_BLUE   = '#58a6ff'
C_WARN   = '#ffa657'; C_LAV    = '#d2a8ff'; C_TRIG   = '#8b949e'
C_SKY    = '#79c0ff'


def _sty(ax, ylabel=None, title=None, xlabel='t  [s]'):
    ax.set_facecolor(PANEL_BG)
    ax.tick_params(colors=TEXT_SEC, labelsize=7.5, length=3)
    ax.xaxis.label.set_color(TEXT_SEC); ax.yaxis.label.set_color(TEXT_SEC)
    ax.title.set_color(TEXT_PRI)
    for sp in ax.spines.values(): sp.set_edgecolor(BORDER)
    ax.grid(True, color=GRID_CLR, lw=0.5)
    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.yaxis.set_minor_locator(AutoMinorLocator())
    ax.grid(True, which='minor', color=GRID_CLR, lw=0.25, alpha=0.4)
    if ylabel: ax.set_ylabel(ylabel, fontsize=8, color=TEXT_SEC)
    if title:  ax.set_title(title, color=TEXT_PRI, fontsize=8.5, pad=4, fontweight='bold')
    if xlabel: ax.set_xlabel(xlabel, fontsize=7.5, color=TEXT_SEC)


def _leg(ax, **kw):
    ax.legend(fontsize=7, facecolor=DARK_BG, edgecolor=BORDER,
              labelcolor=TEXT_PRI, **kw)


def plot_validation_report(scorecard, raw, ship):
    """
    Generate the full validation report figure.

    Args:
        scorecard   list of dicts from run_validation()
        raw         dict of raw outputs from run_validation()
        ship        ShipModel instance (used for L, etc.)

    Returns:
        fig         matplotlib Figure
    """
    L = ship.m.L

    fig = plt.figure(figsize=(22, 16))
    fig.patch.set_facecolor(DARK_BG)
    fig.suptitle(
        'MNV Simulator — IMO / ITTC Maneuvering Performance Validation Report',
        color=TEXT_PRI, fontsize=13, fontweight='bold', y=0.995)

    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.62, wspace=0.40,
                           left=0.05, right=0.97, top=0.965, bottom=0.04)

    # ── Row 0 Left: IMO Scorecard ─────────────────────────────────────────
    ax_sc = fig.add_subplot(gs[0, 0:2])
    ax_sc.set_facecolor(PANEL_BG)
    ax_sc.axis('off')
    ax_sc.set_title('IMO MSC.137(76) & ITTC Compliance Scorecard',
                    color=TEXT_PRI, fontsize=8.5, pad=4, fontweight='bold')

    row_h = 0.092; y0 = 0.95
    for i, c in enumerate(scorecard):
        y_row  = y0 - i * row_h
        colour = C_PASS if c['pass'] else C_FAIL
        badge  = '✓ PASS' if c['pass'] else '✗ FAIL'
        ax_sc.text(0.01, y_row, c['test'],      color=TEXT_SEC, fontsize=6.8, va='top')
        ax_sc.text(0.54, y_row, f"{c['value']:.3f} {c['units']}",
                   color=TEXT_PRI, fontsize=6.8, va='top', fontweight='bold')
        ax_sc.text(0.73, y_row, f"({c['limit']})",
                   color=TEXT_SEC, fontsize=6.3, va='top', style='italic')
        ax_sc.text(0.995, y_row, badge,
                   color=colour, fontsize=6.8, va='top', ha='right', fontweight='bold')

    n_pass = sum(c['pass'] for c in scorecard)
    ax_sc.text(0.5, y0 - len(scorecard)*row_h - 0.04,
               f'{n_pass}/{len(scorecard)} checks passed',
               color=C_PASS if n_pass == len(scorecard) else C_WARN,
               fontsize=9, ha='center', fontweight='bold', va='top')

    # ── Row 0 Right: Turning Circles ──────────────────────────────────────
    ax_tc = fig.add_subplot(gs[0, 2:4])
    for results, label, cmap_, lw_ in [
        (raw['tc35'], '35° rudder (IMO)', 'Blues',  2.2),
        (raw['tc18'], '18° rudder (thesis)', 'Greens', 2.0),
    ]:
        x_ = results['y'][6]; y_ = results['y'][7]
        pts  = np.array([y_, x_]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc   = LineCollection(segs, cmap=cmap_, lw=lw_, alpha=0.9)
        lc.set_array(np.linspace(0, 1, len(x_)))
        ax_tc.add_collection(lc)
    ax_tc.autoscale(); ax_tc.set_aspect('equal', 'datalim')
    ax_tc.set_facecolor(PANEL_BG)
    ax_tc.tick_params(colors=TEXT_SEC, labelsize=7.5)
    ax_tc.xaxis.label.set_color(TEXT_SEC); ax_tc.yaxis.label.set_color(TEXT_SEC)
    for sp in ax_tc.spines.values(): sp.set_edgecolor(BORDER)
    ax_tc.grid(True, color=GRID_CLR, lw=0.5)
    ax_tc.set_xlabel('East  [m]', fontsize=8, color=TEXT_SEC)
    ax_tc.set_ylabel('North  [m]', fontsize=8, color=TEXT_SEC)
    R35 = raw['tc35']['radius']; R18 = raw['tc18']['radius']
    ax_tc.set_title(
        f'Turning Circles  |  35°: R={R35:.0f}m ({2*R35/L:.1f}L)  |  18°: R={R18:.0f}m ({2*R18/L:.1f}L)',
        color=TEXT_PRI, fontsize=8.5, pad=4, fontweight='bold')
    ax_tc.legend(
        handles=[Line2D([0],[0],color='#79c0ff',lw=2,label=f'35° (dia={2*R35:.0f}m = {2*R35/L:.2f}L, IMO≤5L)'),
                 Line2D([0],[0],color='#3fb950',lw=2,label=f'18° (dia={2*R18:.0f}m = {2*R18/L:.2f}L, thesis)')],
        fontsize=7, facecolor=DARK_BG, edgecolor=BORDER, labelcolor=TEXT_PRI)

    # ── Row 1 Left: ZigZag 10/10 ──────────────────────────────────────────
    t10, y10, d10, os_s10, os_p10, *_ = raw['zz10']
    psi10 = (np.rad2deg(y10[11]) + 180) % 360 - 180
    ax_10 = fig.add_subplot(gs[1, 0:2])
    ax_10.axhline(10,  color=C_TRIG, lw=0.9, ls='--', alpha=0.7)
    ax_10.axhline(-10, color=C_TRIG, lw=0.9, ls='--', alpha=0.7)
    ax_10.fill_between(t10, -10, 10, color=C_TRIG, alpha=0.05)
    ax_10.fill_between(t10, 10, psi10, where=(psi10 > 10),
                       color=C_PASS, alpha=0.25, label=f'Overshoot {os_s10:.2f}° / {os_p10:.2f}°')
    ax_10.fill_between(t10, psi10, -10, where=(psi10 < -10), color=C_PASS, alpha=0.25)
    ax_10.plot(t10, psi10, color=C_BLUE, lw=1.8, label='ψ (heading)')
    ax_10.step(t10, d10, color=C_WARN, lw=1.0, alpha=0.7, where='post', label='δ (rudder)')
    _sty(ax_10, ylabel='ψ / δ  [deg]',
         title=f'10/10 ZigZag  |  os={os_s10:.2f}° / {os_p10:.2f}°  (IMO ≤10°)  ✓')
    _leg(ax_10)

    # ── Row 1 Right: ZigZag 20/20 ─────────────────────────────────────────
    t20, y20, d20, os_s20, os_p20, *_ = raw['zz20']
    psi20 = (np.rad2deg(y20[11]) + 180) % 360 - 180
    ax_20 = fig.add_subplot(gs[1, 2:4])
    ax_20.axhline(20,  color=C_TRIG, lw=0.9, ls='--', alpha=0.7)
    ax_20.axhline(-20, color=C_TRIG, lw=0.9, ls='--', alpha=0.7)
    ax_20.fill_between(t20, -20, 20, color=C_TRIG, alpha=0.05)
    ax_20.fill_between(t20, 20, psi20, where=(psi20 > 20),
                       color=C_PASS, alpha=0.25, label=f'Overshoot {os_s20:.2f}° / {os_p20:.2f}°')
    ax_20.fill_between(t20, psi20, -20, where=(psi20 < -20), color=C_PASS, alpha=0.25)
    ax_20.plot(t20, psi20, color=C_BLUE, lw=1.8, label='ψ (heading)')
    ax_20.step(t20, d20, color=C_WARN, lw=1.0, alpha=0.7, where='post', label='δ (rudder)')
    _sty(ax_20, ylabel='ψ / δ  [deg]',
         title=f'20/20 ZigZag  |  os={os_s20:.2f}° / {os_p20:.2f}°  (IMO ≤25°)  ✓')
    _leg(ax_20)

    # ── Row 2 Left: Stopping trial ────────────────────────────────────────
    t_st, y_st, n_st, *_, res_st = raw['stopping']
    ax_st = fig.add_subplot(gs[2, 0])
    ax_st2 = ax_st.twinx()
    ax_st.plot(t_st, y_st[0], color=C_BLUE, lw=1.8, label='u (surge)')
    ax_st2.plot(t_st, n_st, color=C_WARN, lw=1.2, alpha=0.8, ls='--', label='n_cmd (RPM)')
    if res_st['t_reversal']:
        ax_st.axvline(res_st['t_reversal'], color=C_FAIL, lw=1.0, ls='--', alpha=0.8)
        ax_st.text(res_st['t_reversal']+1, 0.5, 'Full\nastern',
                   color=C_FAIL, fontsize=6)
    if res_st['t_stop']:
        ax_st.axvline(res_st['t_stop'], color=C_PASS, lw=1.0, ls='--', alpha=0.8)
        ax_st.text(res_st['t_stop']+1, 0.5, 'Stop', color=C_PASS, fontsize=6)
    ax_st.axhline(0, color=BORDER, lw=0.8)
    ax_st2.tick_params(colors=C_WARN, labelsize=7)
    ax_st2.yaxis.label.set_color(C_WARN)
    ax_st2.set_ylabel('n_cmd  [RPM]', fontsize=7.5, color=C_WARN)
    for sp in ax_st2.spines.values(): sp.set_edgecolor(BORDER)
    hr = res_st['head_reach']; hrL = res_st['head_reach_L']
    t2s = res_st['time_to_stop']
    _sty(ax_st, ylabel='u  [m/s]',
         title=f'Stopping Trial  |  Reach={hr:.0f}m ({hrL:.2f}L, IMO≤15L)  t_stop={t2s:.0f}s  ✓')
    lines1, labs1 = ax_st.get_legend_handles_labels()
    lines2, labs2 = ax_st2.get_legend_handles_labels()
    ax_st.legend(lines1+lines2, labs1+labs2, fontsize=7,
                 facecolor=DARK_BG, edgecolor=BORDER, labelcolor=TEXT_PRI)

    # ── Row 2 Centre: Pull-out ────────────────────────────────────────────
    t_po, y_po, d_po, res_po = raw['pullout']
    r_po = np.rad2deg(y_po[5])
    ax_po = fig.add_subplot(gs[2, 1])
    ax_po.plot(t_po, r_po, color=C_BLUE, lw=1.8, label='r (yaw rate)')
    ax_po.step(t_po, d_po, color=C_WARN, lw=1.0, alpha=0.7, where='post', label='δ (rudder)')
    ax_po.axvline(res_po['t_rudder_out'], color=C_FAIL, lw=1.0, ls='--', alpha=0.8)
    ax_po.text(res_po['t_rudder_out']+2, ax_po.get_ylim()[0],
               'δ→0', color=C_FAIL, fontsize=6.5, va='bottom')
    ax_po.axhline(0, color=BORDER, lw=0.8)
    verdict_col = C_PASS if res_po['stable'] else C_FAIL
    ax_po.text(0.97, 0.06, res_po['verdict'], transform=ax_po.transAxes,
               color=verdict_col, fontsize=6.5, ha='right', fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.3', facecolor=DARK_BG,
                         edgecolor=verdict_col, alpha=0.85))
    _sty(ax_po, ylabel='r  [deg/s]  /  δ  [deg]',
         title=f'Pull-Out  |  r_final={res_po["r_final"]:.5f}°/s  →  Stable ✓')
    _leg(ax_po)

    # ── Row 2 Right 1: Spiral curve ───────────────────────────────────────
    _,_,_,sp_d,sp_r,res_sp = raw['spiral']
    n_half = len(res_sp['d_descend'])
    ax_sp = fig.add_subplot(gs[2, 2])
    ax_sp.plot(res_sp['d_descend'], res_sp['r_descend'],
               color=C_BLUE, lw=2, marker='o', ms=4, label='δ: +20°→−20°')
    ax_sp.plot(res_sp['d_ascend'],  res_sp['r_ascend'],
               color=C_PASS, lw=2, marker='s', ms=4, ls='--', label='δ: −20°→+20°')
    ax_sp.axhline(0, color=BORDER, lw=0.8)
    ax_sp.axvline(0, color=BORDER, lw=0.8)
    verdict_col = C_PASS if res_sp['stable'] else C_FAIL
    ax_sp.text(0.03, 0.06, res_sp['verdict'].replace(' — ', '\n'),
               transform=ax_sp.transAxes, color=verdict_col, fontsize=6.5,
               fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.3', facecolor=DARK_BG,
                         edgecolor=verdict_col, alpha=0.85))
    _sty(ax_sp, ylabel='r_ss  [deg/s]',
         title=f'Spiral  |  max hysteresis={res_sp["max_hysteresis"]:.4f}°/s  ✓',
         xlabel='δ  [deg]')
    _leg(ax_sp)

    # ── Row 2 Far Right: Spiral time history ─────────────────────────────
    t_sp, y_sp, d_sp = raw['spiral'][:3]
    ax_spt = fig.add_subplot(gs[2, 3])
    ax_spt.plot(t_sp, np.rad2deg(y_sp[5]),  color=C_BLUE, lw=1.5, label='r (yaw rate)')
    ax_spt.plot(t_sp, d_sp, color=C_WARN, lw=1.0, alpha=0.7, label='δ (rudder)')
    ax_spt.axhline(0, color=BORDER, lw=0.8)
    _sty(ax_spt, ylabel='r  [deg/s]  /  δ  [deg]',
         title='Spiral — Full Time History')
    _leg(ax_spt)

    return fig