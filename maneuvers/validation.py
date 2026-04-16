# maneuvers/validation.py
"""
IMO / ITTC Maneuvering Performance Validation.

Runs all standard maneuvering trials in both open-loop and (optionally)
closed-loop PID modes, compares against IMO MSC.137(76) limits, and
returns a structured scorecard.

Standards referenced
--------------------
[1] IMO Resolution MSC.137(76) (4 December 2002).
    "Standards for Ship Maneuverability."
    — Turning ability:  tactical diameter ≤ 5.0 L          (para 3.1)
    — Yaw-checking:     10/10 ZZ 1st overshoot ≤ 10° L<200m (para 3.2a)
                        20/20 ZZ 1st overshoot ≤ 25° L<200m (para 3.2b)
    — Stopping ability: track reach ≤ 15 L from full ahead  (para 3.3)

[2] ITTC Recommended Procedures and Guidelines (2002).
    "Maneuvering Trial Code." Procedure 7.5-04-02-01.
    23rd ITTC, Venice.
    — Defines protocols for turning circle, zigzag, pull-out, spiral,
      and stopping trial.

[3] IMO MSC/Circ.1053 (16 December 2002).
    "Explanatory Notes to the Standards for Ship Maneuverability."

[4] Pakkan, S. (2007). "Modeling and Simulation of a Maneuvering Ship."
    M.Sc. Thesis, Middle East Technical University, Ankara.
    — Section 5.3.2, pp.115-127: test protocols and results.
"""

import numpy as np
from maneuvers.turning_circle    import run_turning_circle
from maneuvers.zigzag            import run_zigzag_openloop, compute_overshoot
from maneuvers.zigzag_closedloop import run_zigzag_closedloop, compute_overshoot_cl
from maneuvers.stopping_trial    import run_stopping_trial
from maneuvers.pullout           import run_pullout
from maneuvers.spiral            import run_spiral


def run_validation(ship, init_state, n_cmd=95.0, use_pid=False,
                   Kp=20.0, Ki=0.3, Kd=3.0, delta_max=35.0,
                   verbose=True):
    """
    Run the full IMO/ITTC maneuvering validation suite.

    Args:
        ship         ShipModel instance
        init_state   14-element initial state
        n_cmd        [RPM]  Operating propeller speed
        use_pid      bool   Run each maneuver in closed-loop PID mode where applicable
        Kp, Ki, Kd   PID gains (used when use_pid=True)
        delta_max    [deg]  Rudder saturation limit for PID
        verbose      bool   Print results to console

    Returns:
        scorecard    list of dicts ('test', 'value', 'limit', 'units', 'pass',
                                    'reference', 'note', 'mode')
        raw          dict of raw simulation outputs keyed by test name
    """
    L     = ship.m.L
    init  = list(init_state)
    pid_kw = dict(use_pid=use_pid, Kp=Kp, Ki=Ki, Kd=Kd, delta_max=delta_max)
    mode_label = 'Closed-Loop PID' if use_pid else 'Open-Loop'

    scorecard = []
    raw       = {}

    def _check(test, value, limit, units, passed, reference, note='', mode=''):
        scorecard.append({
            'test'      : test,
            'value'     : value,
            'limit'     : limit,
            'units'     : units,
            'pass'      : passed,
            'reference' : reference,
            'note'      : note,
            'mode'      : mode or mode_label,
        })
        if verbose:
            col  = '\033[92m✓ PASS\033[0m' if passed else '\033[91m✗ FAIL\033[0m'
            print(f'  {col}  [{mode or mode_label}]  {test}')
            print(f'         Value:  {value:.3f} {units}')
            print(f'         Limit:  {limit}  ({reference})')
            if note: print(f'         Note:   {note}')

    # ── 1. TURNING CIRCLE — 35° (IMO) ─────────────────────────────────────
    # Turning circle is always open-loop (rudder IS the test input)
    if verbose: print('\n[1] Turning Circle — 35° rudder (IMO, always open-loop)')
    tc35 = run_turning_circle(ship, init, n_cmd=n_cmd, rudder_deg=35.0)
    tdia35 = 2.0 * tc35['radius']
    _check('Tactical diameter (35° rudder)', tdia35/L, '≤ 5.0 L', 'L',
           tdia35/L <= 5.0, 'IMO MSC.137(76) para 3.1',
           note=f'{tdia35:.0f} m = {tdia35/L:.2f} L', mode='Open-Loop')
    raw['tc35'] = tc35

    # ── 2. TURNING CIRCLE — 18° (thesis calibration) ──────────────────────
    if verbose: print('\n[2] Turning Circle — 18° rudder (thesis calibration, open-loop)')
    tc18 = run_turning_circle(ship, init, n_cmd=n_cmd, rudder_deg=18.0)
    tdia18  = 2.0 * tc18['radius']
    phi_ss18 = np.rad2deg(tc18['y'][9, -1])
    _check('Tactical diameter (18° rudder)', tdia18/L, '≤ 7.0 L', 'L',
           tdia18/L <= 7.0, 'Pakkan (2007) p.116-117',
           note=f'{tdia18:.0f} m, phi_ss={phi_ss18:.1f}°', mode='Open-Loop')
    raw['tc18'] = tc18

    # ── 3. ZIGZAG 10/10 ───────────────────────────────────────────────────
    if verbose: print(f'\n[3] 10/10 ZigZag ({mode_label})')
    if use_pid:
        t10,y10,_,_,_,_,d10,sp10 = run_zigzag_closedloop(
            ship, init[0], delta_zz_deg=10., t_end=300, n_rpm=n_cmd,
            Kp=Kp, Ki=Ki, Kd=Kd, delta_max=delta_max)
        os_s10, os_p10 = compute_overshoot_cl(y10, 10.)
        raw['zz10'] = (t10, y10, d10, os_s10, os_p10, sp10)
    else:
        t10,y10,_,_,_,_,d10 = run_zigzag_openloop(
            ship, init[0], delta_zz_deg=10., t_end=300, n_rpm=n_cmd)
        os_s10, os_p10 = compute_overshoot(y10, 10.)
        raw['zz10'] = (t10, y10, d10, os_s10, os_p10, None)

    _check('10/10 ZigZag — stbd overshoot', os_s10, '≤ 10° (L<200m)', 'deg',
           os_s10 <= 10., 'IMO MSC.137(76) para 3.2a')
    _check('10/10 ZigZag — port overshoot', os_p10, '≤ 10° (L<200m)', 'deg',
           os_p10 <= 10., 'IMO MSC.137(76) para 3.2a')

    # ── 4. ZIGZAG 20/20 ───────────────────────────────────────────────────
    if verbose: print(f'\n[4] 20/20 ZigZag ({mode_label})')
    if use_pid:
        t20,y20,_,_,_,_,d20,sp20 = run_zigzag_closedloop(
            ship, init[0], delta_zz_deg=20., t_end=300, n_rpm=n_cmd,
            Kp=Kp, Ki=Ki, Kd=Kd, delta_max=delta_max)
        os_s20, os_p20 = compute_overshoot_cl(y20, 20.)
        raw['zz20'] = (t20, y20, d20, os_s20, os_p20, sp20)
    else:
        t20,y20,_,_,_,_,d20 = run_zigzag_openloop(
            ship, init[0], delta_zz_deg=20., t_end=300, n_rpm=n_cmd)
        os_s20, os_p20 = compute_overshoot(y20, 20.)
        raw['zz20'] = (t20, y20, d20, os_s20, os_p20, None)

    _check('20/20 ZigZag — stbd overshoot', os_s20, '≤ 25° (L<200m)', 'deg',
           os_s20 <= 25., 'IMO MSC.137(76) para 3.2b')
    _check('20/20 ZigZag — port overshoot', os_p20, '≤ 25° (L<200m)', 'deg',
           os_p20 <= 25., 'IMO MSC.137(76) para 3.2b')

    # ── 5. STOPPING TRIAL ─────────────────────────────────────────────────
    if verbose: print(f'\n[5] Stopping Trial ({mode_label})')
    t_st,y_st,n_st,d_st,res_st = run_stopping_trial(
        ship, init, n_full=ship.m.nmax, t_buildup=100., t_end=300., **pid_kw)
    if res_st['head_reach'] is not None:
        _check('Stopping — head reach', res_st['head_reach_L'], '≤ 15 L', 'L',
               res_st['imo_pass'], 'IMO MSC.137(76) para 3.3',
               note=f'{res_st["head_reach"]:.0f} m, stopped in {res_st["time_to_stop"]:.0f} s')
    raw['stopping'] = (t_st, y_st, n_st, d_st, res_st)

    # ── 6. PULL-OUT ───────────────────────────────────────────────────────
    if verbose: print(f'\n[6] Pull-Out Maneuver ({mode_label})')
    t_po,y_po,d_po,res_po = run_pullout(
        ship, init, rudder_deg=20., t_straight=70., t_turn=200., t_coast=150.,
        n_cmd=n_cmd, **pid_kw)
    _check('Pull-out — residual yaw rate', abs(res_po['r_final']), '≈ 0 (stable)', 'deg/s',
           res_po['stable'], 'ITTC 7.5-04-02-01 §4.3', note=res_po['verdict'])
    raw['pullout'] = (t_po, y_po, d_po, res_po)

    # ── 7. SPIRAL ─────────────────────────────────────────────────────────
    if verbose: print(f'\n[7] Spiral Maneuver ({mode_label})')
    t_sp,y_sp,d_sp,sp_d,sp_r,res_sp = run_spiral(
        ship, init, n_cmd=n_cmd, delta_step=5., t_settle=150., **pid_kw)
    _check('Spiral — hysteresis', res_sp['max_hysteresis'], '< 0.05 deg/s (stable)',
           'deg/s', res_sp['stable'], 'ITTC 7.5-04-02-01 §4.4',
           note=res_sp['verdict'])
    raw['spiral'] = (t_sp, y_sp, d_sp, sp_d, sp_r, res_sp)

    # ── Summary ───────────────────────────────────────────────────────────
    n_pass = sum(c['pass'] for c in scorecard)
    n_total = len(scorecard)
    if verbose:
        print(f'\n{"="*56}')
        print(f'  {mode_label} VALIDATION SUMMARY: {n_pass}/{n_total} passed')
        print(f'{"="*56}')

    return scorecard, raw