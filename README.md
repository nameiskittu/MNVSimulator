```python?code_reference&code_event_index=2
readme_content = """# MNVSimulator

A 6-DOF maneuvering simulator for a Multipurpose Naval Vessel (MNV), implemented in Python. The simulator reproduces the mathematical ship model from Sinan Pakkan's 2007 M.Sc. thesis (*Modeling and Simulation of a Maneuvering Ship*, METU) and validates outputs against IMO MSC.137(76) and ITTC 7.5-04-02-01 standards.

---

## Table of Contents

1. [Overview](#overview)
2. [Vessel Specifications](#vessel-specifications)
3. [Mathematical Model](#mathematical-model)
4. [Project Structure](#project-structure)
5. [Installation](#installation)
6. [Quick Start](#quick-start)
7. [Main Controls Reference](#main-controls-reference)
8. [Module Reference](#module-reference)
9. [Maneuvering Trials](#maneuvering-trials)
10. [Simulation Results Summary](#simulation-results-summary)
11. [PID Closed-Loop Control](#pid-closed-loop-control)
12. [Open-Loop vs Closed-Loop Comparison](#open-loop-vs-closed-loop-comparison)
13. [2D Trajectory Plots](#2d-trajectory-plots)
14. [Spiral Maneuver — Trajectory Notes](#spiral-maneuver-trajectory-notes)
15. [Known Limitations and Calibration Notes](#known-limitations-and-calibration-notes)
16. [Bugs Fixed](#bugs-fixed)
17. [References and Standards](#references-and-standards)

---

## Overview

MNVSimulator integrates the nonlinear 6-DOF equations of motion for a surface ship in the body-fixed frame, driven by hydrodynamic hull forces, a quadratic propeller thrust model, a first-order rudder actuator, and optional environmental disturbances. Seven standard maneuvering tests are implemented, each available in open-loop and closed-loop PID modes, with side-by-side comparison plotting.

**Integrator:** SciPy RK45 adaptive step-size solver via `solve_ivp`.

**Tests implemented:**
- Straight-line acceleration
- Turning circle (18 deg thesis calibration + 35 deg IMO standard)
- ZigZag 10/10 and 20/20
- Stopping trial
- Pull-out maneuver
- Spiral maneuver (Dieudonne)

---

## Vessel Specifications

| Parameter | Value | Unit |
|---|---|---|
| Total mass | 360,000 | kg |
| Length (LBP) | 48.0 | m |
| Beam | 8.6 | m |
| Draft | 2.2 | m |
| Waterplane area | 105.6 | m2 |
| Roll inertia Ixx | 3.4e6 | kg m2 |
| Pitch inertia Iyy | 60.0e6 | kg m2 |
| Yaw inertia Izz | 60.0e6 | kg m2 |
| CoG (xG, yG, zG) | -3.38, 0.0, -1.75 | m |
| GM_T (transverse) | 0.776 | m |
| GM_L (longitudinal) | 7.76 | m |
| Max propeller speed | 160 | RPM |
| Max rudder angle | 45 | deg |
| Max rudder rate | 20 | deg/s |
| Max thrust (quadratic) | 635,000 | N |

---

## Mathematical Model

### Equations of Motion

```

M \* v\_dot  =  tau\_hull + tau\_thrust + tau\_env - g(eta)
eta\_dot    =  J(eta) \* v

```

where M = M_RB + M_A (rigid body + frequency-independent added mass).

### State Vector (14 elements)

| Index | Symbol | Unit | Description |
|---|---|---|---|
| 0 | u | m/s | Surge velocity |
| 1 | v | m/s | Sway velocity |
| 2 | w | m/s | Heave velocity |
| 3 | p | rad/s | Roll rate |
| 4 | q | rad/s | Pitch rate |
| 5 | r | rad/s | Yaw rate |
| 6 | x | m | North position (NED) |
| 7 | y | m | East position (NED) |
| 8 | z | m | Depth |
| 9 | phi | rad | Roll angle |
| 10 | theta | rad | Pitch angle |
| 11 | psi | rad | Heading |
| 12 | n_act | RPM | Actual propeller speed |
| 13 | delta_act | rad | Actual rudder angle |

### Propeller Thrust

```

T = TA\_max \* (|n\_act| / n\_max)^2 \* sign(n\_act)

```

### Actuator Dynamics

Propeller shaft (variable time constant):
```

Tm = 18.83 s           when |n\_act| \< 20 RPM
Tm = 5.65 / |n\_act|    otherwise
n\_dot = (n\_cmd - n\_act) / Tm

```

Rudder (first-order lag with rate limiting):
```

Tc = 1.0 s
delta\_dot = clip((delta\_cmd - delta\_act) / Tc,  +/- 20 deg/s)

```

### Sign Convention (important)

Positive rudder (delta > 0) turns the ship to PORT (psi decreases). This affects PID sign, zigzag trigger logic, and interpretation of all rudder plots.

---

## Project Structure

```

MNVSimulator/
|
\+-- main.py                    Entry point — all toggles here
|
\+-- vessel/
|   +-- params.py                  VesselParams: all constants and derivatives
|   +-- model.py                   ShipModel: 6-DOF dynamics, forces
|
\+-- maneuvers/
|   +-- straight\_line.py           run\_straight\_line()
|   +-- turning\_circle.py          run\_turning\_circle()
|   +-- zigzag.py                  run\_zigzag\_openloop(), compute\_overshoot()
|   +-- zigzag\_closedloop.py       run\_zigzag\_closedloop(), compute\_overshoot\_cl()
|   +-- stopping\_trial.py          run\_stopping\_trial()
|   +-- pullout.py                 run\_pullout()
|   +-- spiral.py                  run\_spiral()
|   +-- validation.py              run\_validation() -- full IMO/ITTC suite
|
\+-- control/
|   +-- pid.py                     PIDController -- heading autopilot
|
\+-- utils/
|   +-- plotters.py                All individual maneuver plot functions
|   +-- comparison\_plotter.py      OL vs CL side-by-side comparison plots
|   +-- validation\_plotter.py      IMO compliance report figure
|
\+-- requirements.txt
\+-- README.md

````

---

## Installation

**Requirements:** Python 3.10+, numpy, scipy, matplotlib.

```bash
pip install numpy scipy matplotlib
````

Or pin exact versions:

```bash
pip install -r requirements.txt
```

-----

## Quick Start

```bash
python main.py
```

By default runs the full IMO/ITTC validation suite. Change the toggles at the top of `main.py` to enable individual maneuvers.

-----

## Main Controls Reference

All simulation options are set at the top of `main.py`.

### Maneuver toggles

```python
RUN_STRAIGHT_LINE   = False
RUN_TURNING_CIRCLE  = False
RUN_ZIGZAG          = False
RUN_STOPPING        = False
RUN_PULLOUT         = False
RUN_SPIRAL          = False
RUN_VALIDATION      = True    # runs all 7 IMO/ITTC tests
RUN_COMPARISON      = False   # OL vs CL comparison suite
```

### PID toggles (one per maneuver)

```python
USE_PID_STRAIGHT    = False   # course-keeping on straight run
USE_PID_ZIGZAG      = ...     # controlled by ZIGZAG_MODE below
USE_PID_STOPPING    = False   # heading-hold during crash-stop
USE_PID_PULLOUT     = False   # course-keeps in straight + coast phases
USE_PID_SPIRAL      = False   # PID heading-step variant
USE_PID_VALIDATION  = False   # full validation suite in CL mode
```

### ZigZag mode

```python
ZIGZAG_MODE = 'both'    # 'open-loop' | 'closed-loop' | 'both'
```

### Global PID gains

```python
PID_KP        = 20.0    # [deg/rad]    proportional
PID_KI        = 0.3     # [deg/rad.s]  integral  <- main overshoot lever
PID_KD        = 3.0     # [deg.s/rad]  derivative
PID_DELTA_MAX = 35.0    # [deg]        rudder saturation
```

### Positional argument trap (important)

Both zigzag functions have `dt` as their 5th positional argument and `n_rpm` as the 6th. Always use the keyword:

```python
# WRONG  -- 95. maps to dt, not n_rpm
run_zigzag_openloop(ship, init[0], 10., 300, 95.)

# CORRECT
run_zigzag_openloop(ship, init[0], 10., 300, n_rpm=95.)
run_zigzag_closedloop(ship, init[0], 10., 300, n_rpm=95., Kp=20, Ki=0.3, Kd=3)
```

-----

## Module Reference

*(Detailed module API documentation...)*

-----

## Maneuvering Trials

### Straight-Line Acceleration

Ship accelerates from rest (u=0.1 m/s) to steady state at 95 RPM.

| Quantity | Value |
|---|---|
| Steady-state speed | 10.68 m/s |
| Heading drift (OL) | 0.000 deg |
| East drift (OL) | 0.00 m |

### Turning Circle

| Parameter | 18 deg rudder | 35 deg rudder |
|---|---|---|
| Turning radius | 141 m | 49 m |
| Tactical diameter | 282 m (5.87 L) | 98 m (2.05 L) |
| Steady roll phi\_ss | 23.1 deg | -- |
| Thesis phi\_ss target | 20 deg | -- |
| IMO limit (tact dia) | -- | \<= 5.0 L |
| IMO result | -- | 2.05 L PASS |

### ZigZag

| Test | Stbd overshoot | Port overshoot | IMO limit | Result |
|---|---|---|---|---|
| 10/10 ZigZag (OL) | 5.95 deg | 6.01 deg | \<= 10 deg | PASS |
| 20/20 ZigZag (OL) | 14.30 deg | 14.33 deg | \<= 25 deg | PASS |

**10/10 ZigZag (Open-Loop vs Closed-Loop):**

**20/20 ZigZag (Open-Loop vs Closed-Loop):**

### Stopping Trial

| Quantity | Value | IMO limit |
|---|---|---|
| Head reach | 109 m (2.27 L) | \<= 15 L |
| Time to stop | 11 s | -- |
| Result | PASS | -- |

Protocol: full ahead 100 s then full astern (n\_full = 160 RPM).
Reference: Pakkan (2007) p.119; IMO MSC.137(76) para 3.3.

### Pull-Out Maneuver

| Quantity | Value |
|---|---|
| Steady yaw rate (20 deg rudder) | -5.031 deg/s |
| Residual yaw rate (after pullout) | 0.00000 deg/s |
| Verdict | DIRECTIONALLY STABLE |

Protocol: 70 s straight, 20 deg rudder for 200 s, then rudder to zero.
Reference: Pakkan (2007) pp.120-122; ITTC 7.5-04-02-01 Section 4.3.

### Spiral Maneuver (Dieudonné)

| Quantity | Value |
|---|---|
| Max hysteresis | 0.00004 deg/s |
| Threshold | \< 0.05 deg/s |
| Verdict | DIRECTIONALLY STABLE |
| Course stability C | 1.217 (\> 1 = stable) |

Protocol: rudder swept +20 -\> -20 -\> +20 in 5 deg steps, 150 s settle.
Reference: Pakkan (2007) pp.122-126; ITTC 7.5-04-02-01 Section 4.4.

-----

## Simulation Results Summary

All 9/9 IMO/ITTC checks pass:

| Test | Result | Limit | Status |
|---|---|---|---|
| Tactical diameter (35 deg) | 2.05 L | \<= 5.0 L | PASS |
| Tactical diameter (18 deg) | 5.87 L | \<= 7.0 L (thesis) | PASS |
| 10/10 ZZ stbd overshoot | 5.95 deg | \<= 10 deg | PASS |
| 10/10 ZZ port overshoot | 6.01 deg | \<= 10 deg | PASS |
| 20/20 ZZ stbd overshoot | 14.30 deg | \<= 25 deg | PASS |
| 20/20 ZZ port overshoot | 14.33 deg | \<= 25 deg | PASS |
| Stopping head reach | 2.27 L | \<= 15 L | PASS |
| Pull-out r\_final | 0.00000 deg/s | \~= 0 | PASS |
| Spiral hysteresis | 0.00004 deg/s | \< 0.05 deg/s | PASS |

-----

## PID Closed-Loop Control

All maneuvers (except turning circle) support a `use_pid=True` flag that switches from open-loop rudder commands to PID heading control.

### What PID does per maneuver

| Maneuver | Open-loop | Closed-loop PID |
|---|---|---|
| Straight line | delta = 0 throughout | PID holds psi = 0 — eliminates heading drift |
| Turning circle | Fixed rudder (always OL) | N/A — fixed rudder IS the test |
| ZigZag | Bang-bang +/-delta\_zz | PID tracks psi\_desired = +/-delta\_zz setpoint |
| Stopping | delta = 0 throughout | PID holds heading at moment of engine reversal |
| Pull-out | delta = 0 in straight/coast | PID course-keeps in straight + coast phases |
| Spiral | Fixed rudder steps | PID heading-step variant |

### ZigZag PID results

| Test | OL overshoot | CL overshoot | Reduction |
|---|---|---|---|
| 10/10 ZigZag | 5.95 deg | 1.47 deg | 75% |
| 20/20 ZigZag | 14.30 deg | 3.18 deg | 78% |

Recommended gains: **Kp=20, Ki=0.3, Kd=3, delta\_max=35 deg**.
Ki is the primary overshoot lever. Kd has minimal effect on steady-state overshoot.

### Secondary PID benefits (ZigZag)

  - Peak roll angle reduced: 13.7 deg OL -\> 7.4 deg CL (46% reduction)
  - Peak sway force reduced \~30%
  - Rudder continuously modulated vs bang-bang: smoother actuator usage

-----

## Open-Loop vs Closed-Loop Comparison

Set `RUN_COMPARISON = True` in `main.py` to generate side-by-side comparison figures for all maneuvers simultaneously.

The comparison produces 7 figures:

  - `plot_comparison_straightline` — heading drift and NE trajectory
  - `plot_comparison_zigzag` (10/10 and 20/20) — heading, rudder, roll, yaw rate, NE trajectories
  - `plot_comparison_stopping` — surge velocity, heading hold, NE trajectory
  - `plot_comparison_pullout` — yaw rate, heading, NE trajectories
  - `plot_comparison_spiral` — r(delta) overlay and NE trajectories
  - `plot_comparison_dashboard` — all maneuvers in one 7-row figure

Colour convention: **coral (\#f78166) = open-loop**, **green (\#3fb950) = closed-loop PID**.

**Runtime note:** The spiral runs twice (OL + CL) and is the longest step. Reduce `t_settle` to 60 s and `delta_step` to 10 deg in the comparison block of `main.py` for faster exploratory runs.

-----

## 2D Trajectory Plots

Every maneuver plot now includes a North-East plane trajectory panel showing the ship's physical path, coloured by simulation progress (t0 = dark, te = bright).

| Maneuver | Trajectory characteristics |
|---|---|
| Straight line | Nearly vertical line — ship goes North, minimal East drift |
| Turning circle | Full circular arc with equal-aspect scaling |
| ZigZag | Snake-like lateral path; OL wanders more than CL |
| Stopping | Near-vertical line with reversal and stop markers annotated |
| Pull-out | Straight run, arc turn, coast path; OL vs CL overlaid |
| Spiral | Multi-cluster pattern: each rudder step produces one circle |

All trajectory panels are produced by the shared `_traj_panel()` helper in `plotters.py`, which enforces a minimum 30 m East span to prevent degenerate aspect ratios on straight-line maneuvers.

-----

## Spiral Maneuver — Trajectory Notes

The spiral trajectory shape depends critically on `t_settle`:

**t\_settle = 150 s (default for r\_ss accuracy):**
One circle takes \~83 s at 20 deg rudder, so 150 s gives 1.8 circles per step. The trajectory shows dense multi-loop clusters at each rudder position.

**t\_settle = 83 s (thesis Figure 36 match):**
Approximately 1 circle per step. Produces the same visual pattern as thesis Figure 36 — one large nested cluster for the starboard steps, a long transition curve, and one cluster for the port steps.

**t\_straight = 200 s:**
Including a straight pre-run replicates the thesis figure which shows the ship travelling several hundred meters forward before the spiral begins.

```python
# For thesis-equivalent trajectory:
run_spiral(ship, init_state, t_settle=83., t_straight=200., delta_step=5.)

# For accurate r_ss measurement (validation):
run_spiral(ship, init_state, t_settle=150., t_straight=0., delta_step=5.)
```

Both settings give r(delta) hysteresis well below the 0.05 deg/s stability threshold.

-----

## Known Limitations and Calibration Notes

### Kur and Kp scaling

`Kur` is set to **14x its Appendix A value** and `Kp` to **10x**. These are deliberate calibrations to compensate for the change from linear (thesis: u\_ss\~7.2 m/s at 80 RPM) to quadratic thrust (simulator: u\_ss\~10.7 m/s at 95 RPM). Because `K_centripetal = Kur * u * r`, the heeling moment is proportionally larger at higher speed. Without scaling, the turning circle roll would be less than 2 deg instead of the thesis target of 20 deg.

### Pitch oscillations

Undamped pitch oscillations (\~1 deg, \~9 s period) appear throughout all maneuvers due to surge-pitch coupling through `M[surge,pitch] = -m*zG = 630,000 kg.m`. The thesis does not model pitch hydrodynamic damping (Mqq, Mq), so oscillations do not decay. This matches the original thesis behaviour.

### Excluded hydrodynamic terms

Two terms were excluded after stability analysis:

`Narv * |r| * v`: Overrides 57% of primary yaw damping in turning maneuvers, effectively inverting net yaw damping sign and causing yaw divergence. Requires verification against original RPMM dataset before re-enabling.

`Yavr * |v| * r` and `Yarv * r * |v|`: Amplify inboard sway during turning without corresponding roll corrections, causing roll divergence above \~8 deg rudder.

### Wind force guard

Wind forces are applied **only when v\_wind \> 0**. Applying aerodynamic coefficients to the ship's own forward speed in still air produces a spurious \~3.4 kN sway force and \~8 kN.m yaw moment that biases every straight-line run.

-----

## Bugs Fixed

### `vessel/model.py`

| Bug | Fix |
|---|---|
| `get_mass_matrix()`: `m = self.m` assigned params object to mass variable | Changed to `p = self.m`, `mass = p.m` |
| `dynamics()` and `get_forces()`: all `self.xxx` caused AttributeError | Added `p = self.m` alias throughout |
| State unpacking: roll rate variable `p` overwrote params alias | Renamed to `roll_rate` |
| Wind force: V\_rw = u\_ss in calm air -\> spurious 3.4 kN sway, 8 kN.m yaw | Wind forces set to zero when v\_wind = 0 |
| Missing terms: Narr, Yauavf, Yauarf not in model despite being in params | Added to N\_h and Y\_h expressions |

### `maneuvers/straight_line.py`

| Bug | Fix |
|---|---|
| `t_end` parameter accepted but ignored (hardcoded 200 s) | Uses t\_end throughout |

### `maneuvers/zigzag.py`

| Bug | Fix |
|---|---|
| Switch trigger direction inverted (positive rudder turns port, not stbd) | Corrected trigger conditions |
| Accumulated heading used in trigger — false switches at +/-180 wrap | Heading wrapped to [-pi, pi] before comparison |
| `compute_overshoot()` used raw accumulated heading | Same wrap applied before peak detection |

### `control/pid.py`

| Bug | Fix |
|---|---|
| Sign convention inverted: positive error -\> positive rudder -\> wrong direction | Added `delta = -clip(P+I+D, ...)` |
| Gains 100x too small: Kp=2.5 gives 0.44 deg rudder for 10 deg error | Updated defaults Kp=20, Ki=0.3, Kd=3 |

-----

## References and Standards

### IMO Standards

**[1] IMO Resolution MSC.137(76)** (4 December 2002).
*Standards for Ship Maneuverability.*

  - Para 3.1: Tactical diameter \<= 5.0 L (all ships)
  - Para 3.2a: 10/10 ZigZag 1st overshoot \<= 10 deg (L \< 200 m)
  - Para 3.2b: 20/20 ZigZag 1st overshoot \<= 25 deg (L \< 200 m)
  - Para 3.3: Stopping track reach \<= 15 L

**[2] IMO MSC/Circ.1053** (16 December 2002).
*Explanatory Notes to the Standards for Ship Maneuverability.*

### ITTC Procedures

**[3] ITTC Recommended Procedures 7.5-04-02-01** (2002).
*Maneuvering Trial Code.* 23rd ITTC, Venice.

  - Section 4.3: Pull-out maneuver protocol
  - Section 4.4: Spiral (Dieudonne) maneuver protocol

### Thesis

**[4] Pakkan, S.** (2007).
*Modeling and Simulation of a Maneuvering Ship.*
M.Sc. Thesis, Middle East Technical University, Ankara.

  - Appendix A: vessel parameters and hydrodynamic derivatives
  - Section 5.3.2, pp.115-127: maneuvering trial protocols and results
  - Figure 36 (p.125): spiral trajectory reference (use t\_settle=83 s, t\_straight=200 s)

### Supporting References

**[5] Perez, T. and Blanke, M.** (2007).
*Mathematical Ship Modeling for Control Applications.*
Technical Report, Technical University of Denmark.

**[6] Fossen, T. I.** (2011).
*Handbook of Marine Craft Hydrodynamics and Motion Control.* Wiley.

**[7] Dieudonne, J.** (1953).
*Etude systematique sur la stabilite en route des navires.* ATMA Bulletin.
(Original source of the spiral maneuver method.)