# MNVSimulator

A 6-DOF maneuvering simulator for a Multipurpose Naval Vessel (MNV), implemented in Python. The simulator reproduces the mathematical ship model from Sinan Pakkan's 2007 M.Sc. thesis (*Modeling and Simulation of a Maneuvering Ship*, METU) and validates it against standard ITTC maneuvering trials.

---

## Table of Contents

1. [Overview](#overview)
2. [Vessel Specifications](#vessel-specifications)
3. [Mathematical Model](#mathematical-model)
4. [Project Structure](#project-structure)
5. [Installation](#installation)
6. [Quick Start](#quick-start)
7. [Module Reference](#module-reference)
8. [Maneuvering Trials](#maneuvering-trials)
9. [Simulation Results](#simulation-results)
10. [Known Limitations and Calibration Notes](#known-limitations-and-calibration-notes)
11. [Bugs Fixed](#bugs-fixed)
12. [References](#references)

---

## Overview

MNVSimulator integrates the nonlinear 6-DOF equations of motion for a surface ship in the body-fixed frame, driven by hydrodynamic hull forces, a quadratic propeller thrust model, a first-order rudder actuator, and optional environmental disturbances (wind, waves, ocean current). Three standard ITTC maneuvering tests are implemented:

- **Straight-line acceleration** — speed build-up and force balance verification
- **Turning circle** — steady roll angle, tactical diameter, turning radius
- **ZigZag maneuver (10/10 and 20/20)** — heading overshoot against IMO criteria

The integrator is SciPy's `solve_ivp` with an RK45 adaptive step-size solver.

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
| Transverse metacentric height GM_T | 0.776 | m |
| Longitudinal metacentric height GM_L | 7.76 | m |
| Max propeller speed n_max | 160 | RPM |
| Max rudder angle d_max | 45 | deg |
| Max rudder rate d_dot_max | 20 | deg/s |
| Max thrust (quadratic law) TA_max | 635,000 | N |

Hydrodynamic derivatives are taken from Appendix A of Pakkan (2007), which references RPMM (Roll Planar Motion Mechanism) data for this vessel from Perez & Blanke (2007).

---

## Mathematical Model

### Equations of Motion

The simulator solves the nonlinear 6-DOF body-frame equations:

```
M * v_dot  =  tau_hull + tau_thrust + tau_env - g(eta)
eta_dot    =  J(eta) * v
```

where:
- `v  = [u, v, w, p, q, r]`  body-frame velocities
- `eta = [x, y, z, phi, theta, psi]`  NED position and Euler angles
- `M  = M_RB + M_A`  total mass matrix (rigid body + frequency-independent added mass)
- `g(eta)`  gravitational and buoyancy restoring forces
- `J(eta)`  kinematic transformation matrix (yaw-pitch-roll Euler sequence)

### State Vector (14 elements)

| Index | Symbol | Unit | Description |
|---|---|---|---|
| 0 | u | m/s | Surge velocity (forward) |
| 1 | v | m/s | Sway velocity (port positive) |
| 2 | w | m/s | Heave velocity (up positive) |
| 3 | p | rad/s | Roll rate |
| 4 | q | rad/s | Pitch rate |
| 5 | r | rad/s | Yaw rate |
| 6 | x | m | North position (NED) |
| 7 | y | m | East position (NED) |
| 8 | z | m | Depth (NED, positive down) |
| 9 | phi | rad | Roll angle |
| 10 | theta | rad | Pitch angle |
| 11 | psi | rad | Heading (yaw) angle |
| 12 | n_act | RPM | Actual propeller shaft speed |
| 13 | delta_act | rad | Actual rudder angle |

### Hull Force Expressions

**Surge (X):**
```
X_h = Xauu*|u_r|*u_r  +  Xvr*v_r*r
```

**Sway (Y):**
```
Y_h = Yauv*|u_r|*v_r  +  Yur*u_r*r  +  Yavv*|v_r|*v_r
    + Yauavf*|u_r|*|v_r|*delta  +  Yauarf*|u_r|*|r|*delta
    + (Yuuf + Yduu)*u_r^2*delta
```

**Roll (K):**
```
K_h = Kauv*|u_r|*v_r  +  Kur*u_r*r  +  Kavv*|v_r|*v_r
    + Kavr*|v_r|*r  +  Karv*r*|v_r|
    + Kauavf*|u_r|*|v_r|*delta  +  Kauarf*|u_r|*|r|*delta  +  Kuuf*u_r^2*delta
    + Kaup*|u_r|*p  +  Kapp*|p|*p  +  Kp*p  +  Kfff*phi^3
```

**Yaw (N):**
```
N_h = Nauv*|u_r|*v_r  +  Naur*|u_r|*r  +  Narr*|r|*r
    + Nauavf*|u_r|*|v_r|*delta  +  Nauuf*u_r^2*delta  +  Naruf*|u_r|*|r|*delta
```

Where `u_r = u - u_current` and `v_r = v - v_current` are current-relative velocities.

### Propeller Thrust Model

Quadratic (propeller law) scaling:

```
T = TA_max * (|n_act| / n_max)^2 * sign(n_act)
```

Calibrated to give `u_ss ≈ 10.7 m/s` at 95 RPM in calm water.

### Actuator Dynamics

**Propeller shaft** — variable time constant:

```
Tm = 18.83 s           when |n_act| < 20 RPM  (idle)
Tm = 5.65 / |n_act|    otherwise               (running)
n_dot = (n_cmd - n_act) / Tm
```

**Rudder** — first-order lag with rate limit:

```
Tc = 1.0 s
delta_dot = clip( (delta_cmd - delta_act) / Tc,  +/- 20 deg/s )
```

### Restoring Forces

```
g_roll  = m * g * GM_T * sin(phi)    (linear gravity restoring, always active)
K_Kfff  = Kfff * phi^3               (nonlinear correction in K_h)
g_pitch = m * g * GM_L * sin(theta)  (pitch restoring)
```

`Kfff = -0.325 * rho_w * g * (m / rho_w)` is derived from first principles.

### Environmental Disturbances

All disturbances are zero by default. They are activated by setting parameters in `VesselParams`:

| Parameter | Default | Description |
|---|---|---|
| `v_curr` | 0.0 m/s | Ocean current speed |
| `psi_curr` | 0.0 rad | Current direction |
| `v_wind` | 0.0 m/s | Wind speed — **set > 0 to activate wind forces** |
| `psi_wind` | pi/2 rad | Wind direction |
| `X_wave_amp` | 0.0 N | Sinusoidal wave surge force amplitude |
| `Y_wave_amp` | 0.0 N | Sinusoidal wave sway force amplitude |

> **Important:** Wind forces are applied **only when `v_wind > 0`**. The relative wind velocity formula subtracts the ship's own velocity from the true wind vector. When `v_wind = 0`, applying these coefficients to the ship's forward speed would produce a spurious ~3.4 kN sway force and ~8 kN·m yaw moment in every straight-line run — a model artifact that has been explicitly guarded against.

---

## Project Structure

```
MNVSimulator/
|
+-- main.py                    Entry point — toggle maneuvers, set initial state
|
+-- vessel/
|   +-- params.py              VesselParams: all physical and hydrodynamic constants
|   +-- model.py               ShipModel: 6-DOF dynamics, mass matrix, force equations
|
+-- maneuvers/
|   +-- straight_line.py       run_straight_line()
|   +-- turning_circle.py      run_turning_circle()
|   +-- zigzag.py              run_zigzag_openloop(), compute_overshoot()
|
+-- control/
|   +-- pid.py                 PIDController: heading autopilot for closed-loop use
|
+-- utils/
|   +-- plotters.py            Dark-theme publication-quality plot functions
|
+-- requirements.txt
+-- README.md
```

---

## Installation

**Requirements:** Python 3.10+, numpy, scipy, matplotlib.

```bash
pip install numpy scipy matplotlib
```

To pin exact versions:

```bash
pip install -r requirements.txt
```

---

## Quick Start

```bash
python main.py
```

Toggle maneuvers at the top of `main.py`:

```python
RUN_STRAIGHT_LINE  = True
RUN_TURNING_CIRCLE = False   # set True to enable
RUN_ZIGZAG         = True
```

### Running from a script

```python
from vessel.params import VesselParams
from vessel.model import ShipModel
from maneuvers.zigzag import run_zigzag_openloop, compute_overshoot

# Set up vessel
params = VesselParams()
ship   = ShipModel(params)

# 14-element initial state
# [u, v, w, p, q, r, x, y, z, phi, theta, psi, n_act, delta_act]
init_state = [0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 95.0, 0]

# Run 10/10 ZigZag
t, y, X, Y, K, N, delta = run_zigzag_openloop(
    ship,
    init_speed=init_state[0],
    delta_zz_deg=10.0,
    t_end=300,
    n_rpm=95.0
)

os_stbd, os_port = compute_overshoot(y, 10.0)
print(f"Overshoot  Stbd: {os_stbd:.2f} deg   Port: {os_port:.2f} deg")
```

### Enabling environmental disturbances

```python
params = VesselParams()

# 2-knot beam wind from 090 degrees
params.v_wind   = 1.03     # m/s
params.psi_wind = 1.5708   # rad (090 deg)

# 0.5-knot head current
params.v_curr   = 0.257    # m/s
params.psi_curr = 0.0      # rad (000 deg, head-on)

# Irregular wave amplitudes
params.X_wave_amp = 5000.0  # N
params.Y_wave_amp = 3000.0  # N
```

---

## Module Reference

### `vessel/params.py` — `VesselParams`

Stores every physical constant and hydrodynamic derivative. All parameters are dimensional (SI). Modify this file to change the vessel or its environment.

**Key parameter groups:**

| Group | Examples | Purpose |
|---|---|---|
| Geometry | `L`, `B`, `T`, `m` | Hull dimensions and mass |
| Inertia | `Ixx`, `Iyy`, `Izz`, `xG`, `zG` | Moments of inertia and CoG offsets |
| Stability | `GM_T`, `GM_L` | Metacentric heights |
| Added mass | `Xud`, `Yvd`, `Nrd` | Frequency-independent added inertia |
| Surge derivatives | `Xauu`, `Xvr` | Hull drag and coupling |
| Sway derivatives | `Yauv`, `Yur`, `Yavv`, `Yauavf`, `Yauarf` | Sway hull forces |
| Roll derivatives | `Kauv`, `Kur`, `Kavv`, `Kavr`, `Kp`, `Kfff` | Roll moments |
| Yaw derivatives | `Nauv`, `Naur`, `Narr`, `Nauuf`, `Naruf` | Yaw moments |
| Actuators | `TA_max`, `nmax`, `deltamax`, `deltadotmax` | Propulsion and rudder limits |
| Environment | `v_wind`, `psi_wind`, `v_curr`, `psi_curr` | Disturbance inputs |

### `vessel/model.py` — `ShipModel`

```python
ship = ShipModel(params)

# 6x6 total mass matrix (rigid body + added mass)
M = ship.get_mass_matrix()

# Right-hand side of equations of motion — pass directly to solve_ivp
dxdt = ship.dynamics(t, state, n_cmd, delta_deg)

# Instantaneous hull + thrust forces at a given state
X, Y, K, N = ship.get_forces(state, delta_deg)
```

`dynamics()` returns the 14-element derivative vector:
`[u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, n_dot, delta_dot]`

### `maneuvers/straight_line.py`

```python
from maneuvers.straight_line import run_straight_line

sol = run_straight_line(ship, init_state, n_cmd=95.0, t_end=300)
# Returns scipy OdeSolution
# sol.t   time array [s]
# sol.y   state array [14 x N_points]
```

### `maneuvers/turning_circle.py`

```python
from maneuvers.turning_circle import run_turning_circle

results = run_turning_circle(ship, init_state, n_cmd=95.0, rudder_deg=18.0)
```

Returns a dictionary:

| Key | Type | Description |
|---|---|---|
| `t` | array | Combined time history |
| `y` | array [14 x N] | Combined state history |
| `t_rudder` | float | Time at which rudder was applied [s] |
| `radius` | float | Tactical turning radius [m] |
| `advance` | float | Advance distance [m] |
| `transfer` | float | Transfer distance [m] |
| `sol_pre` | OdeSolution | Straight-run phase |
| `sol_turn` | OdeSolution | Turning phase |

The maneuver runs a 300 s straight pre-run to build speed, applies the rudder once the ship has travelled 200 m, then turns for 300 s.

### `maneuvers/zigzag.py`

```python
from maneuvers.zigzag import run_zigzag_openloop, compute_overshoot

t, y, X, Y, K, N, delta = run_zigzag_openloop(
    ship,
    init_speed=0.1,       # m/s
    delta_zz_deg=10.0,    # deg
    t_end=300.0,          # s
    dt=0.05,              # s — fixed integration sub-step
    n_rpm=95.0            # RPM
)

# y  shape [14 x N_points] — state history
# X, Y, K, N — force/moment histories [N] or [N·m]
# delta — rudder command history [deg]

os_stbd, os_port = compute_overshoot(y, delta_zz_deg=10.0)
```

**Switching logic:** Positive rudder command turns the ship to port (heading ψ decreases).
- When `psi <= -psi_trigger` → flip to negative rudder
- When `psi >= +psi_trigger` → flip to positive rudder

Heading is wrapped to `[-180, +180]` degrees before trigger comparison to prevent false switches from accumulated angular drift.

### `control/pid.py` — `PIDController`

A heading autopilot for closed-loop maneuvers. Not used in the default open-loop ITTC tests, but available for custom control scenarios.

```python
from control.pid import PIDController

pid = PIDController(Kp=2.5, Ki=0.01, Kd=1.0, delta_max=35.0)
pid.reset()   # call before each new maneuver

# Inside integration loop:
delta_cmd = pid.compute(t, psi_current, psi_desired)  # returns [deg]
```

Heading error is wrapped to `[-pi, pi]` before PID computation to handle 0/360 crossings.

### `utils/plotters.py`

All functions accept simulation output directly and return `matplotlib.figure.Figure` objects rendered with a dark theme.

| Function | Description |
|---|---|
| `plot_straight_line(sol)` | 3x3 grid of all state variables vs time |
| `plot_trajectory(x, y)` | NED trajectory with progress colourmap |
| `plot_turning_circle(results)` | Trajectory + roll/yaw rate/speed panels |
| `plot_zigzag_states(t, y, delta_zz)` | 3x3 state grid with heading trigger lines |
| `plot_zigzag_forces(t, X, Y, K, N)` | 2x2 force and moment panels |
| `plot_zigzag_standard(t, y, d, delta_zz)` | ITTC standard psi + delta with overshoot shading |
| `plot_zigzag_trajectory(y)` | NED trajectory with progress colourmap |

**Colour convention:**

| Channel | Colour |
|---|---|
| Surge u | Electric blue |
| Sway v | Coral |
| Roll phi | Lavender |
| Pitch theta | Amber |
| Heading psi | Sky blue |
| Rudder delta | Amber |
| X force | Blue |
| Y force | Coral |
| K moment | Lavender |
| N moment | Green |

---

## Maneuvering Trials

### Straight-Line Acceleration

Starting from nearly-stopped (`u0 = 0.1 m/s`), the ship accelerates to steady state at 95 RPM. At `t = 300 s` the sway velocity, roll angle, yaw rate and heading drift are all exactly zero in calm water. The net X force approaches zero as thrust balances quadratic drag. The true steady-state speed is:

```
u_ss = sqrt( TA_max * (n/n_max)^2 / |Xauu| ) = 10.687 m/s
```

The 300 s simulation reaches `u ≈ 10.68 m/s`; extending to 400 s gives full convergence.

### Turning Circle

Applied after a 300 s straight pre-run. Rudder: 18° starboard, RPM: 95.

| Metric | Simulated | Thesis Target |
|---|---|---|
| Turning radius R | ~141 m | ~160 m |
| Tactical diameter | ~283 m | 320 m (< 7 L ✓) |
| Steady roll phi_ss | ~23° | 20° (corvette limiting case) |
| Advance | ~252 m | — |
| Transfer | ~281 m | — |

The ~15% difference from the thesis radius is attributable to additional hydrodynamic terms (`Narr`, `Yauavf`, `Yauarf`) added in this implementation that were absent from the original MATLAB code. These terms increase yaw and sway damping, tightening the turn slightly.

### ZigZag Maneuver (ITTC Open-Loop)

| Test | Stbd Overshoot | Port Overshoot | IMO Limit |
|---|---|---|---|
| 10/10 ZigZag | 5.95 deg | 6.01 deg | <= 10 deg [PASS] |
| 20/20 ZigZag | 14.30 deg | 14.33 deg | <= 25 deg [PASS] |

Both tests pass IMO MSC.137(76) criteria. The overshoot is physically explained as:

- **71% from yaw inertia:** The ship carries angular momentum (~2.24e6 N·m·s) at the trigger point that cannot be stopped instantly; heading continues past the threshold for ~6 s while the yaw rate decelerates.
- **29% from rudder actuator lag:** A 20° reversal at 20 deg/s takes ~1 s, during which the original rudder direction is still partially active.

The vessel is directionally stable (course stability index C = 1.22 > 1), so overshoot is a normal inertial phenomenon, not an instability.

---

## Simulation Results Summary

| Quantity | Value | Unit |
|---|---|---|
| Steady cruise speed at 95 RPM | 10.68 | m/s |
| Turning radius (18 deg rudder) | 141 | m |
| Tactical diameter | 283 | m |
| Steady-state roll in turn | 23 | deg |
| 10/10 ZZ overshoot (stbd / port) | 5.95 / 6.01 | deg |
| 20/20 ZZ overshoot (stbd / port) | 14.30 / 14.33 | deg |
| Course stability index C | 1.22 | — |
| Nomoto time constant T | 1.96 | s |

---

## Known Limitations and Calibration Notes

### Kur and Kp scaling

`Kur` is set to **14x its Appendix A value** (-1,428,000 vs -102,000 kg·m) and `Kp` to **10x** (-5,000,000 vs -500,000 kg·m2/s). These are deliberate calibrations to compensate for the difference in thrust model relative to the thesis:

- **Thesis:** linear thrust `T = TA * (n/n_max)`, `TA = 200,000 N`, operating at 80 RPM → `u_ss ≈ 7.2 m/s`
- **This simulator:** quadratic thrust `T = TA_max * (n/n_max)^2`, `TA_max = 635,000 N`, operating at 95 RPM → `u_ss ≈ 10.7 m/s`

Because `K_centripetal = Kur * u * r`, the heeling moment is proportionally larger at higher speed. The 14x factor restores the thesis target of `phi_ss ≈ 20°` at the simulator's operating point. Without the scaling, the turning circle roll would be less than 2°.

### Pitch oscillations

Undamped pitch oscillations appear throughout all maneuvers (amplitude ~1°, period ~9 s). They arise from surge-pitch coupling through the off-diagonal mass matrix term `M[surge, pitch] = -m * zG = 630,000 kg·m`. The thesis does not model pitch hydrodynamic damping (`Mqq`, `Mq`), so oscillations do not decay. This matches the original thesis behaviour exactly.

### Excluded hydrodynamic terms

Two coefficient groups defined in `params.py` were excluded from `model.py` after stability analysis:

**`Narv * |r| * v` (yaw-sway cross coupling):** In a turning maneuver, with `r < 0` (port turn) and `v > 0` (outboard sway), this term produces a negative N moment — same direction as the turn. Its magnitude (`|Narv| = 15.6e6`) overrides 57% of the primary yaw damping `Naur * |u| * r`, effectively inverting the net yaw damping sign and driving the ship to spin uncontrollably. The physical sign requires verification against the original Perez & Blanke (2007) RPMM dataset before this term can be safely re-enabled.

**`Yavr * |v| * r` and `Yarv * r * |v|` (sway-yaw cross coupling):** These amplify inboard sway during turning without corresponding roll correction terms, leading to roll divergence at rudder angles above ~8°. Matching roll coupling terms from [Perez & Blanke, 2007] would be needed to safely include them.

---

## Bugs Fixed

### `vessel/model.py`

| # | Bug | Fix |
|---|---|---|
| 1 | `get_mass_matrix()`: `m = self.m` assigned the VesselParams object to `m`, so mass matrix elements contained a Python object instead of the float mass value | Changed to `p = self.m` (params alias) and `mass = p.m` (kg scalar) |
| 2 | `dynamics()` and `get_forces()`: every parameter access used `self.xxx`, causing AttributeError at runtime — all hydrodynamic parameters live in `self.m` | Added `p = self.m` alias; all accesses changed to `p.xxx` |
| 3 | State unpacking used `p` for roll rate: `u, v, w, p, q, r, ...` silently overwrote the params alias | Renamed roll rate to `roll_rate` in unpacking |
| 4 | Wind force formula `u_rw = v_wind*cos(psi_wind-psi) - u` gave `V_rw = u_ss ≈ 10.7 m/s` in calm air, generating a spurious 3.4 kN sway force and 8 kN·m yaw moment that biased every straight-line simulation | Wind forces are now exactly zero when `v_wind = 0` |
| 5 | `Narr`, `Yauavf`, `Yauarf` defined in params.py but absent from model.py | Added: `Narr * |r| * r` to N_h; `Yauavf * |u|*|v|*delta` and `Yauarf * |u|*|r|*delta` to Y_h |

### `maneuvers/straight_line.py`

| # | Bug | Fix |
|---|---|---|
| 6 | `t_end` parameter accepted but ignored — `t_span` and `t_eval` hardcoded to 200 s | Uses `t_end` throughout |

### `maneuvers/zigzag.py`

| # | Bug | Fix |
|---|---|---|
| 7 | Switching trigger direction inverted: positive rudder turns ship to port (psi decreases), so trigger should fire at `psi <= -psi_trigger`, not `psi >= +psi_trigger` | Corrected trigger conditions to match rudder sign convention |
| 8 | Accumulated (unwrapped) heading used in trigger check: ship completed full circles at the +-180 degree wrap crossing | Heading wrapped to [-pi, pi] before comparison |
| 9 | `compute_overshoot()` used raw accumulated heading, reporting overshoot in hundreds of degrees | Same [-180, 180] wrap applied before peak detection |

---

## References

1. Pakkan, S. (2007). *Modeling and Simulation of a Maneuvering Ship*. M.Sc. Thesis, Middle East Technical University, Ankara.

2. Perez, T. and Blanke, M. (2007). *Mathematical Ship Modeling for Control Applications*. Technical Report, Technical University of Denmark.

3. ITTC (2002). *Recommended Procedures and Guidelines — Maneuvering Trial Code*. 23rd ITTC, Venice.

4. IMO (2002). *Standards for Ship Maneuverability*. Resolution MSC.137(76).

5. Fossen, T. I. (2011). *Handbook of Marine Craft Hydrodynamics and Motion Control*. Wiley.