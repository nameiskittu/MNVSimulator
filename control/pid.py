# control/pid.py
import numpy as np


class PIDController:
    """
    Heading autopilot for surface ship maneuvering.

    Computes a rudder angle command [deg] from a heading error [rad].

    Sign convention (important):
        Positive rudder (delta > 0) turns the ship to PORT (psi decreases).
        Therefore a POSITIVE heading error (want to go starboard) requires a
        NEGATIVE rudder command.  The controller applies this sign internally:

            delta_cmd = -clip(P + I + D, ±delta_max)

    Gains (units):
        Kp  [deg/rad]     proportional — maps heading error to rudder angle
        Ki  [deg/(rad·s)] integral     — eliminates steady-state offset
        Kd  [deg·s/rad]   derivative   — damps oscillation

    Tuning guidance (Nomoto 1st-order heading plant, K=0.878 (°/s)/°, T=1.96 s):
        For a 10° heading error to command ~10° rudder: Kp ≈ 57 deg/rad.
        Use lower Kp (≈20) for gentler response; add Ki (≈0.3) to eliminate
        the resulting proportional droop.  Add Kd (≈3) for turn-rate damping.

        Recommended defaults: Kp=20, Ki=0.3, Kd=3.0, delta_max=35 deg
    """

    def __init__(self, Kp=20.0, Ki=0.3, Kd=3.0, delta_max=35.0):
        self.Kp        = Kp           # [deg/rad]   - Proportional gain
        self.Ki        = Ki           # [deg/rad·s] - Integral gain
        self.Kd        = Kd           # [deg·s/rad] - Derivative gain
        self.delta_max = delta_max    # [deg]        - Rudder saturation limit

        self._integral   = 0.0        # [rad·s]  - Accumulated heading error
        self._prev_error = 0.0        # [rad]    - Error from previous step
        self._prev_t     = None       # [s]      - Previous call time

    def reset(self):
        """Reset integrator and memory — call before each new maneuver."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_t     = None

    def compute(self, t, psi_current, psi_desired):
        """
        Compute rudder command from current and desired heading.

        Args:
            t           [s]   - Current simulation time
            psi_current [rad] - Current ship heading (wrapped to [-pi, pi])
            psi_desired [rad] - Desired heading

        Returns:
            delta_cmd   [deg] - Rudder angle command, clamped to ±delta_max
        """
        # Heading error wrapped to [-pi, pi] to handle 0/360 crossings
        error = psi_desired - psi_current
        error = (error + np.pi) % (2 * np.pi) - np.pi   # [rad]

        # Time step
        if self._prev_t is None:
            dt = 0.05
        else:
            dt = max(t - self._prev_t, 1e-6)

        # PID terms
        P = self.Kp * error                                # [deg]
        self._integral += error * dt                       # [rad·s]
        I = self.Ki * self._integral                       # [deg]
        D = self.Kd * (error - self._prev_error) / dt     # [deg]

        self._prev_error = error
        self._prev_t     = t

        # Sign flip: positive error (want starboard) requires negative rudder
        # (positive rudder turns ship to port in this vessel's convention)
        delta_cmd = np.clip(-(P + I + D), -self.delta_max, self.delta_max)
        return delta_cmd