import numpy as np

class PIDController:
    def __init__(self, Kp=2.5, Ki=0.01, Kd=1.0, delta_max=35.0):
        self.Kp        = Kp           # [deg/rad]   - Proportional gain
        self.Ki        = Ki           # [deg/rad·s] - Integral gain
        self.Kd        = Kd           # [deg·s/rad] - Derivative gain
        self.delta_max = delta_max    # [deg]       - Rudder angle saturation limit

        self._integral    = 0.0       # [rad·s]     - Accumulated heading error over time
        self._prev_error  = 0.0       # [rad]       - Heading error from previous time step
        self._prev_t      = None      # [s]         - Previous time (for dt calculation)

    def reset(self):
        """Reset integrator and memory — call before each new maneuver."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_t     = None

    def compute(self, t, psi_current, psi_desired):
        """
        Compute rudder command to reach desired heading.

        Args:
            t           [s]   - Current simulation time
            psi_current [rad] - Current ship heading
            psi_desired [rad] - Target heading

        Returns:
            delta_cmd   [deg] - Rudder angle command (clamped to ±delta_max)
        """
        # Heading error — wrapped to [-pi, pi] to handle 0/360 crossings
        error = psi_desired - psi_current                    # [rad]
        error = (error + np.pi) % (2 * np.pi) - np.pi       # [rad] - wrap to [-pi, pi]

        # Time step
        if self._prev_t is None:
            dt = 0.05                                        # [s] - default first step
        else:
            dt = max(t - self._prev_t, 1e-6)                # [s] - avoid division by zero

        # PID terms
        P = self.Kp * error                                  # [deg] - Proportional term
        self._integral += error * dt                         # [rad·s]
        I = self.Ki * self._integral                         # [deg] - Integral term
        D = self.Kd * (error - self._prev_error) / dt       # [deg] - Derivative term

        # Save state for next call
        self._prev_error = error                             # [rad]
        self._prev_t     = t                                 # [s]

        # Rudder command = sum of PID, clamped to physical rudder limits
        delta_cmd = np.clip(P + I + D, -self.delta_max, self.delta_max)  # [deg]
        return delta_cmd