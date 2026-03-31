# vessel/model.py
import numpy as np
from vessel.params import VesselParams

class ShipModel:
    def __init__(self, params):
        self.m = params

    def get_mass_matrix(self):
        m = self.m
        xG, yG, zG = self.xG, self.yG, self.zG  # [m]

        # Rigid body mass matrix [6x6]
        M_RB = np.array([
            [m,      0,      0,       0,      m*zG,  -m*yG ],
            [0,      m,      0,      -m*zG,   0,      m*xG ],
            [0,      0,      m,       m*yG,  -m*xG,  0     ],
            [0,     -m*zG,   m*yG,   self.Ixx, 0,    0     ],
            [m*zG,   0,     -m*xG,   0,   self.Iyy,  0     ],
            [-m*yG,  m*xG,   0,      0,       0,  self.Izz ]
        ])  # Units: [kg / kg·m / kg·m2]

        # Added mass matrix [6x6]
        M_A = np.zeros((6, 6))
        M_A[0, 0] = -self.Xud    # [kg]
        M_A[1, 1] = -self.Yvd    # [kg]
        M_A[1, 3] = -self.Ypd    # [kg·m]
        M_A[1, 5] = -self.Yrd    # [kg·m]
        M_A[3, 1] = -self.Ypd    # [kg·m]
        M_A[3, 3] = -self.Kpd    # [kg·m2]
        M_A[3, 5] = -self.Krd    # [kg·m2]
        M_A[5, 1] = -self.Nvd    # [kg·m]
        M_A[5, 3] = -self.Npd    # [kg·m2]
        M_A[5, 5] = -self.Nrd    # [kg·m2]

        return M_RB + M_A  # Total mass matrix [6x6]

    def dynamics(self, t, state, n_cmd, delta_deg):
        """
        State vector (14 elements):
            [0]  u         [m/s]   - Surge velocity
            [1]  v         [m/s]   - Sway velocity
            [2]  w         [m/s]   - Heave velocity
            [3]  p         [rad/s] - Roll rate
            [4]  q         [rad/s] - Pitch rate
            [5]  r         [rad/s] - Yaw rate
            [6]  x         [m]     - North position
            [7]  y         [m]     - East position
            [8]  z         [m]     - Depth
            [9]  phi       [rad]   - Roll angle
            [10] theta     [rad]   - Pitch angle
            [11] psi       [rad]   - Heading angle
            [12] n_act     [RPM]   - Actual propeller speed
            [13] delta_act [rad]   - Actual rudder angle

        Inputs:
            n_cmd      [RPM]     - Commanded propeller speed
            delta_deg  [degrees] - Commanded rudder angle
        """
        u, v, w, p, q, r, x, y, z, phi, theta, psi, n_act, delta_act = state

        # Environmental Disturbances
        u_c = self.v_curr * np.cos(self.psi_curr - psi)   # [m/s] - Current surge component
        v_c = self.v_curr * np.sin(self.psi_curr - psi)   # [m/s] - Current sway component
        u_r = u - u_c                                      # [m/s] - Relative surge velocity
        v_r = v - v_c                                      # [m/s] - Relative sway velocity

        u_rw = self.v_wind * np.cos(self.psi_wind - psi) - u   # [m/s]
        v_rw = self.v_wind * np.sin(self.psi_wind - psi) - v   # [m/s]
        V_rw = np.sqrt(u_rw**2 + v_rw**2)                      # [m/s] - Relative wind speed
        X_wind = 0.5 * self.rho_a * V_rw**2 * self.Cw_X * self.A_x           # [N]
        Y_wind = 0.5 * self.rho_a * V_rw**2 * self.Cw_Y * self.A_y           # [N]
        N_wind = 0.5 * self.rho_a * V_rw**2 * self.Cw_N * self.A_y * self.L  # [N·m]

        X_wave = self.X_wave_amp * np.sin(0.5 * t)   # [N]
        Y_wave = self.Y_wave_amp * np.cos(0.5 * t)   # [N]

        X_env = X_wind + X_wave   # [N]
        Y_env = Y_wind + Y_wave   # [N]
        N_env = N_wind            # [N·m]

        X_h = (self.Xauu * abs(u_r) * u_r                        # [N] - Quadratic surge drag
             + self.Xvr  * v_r * r)                              # [N] - Sway-yaw coupling

        Y_h = (self.Yauv  * abs(u_r) * v_r                      # [N] - Sway from |u|*v
             + self.Yur   * u_r * r                              # [N] - Centripetal sway
             + self.Yavv  * abs(v_r) * v_r                      # [N] - Nonlinear sway drag
             + (self.Yuuf + self.Yduu) * delta_act * u_r**2)    # [N] - Rudder sway at speed

        K_h = (self.Kauv  * abs(u_r) * v_r              # [N·m] - Roll from |u|*v
             + self.Kur   * u_r * r                      # [N·m] - Centrifugal heeling (main term)
             + self.Kavv  * abs(v_r) * v_r               # [N·m] - Roll from |v|*v
             + self.Kavr  * abs(v_r) * r                 # [N·m] - Roll from |v|*r
             + self.Karv  * r * abs(v_r)                 # [N·m] - Roll from r*|v|
             + self.Kauavf * abs(u_r) * abs(v_r) * delta_act   # [N·m] - Rudder+sway roll
             + self.Kauarf * abs(u_r) * abs(r)  * delta_act    # [N·m] - Rudder+yaw roll
             + self.Kuuf  * u_r**2 * delta_act           # [N·m] - Rudder roll at speed
             + self.Kaup  * abs(u_r) * p                 # [N·m] - Roll rate damping at speed
             + self.Kapp  * abs(p) * p                   # [N·m] - Nonlinear roll damping
             + self.Kp    * p                             # [N·m] - Linear roll damping (calibrated)
             + self.Kfff  * (phi**3))                    # [N·m] - Nonlinear roll restoring

        N_h = (self.Nauv  * abs(u_r) * v_r               # [N·m] - Yaw from |u|*v
             + self.Naur  * abs(u_r) * r                  # [N·m] - Yaw damping at speed
             + self.Nauavf * abs(u_r) * abs(v_r) * delta_act   # [N·m] - Rudder+sway yaw
             + self.Nauuf * u_r**2 * delta_act             # [N·m] - Rudder yaw at speed
             + self.Naruf * abs(u_r) * abs(r) * delta_act) # [N·m] - Rudder+yaw coupling (was missing!)

        # Propeller Thrust — QUADRATIC scaling (propeller law: T ~ n^2)
        # Calibrated: TA_max*(n/nmax)^2 gives 18 m/s at full ahead (160 RPM)
        thrust = self.TA_max * (abs(n_act) / self.nmax)**2 * np.sign(n_act)  # [N]

        # Total Force Vector tau [6x1] -> [N, N, N, N·m, N·m, N·m]
        tau = np.array([
            thrust + X_h + X_env,   # [N]   - Surge
            Y_h    + Y_env,         # [N]   - Sway
            0,                      # [N]   - Heave (not modelled)
            K_h,                    # [N·m] - Roll
            0,                      # [N·m] - Pitch (not modelled)
            N_h    + N_env          # [N·m] - Yaw
        ])

        # Restoring Forces (Buoyancy)
        # Roll: linear restoring m*g*GM_T*sin(phi) is kept in g_eta.
        # Kfff*phi^3 in K_h provides the nonlinear correction term.
        # With Kur scaled to 17.3x, the centrifugal heeling force balances
        # the full restoring at phi_ss=20 deg, giving fast settling (~15s).
        g_eta = np.array([
            0,
            0,
            0,
            self.m * 9.81 * self.GM_T * np.sin(phi),      # [N·m] - Linear roll restoring
            self.m * 9.81 * self.GM_L * np.sin(theta),    # [N·m] - Pitch restoring
            0
        ])

        # Equations of Motion: nu_dot = M^-1 * (tau - g_eta)
        M      = self.get_mass_matrix()
        nu_dot = np.linalg.inv(M) @ (tau - g_eta)  # [m/s2 or rad/s2]

        # ------------------------------------------
        # SHAFT DYNAMICS  (Appendix B — variable time constant)
        # Tm = 18.83 s  when |n| < 20 RPM  (idle / low speed)
        # Tm = 5.65/|n| when |n| >= 20 RPM (running speed — faster response at higher RPM)
        # ------------------------------------------
        n_cmd_sat = np.clip(n_cmd, -self.nmax, self.nmax)   # [RPM] - saturated command
        Tm   = 18.83 if abs(n_act) < 20.0 else 5.65 / max(abs(n_act), 0.01)  # [s]
        ndot = (n_cmd_sat - n_act) / Tm                      # [RPM/s]

        # ------------------------------------------
        # RUDDER DYNAMICS  (Appendix B)
        # Transfer function: delta/delta_c = 1/(s+1)  → time constant Tc = 1s
        # Rate limited to deltadotmax = 20 deg/s
        # ------------------------------------------
        delta_cmd_sat = np.clip(np.deg2rad(delta_deg),
                                -np.deg2rad(self.deltamax),
                                 np.deg2rad(self.deltamax))  # [rad] - angle saturation
        Tc            = 1.0                                   # [s]   - rudder time constant (thesis Appendix B)
        deltadot_raw  = (delta_cmd_sat - delta_act) / Tc     # [rad/s] - first-order lag
        deltadot      = np.clip(deltadot_raw,
                                -np.deg2rad(self.deltadotmax),
                                 np.deg2rad(self.deltadotmax))  # [rad/s] - rate saturation

        # Kinematics: Body frame -> NED frame
        cp  = np.cos(phi);   sp  = np.sin(phi)
        ct  = np.cos(theta); st  = np.sin(theta)
        cps = np.cos(psi);   sps = np.sin(psi)

        x_dot     = u*(cps*ct) + v*(cps*st*sp - sps*cp) + w*(sps*sp + cps*cp*st)  # [m/s]
        y_dot     = u*(sps*ct) + v*(cps*cp + sp*st*sps) + w*(sps*st*cp - cps*sp)  # [m/s]
        z_dot     = -u*st + v*ct*sp + w*ct*cp                                      # [m/s]
        phi_dot   = p    # [rad/s]
        theta_dot = q    # [rad/s]
        psi_dot   = r    # [rad/s]

        return [*nu_dot, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, ndot, deltadot]

    def get_forces(self, state, delta_deg):
        """
        Compute and return all forces [N] and moments [N·m] at a given state.
        Returns: X [N], Y [N], K [N·m], N [N·m]  (heave Z and pitch M excluded)
        """
        u, v, w, p, q, r, x, y, z, phi, theta, psi, n_act, delta_act = state

        u_r = u - self.v_curr * np.cos(self.psi_curr - psi)
        v_r = v - self.v_curr * np.sin(self.psi_curr - psi)

        X_h = self.Xauu*abs(u_r)*u_r + self.Xvr*v_r*r
        Y_h = (self.Yauv*abs(u_r)*v_r + self.Yur*u_r*r + self.Yavv*abs(v_r)*v_r
               + (self.Yuuf + self.Yduu)*delta_act*u_r**2)
        K_h = (self.Kauv*abs(u_r)*v_r + self.Kur*u_r*r
               + self.Kavv*abs(v_r)*v_r + self.Kavr*abs(v_r)*r + self.Karv*r*abs(v_r)
               + self.Kauavf*abs(u_r)*abs(v_r)*delta_act + self.Kauarf*abs(u_r)*abs(r)*delta_act
               + self.Kuuf*u_r**2*delta_act + self.Kaup*abs(u_r)*p + self.Kapp*abs(p)*p
               + self.Kp*p + self.Kfff*(phi**3))
        N_h = (self.Nauv*abs(u_r)*v_r + self.Naur*abs(u_r)*r
               + self.Nauavf*abs(u_r)*abs(v_r)*delta_act + self.Nauuf*u_r**2*delta_act
               + self.Naruf*abs(u_r)*abs(r)*delta_act)
        thrust = self.TA_max * (abs(n_act)/self.nmax)**2 * np.sign(n_act)

        X_total = thrust + X_h
        Y_total = Y_h
        K_total = K_h - self.m*9.81*self.GM_T*np.sin(phi)
        N_total = N_h
        return X_total, Y_total, K_total, N_total