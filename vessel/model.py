# vessel/model.py
import numpy as np
from vessel.params import VesselParams

class ShipModel:
    def __init__(self, params):
        self.m = params

    def get_mass_matrix(self):
        p = self.m                          # BUG FIX 1: was `m = self.m`, which shadowed
        mass = p.m                          # ship mass `m` with the params object.
        xG, yG, zG = p.xG, p.yG, p.zG

        # Rigid body mass matrix [6x6]
        M_RB = np.array([
            [mass,     0,       0,        0,          mass*zG,  -mass*yG ],
            [0,        mass,    0,       -mass*zG,    0,         mass*xG ],
            [0,        0,       mass,     mass*yG,   -mass*xG,  0        ],
            [0,       -mass*zG, mass*yG,  p.Ixx,     0,         0        ],
            [mass*zG,  0,      -mass*xG,  0,          p.Iyy,    0        ],
            [-mass*yG, mass*xG, 0,        0,          0,         p.Izz   ]
        ])

        # Added mass matrix [6x6]
        M_A = np.zeros((6, 6))
        M_A[0, 0] = -p.Xud
        M_A[1, 1] = -p.Yvd
        M_A[1, 3] = -p.Ypd
        M_A[1, 5] = -p.Yrd
        M_A[3, 1] = -p.Ypd
        M_A[3, 3] = -p.Kpd
        M_A[3, 5] = -p.Krd
        M_A[5, 1] = -p.Nvd
        M_A[5, 3] = -p.Npd
        M_A[5, 5] = -p.Nrd

        return M_RB + M_A

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
        """
        # BUG FIX 2: All self.xxx -> self.m.xxx (params live in self.m, not self)
        p = self.m

        u, v, w, roll_rate, q, r, x, y, z, phi, theta, psi, n_act, delta_act = state

        # Environmental Disturbances
        u_c = p.v_curr * np.cos(p.psi_curr - psi)
        v_c = p.v_curr * np.sin(p.psi_curr - psi)
        u_r = u - u_c
        v_r = v - v_c

        u_rw = p.v_wind * np.cos(p.psi_wind - psi) - u
        v_rw = p.v_wind * np.sin(p.psi_wind - psi) - v
        V_rw = np.sqrt(u_rw**2 + v_rw**2)
        X_wind = 0.5 * p.rho_a * V_rw**2 * p.Cw_X * p.A_x
        Y_wind = 0.5 * p.rho_a * V_rw**2 * p.Cw_Y * p.A_y
        N_wind = 0.5 * p.rho_a * V_rw**2 * p.Cw_N * p.A_y * p.L

        X_wave = p.X_wave_amp * np.sin(0.5 * t)
        Y_wave = p.Y_wave_amp * np.cos(0.5 * t)

        X_env = X_wind + X_wave
        Y_env = Y_wind + Y_wave
        N_env = N_wind

        X_h = (p.Xauu * abs(u_r) * u_r
             + p.Xvr  * v_r * r)

        Y_h = (p.Yauv  * abs(u_r) * v_r
             + p.Yur   * u_r * r
             + p.Yavv  * abs(v_r) * v_r
             + (p.Yuuf + p.Yduu) * delta_act * u_r**2)

        K_h = (p.Kauv  * abs(u_r) * v_r
             + p.Kur   * u_r * r
             + p.Kavv  * abs(v_r) * v_r
             + p.Kavr  * abs(v_r) * r
             + p.Karv  * r * abs(v_r)
             + p.Kauavf * abs(u_r) * abs(v_r) * delta_act
             + p.Kauarf * abs(u_r) * abs(r)   * delta_act
             + p.Kuuf  * u_r**2 * delta_act
             + p.Kaup  * abs(u_r) * roll_rate
             + p.Kapp  * abs(roll_rate) * roll_rate
             + p.Kp    * roll_rate
             + p.Kfff  * (phi**3))

        N_h = (p.Nauv  * abs(u_r) * v_r
             + p.Naur  * abs(u_r) * r
             + p.Nauavf * abs(u_r) * abs(v_r) * delta_act
             + p.Nauuf * u_r**2 * delta_act
             + p.Naruf * abs(u_r) * abs(r) * delta_act)

        # Propeller Thrust
        thrust = p.TA_max * (abs(n_act) / p.nmax)**2 * np.sign(n_act)

        tau = np.array([
            thrust + X_h + X_env,
            Y_h    + Y_env,
            0,
            K_h,
            0,
            N_h    + N_env
        ])

        # Restoring Forces (subtracted — opposes displacement)
        g_eta = np.array([
            0,
            0,
            0,
            p.m * 9.81 * p.GM_T * np.sin(phi),
            p.m * 9.81 * p.GM_L * np.sin(theta),
            0
        ])

        # Equations of Motion
        M      = self.get_mass_matrix()
        nu_dot = np.linalg.inv(M) @ (tau - g_eta)

        # Shaft dynamics
        n_cmd_sat = np.clip(n_cmd, -p.nmax, p.nmax)
        Tm   = 18.83 if abs(n_act) < 20.0 else 5.65 / max(abs(n_act), 0.01)
        ndot = (n_cmd_sat - n_act) / Tm

        # Rudder dynamics
        delta_cmd_sat = np.clip(np.deg2rad(delta_deg),
                                -np.deg2rad(p.deltamax),
                                 np.deg2rad(p.deltamax))
        Tc           = 1.0
        deltadot_raw = (delta_cmd_sat - delta_act) / Tc
        deltadot     = np.clip(deltadot_raw,
                               -np.deg2rad(p.deltadotmax),
                                np.deg2rad(p.deltadotmax))

        # Kinematics: Body -> NED
        cp  = np.cos(phi);   sp  = np.sin(phi)
        ct  = np.cos(theta); st  = np.sin(theta)
        cps = np.cos(psi);   sps = np.sin(psi)

        x_dot     = u*(cps*ct) + v*(cps*st*sp - sps*cp) + w*(sps*sp + cps*cp*st)
        y_dot     = u*(sps*ct) + v*(cps*cp + sp*st*sps) + w*(sps*st*cp - cps*sp)
        z_dot     = -u*st + v*ct*sp + w*ct*cp
        phi_dot   = roll_rate
        theta_dot = q
        psi_dot   = r

        return [*nu_dot, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, ndot, deltadot]

    def get_forces(self, state, delta_deg):
        """
        Compute and return all forces [N] and moments [N·m] at a given state.
        Returns: X [N], Y [N], K [N·m], N [N·m]
        """
        # BUG FIX 4: same self.xxx -> self.m.xxx fix
        p = self.m

        u, v, w, roll_rate, q, r, x, y, z, phi, theta, psi, n_act, delta_act = state

        u_r = u - p.v_curr * np.cos(p.psi_curr - psi)
        v_r = v - p.v_curr * np.sin(p.psi_curr - psi)

        X_h = p.Xauu*abs(u_r)*u_r + p.Xvr*v_r*r
        Y_h = (p.Yauv*abs(u_r)*v_r + p.Yur*u_r*r + p.Yavv*abs(v_r)*v_r
               + (p.Yuuf + p.Yduu)*delta_act*u_r**2)
        K_h = (p.Kauv*abs(u_r)*v_r + p.Kur*u_r*r
               + p.Kavv*abs(v_r)*v_r + p.Kavr*abs(v_r)*r + p.Karv*r*abs(v_r)
               + p.Kauavf*abs(u_r)*abs(v_r)*delta_act + p.Kauarf*abs(u_r)*abs(r)*delta_act
               + p.Kuuf*u_r**2*delta_act + p.Kaup*abs(u_r)*roll_rate + p.Kapp*abs(roll_rate)*roll_rate
               + p.Kp*roll_rate + p.Kfff*(phi**3))
        N_h = (p.Nauv*abs(u_r)*v_r + p.Naur*abs(u_r)*r
               + p.Nauavf*abs(u_r)*abs(v_r)*delta_act + p.Nauuf*u_r**2*delta_act
               + p.Naruf*abs(u_r)*abs(r)*delta_act)
        thrust = p.TA_max * (abs(n_act)/p.nmax)**2 * np.sign(n_act)

        X_total = thrust + X_h
        Y_total = Y_h
        K_total = K_h - p.m*9.81*p.GM_T*np.sin(phi)
        N_total = N_h
        return X_total, Y_total, K_total, N_total