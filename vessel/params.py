import numpy as np
class VesselParams:
    def __init__(self):
        # ==========================================
        # VESSEL PARAMETERS (Appendix A, p. 152)
        # ==========================================
        self.m      = 360000.0   # [kg]        - Total ship mass (360 tonnes)
        self.rho_w  = 1025.0     # [kg/m3]     - Seawater density
        self.rho_a  = 1.225      # [kg/m3]     - Air density at sea level
        self.L      = 48.0       # [m]         - Ship length (bow to stern)
        self.B      = 8.6        # [m]         - Ship beam (port to starboard width)
        self.T      = 2.2        # [m]         - Draft (depth below waterline)
        self.A_w    = 105.6      # [m2]        - Waterplane area (footprint at waterline)

        # Moments of Inertia
        self.Ixx = 3.4e6         # [kg·m2]     - Roll moment of inertia (about x-axis)
        self.Iyy = 60.0e6        # [kg·m2]     - Pitch moment of inertia (about y-axis)
        self.Izz = 60.0e6        # [kg·m2]     - Yaw moment of inertia (about z-axis)

        # Center of Gravity (relative to midship at waterline)
        self.xG  = -3.38         # [m]         - CoG longitudinal (negative = aft of midship)
        self.yG  =  0.0          # [m]         - CoG lateral (0 = on centreline)
        self.zG  = -1.75         # [m]         - CoG vertical (negative = below waterline)

        # Metacentric Heights (Stability)
        self.GM_T = 0.776        # [m]         - Transverse metacentric height (roll stability)
        self.GM_L = 7.76         # [m]         - Longitudinal metacentric height (pitch stability)

        # ==========================================
        # HYDRODYNAMIC DERIVATIVES (Appendix A, p. 153)
        # ==========================================
        # Added Mass Coefficients (M_A)
        self.Xud  = -17400.0     # [kg]        - Added mass opposing surge acceleration
        self.Yvd  = -393000.0    # [kg]        - Added mass opposing sway acceleration
        self.Nvd  =  538000.0    # [kg·m]      - Yaw moment due to sway acceleration
        self.Yrd  = -1.4e6       # [kg·m]      - Sway force due to yaw acceleration
        self.Nrd  = -38.7e6      # [kg·m2]     - Added yaw inertia
        self.Ypd  = -0.296e6     # [kg·m]      - Sway force due to roll acceleration
        self.Kpd  = -0.774e6     # [kg·m2]     - Added roll inertia
        self.Krd  =  0.0         # [kg·m2]     - Roll moment due to yaw acceleration
        self.Npd  =  0.0         # [kg·m2]     - Yaw moment due to roll acceleration

        # Surge Force Coefficients (X) -> [N]
        self.Xauu = -1960.0      # [kg/m]      - Quadratic surge drag prop to |u|*u
        self.Xvr  =  0.33*self.m # [kg]        - Surge force from sway-yaw coupling v*r

        # Sway Force Coefficients (Y) -> [N]
        self.Yauv  = -11800.0    # [kg/m]      - Sway force prop to |u|*v
        self.Yur   =  131000.0   # [kg]        - Sway force prop to u*r
        self.Yavv  = -3700.0     # [kg/m]      - Nonlinear sway drag prop to |v|*v
        self.Yavr  = -0.794e6    # [kg·m]      - Sway force prop to |v|*r
        self.Yarv  = -0.182e6    # [kg·m]      - Sway force prop to r*|v|
        self.Yauavf =  10800.0   # [kg/m]      - Rudder + sway coupled prop to |u|*|v|*delta
        self.Yauarf =  0.251e6   # [kg]        - Rudder + yaw coupled prop to |u|*|r|*delta
        self.Yuuf  = -74.0       # [kg/m]      - Rudder sway at speed prop to u2*delta
        self.Yduu  =  7008.8     # [kg/m]      - Rudder effectiveness prop to delta*u2

        # Roll Moment Coefficients (K) -> [N·m]
        self.Kauv  =  9260.0              # [kg]        - Roll moment prop to |u|*v
        # Kur calibrated 14.0x: original -102000 appears non-dimensionalized in source data.
        # With Naruf added + linear restoring: Kur*14 gives phi_ss=19.8 deg, R=157m (thesis ~160m)
        self.Kur   = -102000.0 * 14.0    # [kg·m]      - Centrifugal heeling moment (calibrated)
        self.Kavv  =  29300.0            # [kg]        - Roll moment prop to |v|*v
        self.Kavr  =  0.621e6    # [kg·m2]     - Roll moment prop to |v|*r
        self.Karv  =  0.142e6    # [kg·m2]     - Roll moment prop to r*|v|
        self.Kauavf = -8400.0    # [kg]        - Roll moment prop to |u|*|v|*delta
        self.Kauarf = -0.196e6   # [kg·m]      - Roll moment prop to |u|*|r|*delta
        self.Kuuf  = -1180.0     # [kg]        - Roll moment prop to u2*delta
        self.Kaup  = -15500.0    # [kg]        - Roll moment prop to |u|*p
        self.Kapp  = -0.416e6    # [kg·m]      - Nonlinear roll damping prop to |p|*p
        # Kp calibrated to match thesis Fig 28 transient:
        # Kp=10x gives phi_min=-2.9 deg, phi_ss=19.4 deg, settle=15s (matches thesis)
        self.Kp    = -0.5e6 * 10  # [kg·m2/s]  - Linear roll damping (calibrated)
        self.Kfff  = -0.325 * self.rho_w * 9.81 * (self.m / self.rho_w)
                                 # [N·m]       - Nonlinear roll restoring prop to phi^3

        # Yaw Moment Coefficients (N) -> [N·m]
        self.Nauv  = -92000.0    # [kg]        - Yaw moment prop to |u|*v
        self.Naur  = -4.71e6     # [kg·m]      - Yaw damping at speed prop to |u|*r
        self.Narr  = -202e6      # [kg·m2]     - Nonlinear yaw drag prop to |r|*r
        self.Narv  = -15.6e6     # [kg·m]      - Yaw moment prop to |r|*v
        self.Nauavf = -0.214e6   # [kg]        - Yaw moment prop to |u|*|v|*delta (Appendix A)
        self.Nauuf = -8000.0     # [kg/m]      - Rudder yaw at speed prop to u2*delta
        self.Naruf = -4.98e6     # [kg·m]      - Rudder yaw moment (Naruf from Appendix A)

        # Environmental Force Coefficients
        self.Cw_X  = 0.4         # [dimensionless] - Wind drag coefficient, longitudinal
        self.Cw_Y  = 0.4         # [dimensionless] - Wind drag coefficient, lateral
        self.Cw_N  = 0.02        # [dimensionless] - Wind yaw moment coefficient
        self.A_x   = 25.0        # [m2]            - Frontal wind exposure area
        self.A_y   = 120.0       # [m2]            - Lateral wind exposure area

        # Propulsion & Control Limits  (Appendix A + calibrated from thesis p.122)
        # Thrust law: T = TA_max * (n/nmax)^2  — quadratic (propeller law)
        # TA_max calibrated so full ahead (160 RPM) gives 18 m/s straight course (thesis p.122)
        # "Half ahead" in thesis = ~90 RPM giving ~10 m/s (matches Figs 28-31)
        self.TA_max      = 635000.0  # [N]     - Max thrust at full ahead (quadratic scaling)
        self.nmax        = 160.0     # [RPM]   - Max propeller shaft speed
        self.deltamax    = 45.0      # [deg]   - Rudder angle hard limit (Appendix A)
        self.deltadotmax = 20.0      # [deg/s] - Rudder rate limit (Appendix A)

        # Environmental Input Parameters (set to zero for calm water)
        self.v_curr      = 0.0            # [m/s]   - Ocean current speed
        self.psi_curr    = 0.0            # [rad]   - Current direction
        self.v_wind      = 0.0            # [m/s]   - Wind speed
        self.psi_wind    = np.deg2rad(90) # [rad]   - Wind direction
        self.X_wave_amp  = 0.0            # [N]     - Wave surge force amplitude
        self.Y_wave_amp  = 0.0            # [N]     - Wave sway force amplitude