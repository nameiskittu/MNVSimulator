# utils/plotters.py

import numpy as np
import matplotlib.pyplot as plt


# ==========================================
# STRAIGHT LINE PLOTS
# ==========================================
def plot_straight_line(sol):
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle("Straight Line — State Variables", fontsize=15)

    t = sol.t
    y = sol.y

    axes[0,0].plot(t, y[6]); axes[0,0].set_title("x [m]")
    axes[0,1].plot(t, y[7]); axes[0,1].set_title("y [m]")
    axes[0,2].plot(t, y[8]); axes[0,2].set_title("z [m]")

    axes[1,0].plot(t, y[0]); axes[1,0].set_title("u [m/s]")
    axes[1,1].plot(t, y[1]); axes[1,1].set_title("v [m/s]")
    axes[1,2].plot(t, y[2]); axes[1,2].set_title("w [m/s]")

    axes[2,0].plot(t, np.rad2deg(y[11])); axes[2,0].set_title("psi [deg]")
    axes[2,1].plot(t, np.rad2deg(y[9]));  axes[2,1].set_title("phi [deg]")
    axes[2,2].plot(t, np.rad2deg(y[10])); axes[2,2].set_title("theta [deg]")

    for ax in axes.flat:
        ax.grid(True)
        ax.set_xlabel("t [s]")

    plt.tight_layout()
    return fig


def plot_trajectory(x, y):
    fig = plt.figure(figsize=(8, 6))
    plt.plot(y, x, 'b-', linewidth=2)
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    plt.title("Trajectory")
    plt.grid(True)
    plt.axis('equal')
    return fig


# ==========================================
# TURNING CIRCLE PLOTS
# ==========================================
def plot_turning_circle(results):
    t = results["t"]
    y = results["y"]
    t_rudder = results["t_rudder"]

    phi = np.rad2deg(y[9])
    r   = np.rad2deg(y[5])

    u, v, w = y[0], y[1], y[2]
    V = np.sqrt(u**2 + v**2 + w**2)

    x = y[6]
    y_pos = y[7]

    fig = plt.figure(figsize=(16, 9))
    fig.suptitle("Turning Circle", fontsize=14)

    # Trajectory
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.plot(x, y_pos, 'b-', linewidth=2)
    ax1.set_title("Trajectory")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.axis('equal')
    ax1.grid(True)

    # Roll
    ax2 = fig.add_subplot(3, 2, 2)
    ax2.plot(t, phi)
    ax2.axvline(t_rudder, linestyle='--')
    ax2.set_ylabel("phi [deg]")
    ax2.grid(True)

    # Yaw rate
    ax3 = fig.add_subplot(3, 2, 4)
    ax3.plot(t, r)
    ax3.axvline(t_rudder, linestyle='--')
    ax3.set_ylabel("r [deg/s]")
    ax3.grid(True)

    # Velocity
    ax4 = fig.add_subplot(3, 2, 6)
    ax4.plot(t, V)
    ax4.axvline(t_rudder, linestyle='--')
    ax4.set_ylabel("V [m/s]")
    ax4.set_xlabel("t [s]")
    ax4.grid(True)

    plt.tight_layout()
    return fig


# ==========================================
# ZIGZAG PLOTS
# ==========================================
def plot_zigzag_states(t, y, delta_zz):
    fig, axes = plt.subplots(3, 3, figsize=(16, 11))
    fig.suptitle(f"{delta_zz}/{delta_zz} ZigZag — States", fontsize=14)

    axes[0,0].plot(t, y[0]); axes[0,0].set_title("u")
    axes[0,1].plot(t, y[1]); axes[0,1].set_title("v")
    axes[0,2].plot(t, np.rad2deg(y[3])); axes[0,2].set_title("p")

    axes[1,0].plot(t, np.rad2deg(y[4])); axes[1,0].set_title("q")
    axes[1,1].plot(t, np.rad2deg(y[5])); axes[1,1].set_title("r")
    axes[1,2].set_visible(False)

    axes[2,0].plot(t, np.rad2deg(y[9])); axes[2,0].set_title("phi")
    axes[2,1].plot(t, np.rad2deg(y[10])); axes[2,1].set_title("theta")

    axes[2,2].plot(t, np.rad2deg(y[11]))
    axes[2,2].axhline(delta_zz, linestyle='--')
    axes[2,2].axhline(-delta_zz, linestyle='--')
    axes[2,2].set_title("psi")

    for ax in axes.flat:
        ax.grid(True)
        ax.set_xlabel("t [s]")

    plt.tight_layout()
    return fig


def plot_zigzag_forces(t, X, Y, K, N):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle("ZigZag — Forces & Moments", fontsize=14)

    axes[0,0].plot(t, X/1000); axes[0,0].set_title("X [kN]")
    axes[0,1].plot(t, Y/1000); axes[0,1].set_title("Y [kN]")
    axes[1,0].plot(t, K/1000); axes[1,0].set_title("K [kN·m]")
    axes[1,1].plot(t, N/1000); axes[1,1].set_title("N [kN·m]")

    for ax in axes.flat:
        ax.grid(True)
        ax.set_xlabel("t [s]")

    plt.tight_layout()
    return fig


def plot_zigzag_standard(t, y, delta_cmd, delta_zz):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("ZigZag — ITTC Standard Plot", fontsize=14)

    psi = np.rad2deg(y[11])

    axes[0].plot(t, psi, label="psi")
    axes[0].axhline(delta_zz, linestyle='--')
    axes[0].axhline(-delta_zz, linestyle='--')
    axes[0].set_ylabel("psi [deg]")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, delta_cmd, label="delta", color='orange')
    axes[1].set_ylabel("delta [deg]")
    axes[1].set_xlabel("t [s]")
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    return fig


def plot_zigzag_trajectory(y):
    fig = plt.figure(figsize=(8, 6))

    plt.plot(y[7], y[6], 'b-')
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    plt.title("ZigZag Trajectory")
    plt.axis('equal')
    plt.grid(True)

    return fig