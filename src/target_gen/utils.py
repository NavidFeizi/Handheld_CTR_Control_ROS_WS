import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import LineString

def figsize(scale, nplots=1):
    fig_width_pt = 390.0  # Get this from LaTeX using \the\textwidth
    inches_per_pt = 1.0 / 72.27  # Convert pt to inch
    golden_mean = ((5.0) ** 0.5 - 1.0) / 2.0  # Aesthetic ratio (you could change this)
    fig_width = fig_width_pt * inches_per_pt * scale  # width in inches
    fig_height = nplots * fig_width * golden_mean  # height in inches
    fig_size = [fig_width, fig_height]
    return fig_size


pgf_with_latex = {  # setup matplotlib to use latex for output
    "pgf.texsystem": "pdflatex",  # change this if using xetex or lautex
    "text.usetex": True,  # use LaTeX to write all text
    "font.family": "serif",
    "font.serif": [],  # blank entries should cause plots to inherit fonts from the document
    "font.sans-serif": [],
    "font.monospace": [],
    "axes.labelsize": 9,  # LaTeX default is 10pt font.
    "font.size": 9,
    "legend.fontsize": 9,  # Make the legend/label fonts a little smaller
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "grid.color": "lightgray",
    "grid.linewidth": 0.5,
    "figure.figsize": figsize(1.0),  # default fig size of 0.9 textwidth
    "pgf.preamble": r"\usepackage[utf8x]{inputenc}, \usepackage[T1]{fontenc}",
}



def circular_ref_trajectory(t, h, dt, vary_in_horizon: bool = True) -> np.ndarray:
    """
    Return stacked refs of shape (N*m,), here: hold current position
    or make a simple target (e.g., +2 cm in z over the horizon).
    """
    # Circle in x-y plane centered at current position (x0, y0, z0)
    x0, y0, z0 = 0.00, -0.00, 0.125

    r = 0.022  # radius [m]
    A_r = 0.005
    A_z = 0.01
    omega = 0.25 * 0.8  # angular speed [rad/s]
    omega_r = 1.5 * 0.8  # angular speed in z [rad/s]
    omega_z = 1.1 * 0.8  # angular speed in z [rad/s]

    theta0 = omega * t
    theta0_r = omega_r * t
    theta0_z = omega_z * t
    refs = []

    if vary_in_horizon:
        for k in range(1, h + 1):
            th = theta0 + k * omega * dt
            theta = -1.25 * np.pi + 0.5 * (1 + np.sin(-np.pi / 2 + th)) * np.pi * 1.5
            th_r = theta0_r + k * omega_r * dt
            th_z = theta0_z + k * omega_z * dt

            refs.append(
                np.array(
                    [
                        x0 + (r + A_r * np.sin(th_r)) * np.cos(theta),
                        y0 + (r + A_r * np.sin(th_r)) * np.sin(theta),
                        z0 + A_z * -1 * np.sin(th_z),
                    ],
                    dtype=float,
                )
            )
    else:
        for k in range(1, h + 1):
            th = theta0
            theta = -1.25 * np.pi + 0.5 * (1 + np.sin(-np.pi / 2 + th)) * np.pi * 1.5
            th_r = theta0_r
            th_z = theta0_z
            ref = np.array(
                [
                    x0 + (r + A_r * np.sin(th_r)) * np.cos(theta),
                    y0 + (r + A_r * np.sin(th_r)) * np.sin(theta),
                    z0 + A_z * -1 * np.sin(th_z),
                ],
                dtype=float,
            )

            refs = np.tile(ref, (h, 1))

    return np.concatenate(refs, axis=0)  # (N*m,)

def infinity_ref_trajectory(t, h, dt, vary_in_horizon: bool = True) -> np.ndarray:
    """
    Return stacked refs of shape (h*3,) following an infinity (figure-eight) trajectory
    in the x-y plane, with optional small modulation in z.

    Parameters
    ----------
    t : float
        Current time [s].
    h : int
        Horizon length (number of steps).
    dt : float
        Time step [s].
    vary_in_horizon : bool
        If True, the reference moves along the infinity curve over the horizon.
        If False, the same target (at time t) is repeated over the horizon.

    Returns
    -------
    refs_flat : np.ndarray
        Stacked reference positions of shape (h*3,) = [x0, y0, z0, x1, y1, z1, ...].
    """
    # Center position
    x0, y0, z0 = 0.000, -0.014, 0.125
    t = t + 0.5  # phase shift to start at a specific point on the curve

    # Infinity shape (lemniscate of Gerono) parameters
    A_xy = 0.025     # size of the infinity in x-y [m]
    A_z  = -0.012     # small oscillation in z [m] (can be set to 0.0 to keep z constant)
    A_z  = 0.0
    omega_xy = 0.2   # angular speed for x-y [rad/s]
    omega_z  = 0.4   # angular speed for z [rad/s]

    theta0_xy = omega_xy * t
    theta0_z  = omega_z * t

    refs = []

    if vary_in_horizon:
        # Move along the infinity curve across the horizon
        for k in range(1, h + 1):
            th_xy = theta0_xy + k * omega_xy * dt
            th_z  = theta0_z  + k * omega_z  * dt

            # Lemniscate of Gerono in x-y:
            # x = A * sin(θ)
            # y = A * sin(θ) * cos(θ)  (i.e., 0.5 * A * sin(2θ))
            x = x0 + A_xy * np.sin(th_xy)
            y = y0 + A_xy * np.sin(th_xy) * np.cos(th_xy)
            z = z0 + A_z  * np.sin(th_z)

            refs.append(np.array([x, y, z], dtype=float))
    else:
        # Hold the *same* target (at current time t) over the whole horizon
        th_xy = theta0_xy
        th_z  = theta0_z

        x = x0 + A_xy * np.sin(th_xy)
        y = y0 + A_xy * np.sin(th_xy) * np.cos(th_xy)
        z = z0 + A_z  * np.sin(th_z)

        ref = np.array([x, y, z], dtype=float)
        refs = np.tile(ref, (h, 1))

    return np.concatenate(refs, axis=0)  # (h*3,)

def H_ref_trajectory(t, h, dt, vary_in_horizon: bool = True) -> np.ndarray:
    """
    Return stacked refs of shape (h*3,) following the *outer edge* of a thick 'H'
    in the x-y plane as a single closed loop (start and end at same corner, no pen lift).

    Parameters
    ----------
    t : float
        Current time [s].
    h : int
        Horizon length (number of steps).
    dt : float
        Time step [s].
    vary_in_horizon : bool
        If True, the reference advances along the H over the horizon.
        If False, the same target (at current time t) is repeated over the horizon.

    Returns
    -------
    refs_flat : np.ndarray
        Stacked reference positions of shape (h*3,) = [x0,y0,z0, x1,y1,z1, ...].
    """

    # Center and base z
    x0, y0, z0 = 0.00, -0.010, 0.130

    # H geometry (all in meters)
    H_height = 0.040       # total H height
    H_width  = 0.035       # distance between centers of the two vertical stems
    stroke   = 0.005       # thickness of stems and bar

    # How many full H loops per second
    omega = 1.0/30.0           # 0.1 ⇒ one loop every 10 seconds

    # Precompute key coordinates
    y_bot = y0 - H_height / 2.0
    y_top = y0 + H_height / 2.0
    y_bar_bot = y0 - stroke / 2.0
    y_bar_top = y0 + stroke / 2.0

    # Outer/inner x-positions of stems
    xL_center = x0 - H_width / 2.0
    xR_center = x0 + H_width / 2.0

    xL_outer = xL_center - stroke / 2.0
    xL_inner = xL_center + stroke / 2.0
    xR_inner = xR_center - stroke / 2.0
    xR_outer = xR_center + stroke / 2.0

    # Define polygon for the outer edge of the thick H (closed loop).
    # Order: go around counter-clockwise starting from bottom-left outer corner.
    vertices = np.array([
        [xL_outer, y_bot],       # 0: bottom-left outer (start)
        [xL_outer, y_top],       # 1: up outer left
        [xL_inner, y_top],       # 2: right along top of left stem
        [xL_inner, y_bar_top],   # 3: down inner-left to top of bar
        [xR_inner, y_bar_top],   # 4: across bar top
        [xR_inner, y_top],       # 5: up inner-right to top
        [xR_outer, y_top],       # 6: out to outer right top
        [xR_outer, y_bot],       # 7: down outer right
        [xR_inner, y_bot],       # 8: left along bottom of right stem
        [xR_inner, y_bar_bot],   # 9: up inner-right to bottom of bar
        [xL_inner, y_bar_bot],   # 10: across bar bottom
        [xL_inner, y_bot],       # 11: down inner-left to bottom
        [xL_outer, y_bot],       # 12: back to start (close loop)
    ], dtype=float)

    # Precompute segment lengths along the polygon
    diffs = vertices[1:] - vertices[:-1]          # (num_segments, 2)
    seg_lengths = np.linalg.norm(diffs, axis=1)   # (num_segments,)
    total_length = np.sum(seg_lengths)
    cum_lengths = np.concatenate(([0.0], np.cumsum(seg_lengths)))  # (num_segments+1,)

    def point_on_H(phi: float) -> np.ndarray:
        """
        Map a normalized phase phi in [0,1) to a point on the H edge loop,
        moving with approximately constant speed along the edges.
        """
        # distance along the loop
        phi = phi % 1.0
        d = phi * total_length

        # find which segment contains distance d
        # index i such that cum_lengths[i] <= d < cum_lengths[i+1]
        i = np.searchsorted(cum_lengths, d, side="right") - 1
        i = min(max(i, 0), len(seg_lengths) - 1)  # clamp just in case

        seg_start = vertices[i]
        seg_end = vertices[i + 1]
        seg_len = seg_lengths[i]

        if seg_len <= 1e-9:
            # degenerate segment, just return its start
            xy = seg_start
        else:
            u = (d - cum_lengths[i]) / seg_len  # in [0,1]
            xy = seg_start + u * (seg_end - seg_start)

        x, y = xy
        z = z0
        return np.array([x, y, z], dtype=float)

    refs = []

    if vary_in_horizon:
        # move along the H loop over the horizon
        for k in range(1, h + 1):
            t_k = t + k * dt
            phi_k = (omega * t_k) % 1.0
            refs.append(point_on_H(phi_k))
    else:
        # hold the same point on the H loop for the whole horizon
        phi = (omega * t) % 1.0
        ref = point_on_H(phi)
        refs = np.tile(ref, (h, 1))

    return np.concatenate(refs, axis=0)  # (h*3,)

def W_ref_trajectory(
    t: float,
    h: int,
    dt: float,
    vary_in_horizon: bool = True,
    width: float = 0.050,
    outer_height: float = 0.040,
    inner_height: float = 0.015,
    inner_offset_down: float = 0.010,
    stroke: float = 0.005,
) -> np.ndarray:
    """
    Return stacked refs of shape (h*3,) following the *outer edge* of a thick, symmetric 'W'
    in the x-y plane as a single closed loop.

    The W is symmetric about x = x0. The outer legs are taller; the inner legs are shorter
    and shifted downward so the trajectory stays away from (0,0).

    Parameters
    ----------
    t : float
        Current time [s].
    h : int
        Horizon length (number of steps).
    dt : float
        Time step [s].
    vary_in_horizon : bool
        If True, the reference advances along the W over the horizon.
        If False, the same target (at current time t) is repeated over the horizon.
    width : float
        Total horizontal width of the W (distance from left to right outer tips).
    outer_height : float
        Vertical extent of the two outer legs.
    inner_height : float
        Vertical extent of the two inner legs.
    inner_offset_down : float
        Vertical shift applied to the inner legs (positive moves them downward).
    stroke : float
        Thickness of the W stroke (similar to H's stroke).

    Returns
    -------
    refs_flat : np.ndarray
        Stacked reference positions of shape (h*3,) = [x0,y0,z0, x1,y1,z1, ...].
    """

    # Center and base z
    x0, y0, z0 = 0.00, -0.010, 0.132

    # One full loop every 30 seconds (like H)
    omega = 1.0 / 30.0

    # ---- Symmetric W centerline (5 points, 4 segments) ----
    # x positions: equally spaced so the W is symmetric
    x_left   = x0 - width / 2.0
    x_lmid   = x0 - width / 4.0
    x_center = x0
    x_rmid   = x0 + width / 4.0
    x_right  = x0 + width / 2.0

    # Outer legs centered at y0
    outer_center = y0
    y_outer_top = outer_center + outer_height / 2.0
    y_outer_bot = outer_center - outer_height / 2.0

    # Inner legs centered *below* y0 by inner_offset_down
    inner_center = y0 - inner_offset_down
    y_inner_top = inner_center + inner_height / 2.0
    y_inner_bot = inner_center - inner_height / 2.0

    # W centerline points (P0..P4), symmetric around x0:
    # P0->P1 outer down, P1->P2 inner up, P2->P3 inner down, P3->P4 outer up.
    centerline_pts = np.array([
        [x_left,   y_outer_top],  # P0: left outer top
        [x_lmid,   y_inner_bot],  # P1: left inner bottom
        [x_center, y_inner_top],  # P2: inner top
        [x_rmid,   y_inner_bot],  # P3: right inner bottom
        [x_right,  y_outer_top],  # P4: right outer top
    ], dtype=float)

    # ---- Thicken the W using a buffer around the centerline ----
    line = LineString(centerline_pts)
    # buffer radius = stroke/2, flat caps & joins to look like a thick line
    poly = line.buffer(stroke / 2.0, cap_style=2, join_style=2)

    # Outer boundary of the thick W (closed ring; last point == first point)
    vertices = np.array(poly.exterior.coords)[:, :2]  # (M, 2)

    # ---- Precompute segment lengths along this polygon ----
    diffs = vertices[1:] - vertices[:-1]        # (num_segments, 2)
    seg_lengths = np.linalg.norm(diffs, axis=1) # (num_segments,)
    total_length = float(np.sum(seg_lengths))
    cum_lengths = np.concatenate(([0.0], np.cumsum(seg_lengths)))  # (num_segments+1,)

    def point_on_W(phi: float) -> np.ndarray:
        """
        Map a normalized phase phi in [0,1) to a point on the W edge loop
        with approximately constant speed.
        """
        phi = phi % 1.0
        d = phi * total_length

        # Find segment index
        i = np.searchsorted(cum_lengths, d, side="right") - 1
        i = min(max(i, 0), len(seg_lengths) - 1)

        seg_start = vertices[i]
        seg_end   = vertices[i + 1]
        seg_len   = seg_lengths[i]

        if seg_len <= 1e-9:
            xy = seg_start
        else:
            u = (d - cum_lengths[i]) / seg_len  # in [0,1]
            xy = seg_start + u * (seg_end - seg_start)

        x, y = xy
        return np.array([x, y, z0], dtype=float)

    # ---- Build horizon references ----
    if vary_in_horizon:
        refs = []
        for k in range(1, h + 1):
            t_k = t + k * dt
            phi_k = (omega * t_k) % 1.0
            refs.append(point_on_W(phi_k))
        refs = np.vstack(refs)
    else:
        phi = (omega * t) % 1.0
        ref = point_on_W(phi)
        refs = np.tile(ref, (h, 1))

    return refs.reshape(-1)  # (h*3,)


def y_ref_trajectory_for_grassmann(t, h, dt, vary_in_horizon: bool = True) -> np.ndarray:
    """
    Return stacked refs of shape (N*m,), here: hold current position
    or make a simple target (e.g., +2 cm in z over the horizon).
    """
    # Circle in x-y plane centered at current position (x0, y0, z0)
    x0, y0, z0 = 0.00, -0.00, 0.130

    r = 0.040  # radius [m]
    A_r = 0.020
    A_z = 0.020
    omega = 0.25  # angular speed [rad/s]
    omega_r = 2.0  # angular speed in z [rad/s]
    omega_z = 1.5  # angular speed in z [rad/s]

    theta0 = omega * t
    theta0_r = omega_r * t
    theta0_z = omega_z * t
    refs = []

    if vary_in_horizon:
        for k in range(1, h + 1):
            th = theta0 + k * omega * dt
            theta = -1.25 * np.pi + 0.5 * (1 + np.sin(-np.pi / 2 + th)) * np.pi * 1.5
            th_r = theta0_r + k * omega_r * dt
            th_z = theta0_z + k * omega_z * dt

            refs.append(
                np.array(
                    [
                        x0 + (r + A_r * np.sin(th_r)) * np.cos(theta),
                        y0 + (r + A_r * np.sin(th_r)) * np.sin(theta),
                        z0 + A_z * -1 * np.sin(th_z),
                    ],
                    dtype=float,
                )
            )
    else:
        for k in range(1, h + 1):
            th = theta0
            theta = -1.25 * np.pi + 0.5 * (1 + np.sin(-np.pi / 2 + th)) * np.pi * 1.5
            th_r = theta0_r
            th_z = theta0_z
            ref = np.array(
                [
                    x0 + (r + A_r * np.sin(th_r)) * np.cos(theta),
                    y0 + (r + A_r * np.sin(th_r)) * np.sin(theta),
                    z0 + A_z * -1 * np.sin(th_z),
                ],
                dtype=float,
            )

            refs = np.tile(ref, (h, 1))

    return np.concatenate(refs, axis=0)  # (N*m,)



def plot_results_4D(data_store, save_dir, H_info_store):
    fig = plt.figure(figsize=(12, 7))
    gs = GridSpec(
        8, 2, figure=fig, width_ratios=[3, 1], height_ratios=[1, 1, 1, 1, 1, 1, 1, 1]
    )
    axs = []
    axs.append(fig.add_subplot(gs[0, 0]))
    axs.append(fig.add_subplot(gs[1, 0]))
    axs.append(fig.add_subplot(gs[2, 0]))
    axs.append(fig.add_subplot(gs[3, 0]))
    axs.append(fig.add_subplot(gs[4, 0]))
    axs.append(fig.add_subplot(gs[5, 0]))
    axs.append(fig.add_subplot(gs[6, 0]))
    axs.append(fig.add_subplot(gs[7, 0]))
    axs.append(fig.add_subplot(gs[0:4, 1]))
    axs.append(fig.add_subplot(gs[4:6, 1]))
    axs.append(fig.add_subplot(gs[6:8, 1]))
    
    ## X positions
    axs[0].plot(
        data_store["t"],
        data_store["x_r"],
        c="black",
        label="x_ref",
        linestyle="--",
        linewidth=2.0,
    )
    axs[0].plot(data_store["t"], data_store["x1"], c="orange", label="x", linewidth=1.0)
    ## Y positions
    axs[0].plot(
        data_store["t"],
        data_store["y_r"],
        c="blue",
        label="y_ref",
        linestyle="--",
        linewidth=2.0,
    )
    axs[0].plot(data_store["t"], data_store["y1"], c="red", label="y", linewidth=1.0)
    ## Z positions
    axs[1].plot(
        data_store["t"],
        data_store["z_r"],
        c="black",
        label="z_ref",
        linestyle="--",
        linewidth=2.0,
    )
    axs[1].plot(data_store["t"], data_store["z1"], c="orange", label="z", linewidth=1.0)

    ## Joints positions
    if "q1_min" in data_store.columns and "q1_max" in data_store.columns:
        axs[2].fill_between(
            data_store["t"],
            (data_store["q2_min"]),
            (data_store["q2_max"]),
            color="red",
            alpha=0.2,
        )
        axs[2].fill_between(
            data_store["t"],
            (data_store["q1_min"] + data_store["beta2"]),
            (data_store["q1_max"] + data_store["beta2"]),
            color="black",
            alpha=0.2,
        )

    axs[2].plot(
        data_store["t"], data_store["beta1"], c="black", label="beta1", linewidth=1.0
    )
    axs[2].plot(
        data_store["t"], data_store["beta2"], c="red", label="beta2", linewidth=1.0
    )

    if "q3_min" in data_store.columns and "q3_max" in data_store.columns:
        axs[3].fill_between(
            data_store["t"],
            (data_store["q4_min"]),
            (data_store["q4_max"]),
            color="red",
            alpha=0.2,
        )
        axs[3].fill_between(
            data_store["t"],
            (data_store["q3_min"] + data_store["alpha2"]),
            (data_store["q3_max"] + data_store["alpha2"]),
            color="black",
            alpha=0.2,
        )

    axs[3].plot(
        data_store["t"], data_store["alpha1"], c="black", label="alpha1", linewidth=2.0
    )
    axs[3].plot(
        data_store["t"], data_store["alpha2"], c="red", label="alpha2", linewidth=2.0
    )

    ## Joints velocities
    if "u1_min" in data_store.columns and "u1_max" in data_store.columns:
        axs[4].plot(
            data_store["t"],
            data_store["u1_min"],
            c="black",
            linestyle="--",
            linewidth=2.0,
        )
        axs[4].plot(
            data_store["t"],
            data_store["u1_max"],
            c="black",
            linestyle="--",
            linewidth=2.0,
        )
        axs[4].plot(
            data_store["t"],
            data_store["u2_min"],
            c="red",
            linestyle="--",
            linewidth=2.0,
        )
        axs[4].plot(
            data_store["t"],
            data_store["u2_max"],
            c="red",
            linestyle="--",
            linewidth=2.0,
        )
    axs[4].plot(
        data_store["t"], data_store["beta1_dot"], c="black", label="u1", linewidth=1.0
    )
    axs[4].plot(
        data_store["t"], data_store["beta2_dot"], c="red", label="u2", linewidth=1.0
    )

    if "u1_min" in data_store.columns and "u1_max" in data_store.columns:
        axs[5].plot(
            data_store["t"],
            data_store["u3_min"],
            c="black",
            linestyle="--",
            linewidth=1.0,
        )
        axs[5].plot(
            data_store["t"],
            data_store["u3_max"],
            c="black",
            linestyle="--",
            linewidth=1.0,
        )
        axs[5].plot(
            data_store["t"],
            data_store["u4_min"],
            c="red",
            linestyle="--",
            linewidth=1.0,
        )
        axs[5].plot(
            data_store["t"],
            data_store["u4_max"],
            c="red",
            linestyle="--",
            linewidth=1.0,
        )
    axs[5].plot(
        data_store["t"], data_store["alpha1_dot"], c="black", label="u4", linewidth=1.0
    )
    axs[5].plot(
        data_store["t"], data_store["alpha2_dot"], c="red", label="u5", linewidth=1.0
    )

    axs[6].fill_between(
        data_store["t"],
        data_store["beta1_ddot_min"],
        data_store["beta1_ddot_max"],
        color="black",
        alpha=0.1,
    )
    axs[6].fill_between(
        data_store["t"],
        data_store["beta2_ddot_min"],
        data_store["beta2_ddot_max"],
        color="red",
        alpha=0.1,
    )
    axs[6].plot(
        data_store["t"], data_store["beta1_ddot"], c="black", label="acc1", linewidth=1.0
    )
    axs[6].plot(
        data_store["t"], data_store["beta2_ddot"], c="red", label="acc2", linewidth=1.0
    )

    axs[7].fill_between(
        data_store["t"],
        data_store["alpha1_ddot_min"],
        data_store["alpha1_ddot_max"],
        color="black",
        alpha=0.1,
    )
    axs[7].fill_between(
        data_store["t"],
        data_store["alpha2_ddot_min"],
        data_store["alpha2_ddot_max"],
        color="red",
        alpha=0.1,
    )
    axs[7].plot(
        data_store["t"], data_store["alpha1_ddot"], c="black", label="acc4", linewidth=1.0
    )
    axs[7].plot(
        data_store["t"], data_store["alpha2_ddot"], c="red", label="acc5", linewidth=1.0
    )

    ## XY Trajectory
    axs[8].plot(
        data_store["x_r"],
        data_store["y_r"],
        c="black",
        label="ref",
        linestyle="--",
        linewidth=2.0,
    )
    axs[8].plot(
        data_store["x1"],
        data_store["y1"],
        c="red",
        label="robot",
        linestyle="-",
        linewidth=1.0,
    )
    axs[8].scatter(
        data_store["x2"],
        data_store["y2"],
        c="orange",
        s=5,
        label="l1",
        linestyle="-",
        linewidth=1.0,
    )
    axs[8].scatter(
        data_store["x3"],
        data_store["y3"],
        c="green",
        s=5,
        label="l2",
        linestyle="-",
        linewidth=1.0,
    )

    axs[9].plot(
        H_info_store['t'],
        H_info_store['cond'],
        label="H condition number",
        color="green",
        linewidth=1.5,
    )

    axs[10].plot(
        H_info_store['t'],
        H_info_store['diag_min'],
        label="H diagonal min",
        color="green",
        linewidth=1.5,
    )
    axs[10].plot(
        H_info_store['t'],
        H_info_store['diag_max'],
        label="H diagonal max",
        color="blue",
        linewidth=1.5,
    )
    
    


    axs[0].set_ylabel("X Y [m]")
    axs[0].legend()
    axs[0].grid("both")
    axs[1].set_ylabel("Z [m]")
    axs[1].legend()
    axs[1].grid("both")

    axs[2].set_ylabel("beta [m]")
    axs[2].legend()
    axs[2].grid("both")
    axs[3].set_ylabel("alpha [rad]")
    axs[3].legend()
    axs[3].grid("both")

    axs[4].set_ylabel("u [m/s]")
    axs[4].legend()
    axs[4].grid("both")

    axs[5].set_ylabel("u [rad/s]")
    axs[5].legend()
    axs[5].grid("both")

    axs[6].set_ylabel("acc [m/s2]")
    axs[6].legend()
    axs[6].grid("both")

    axs[7].set_xlabel("Time [s]")
    axs[7].set_ylabel("acc [rad/s2]")
    axs[7].legend()
    axs[7].grid("both")

    for ax in axs[:7]:
        ax.set_xticklabels([])
    axs[7].set_xlabel("Time [s]")

    axs[8].set_xlabel("X [m]")
    axs[8].set_ylabel("Y [m]")
    axs[8].legend()
    axs[8].grid("both")
    axs[8].set_aspect("equal", adjustable="box")

    axs[9].set_ylabel("H Condition")
    axs[9].legend()
    axs[9].grid("both")

    axs[10].set_xlabel("Time [s]")
    axs[10].set_ylabel("H Diagonal")
    axs[10].set_yscale('log')
    axs[10].legend()
    axs[10].grid("both")


    # axs[5].set_xlabel("X [m]")
    # axs[5].set_ylabel("Y [m]")
    # axs[5].legend()
    # axs[5].grid("both")
    # axs[5].set_aspect("equal", adjustable="box")

    # plt.tight_layout()

    plt.subplots_adjust(right=0.98, top=1.0, left=0.06, bottom=0.05)

    plt.draw()
    plt.pause(0.1)
    plt.savefig(
        os.path.join(save_dir, "time_domain_results.pdf"),
        format="pdf",
    )
    plt.savefig(
        os.path.join(save_dir, "time_domain_results.png"),
        format="png",
        dpi=600,
    )
    plt.show(block=False)


def plot_shape_4D(shapes, save_dir, data_store=None, ws_mesh=None):
    fig = plt.figure(figsize=(8, 8))
    gs = GridSpec(1, 1, figure=fig)
    axs = []
    axs.append(fig.add_subplot(gs[0], projection="3d"))  # 3D plot

    xdata, ydata, zdata = [], [], []

    for i in range(0, len(shapes), 10):
        axs[0].plot3D(
            shapes[i][0][:, 0],
            shapes[i][0][:, 1],
            shapes[i][0][:, 2],
            # c="#c92435",
            c="#3c89d0",
            # c="#3cd061",
            label="Tube 1" if i == 0 else None,
            linewidth=0.9,
        )

        axs[0].plot3D(
            shapes[i][1][:, 0],
            shapes[i][1][:, 1],
            shapes[i][1][:, 2],
            c="#f6a400",
            # c="#3c89d0",
            label="Tube 2" if i == 0 else None,
            linewidth=0.8,
        )

        axs[0].plot3D(
            shapes[i][2][:, 0],
            shapes[i][2][:, 1],
            shapes[i][2][:, 2],
            # c="#3c89d0",
            c="#3cd061",
            # c="#f6a400",
            label="Tube 3" if i == 0 else None,
            linewidth=1.1,
        )

        xdata.append(shapes[i][0][:, 0])
        ydata.append(shapes[i][0][:, 1])
        zdata.append(shapes[i][0][:, 2])

    axs[0].scatter(0.0, 0.0, 0.0, color="#000000", label="base")

    if data_store is not None and "x_r" in data_store:
        axs[0].plot3D(
            data_store["x_r"],
            data_store["y_r"],
            data_store["z_r"],
            # s=2,
            # c="#c92435",
            c="#000000",
            # marker="o",
            label="Ref",
            linewidth=1.8,
            linestyle="--",
            alpha=0.6,
        )
        axs[0].plot3D(
            data_store["x1"],
            data_store["y1"],
            data_store["z1"],
            # s=2,
            # c="#c92435",
            c="#F80101",
            # marker="o",
            label="Tip",
            linewidth=1.2,
            linestyle="-",
            alpha=0.8,
        )

    
    # 3. Create a Poly3DCollection from the triangles
    if ws_mesh is not None:
        triangles = ws_mesh.vectors  # (N, 3, 3) -> N triangles with 3 vertices
        mesh_collection = Poly3DCollection(
            triangles,
            linewidths=0.08,
            edgecolors="#0000004E",      # black edges (set to None if you want no edges)
            alpha=0.1            # transparency (0 = fully transparent, 1 = opaque)
        )
        mesh_collection.set_facecolor("#F8D301")  # light blue, semi-transparent
        axs[0].add_collection3d(mesh_collection)
        scale = ws_mesh.points.flatten()  #   Auto-scale to the mesh size
        axs[0].auto_scale_xyz(scale, scale, scale)

    axs[0].set_xlabel("x [m]")
    axs[0].set_ylabel("y [m]")
    axs[0].set_zlabel("z [m]")
    axs[0].set_title("CTR shape")
    axs[0].legend()
    axs[0].set_box_aspect([1, 1, 1])

    # Equal aspect ratio: set data limits to be the same
    xdata = np.concatenate(xdata, axis=0)
    ydata = np.concatenate(ydata, axis=0)
    zdata = np.concatenate(zdata, axis=0)

    max_range = (
        np.array(
            [
                xdata.max() - xdata.min(),
                ydata.max() - ydata.min(),
                zdata.max() - zdata.min(),
            ]
        ).max()
        / 2.0
    )
    mid_x = (xdata.max() + xdata.min()) / 2
    mid_y = (ydata.max() + ydata.min()) / 2
    mid_z = (zdata.max() + zdata.min()) / 2
    axs[0].set_xlim(mid_x - max_range, mid_x + max_range)
    axs[0].set_ylim(mid_y - max_range, mid_y + max_range)
    axs[0].set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()
    plt.draw()
    plt.pause(0.1)
    plt.savefig(
        os.path.join(save_dir, "shape.pdf"),
        format="pdf",
    )
    plt.savefig(
        os.path.join(save_dir, "shape.png"),
        format="png",
        dpi=600,
    )
    plt.show(block=True)
    # plt.show(block=True)
