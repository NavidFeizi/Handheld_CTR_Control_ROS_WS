import sys, os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import pandas as pd
from utils import pgf_with_latex, plot_results_4D, plot_shape_4D

from stl import mesh
import trimesh

from mpl_toolkits.mplot3d.art3d import Poly3DCollection

plt.rcParams.update(pgf_with_latex)


def main():
    output_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        "Input_Files",
    )
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)


    # 1. Load STL file
    stl_path = os.path.join(os.path.dirname(__file__), "workspace_surface_poisson.stl")
    ws_mesh = mesh.Mesh.from_file(stl_path)  # shape: (n_triangles, 3, 3)
    ws_mesh.vectors /= 1000.0  # 🔹 Convert mm to m (1 m = 1000 mm)

    # # Optional: rotate (your code)
    # rotation_matrix = np.array([
    #     [0, 0, 1],  # new_x = old_z
    #     [1, 0, 0],  # new_y = old_x
    #     [0, 1, 0]   # new_z = old_y
    # ])
    # for i in range(len(ws_mesh.vectors)):
    #     for j in range(3):  # 3 vertices per triangle
    #         ws_mesh.vectors[i, j] = rotation_matrix @ ws_mesh.vectors[i, j]

    # 2. Convert numpy-stl mesh to trimesh
    triangles = ws_mesh.vectors  # (n_triangles, 3, 3)

    # unique-ify vertices and build faces
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    tm = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    print("Watertight:", tm.is_watertight)

    # 3A. Sample random points INSIDE the volume
    target_n = 50
    margin = 3e-3
    z_min = 80e-3

    # points_inside = trimesh.sample.volume_mesh(tm, target_n)
    interior_points = sample_interior_with_margin(
        tm, target_n=target_n, margin=margin, z_min=z_min
    )

    # points_inside is (n_points, 3)
    print(interior_points.shape)
    print(f"Points after filtering (z >= 0.100): {interior_points.shape}")

    # Save points to CSV
    csv_path = os.path.join(output_dir, "random_interior_points.csv")
    df = pd.DataFrame(interior_points, columns=['x', 'y', 'z'])
    df.to_csv(csv_path, index=False, float_format='%.4f')
    print(f"Saved {len(interior_points)} points to {csv_path}")

    plot_shape(
        interior_points, ws_mesh, os.path.join(output_dir, "random_interior_points.pdf")
    )

    
def sample_interior_with_margin(
    mesh: trimesh.Trimesh,
    target_n: int,
    margin: float,
    z_min: float,
    batch_factor: float = 5.0,
) -> np.ndarray:

    collected = []

    while sum(c.shape[0] for c in collected) < target_n:
        remaining = target_n - sum(c.shape[0] for c in collected)
        batch_count = int(max(remaining * batch_factor, remaining + 10))

        # Sample interior candidates
        batch = trimesh.sample.volume_mesh(mesh, batch_count)
        if isinstance(batch, tuple):
            batch = batch[0]  # first element is points

        # Signed distance to mesh surface (negative = inside)
        sd = trimesh.proximity.signed_distance(mesh, batch)

        # Keep points with at least 'margin' clearance from surface
        mask = (sd > margin) & (batch[:, 2] >= z_min)
        deep_points = batch[mask]

        if deep_points.size > 0:
            collected.append(deep_points)

    points = np.vstack(collected)[:target_n]
    return points



def plot_shape(points_inside, ws_mesh, save_dir):
    fig = plt.figure(figsize=(8, 8))
    gs = GridSpec(1, 1, figure=fig)
    axs = []
    axs.append(fig.add_subplot(gs[0], projection="3d"))  # 3D plot

    xdata, ydata, zdata = [], [], []

    axs[0].scatter(
        points_inside[:, 0],
        points_inside[:, 1],
        points_inside[:, 2],
        c="#3cd061",
        label="Random Target Points",
        s=5,
    )

    # 3. Create a Poly3DCollection from the triangles
    if ws_mesh is not None:
        triangles = ws_mesh.vectors  # (N, 3, 3) -> N triangles with 3 vertices
        mesh_collection = Poly3DCollection(
            triangles,
            linewidths=0.03,
            edgecolors="#4646464E",  # black edges (set to None if you want no edges)
            alpha=0.05,  # transparency (0 = fully transparent, 1 = opaque)
        )
        mesh_collection.set_facecolor("#6363635C")  # light blue, semi-transparent
        axs[0].add_collection3d(mesh_collection)
        scale = ws_mesh.points.flatten()  #   Auto-scale to the mesh size
        axs[0].auto_scale_xyz(scale, scale, scale)

    axs[0].set_xlabel("x [mm]", labelpad=-2)
    # axs[0].set_xticks([0, 40, 80, 120, 160])
    # axs[0].tick_params(axis="x", which="both", pad=-2)

    axs[0].set_ylabel("y [mm]", labelpad=-3)
    # axs[0].set_yticks([-40, -20, 0, 20, 40])
    # axs[0].tick_params(axis="y", which="both", pad=-2)

    axs[0].set_zlabel("z [mm]", labelpad=-1)
    # axs[0].set_zticks([-60, -40, -20, 0, 20])
    # axs[0].tick_params(axis="z", which="both", pad=-0)

    axs[0].set_title("3D Backbone")
    axs[0].set_box_aspect([1, 1, 1])

    # Equal aspect ratio: set data limits to be the same

    # xdata = np.concatenate([np.array([0]), *xdata], axis=0)
    # ydata = np.concatenate([np.array([0]), *ydata], axis=0)
    # zdata = np.concatenate([np.array([120]), *zdata], axis=0)

    # max_range = (
    #     np.array(
    #         [
    #             xdata.max() - xdata.min(),
    #             ydata.max() - ydata.min(),
    #             zdata.max() - zdata.min(),
    #         ]
    #     ).max()
    #     / 2.0
    # )
    # mid_x = (xdata.max() + xdata.min()) / 2
    # mid_y = (ydata.max() + ydata.min()) / 2
    # mid_z = (zdata.max() + zdata.min()) / 2
    # axs[0].set_xlim(mid_x - max_range, mid_x + max_range)
    # axs[0].set_ylim(mid_y - max_range / 1.5, mid_y + max_range / 1.5)
    # axs[0].set_zlim(mid_z - max_range / 1.5, mid_z + max_range / 1.5)

    leg = axs[0].legend(
        ncols=2,
        fancybox=False,
        fontsize=7,
        loc="upper left",
        bbox_to_anchor=(0.55, 0.9),
        borderaxespad=0.0,
    )
    leg.get_frame().set_linewidth(0.2)
    leg.get_frame().set_edgecolor("gray")

    axs[0].set_facecolor("white")
    for ax in axs:
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False

    # Adjust border width for all subplots
    for ax in axs:
        for spine in ax.spines.values():
            spine.set_linewidth(0.3)  # Adjust this value to control border thickness

    axs[0].view_init(elev=15, azim=-30)

    plt.subplots_adjust(right=0.89, top=1.04, left=-0.06, bottom=0.05)
    plt.draw()
    plt.pause(0.1)
    plt.savefig(
        os.path.join(save_dir),
        format="pdf",
    )
    plt.show(block=True)


if __name__ == "__main__":
    main()
