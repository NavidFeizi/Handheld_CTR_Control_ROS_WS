# target_gen

Standalone Python utilities for generating target points inside the robot's reachable
workspace.

**This is not a ROS package.** It has no `package.xml` and no `CMakeLists.txt`, so
colcon does not build, install, or test it, and nothing in `src/` depends on it. It
lives under `src/` for proximity only. Run the scripts directly.

## Generating targets

```bash
python3 src/target_gen/random_target.py
```

Writes two files to `<workspace_root>/Input_Files/` (created if absent):

| File | Contents |
|---|---|
| `random_interior_points.csv` | 50 sampled points, columns `x,y,z`, 4 decimal places |
| `random_interior_points.pdf` | 3D scatter of the points against the workspace mesh |

That CSV is what `manager`'s `targets_csv` parameter defaults to, so regenerating it
changes the target list the next procedure runs against.

### How it samples

1. Loads `workspace_surface_poisson.stl` (the reachable-workspace surface) and
   converts it from millimetres to metres.
2. Rebuilds it as a watertight `trimesh.Trimesh`, deduplicating vertices.
3. Rejection-samples interior points with `trimesh.sample.volume_mesh`, keeping only
   those with at least `margin` clearance from the surface (via
   `trimesh.proximity.signed_distance`) and above `z_min`. Batches until it has
   `target_n` points.

Defaults are constants near the top of `main()`:

| Constant | Value | Meaning |
|---|---|---|
| `target_n` | 50 | Points to collect |
| `margin` | 3e-3 | m — minimum clearance from the workspace surface |
| `z_min` | 80e-3 | m — floor on the z coordinate |

Edit them in the script; there is no CLI.

## Dependencies

Not declared anywhere — there is no `package.xml` or `requirements.txt`. Install by
hand:

```bash
pip install numpy pandas matplotlib numpy-stl trimesh shapely
```

## Files

| File | Purpose |
|---|---|
| `random_target.py` | Entry point — the sampler described above |
| `utils.py` | Shared plotting and trajectory helpers |
| `workspace_surface_poisson.stl` | Reachable-workspace surface mesh (binary STL, mm) |

## Status notes

`utils.py` is a grab-bag from an earlier MPC plotting workflow, and most of it is
unused here. `random_target.py` imports `plot_results_4D` and `plot_shape_4D` but
never calls either — only `pgf_with_latex` (LaTeX-styled matplotlib rcParams) is
actually used. The MPC reference-trajectory generators in `utils.py`
(`circular_ref_trajectory`, `infinity_ref_trajectory`, `H_ref_trajectory`,
`W_ref_trajectory`, `y_ref_trajectory_for_grassmann`) and `plot_results_4D` /
`plot_shape_4D` are kept for ad-hoc scripts, not because this script needs them.
