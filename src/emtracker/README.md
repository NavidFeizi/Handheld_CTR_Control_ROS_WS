# emtracker

NDI Aurora electromagnetic tracker driver. Reads the tool, robot-base, and phantom
sensors, publishes their poses in the robot frame, broadcasts the TF tree the rest of
the system localizes against, and performs landmark registration.

This is the first node to start in system bring-up: the tracker takes several seconds
to initialize, which is why `ctr_bringup` delays the robot nodes by 14 s.

## Quick start

```bash
source install/setup.bash
ros2 launch emtracker launch.py
ros2 launch emtracker launch.py host_name:=/dev/ttyUSB0    # different serial port
```

## Nodes

| Executable | Node name in code | Name at launch | Role |
|---|---|---|---|
| `track` | `emtracker` | `emt_node` | Aurora driver, pose publication, TF broadcast, registration |

Use `emt_node` — the launch name — with `ros2 param set`.

## Launch file

`launch/launch.py` starts `emt_node` pinned to CPU core 10.

| Argument | Default |
|---|---|
| `host_name` | `/dev/ttyUSB1` |
| `send_on_igtl` | `false` |
| `enable_position_logging` | `false` |

## Parameters

From `config/emtracker_params.yaml`; the three launch arguments above override.

| Parameter | Default | Meaning |
|---|---|---|
| `sample_time` | 0.025 | s (~40 Hz) |
| `cutoff_freq` | 6.6 | Hz — node-side Butterworth on the tool poses |
| `send_on_igtl` | `false` | Push poses over OpenIGTLink directly from this node |
| `enable_position_logging` | `false` | Log tool positions to disk |
| `host_name` | `/dev/ttyUSB1` | NDI Aurora serial device |
| `emtracker_data_dir` | `""` | Empty → `$HOME/Documents/handheld_CTR/emtracker_config/`, seeded from the installed share directory on first run |

Filter cutoff can be changed while running:

```bash
ros2 param set emt_node cutoff_freq 1.0
```

## Topics

| Direction | Topic | Type |
|---|---|---|
| pub | `task_space/feedback/base_tool` | `interfaces/msg/Taskspace` |
| pub | `task_space/feedback/phantom_tool` | `interfaces/msg/Taskspace` |
| pub | `task_space/feedback/phantom_base` | `interfaces/msg/Taskspace` |

`task_space/feedback/base_tool` is the tool pose in the robot frame — the primary
feedback signal for the EKF, MPC, GUI, and recorder.

TF frames broadcast: `em_tracker`, `robot_base`, `ctr_tip`, `phantom`, `probe`,
`us_probe`, `sensor_1`, `sensor_2`, `sensor_3`.

## Services

| Service | Type | Effect |
|---|---|---|
| `freeze_robot` | `std_srvs/srv/SetBool` | Hold the robot-base transform at its current value |
| `freeze_phantom` | `std_srvs/srv/SetBool` | Hold the phantom transform |
| `get_transformation` | `interfaces/srv/Transformation` | Returns the current CTR transform as a 16-element row-major matrix |

```bash
ros2 service call /freeze_phantom std_srvs/srv/SetBool "{data: true}"
```

Freezing is useful when a sensor drops out mid-procedure and you would rather hold
the last good registration than track noise.

## Hardware

A real NDI Aurora tracker on a serial/USB port. Port permissions and enumeration are
covered in the [root README](../../README.md).

For older devices without USB support, adjust the baud rate in the vendored NDI API
(`lib_emtracker/ndi_api/CombinedApi.cpp`).

Sensor serial numbers, SROM tool-definition files, and the active registration files
are configured in `lib_emtracker/config/config.yaml`.

## Landmark registration

Registration is a build-time-gated procedure, not a service — it requires editing and
rebuilding:

1. Touch each landmark of the experimental setup with a pre-calibrated probe.
2. Save the true landmark values as a `.csv` in the config path.
3. Uncomment the landmark-registration section in `setup_emtracker()` and point
   `std::string landmarks` at your truth file.
4. Build and run. Touch each landmark in the order given in the truth CSV and hold
   still; the terminal prompts you onward once enough samples are collected.
5. The transformation is solved by optimization and written to
   `registration_results.csv` in the config path.
6. Rename that file and set the new name in `config.yaml` so the node loads it next
   time.
7. Comment the registration section back out and rebuild.

## Tests

```bash
colcon test --packages-select emtracker
```

- `test/test_butterworth.cpp` — filter settling and attenuation behavior.
- `test/test_rigid_transformation.cpp` — quaternion rigid transforms: identity,
  inverse, associativity, translation and rotation composition.

Both are pure math, with no ROS or hardware dependency.

## Status notes

- **`host_name` defaults disagree**: the in-code `declare_parameter` default is
  `/dev/ttyUSB0`, while the YAML and launch file both say `/dev/ttyUSB1`. The YAML
  wins in normal operation, so this only bites if you run the executable directly
  with `ros2 run`.
- **`enable_position_logging` defaults disagree** the same way: `true` in code,
  `false` in the YAML and launch file.
- Two stale files sit in `lib_emtracker/config/`:
  `landmarks_truth_ctr_robot-depreciated.csv` (named as deprecated) and
  `landmarks_truth_ctr_robot_v3.5 copy.csv` (a stray duplicate). Neither is loaded by
  default — `config.yaml` selects the active registration files.
- `config.yaml` retains commented-out sensor serial numbers from earlier hardware
  revisions.
