# roboracer_unita_ws

F1TENTH / RoboRacer 2026 stack, built on
[ForzaETH race_stack](https://github.com/ForzaETH/race_stack).

---

## 1. Environment

| | |
|---|---|
| Car | `albomb` — Traxxas 1/10, VESC 6 MkVI, Hokuyo UST-10LX, Logitech F710 |
| Computer | Jetson Orin Nano Super (arm64, JetPack 6 / Ubuntu 22.04) |
| ROS | Humble |

**Everything runs on the Jetson**, raceline generation included. A laptop VM is
only a stand-in until the car exists — one install path, not two. arm64 is the
deployment target, so an x86-only wheel is never acceptable.

TF frames are `ego_racecar/*` in both sim and real, deliberately, so the two
share one set of frames with no remapping.

### Layout

```
albomb_description/   URDF, meshes
f1tenth_gym_ros/      simulator (vendored gym subtree)
stack_master/         launch + config + maps        <- start here
planner/raceline/     raceline generation + publishing
state_estimation/     mapping (cartographer), particle_filter
sensor/vesc/          VESC driver (patched, see section 3)
utilities/            f110_msgs, sector_tuner, lap_analyser, ...
```

### Pipeline

Each stage is a launch file, not a mode flag on one node:

```
mapping.launch.xml            drive the track      -> maps/<map>/<map>.{png,yaml,pbstream}
raceline_generator.launch.xml map                  -> global_waypoints.json + sector yamls
base_system.launch.xml        sim:=true|false      -> drivers + raceline, ready to drive
```

Under those, `hardware.launch.xml` brings up the LiDAR, VESC, teleop and robot
description. The map folder is the contract between stages:

```
maps/<map>/
  <map>.png  <map>.yaml    nav2 standard, kept pristine
  track_meta.yaml          start pose + direction
  centerline.csv           x_m,y_m,w_tr_right_m,w_tr_left_m
  global_waypoints.json    the raceline
  speed_scaling.yaml  ot_sectors.yaml
```

---

## 2. Setup

Run everything from the workspace root.

```bash
# 0. prerequisites: ROS 2 Humble installed, rosdep initialised
sudo apt install python3-venv python3-pip

# 1. clone (no submodules)
git clone https://github.com/hyunyoung1018/roboracer_unita_ws.git
cd roboracer_unita_ws

# 2. venv - --system-site-packages is mandatory, or rclpy is invisible
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

# 3. apt dependencies (nav2, cartographer, urg_node, rviz, skimage, ...)
rosdep install --from-paths src --ignore-src -y

# 4. the vendored simulator, without its renderer
pip install -e src/f1tenth_gym_ros/f1tenth_gym --no-deps
pip install "numpy<2" "gymnasium>=0.29.1,<0.30" "numba>=0.59.0,<0.61" \
            "pandas>=2.0.0" "pillow>=9.1.0" "requests>=2.31.0" \
            "scipy>=1.13.0" "yamldataclassconfig>=1.5.0,<2"

`transforms3d` in `requirements.txt` belongs to the same problem: apt's copy
uses `np.float`, removed in numpy 1.24, and every node that touches
`tf_transformations` dies on it. There is no numpy that satisfies both apt's
transforms3d and the gym's `scipy>=1.13`, so the newer transforms3d wins.

**`numpy<2` is the one that matters.** `skimage` and `opencv` come from apt,
built against numpy 1.x. A pip numpy 2 in the venv shadows apt's copy and breaks
every one of them at import - `numpy.dtype size changed` from skimage,
`_ARRAY_API not found` from cv2. Nothing here needs numpy 2. `numba<0.61`
follows: 0.61 and later require it, and 0.66 additionally wants a newer
`coverage` than apt ships and dies on `module 'coverage' has no attribute
'types'`.

# 5. the optimizer's solvers
pip install --no-deps -r requirements.txt

# 6. range_libc, for the particle filter. Cython extension, not a pip package.
#    --no-build-isolation is required: the repo has no pyproject.toml, so pip
#    would build in a clean environment where the cython just installed is not
#    visible. Do NOT use its compile.sh, which runs sudo and would install to
#    the system python rather than this venv.
pip install cython
git clone https://github.com/f1tenth/range_libc /tmp/range_libc
cd /tmp/range_libc/pywrapper && pip install --no-build-isolation . && cd -
#    On the Jetson, for the much faster rmgpu ray casting (then set
#    range_method: 'rmgpu' in config/car/pf.yaml):
#      WITH_CUDA=ON pip install --no-build-isolation .

# 7. build - `python -m colcon`, never bare `colcon`
python -m colcon build --symlink-install
source install/setup.bash

# 8. verify
python -c "from f1tenth_gym.envs import F110Env; print('gym OK')"
python -c "from raceline.raceline_generator import trajectory_optimizer; print('raceline OK')"
ros2 launch f1tenth_gym_ros unita_gym_bridge_launch.py
```

**Pick the workspace location before step 2.** A venv bakes absolute paths into
`bin/activate`, every console script's shebang, and editable-install `.pth`
files. Move it afterwards and `source .venv/bin/activate` appears to succeed
while doing nothing.

Both `--no-deps` flags are load-bearing; section 3 says why.

### Running

```bash
# simulator
ros2 launch f1tenth_gym_ros unita_gym_bridge_launch.py

# map a new track (real car; needs a display for RViz)
ros2 launch stack_master mapping.launch.xml map:=<name>
#   drive with the F710 (hold LB), then from another terminal:
ros2 service call /finish_mapping std_srvs/srv/Trigger {}

# localisation on a saved map - everything above it needs this
ros2 launch stack_master localization.launch.xml map:=<name>
ros2 launch stack_master localization.launch.xml map:=<name> rviz:=false foxglove:=true
#   drivers come up with it. Place the car with "2D Pose Estimate", then watch
#   /pf/viz/particles: a tight cloud following the car means it has converged.

# raceline from an existing map
ros2 launch stack_master raceline_generator.launch.xml map:=<name>
#   first run waits 30 s for an RViz "2D Pose Estimate" - click the start line,
#   pointing the way the car will drive. Saved to track_meta.yaml and reused.

# bring the stack up
ros2 launch stack_master base_system.launch.xml map:=<name>          # car
ros2 launch stack_master base_system.launch.xml map:=<name> sim:=true
```

| Argument | Default | Notes |
|---|---|---|
| `safety_width` | `0.7` | use `0.45` on `26_inu_track_6x12` (0.9 m wide) |
| `sectors` | `true` | `false` to redo only the raceline |
| `hardware` | `true` | mapping: `false` if drivers already run |

---

## 3. Watch out

Things that have actually broken this workspace, or will.

### Build and install

- **`python -m colcon build`, not `colcon build`.** `colcon` is installed
  system-wide with a `/usr/bin/python3` shebang, and running it directly bakes
  that interpreter into every generated node script — which then cannot see the
  venv, `f1tenth_gym` included.

  Rebuilding does not undo it: the shebang is written into scripts that already
  exist, so the wrong one survives every later build. Recover with

  ```bash
  rm -rf build install log
  python -m colcon build --symlink-install
  ```

  The symptom is a node dying on `ModuleNotFoundError` for a package that
  imports perfectly well in your shell. That mismatch is the tell — same
  machine, same venv, different interpreter.
- **Never `pip install matplotlib` or `opencv-python`.** apt's matplotlib ships
  `mpl_toolkits` as a regular package and pip's ships it as a namespace one; a
  pip matplotlib wins for `matplotlib` while apt still wins for `mpl_toolkits`,
  and the mismatched pair takes down the whole raceline import chain. For
  OpenCV it is worse: JetPack's build is CUDA-enabled and a pip one shadows it.
  Both come from apt via rosdep.
- **Keep the version bounds in `requirements.txt`.** With
  `--system-site-packages`, pip counts anything in `~/.local` as satisfying an
  unpinned requirement and installs nothing — even if what it found is broken.
- **Do not mix `--symlink-install` with a plain build.** A `build/` tree from
  one cannot be converted; recovery is `rm -rf build install log`.

### Wiring

- **No hardcoded repo paths.** race_stack's `get_data_path()` resolved to
  `src/race_stack/stack_master`, which does not exist here. Take an explicit
  `map_dir` / `save_dir` parameter and resolve symlinks back to src
  (`raceline/paths.py`, `sector_tuner/paths.py`, `mapping/paths.py`).
- **No `~/.ros` side channels.** Stages talk through files in the map folder.
  race_stack passed the centerline via `~/.ros/map_centerline.csv`, which made
  the stages impossible to run separately.
- **The start pose is not in the nav2 map yaml.** It lives in `track_meta.yaml`.
  Order: parameter → `track_meta.yaml` → RViz `/initialpose` → map origin, which
  warns. Falling back to the origin silently misplaces `s=0` and therefore every
  sector index.
- **`global_waypoints.json` keeps race_stack's schema** so downstream nodes work
  unmodified.
- **No inline `#` comments inside the `{...}` blocks in `racecar_f110.ini`.**
  They are parsed with `json.loads`. Annotations go in the header.
- **Do not bump `trajectory_planning_helpers` past 0.76** or touch the scipy
  shim in `LOCAL_CHANGES.md` without re-running a full generation. 0.79 changed
  `iqp_handler()`'s signature; scipy ≥ 1.9 broke tph's `dist_to_p`.

### Mapping

Cartographer runs on **LiDAR + IMU, no wheel odometry**. The VESC is sensorless,
so its speed is least trustworthy exactly at the slow, tight-turning pace
mapping is driven at, and its yaw is dead-reckoned from the servo command rather
than measured.

- `tracking_frame` **must** be `ego_racecar/imu`. Cartographer hard-CHECKs that
  the IMU is colocated with the tracking frame, and albomb's IMU sits at
  (0.07, 0, 0.09) from base_link.
- `vesc_driver` is **patched locally** — upstream leaves the IMU `frame_id`
  empty and assigns the VESC's raw units (g, deg/s) straight into
  `sensor_msgs/Imu`, which specifies m/s² and rad/s. Acceleration came out 9.8×
  too small, angular velocity 57× too large, and no consumer can detect it.
  Do not revert those.
- Cartographer owns TF (`map -> ego_racecar/base_link`). Nothing else may
  publish that edge — `vesc_to_odom`'s `publish_tf` stays `false`.

### Hardware

- The Jetson needs an interface on `192.168.0.0/24` to reach the LiDAR
  (default `192.168.0.10:10940`). The Windows *IP Discovery* tool in
  `datasheets/UST-10LX/` is only needed to change it; there is also a hardware
  reset (tie IP RESET LINE to COM− for >2 s).
- Put the F710's mode switch on **X (XInput)**. D mode renumbers every axis.
- Offline tools need a display — `raceline_generator` opens RViz and the sector
  slicers open blocking matplotlib windows. Over bare SSH they hang silently.
  Runtime nodes are headless.

---

## 4. TODO

### Before the first real-car run

- [ ] **Measure the four VESC calibration gains** in `config/car/vesc.yaml`.
      They are still ForzaETH's; speed and steering angle will both be wrong.
      Procedure is in the file header.
- [ ] **Settle the IMU Z axis.** The yaw is now +90°, verified on the car. But
      the VESC board's Z reportedly points at the floor, and a Z flip is a roll
      of pi, not a yaw - so the joint may still be wrong for gravity, which is
      what cartographer uses the IMU for. Level and still, `linear_acceleration.z`
      on `/vesc/sensors/imu/raw` should read about +9.8. If it reads −9.8, add
      roll=pi in `albomb_sensors.xacro` and re-check the yaw sign, since the
      flip changes it.
- [ ] Pin the VESC serial port with a udev rule; `/dev/ttyACM0` moves on reboot.
      Give the rule `MODE="0666"` and it also covers the dialout group, which a
      fresh flash does not put you in - the symptom is
      `open: Permission denied` from vesc_driver.
- [ ] Set the VESC's own command timeout (App Settings, General). Releasing the
      joystick deadman stops publication rather than sending zeros, so that
      timeout is what actually stops the car.
- [ ] Match the simulated LiDAR to the real one in `sim.yaml` if it drifts —
      1081 beams, 0.06–10 m, ±135°.

### Not written yet

- [ ] `raceline_editor` — interactive tuning of an existing `global_waypoints.json`.
- [ ] Localisation for `base_system.launch.xml mode:=car`. `particle_filter` is
      ported but unwired, so there is currently no `map -> odom` while racing.
- [ ] A command mux. `joy_teleop` publishes straight to `/vesc/ackermann_cmd`;
      correct while the joystick is the only publisher, but a controller will
      fight it for that topic.
- [ ] `time_trials` / `head_to_head` driving modes — both need a controller and
      state machine that are not ported. `controller` is commented out in
      `stack_master/package.xml`; an `exec_depend` on a missing package fails
      `rosdep install` for the whole workspace.
- [ ] `slam_tuner`'s launch files reference a `state_estimation` package that
      does not exist here (it is a directory). Inherited from race_stack, broken.

### Known and deliberately left

- The spline-normals crossing check is commented out upstream by ForzaETH. With
  it off a raceline can silently leave the track on tight corners; if that shows
  up, re-enable it in `prep_track.py` and raise `s_reg` in `racecar_f110.ini`.
- `urg_node` comes from apt. If it ever needs to be vendored, upstream 1.2.0 is
  missing `#include <unistd.h>` in `urg_c_wrapper.cpp` on some toolchains.
- `resolve_source_dir` is duplicated in three packages. Deliberate — it avoids a
  dependency edge for 15 lines. A fourth copy means it should move to
  `utilities/libraries/`.
