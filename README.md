# roboracer_sim_ws

F1TENTH / RoboRacer 2026 stack, built on [ForzaETH race_stack](https://github.com/ForzaETH/race_stack)
with the global optimizer subtree from ForzaETH's fork of TUM FTM's
`global_racetrajectory_optimization`.

`CLAUDE.md` covers architecture and design decisions. This file covers getting it
to run.

**Target platform: Ubuntu 22.04 / ROS 2 Humble.** Nothing here runs on macOS.

---

## First-time setup

Run every step from the workspace root.

### 0. Prerequisites

ROS 2 Humble installed and `rosdep` initialised, plus:

```bash
sudo apt install python3-venv python3-pip
```

### 1. Clone

```bash
git clone https://github.com/hyunyoung1018/roboracer_sim_ws.git
cd roboracer_sim_ws
```

**Pick the final location now.** A venv records absolute paths in
`bin/activate`, in every console script's shebang, and in editable-install
`.pth` files. Moving the workspace after step 2 silently breaks all of them —
`source .venv/bin/activate` still appears to succeed while doing nothing,
because the directory it puts on `PATH` no longer exists.

### 2. Virtualenv — `--system-site-packages` is mandatory

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
```

Without `--system-site-packages` the venv cannot see `rclpy` or any other apt
ROS package, and nothing works.

### 3. System dependencies

```bash
rosdep install --from-paths src --ignore-src -y
```

This installs the apt side: `nav2_map_server`, `foxglove_bridge`, `rviz2`,
`python3-skimage`, `python3-matplotlib`, and so on. All of it is declared in the
`package.xml` files.

### 4. The vendored f1tenth_gym

The simulator is vendored as a subtree and is **not** installed by
`requirements.txt`. Install it editable, with dependencies:

```bash
pip install -e src/f1tenth_gym_ros/f1tenth_gym
```

This is what brings `numpy`, `scipy`, `gymnasium` and `numba` into the venv. Do
this **before** step 5.

### 5. Python dependencies

```bash
pip install --no-deps -r requirements.txt
```

`--no-deps` is required, not an optimisation. `requirements.txt` explains why in
detail; the short version is that `trajectory_planning_helpers` declares
`quadprog==0.1.7` (a broken build on aarch64) and `matplotlib>=3.3.1` (which
breaks `mpl_toolkits` — see *Gotchas*).

### 6. Build

```bash
python -m colcon build --symlink-install
source install/setup.bash
```

`python -m colcon`, **not** `colcon`. See *Gotchas*.

### 7. Verify

```bash
ros2 launch f1tenth_gym_ros unita_gym_bridge_launch.py
```

`/scan` and `/ego_racecar/odom` should be publishing.

---

## Everyday use

```bash
source .venv/bin/activate
source install/setup.bash          # alias: sb
python -m colcon build --symlink-install
```

With `--symlink-install`, edits to Python sources and to config/launch files
take effect without rebuilding. Adding or renaming a node still needs a build.

---

## Running

### Simulator

```bash
ros2 launch f1tenth_gym_ros unita_gym_bridge_launch.py
```

Which map it loads is set by `map_path` in
`src/f1tenth_gym_ros/config/sim.yaml`. Give it a bare map name — it is resolved
against `stack_master`'s share directory as `maps/<map>/<map>.yaml`.

A bare name that does not resolve makes the vendored gym fall back to
downloading a track from `api.f1tenth.org`, so a typo shows up as a network
fetch rather than a clean error.

Foxglove opens automatically; `open_foxglove:=false` suppresses it.

### Raceline generation

```bash
ros2 launch stack_master raceline_generator.launch.xml map:=26_track_22x8
```

Writes `centerline.csv`, `global_waypoints.json`, `track_meta.yaml`,
`speed_scaling.yaml` and `ot_sectors.yaml` into `stack_master/maps/<map>/`.

On the first run for a map, RViz waits 30 s for a **2D Pose Estimate** — click
the start line, pointing the way the car will drive. The choice is saved to
`track_meta.yaml` and reused automatically afterwards.

Do not skip this with `start_pose_timeout:=0` on a map you care about. It falls
back to the map origin, which puts `s=0` at an arbitrary point and therefore
shifts every sector index — and then persists that to `track_meta.yaml`.

Useful arguments:

| Argument | Default | Notes |
|---|---|---|
| `map` | required | folder name under `stack_master/maps/` |
| `mode` | `car` | vehicle parameter set: `car` or `sim` |
| `sectors` | `true` | `false` to redo only the raceline |
| `rviz` | `true` | |
| `safety_width` | `0.7` | use `0.45` on `26_inu_track_6x12`, which is only 0.9 m wide |

---

## Gotchas

These are all things that have actually broken this workspace.

### Build with `python -m colcon`, inside the venv

`colcon` is installed system-wide, so its shebang is `/usr/bin/python3`. Running
it directly bakes that interpreter into every generated node script, and those
nodes then cannot see anything in the venv — `f1tenth_gym` included. Going
through `python -m colcon` makes the venv interpreter the one that gets
recorded.

### Never `pip install matplotlib`

apt's `python3-matplotlib` ships `mpl_toolkits` as a *regular* package (it has
an `__init__.py`), while pip's ships it as a *namespace* package. Python's
import rules let a regular package beat a namespace portion found earlier on
`sys.path`, so a pip matplotlib wins for `matplotlib` while apt still wins for
`mpl_toolkits`. The mismatched pair fails at
`from mpl_toolkits.mplot3d import Axes3D`, which takes down the entire raceline
import chain. `pip install --user` has the same effect.

Matplotlib comes from apt, via rosdep, only.

### Do not mix `--symlink-install` with a plain build

Pick one and stay with it. A `build/` tree created by plain `colcon build`
cannot be converted, and the next `--symlink-install` fails with
`failed to create symbolic link ... existing path cannot be removed: Is a
directory`. Recovery is `rm -rf build install log` and a full rebuild.

### Do not move the workspace after setup

See step 1. If it has already happened, the fix is to rewrite the old absolute
path everywhere in `.venv`:

```bash
grep -rl "/old/path" .venv/ | xargs sed -i 's|/old/path|/new/path|g'
```

That works because the venv's `python` is a symlink to the system interpreter
and `pyvenv.cfg` points at `/usr/bin`; only the recorded paths are stale.

### `rosdep` and packages that do not exist yet

An `exec_depend` on a package that is not in the workspace makes
`rosdep install` fail for the **whole workspace**, not just that package, and it
reports only the first unresolved key per package — so fixing one reveals the
next. `controller` is commented out in `stack_master/package.xml` for this
reason; uncomment it when the package is ported.

### `slam_tuner` launch files are broken

`ekf_sim_launch.py` and `tuner_launch.xml` both call
`find-pkg-share state_estimation`. No such package exists here — `state_estimation`
is a directory containing `particle_filter`. Inherited from race_stack, not yet
repaired.
