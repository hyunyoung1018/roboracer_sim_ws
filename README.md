# roboracer_unita_ws

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
git clone https://github.com/hyunyoung1018/roboracer_unita_ws.git
cd roboracer_unita_ws
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
`requirements.txt`. Install it without dependency resolution, then install the
dependencies it actually uses:

```bash
pip install -e src/f1tenth_gym_ros/f1tenth_gym --no-deps
pip install gymnasium numba pandas pillow requests "scipy>=1.13" yamldataclassconfig
```

Do this **before** step 5.

`--no-deps` on the first line skips the gym's rendering stack — `pyqt6`,
`pyqtgraph`, `PyOpenGL`, `PyOpenGL-accelerate` — which this workspace never
loads. The second line resolves normally, so the real dependencies still bring
their own transitive requirements.

Skipping the renderer is not just an optimisation on arm64, where it does not
build at all (see *Architecture notes*). Nothing here can reach it: `gym_bridge.py` builds
the environment with `render_enabled=False`, and `F110Env` only calls
`make_renderer()` — the sole place PyQt6 is imported — when that flag is set.
Visualisation is RViz and Foxglove.

The second line is every third-party module the gym imports outside
`envs/rendering/`, minus `numpy`, `opencv-python` and `PyYAML`, which step 3
already installed from apt. `scipy` is in the list despite also coming from apt:
22.04 ships 1.8.0 and the gym needs `>=1.13`, so pip has to shadow it. The list
was derived by reading the imports, so if something still surfaces at runtime,
add it here.

If you specifically want the gym's own renderer — for standalone (non-ROS) use
of the simulator — install it separately on x86_64:

```bash
pip install "pyqt6>=6.7.1,<7" "pyqtgraph>=0.13.7,<0.14" "PyOpenGL>=3.1.9" "PyOpenGL-accelerate>=3.1.9"
```

On arm64 that will try to compile PyQt6 from source; see *Architecture notes*
before you do.

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
python -c "from f1tenth_gym.envs import F110Env; print('gym OK')"
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

## Where this runs

**The whole workspace runs on the car — a Jetson Orin Nano Super.** Raceline
generation, the simulator and the sector tuners all run there too; nothing is
split off onto a separate machine. A laptop VM is only a stand-in until the car
is available.

That makes the setup single-path: the steps above are the steps, everywhere.
Both targets are arm64 on Ubuntu 22.04, so the environment is the same one
twice rather than two environments to keep in sync.

### Jetson Orin Nano Super

JetPack 6 is Ubuntu 22.04, so ROS 2 Humble is a native fit and no step changes.
Two things to know:

**Never `pip install opencv-python`.** JetPack ships its own CUDA-enabled
OpenCV in `/usr/lib/python3/dist-packages`, and a pip build shadows it — the
same failure mode as the matplotlib one under *Gotchas*, with the added cost of
silently losing GPU acceleration. Step 4's `--no-deps` is what keeps the gym
from pulling one in.

**The offline tools need a display.** `raceline_generator` opens RViz, and the
two sector slicers open blocking matplotlib windows. Over a bare SSH session
they will hang with no visible error. Use the Jetson's own monitor, `ssh -X`,
or a VNC session when generating a raceline. Everything needed while actually
driving — `raceline_publisher` and the runtime nodes — is headless.

### arm64 notes (Jetson, and Apple Silicon VMs)

**PyQt6 has no arm64 wheel on PyPI.** Anything that resolves `pyqt6` — which is
what step 4 would do without `--no-deps` — falls back to the source
distribution, and the build fails before it even starts compiling:

```
ModuleNotFoundError: No module named 'packaging.licenses'
```

`packaging.licenses` was added in packaging 24.2, and pip's build-isolation
environment gets an older one because Ubuntu 22.04 ships pip 22.0.2, which
cannot use current `packaging` wheels.

Upgrading pip clears that error but is the wrong fix: the next step compiles
PyQt6 from source, which needs the Qt6 development headers and takes a long time,
to produce a library nothing here loads. Follow step 4 as written instead.

The same "no arm64 wheel" pattern affects `quadprog==0.1.7`, whose arm64 build
imports with `undefined symbol: _Z7qpgen2_...`. That is why `requirements.txt`
leaves `quadprog` unpinned and why `--no-deps` is mandatory there — see step 5.

### x86_64

No known differences; wheels exist for everything. A full
`pip install -e src/f1tenth_gym_ros/f1tenth_gym` would also succeed here, it
just pulls in the renderer this workspace never loads.

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
