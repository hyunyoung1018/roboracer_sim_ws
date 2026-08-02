# roboracer_unita_ws

F1TENTH / RoboRacer 2026 stack. Built on **ForzaETH race_stack** (ETH Zurich) —
that lineage is intentional and should stay visible in naming and structure.
The global optimizer subtree comes from ForzaETH's fork of TUM FTM's
`global_racetrajectory_optimization`.

UNIST's `unicorn-racing-stack` was studied for reference. Do **not** copy its
names or code — where their solution is right, arrive at it independently or
take it from race_stack instead.

## Environment

- Development happens on macOS; **everything actually runs on Ubuntu 22.04 /
  ROS 2 Humble** in UTM. Do not assume a command can be executed locally.
- Car is `albomb`, on a **Jetson Orin Nano Super** (arm64, JetPack 6 / Ubuntu
  22.04). arm64 is the deployment target, not an edge case — never assume an
  x86-only wheel is acceptable.
- TF frames use `ego_racecar/*` so sim and real share one set of frames with no
  remapping — this is deliberate, don't "fix" it.
- **Everything runs on the Jetson**, raceline generation included. The laptop VM
  is a stand-in until the car exists, not a separate deployment target — so
  there is one install path, not two.
- Never let a pip `opencv-python` in: JetPack's CUDA-enabled build lives in
  `/usr/lib/python3/dist-packages` and a pip one shadows it.
- The offline tools need a display (RViz, and the sector slicers' matplotlib
  windows). Runtime nodes are headless.
- Setup order and the traps are in `README.md`. Both pip steps need `--no-deps`
  and neither is optional — see the file for why. Never install the vendored
  subtree's own requirements.txt (2020-era pins).

## Pipeline: three launch files, not one node with mode flags

| Launch | Status |
|---|---|
| `mapping.launch.xml` — cartographer + `finish_mapping` | not written |
| `raceline_generator.launch.xml` — map → `global_waypoints.json` + sectors | **done** |
| `raceline_editor.launch.xml` — edit an existing json | not written |

Also missing: `base_system.launch.xml`, `hardware.launch.xml`, a launch for
`raceline_publisher`, and the `vesc_driver` IMU `frame_id` fix (it publishes an
empty frame_id, so robot_localization silently drops every IMU sample).

race_stack does all of this in one node behind `create_map` /
`create_global_path` / `map_editor` flags. Those flags are gone: which stage
runs is decided by which launch you start, so each node does exactly one job.

`raceline/` splits into `map_io`, `map_processing`, `markers` + two entry
points. These are **modules inside one node**, not separate stages — don't
mistake the file count for pipeline steps.

## Non-negotiables (undoing these re-breaks known bugs)

- **No hardcoded repo paths.** race_stack's `get_data_path()` resolved to
  `src/race_stack/stack_master`, a directory that does not exist here. Always
  take an explicit `map_dir` / `save_dir` parameter and resolve symlinks back to
  src (`map_io.resolve_source_dir`, `sector_tuner/paths.py`).
- **No `~/.ros` side channels.** Stages communicate through files in the map
  folder. race_stack passed the centerline via `~/.ros/map_centerline.csv`,
  which made the stages impossible to run separately.
- **Start pose is not in the nav2 map yaml.** It lives in `track_meta.yaml`.
  Resolution order: parameter → `track_meta.yaml` → RViz `/initialpose` →
  map origin (warns loudly). race_stack and UNIST both hardcode the origin,
  which silently misplaces s=0 and therefore every sector index.
- **`global_waypoints.json` keeps race_stack's schema** so downstream nodes
  (controller, state machine, sector tuner) work unmodified.
- **No blocking `input()` or blocking matplotlib in a node's main path.**
  RViz is the GUI: `/initialpose` for input, markers for output.
- **Don't edit `.ini` values inside the `{...}` blocks with `#` comments.**
  They are parsed by `json.loads`. Annotations go in the header.

## Map folder is the contract

```
maps/<map>/
  <map>.png <map>.yaml     from mapping (nav2 standard, kept pristine)
  track_meta.yaml          start pose + direction
  centerline.csv           x_m,y_m,w_tr_right_m,w_tr_left_m
  global_waypoints.json    the raceline
  speed_scaling.yaml  ot_sectors.yaml
```

## State estimation

EKF sits **before** cartographer (race_stack's "early fusion"): VESC wheel odom
+ IMU → `/early_fusion/odom` → cartographer's odom input. Cartographer owns TF
(`publish_to_tf = true`); the EKF never touches TF. This is the opposite of
UNIST's arrangement — see the comments in `config/common/mapping_2d.lua`.

IMU goes to the EKF, not to cartographer: `use_imu_data = true` would force
`tracking_frame` to be the IMU frame, because cartographer hard-CHECKs that the
IMU is colocated with the tracking frame and albomb's IMU sits at (0.07, 0, 0.09).

Odometry is a front-end hint only — `POSE_GRAPH.optimization_problem.odometry_*_weight = 0`.
The VESC is sensorless (ERPM from a back-EMF observer, 0.05 m/s deadband) and its
yaw is dead-reckoned from the servo command, so it must never sit in the TF path.

## Known rough edges

- The optimization runs end to end (verified on `26_track_22x8`: IQP converged
  in 22 iterations to 0.0197 rad/m, 6.79 s estimated lap time). Getting there
  needed two third-party fixes, both recorded — the `tph` 0.76 pin in
  `requirements.txt` and the scipy shim in `LOCAL_CHANGES.md`. Do not bump
  either without re-running a full generation.
- `26_inu_track_6x12` is only 0.9 m wide. Default `safety_width: 0.7` leaves
  ±0.10 m; use `safety_width:=0.45` on that map.
- The spline-normals crossing check is commented out upstream by ForzaETH. With
  it off, a raceline can leave the track silently on tight corners. See
  `global_racetrajectory_optimization/LOCAL_CHANGES.md`.
- gym publishes `base_link→laser` at x=0.275 while albomb's URDF says 0.259.
  Known, 16 mm, deliberately left alone — irrelevant to a 2D pipeline.
