"""
finish_mapping - close a cartographer mapping run and write the map folder.

Runs alongside cartographer for the whole mapping session and does nothing until
asked. When the operator is done driving:

    ros2 service call /finish_mapping std_srvs/srv/Trigger {}

it finishes the trajectory, lets the pose graph settle, and writes into
`maps/<map_name>/`:

    <map_name>.pbstream    cartographer state, for later SLAM localisation
    <map_name>.png         nav2 occupancy grid
    <map_name>.yaml        nav2 map metadata

Replaces race_stack's `scripts/finish_map.sh`, which called /finish_trajectory
and /write_state from bash and stopped there - it never produced the nav2
png/yaml pair that the rest of this pipeline treats as the map. It also had no
way to know where the map folder was; the path was passed in by hand.

A service rather than a prompt, because a node on the main path must not block
on `input()`, and because mapping is driven from a second terminal anyway.
"""

import os

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

from .paths import is_inside_install_tree, resolve_maps_source_dir

# cartographer_ros_msgs/StatusResponse uses google.rpc codes; 0 is OK.
STATUS_OK = 0

# How long to wait for the pose graph to absorb the final optimisation after the
# trajectory is closed. /finish_trajectory returns as soon as the trajectory is
# marked finished, not when the resulting optimisation has been applied, so
# saving immediately can catch a map that is still mid-update.
SETTLE_SEC = 3.0

SERVICE_TIMEOUT_SEC = 10.0


class FinishMapping(Node):

    def __init__(self):
        super().__init__('finish_mapping')

        self.declare_parameter('map_name', '')
        # `$(find-pkg-share stack_master)/maps` from the launch file. Resolved
        # back to src so the map is version controlled rather than written into
        # the install tree - same rule as the raceline stage.
        self.declare_parameter('maps_dir', '')
        # Explicit escape hatch; skips resolution entirely when set.
        self.declare_parameter('map_dir', '')
        self.declare_parameter('trajectory_id', 0)
        self.declare_parameter('save_pbstream', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('image_format', 'png')
        self.declare_parameter('map_mode', 'trinary')
        self.declare_parameter('free_thresh', 0.196)
        self.declare_parameter('occupied_thresh', 0.65)

        self.map_name = self.get_parameter('map_name').value
        if not self.map_name:
            raise RuntimeError('map_name parameter is required')

        # Reentrant: the Trigger callback calls other services and blocks on
        # them. With the default (mutually exclusive) group and a single-threaded
        # executor that deadlocks - the responses can never be processed while
        # the callback holding the thread is waiting for them.
        self.cb_group = ReentrantCallbackGroup()

        self.cli_finish = self.create_client(
            FinishTrajectory, '/finish_trajectory', callback_group=self.cb_group)
        self.cli_write_state = self.create_client(
            WriteState, '/write_state', callback_group=self.cb_group)
        self.cli_save_map = self.create_client(
            SaveMap, '/map_saver/save_map', callback_group=self.cb_group)

        # Held only to answer "is there a map yet?" before doing anything
        # irreversible. cartographer_occupancy_grid_node latches /map, so the
        # first grid it ever publishes arrives here even if it predates this
        # subscription - hence transient local.
        self._last_map = None
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._map_cb,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.cb_group)

        self.create_service(
            Trigger, '/finish_mapping', self._finish_cb, callback_group=self.cb_group)

        self.get_logger().info(
            f'Mapping "{self.map_name}". Drive the track, then run:\n'
            f'    ros2 service call /finish_mapping std_srvs/srv/Trigger {{}}')

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._last_map = msg

    # ------------------------------------------------------------------ paths

    def _resolve_map_dir(self) -> str:
        explicit = self.get_parameter('map_dir').value
        if explicit:
            return explicit

        maps_dir = self.get_parameter('maps_dir').value
        if not maps_dir:
            raise RuntimeError('either map_dir or maps_dir must be set')

        source_maps = resolve_maps_source_dir(maps_dir)
        if is_inside_install_tree(source_maps):
            self.get_logger().warn(
                f'Could not resolve {maps_dir} back to src; writing into the '
                f'install tree at {source_maps}. The map will be lost on the '
                f'next clean build - copy it into src/ before rebuilding.')
        return os.path.join(source_maps, self.map_name)

    # --------------------------------------------------------------- services

    def _call(self, client, request, what: str):
        """Call a service synchronously, with a clear message on failure."""
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            raise RuntimeError(
                f'{what}: service {client.srv_name} is not available. '
                f'Is the mapping launch still running?')
        response = client.call(request)
        if response is None:
            raise RuntimeError(f'{what}: call to {client.srv_name} returned nothing')
        return response

    def _finish_cb(self, _request, response):
        try:
            message = self._run()
            response.success = True
            response.message = message
            self.get_logger().info(message)
        except Exception as exc:  # noqa: BLE001 - reported to the caller
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Finishing the map failed: {exc}')
        return response

    def _run(self) -> str:
        map_dir = self._resolve_map_dir()

        # Pre-flight, before anything irreversible happens. Finishing a
        # trajectory cannot be undone - cartographer will not accept new scans
        # into it afterwards - so discovering at the save step that there is no
        # occupancy grid to save would cost the whole drive. Everything below
        # this point is either reversible or already committed.
        # Counting publishers is not enough: cartographer_occupancy_grid_node
        # advertises /map as soon as it starts, whether or not it has any
        # submaps to put in it. Only an actual message proves there is a map.
        if self._last_map is None:
            raise RuntimeError(
                f'No message has arrived on {self.get_parameter("map_topic").value}, '
                f'so there is no map to save and the trajectory has NOT been '
                f'finished - keep driving. Check that cartographer is receiving '
                f'/scan and that TF from ego_racecar/base_link to the laser frame '
                f'exists.')

        os.makedirs(map_dir, exist_ok=True)

        trajectory_id = int(self.get_parameter('trajectory_id').value)
        self.get_logger().info(f'Finishing trajectory {trajectory_id}...')
        result = self._call(
            self.cli_finish,
            FinishTrajectory.Request(trajectory_id=trajectory_id),
            'finish_trajectory')
        if result.status.code != STATUS_OK:
            # Already-finished is not fatal: it happens when the service is
            # called twice, and the map is still perfectly saveable.
            self.get_logger().warn(
                f'/finish_trajectory returned code {result.status.code}: '
                f'{result.status.message}')

        self.get_logger().info(
            f'Waiting {SETTLE_SEC:.0f}s for the final pose graph optimisation...')
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=SETTLE_SEC))

        written = []

        if self.get_parameter('save_pbstream').value:
            pbstream = os.path.join(map_dir, f'{self.map_name}.pbstream')
            result = self._call(
                self.cli_write_state,
                WriteState.Request(filename=pbstream, include_unfinished_submaps=True),
                'write_state')
            if result.status.code != STATUS_OK:
                raise RuntimeError(
                    f'/write_state failed ({result.status.code}): {result.status.message}')
            written.append(os.path.basename(pbstream))

        # nav2's map_saver writes <map_url>.png and <map_url>.yaml, and puts the
        # image basename in the yaml's `image:` field - which is exactly the
        # maps/<map>/<map>.{png,yaml} layout the rest of the pipeline expects.
        map_url = os.path.join(map_dir, self.map_name)
        result = self._call(
            self.cli_save_map,
            SaveMap.Request(
                map_topic=self.get_parameter('map_topic').value,
                map_url=map_url,
                image_format=self.get_parameter('image_format').value,
                map_mode=self.get_parameter('map_mode').value,
                free_thresh=float(self.get_parameter('free_thresh').value),
                occupied_thresh=float(self.get_parameter('occupied_thresh').value)),
            'save_map')
        if not result.result:
            raise RuntimeError(
                '/map_saver/save_map failed. Is anything publishing '
                f'{self.get_parameter("map_topic").value}?')
        written.extend([f'{self.map_name}.png', f'{self.map_name}.yaml'])

        return (f'Wrote {", ".join(written)} to {map_dir}. '
                f'Next: ros2 launch stack_master raceline_generator.launch.xml '
                f'map:={self.map_name}')


def main(args=None):
    rclpy.init(args=args)
    node = FinishMapping()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
