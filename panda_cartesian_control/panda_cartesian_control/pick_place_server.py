import threading
import math

import rclpy
import rclpy.task
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int32

from panda_cartesian_control_msgs.action import PickPlace, HTMMotion
from panda_cartesian_control_msgs.msg import DetectedObjects
from franka_msgs.action import Move, Grasp

ACTION_SERVER_TIMEOUT_SEC = 5.0

# Minimum drift (m) from the currently-commanded target before we
# cancel + resend the htm_motion goal. Keeps 30Hz camera updates from
# causing goal churn on sub-mm jitter. Tune during testing.
RESEND_THRESHOLD_M = 0.05

# Max number of times we'll cancel-and-resend before giving up on
# locking onto a moving/never-settling target.
MAX_TRACKING_RETRIES = 10


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')

        self.declare_parameter('use_sim', rclpy.Parameter.Type.BOOL)
        use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value

        gripper_ns = '/panda_gripper_sim_node' if use_sim else '/panda_gripper'
        self.get_logger().info(f'use_sim={use_sim}, gripper namespace: {gripper_ns}')

        cb_group = ReentrantCallbackGroup()

        self._latest_positions = {}  # tag_id -> [x, y, z]
        self._latest_yaws = {}       # tag_id -> yaw (rad, about base Z)
        self._positions_lock = threading.Lock()

        self._objects_sub = self.create_subscription(
            DetectedObjects, 'detected_objects/robot_frame',
            self.objects_callback, 10, callback_group=cb_group)

        self._tag_picked_pub = self.create_publisher(Int32, 'tag_picked', 10)

        self._htm_client = ActionClient(
            self, HTMMotion, 'htm_motion', callback_group=cb_group)
        self._gripper_move_client = ActionClient(
            self, Move, f'{gripper_ns}/move', callback_group=cb_group)
        self._gripper_grasp_client = ActionClient(
            self, Grasp, f'{gripper_ns}/grasp', callback_group=cb_group)

        self._server = ActionServer(
            self, PickPlace, 'pick_place', self.execute_callback,
            callback_group=cb_group)

        self.get_logger().info('Pick-and-place action server ready.')

    def objects_callback(self, msg):
        with self._positions_lock:
            for obj in msg.objects:
                self._latest_positions[obj.id] = [obj.x, obj.y, obj.z]
                self._latest_yaws[obj.id] = obj.yaw

    def get_position(self, tag_id):
        with self._positions_lock:
            pos = self._latest_positions.get(tag_id)
            return list(pos) if pos is not None else None

    def get_yaw(self, tag_id):
        with self._positions_lock:
            return self._latest_yaws.get(tag_id, 0.0)

    def make_htm(self, xyz, yaw=0.0):
        """Fixed gripper pointing straight down orientation.

        The gripper is rotated in-plane by `yaw` to align its fingers
        with the detected cube orientation.

        Since a parallel-jaw gripper is symmetric under 180° rotation,
        equivalent cube orientations are mapped into [-90°, +90°].
        """

        # Convert yaw to an equivalent angle in [-90°, +90°]
        yaw_deg = math.degrees(yaw)

        while yaw_deg > 90.0:
            yaw_deg -= 180.0

        while yaw_deg < -90.0:
            yaw_deg += 180.0

        yaw = math.radians(yaw_deg)

        c = math.cos(yaw)
        s = math.sin(yaw)

        r00, r01 = 0.707, -0.707
        r10, r11 = -0.707, -0.707

        n00 = c * r00 - s * r10
        n01 = c * r01 - s * r11
        n10 = s * r00 + c * r10
        n11 = s * r01 + c * r11

        return [
            n00, n01, 0.0, xyz[0],
            n10, n11, 0.0, xyz[1],
            0.0, 0.0, -1.0, xyz[2],
            0.0, 0.0, 0.0, 1.0,
        ]

    async def send_htm(self, xyz, v_scale=0.2, yaw=0.0):
        """Send a single htm_motion goal and await completion. Returns
        (success, error, goal_handle) -- goal_handle is returned so a
        caller doing live tracking can cancel it mid-flight."""
        goal = HTMMotion.Goal()
        goal.htm = self.make_htm(xyz, yaw)
        goal.v_scale = v_scale

        if not self._htm_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            return False, 'htm_motion action server not available', None

        goal_handle = await self._htm_client.send_goal_async(goal)

        if goal_handle is None:
            return False, 'htm_motion send_goal_async returned no result', None
        if not goal_handle.accepted:
            return False, 'htm_motion goal rejected', None

        wrapped_result = await goal_handle.get_result_async()
        if wrapped_result is None:
            return False, 'htm_motion get_result_async returned no result', goal_handle
        result = wrapped_result.result

        if not result.success:
            return False, f'htm_motion failed: {result.error}', goal_handle
        return True, '', goal_handle

    async def send_htm_tracking(self, tag_id, base_xyz, z_offset, v_scale=0.2):
        """Move toward the live position of tag_id (with a fixed z_offset
        applied to x/y/z base), cancelling and resending if the tracked
        position drifts past RESEND_THRESHOLD_M while the goal is still
        in flight. Used for the pick-approach/descend stages where the
        target may still move; not used for the place side.

        base_xyz is the offset to apply relative to the tag's live
        position, e.g. [0, 0, z_offset] for the "above" pose or [0, 0, 0]
        for the direct pick pose.
        """
        current_target = self.get_position(tag_id)
        if current_target is None:
            return False, f'no known position for tag {tag_id}'

        retries = 0
        while retries <= MAX_TRACKING_RETRIES:
            commanded = [current_target[i] + base_xyz[i] for i in range(3)]
            current_yaw = self.get_yaw(tag_id)

            goal_handle = await self._htm_client.send_goal_async(
                self._make_htm_goal(commanded, v_scale, current_yaw))

            if goal_handle is None:
                return False, 'htm_motion send_goal_async returned no result'
            if not goal_handle.accepted:
                return False, 'htm_motion goal rejected'

            result_future = goal_handle.get_result_async()

            while not result_future.done():
                await self._sleep(0.05)

                latest = self.get_position(tag_id)
                if latest is None:
                    continue

                drift = max(abs(latest[i] - current_target[i]) for i in range(3))
                if drift > RESEND_THRESHOLD_M:
                    self.get_logger().info(
                        f'Tag {tag_id} drifted {drift:.3f}m, cancelling and resending.')

                    cancel_future = await goal_handle.cancel_goal_async()

                    # Wait for the goal to actually stop, not just for the
                    # cancel request to be accepted -- otherwise the next
                    # goal can be sent while the old trajectory is still
                    # physically executing underneath, which desyncs the
                    # planner's start state from the robot's real state.
                    while not result_future.done():
                        await self._sleep(0.05)

                    current_target = latest
                    retries += 1
                    break
            else:
                # Goal finished on its own without a reposition-cancel.
                wrapped_result = result_future.result()
                if wrapped_result is None:
                    return False, 'htm_motion get_result_async returned no result'
                result = wrapped_result.result
                if not result.success:
                    return False, f'htm_motion failed: {result.error}'
                return True, ''

        return False, f'tracking on tag {tag_id} did not settle after {MAX_TRACKING_RETRIES} retries'

    def _make_htm_goal(self, xyz, v_scale, yaw=0.0):
        goal = HTMMotion.Goal()
        goal.htm = self.make_htm(xyz, yaw)
        goal.v_scale = v_scale
        return goal

    async def _sleep(self, seconds):
        # rclpy coroutines are NOT run on a real asyncio event loop, so
        # asyncio.sleep() raises "no running event loop" here. Use an
        # rclpy Timer + Future instead, which the executor knows how to
        # await correctly.
        future = rclpy.task.Future()

        def on_timer():
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(seconds, on_timer)
        try:
            await future
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    async def send_gripper_move(self, width, speed=0.1):
        goal = Move.Goal()
        goal.width = width
        goal.speed = speed

        if not self._gripper_move_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            return False, 'gripper move action server not available'

        goal_handle = await self._gripper_move_client.send_goal_async(goal)

        if goal_handle is None:
            return False, 'gripper move send_goal_async returned no result'
        if not goal_handle.accepted:
            return False, 'gripper move goal rejected'

        wrapped_result = await goal_handle.get_result_async()
        if wrapped_result is None:
            return False, 'gripper move get_result_async returned no result'
        result = wrapped_result.result

        if not result.success:
            return False, f'gripper move failed: {result.error}'
        return True, ''

    async def send_gripper_grasp(self, width, force, speed=0.05, eps=0.01):
        goal = Grasp.Goal()
        goal.width = width
        goal.epsilon.inner = eps
        goal.epsilon.outer = eps
        goal.speed = speed
        goal.force = force

        if not self._gripper_grasp_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            return False, 'gripper grasp action server not available'

        goal_handle = await self._gripper_grasp_client.send_goal_async(goal)

        if goal_handle is None:
            return False, 'gripper grasp send_goal_async returned no result'
        if not goal_handle.accepted:
            return False, 'gripper grasp goal rejected'

        wrapped_result = await goal_handle.get_result_async()
        if wrapped_result is None:
            return False, 'gripper grasp get_result_async returned no result'
        result = wrapped_result.result

        if not result.success:
            return False, f'gripper grasp failed: {result.error}'
        return True, ''

    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        result = PickPlace.Result()
        feedback = PickPlace.Feedback()

        tag_id = req.pick_tag_id
        place = list(req.place_xyz)
        z_off = req.z_offset

        if self.get_position(tag_id) is None:
            goal_handle.abort()
            result.success = False
            result.error = f'no known position for tag {tag_id}'
            return result

        place_above = [place[0], place[1], place[2] + z_off]

        SPEED_FAST = 0.2
        SPEED_SLOW = 0.05

        # Pick-side stages track the live tag position; place-side stages
        # use the fixed goal-provided place coordinate.
        # tracking_steps = [
        #     ('approaching pick', [0.0, 0.0, z_off], SPEED_FAST),
        #     ('descending to pick', [0.0, 0.0, 0.0], SPEED_SLOW),
        # ]

        # feedback.stage = 'opening gripper'
        # feedback.progress = 0.0
        # goal_handle.publish_feedback(feedback)
        # ok, err = await self.send_gripper_move(0.08)
        # if not ok:
        #     return self._fail(goal_handle, result, 'opening gripper', err)

        # total_steps = len(tracking_steps) + 7
        # step_i = 1

        # for stage_name, offset, v_scale in tracking_steps:
        #     feedback.stage = stage_name
        #     feedback.progress = float(step_i) / total_steps
        #     goal_handle.publish_feedback(feedback)

        #     ok, err = await self.send_htm_tracking(tag_id, offset, z_off, v_scale=v_scale)
        #     if not ok:
        #         return self._fail(goal_handle, result, stage_name, err)
        #     step_i += 1

        feedback.stage = 'opening gripper'
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)
        ok, err = await self.send_gripper_move(0.08)
        if not ok:
            return self._fail(goal_handle, result, 'opening gripper', err)

        total_steps = 2 + 7
        step_i = 1

        # Only the hover approach is live-tracked -- cancel/resend here
        # always stays at safe height (z_off), so a mid-track reposition
        # never causes a horizontal move at grasp height.
        feedback.stage = 'approaching pick'
        feedback.progress = float(step_i) / total_steps
        goal_handle.publish_feedback(feedback)
        ok, err = await self.send_htm_tracking(tag_id, [0.0, 0.0, z_off], z_off, v_scale=SPEED_FAST)
        if not ok:
            return self._fail(goal_handle, result, 'approaching pick', err)
        step_i += 1

        # Descent is a single untracked vertical move using the position
        # the tag had once the hover phase locked in -- no more
        # cancel/resend, so it can't get redirected sideways low down.
        settled_pos = self.get_position(tag_id)
        settled_yaw = self.get_yaw(tag_id)
        if settled_pos is None:
            return self._fail(goal_handle, result, 'descending to pick', f'no known position for tag {tag_id}')

        feedback.stage = 'descending to pick'
        feedback.progress = float(step_i) / total_steps
        goal_handle.publish_feedback(feedback)
        ok, err, _ = await self.send_htm(settled_pos, v_scale=SPEED_SLOW, yaw=settled_yaw)
        if not ok:
            return self._fail(goal_handle, result, 'descending to pick', err)
        step_i += 1

        feedback.stage = 'grasping'
        feedback.progress = float(step_i) / total_steps
        goal_handle.publish_feedback(feedback)
        ok, err = await self.send_gripper_grasp(req.grasp_width, req.grasp_force)
        if not ok:
            return self._fail(goal_handle, result, 'grasping', err)
        step_i += 1

        # Grasp succeeded -- tell camera_node to forget this tag
        # immediately rather than relying on staleness/timeout.
        picked_msg = Int32()
        picked_msg.data = tag_id
        self._tag_picked_pub.publish(picked_msg)

        # From here on the cube is in the gripper: use the last-known
        # pick position (fixed, above the table) purely to compute a
        # safe lift point -- no more tracking, tag is gone from the topic.
        pick_last = self.get_position(tag_id) or [place[0], place[1], place[2]]
        pick_above = [pick_last[0], pick_last[1], pick_last[2] + z_off]

        fixed_steps = [
            ('lifting', lambda: self.send_htm(pick_above, v_scale=SPEED_FAST)),
            ('approaching place', lambda: self.send_htm(place_above, v_scale=SPEED_FAST)),
            ('descending to place', lambda: self.send_htm(place, v_scale=SPEED_SLOW)),
            ('releasing', lambda: self.send_gripper_move(0.08)),
            ('retreating', lambda: self.send_htm(place_above, v_scale=SPEED_FAST)),
        ]

        for stage_name, action in fixed_steps:
            feedback.stage = stage_name
            feedback.progress = float(step_i) / total_steps
            goal_handle.publish_feedback(feedback)

            action_result = await action()
            ok, err = action_result[0], action_result[1]
            if not ok:
                return self._fail(goal_handle, result, stage_name, err)
            step_i += 1

        goal_handle.succeed()
        result.success = True
        result.error = ''
        return result

    def _fail(self, goal_handle, result, stage_name, err):
        self.get_logger().error(f'Stage "{stage_name}" failed: {err}')
        goal_handle.abort()
        result.success = False
        result.error = f'Failed at stage "{stage_name}": {err}'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()