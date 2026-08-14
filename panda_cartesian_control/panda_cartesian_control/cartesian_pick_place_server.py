import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import OrientationConstraint
from ament_index_python.packages import get_package_share_directory

from panda_cartesian_control_msgs.action import HTMMotion, PickPlace
from franka_msgs.action import Move, Grasp
from panda_cartesian_control.pinocchio_ik import solver

ACTION_SERVER_TIMEOUT_SEC = 5.0  # bounded waits everywhere; no more wait_for_server() hangs


def matrix_to_quaternion(m):
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif (m00 > m11) and (m00 > m22):
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    return x, y, z, w


class CartesianPickPlaceServer(Node):
    """
    Combined node: does the job of both the old `cartesian_moveit_server`
    (Pinocchio IK -> CHOMP/OMPL via move_group, exposed as `htm_motion`) and
    the old `pick_place_server` (9-step pick/place choreography, exposed as
    `pick_place`).

    Both action servers stay independently callable:
      ros2 action send_goal /htm_motion   panda_cartesian_control_msgs/action/HTMMotion   "..."
      ros2 action send_goal /pick_place   panda_cartesian_control_msgs/action/PickPlace   "..."

    `pick_place`'s internal sub-motions call `move_to_pose()` directly
    (a plain method call) instead of going back out through the
    `htm_motion` action interface. That avoids a self-referential
    client -> own-action-server call and the threading subtleties that
    come with it, while giving identical motion behavior. A lock guards
    actual motion dispatch so `htm_motion` and `pick_place` can't both be
    mid-move at once.
    """

    def __init__(self):
        super().__init__('cartesian_pick_place_server')

        # ---- shared robot / motion config (from cartesian_moveit_server) ----
        self.base_frame = 'panda_link0'
        self.ee_link = 'panda_link8'
        self.group_name = 'panda_arm'

        self.declare_parameter('joint_states_topic', '/joint_states')
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value

        # No silent default for use_sim: must be passed explicitly at launch
        # (e.g. -p use_sim:=false), so a missing arg fails loud, not silently.
        self.declare_parameter('use_sim', rclpy.Parameter.Type.BOOL)
        use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value

        urdf_path = get_package_share_directory('panda_cartesian_control') + '/urdf/panda_arm.urdf'
        solver.load_model(urdf_path)
        self.joint_names = solver.get_joint_names()
        self.current_q = solver.Q_PREFERRED.copy()

        # Guards self.current_q and the actual move_action dispatch, since
        # htm_motion and pick_place goals can now run concurrently on this
        # one node (each ActionServer goal executes on its own thread).
        self._motion_lock = threading.Lock()

        self._move_client = ActionClient(self, MoveGroup, 'move_action')

        self._joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_state_callback, 10)

        # ---- gripper config (from pick_place_server) ----
        gripper_ns = '/panda_gripper_sim_node' if use_sim else '/panda_gripper'
        self.get_logger().info(f'use_sim={use_sim}, gripper namespace: {gripper_ns}')

        self._gripper_move_client = ActionClient(self, Move, f'{gripper_ns}/move')
        self._gripper_grasp_client = ActionClient(self, Grasp, f'{gripper_ns}/grasp')

        # ---- both action servers, independently callable ----
        # Separate reentrant callback groups so a long-running pick_place
        # goal doesn't block a directly-issued htm_motion goal (or vice
        # versa) from being accepted and processed.
        htm_cb_group = ReentrantCallbackGroup()
        pick_place_cb_group = ReentrantCallbackGroup()

        self._htm_server = ActionServer(
            self, HTMMotion, 'htm_motion', self.execute_htm_motion,
            callback_group=htm_cb_group)

        self._pick_place_server = ActionServer(
            self, PickPlace, 'pick_place', self.execute_pick_place,
            callback_group=pick_place_cb_group)

        self.get_logger().info(
            f'Combined cartesian/pick-place server ready '
            f'(Pinocchio IK + CHOMP, fallback OMPL RRTConnect). '
            f'Listening for joint states on {joint_states_topic}')

    # ------------------------------------------------------------------
    # Shared robot state
    # ------------------------------------------------------------------

    def joint_state_callback(self, msg):
        pos_map = dict(zip(msg.name, msg.position))
        q = list(self.current_q)
        for i, name in enumerate(self.joint_names):
            if name in pos_map:
                q[i] = pos_map[name]
        self.current_q = np.array(q)

    # ------------------------------------------------------------------
    # htm_motion internals (from cartesian_moveit_server, unchanged)
    # ------------------------------------------------------------------

    def htm_to_pose_and_rot(self, htm):
        rot = [
            [htm[0], htm[1], htm[2]],
            [htm[4], htm[5], htm[6]],
            [htm[8], htm[9], htm[10]],
        ]
        x, y, z, w = matrix_to_quaternion(rot)

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = htm[3]
        pose.pose.position.y = htm[7]
        pose.pose.position.z = htm[11]
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose, np.array(rot)

    def compute_ik_pinocchio(self, target_pos, target_rot):
        q_sol, ok, iters = solver.solve_ik(
            np.array(target_pos), np.array(target_rot), q_init=self.current_q)

        if not ok:
            return None, f'Pinocchio IK did not converge after {iters} iterations'

        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = q_sol.tolist()
        return joint_state, ''

    def build_joint_goal(self, joint_state, target_pose, v_scale, pipeline_id, planner_id, use_orientation_constraint=False):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.pipeline_id = pipeline_id
        goal.request.planner_id = planner_id
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = v_scale
        goal.request.max_acceleration_scaling_factor = v_scale

        constraints = Constraints()
        for name, position in zip(joint_state.name, joint_state.position):
            if name.startswith('panda_joint'):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = position
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

        if use_orientation_constraint:
            path_constraints = Constraints()

            oc = OrientationConstraint()
            oc.header.frame_id = self.base_frame
            oc.link_name = self.ee_link

            oc.orientation = target_pose.pose.orientation

            oc.absolute_x_axis_tolerance = 0.05
            oc.absolute_y_axis_tolerance = 0.05
            oc.absolute_z_axis_tolerance = 3.14

            oc.weight = 1.0

            path_constraints.orientation_constraints.append(oc)
            goal.request.path_constraints = path_constraints
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        return goal

    def send_move_goal(self, move_goal):
        """Send a MoveGroup goal and wait for the result.
        Returns (success: bool, error_code: int)."""
        if not self._move_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            self.get_logger().error('move_action server not available')
            return False, -2  # server unavailable

        send_future = self._move_client.send_goal_async(move_goal)
        rclpy.spin_until_future_complete(self, send_future)
        move_goal_handle = send_future.result()

        if move_goal_handle is None:
            return False, -3  # send_goal_async returned no result
        if not move_goal_handle.accepted:
            return False, -1  # rejected before planning

        result_future = move_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, -4  # get_result_async returned no result
        move_result = wrapped_result.result

        return (move_result.error_code.val == 1), move_result.error_code.val

    def move_to_pose(self, pose: PoseStamped, rot, v_scale: float):
        """Solve IK via Pinocchio, then try CHOMP first; fall back to OMPL
        RRTConnect if CHOMP's plan is invalid/fails. Returns (success, error_message).

        Called both from htm_motion's execute callback and directly from
        pick_place's internal sub-motions. Locked so the two action
        servers can't dispatch overlapping motions to move_group."""
        target_pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

        with self._motion_lock:
            joint_state, err = self.compute_ik_pinocchio(target_pos, rot)
            if joint_state is None:
                return False, err

            # Attempt 1: CHOMP (smooth, consistent, but weaker obstacle avoidance)
            chomp_goal = self.build_joint_goal(joint_state, pose, v_scale, 'chomp', 'CHOMP', use_orientation_constraint=True)
            success, code = self.send_move_goal(chomp_goal)
            if success:
                self.current_q = np.array(joint_state.position)
                return True, ''

            self.get_logger().warn(
                f'CHOMP failed (error code {code}), falling back to OMPL RRTConnect')

            # Attempt 2: OMPL RRTConnect (reliable global obstacle avoidance)
            ompl_goal = self.build_joint_goal(joint_state, None, v_scale, 'ompl', 'RRTConnectkConfigDefault', use_orientation_constraint=False)
            success, code = self.send_move_goal(ompl_goal)
            if success:
                self.current_q = np.array(joint_state.position)
                return True, ''

            return False, f'Both CHOMP and OMPL RRTConnect failed, last error code {code}'

    def execute_htm_motion(self, goal_handle):
        htm = goal_handle.request.htm
        v_scale = goal_handle.request.v_scale
        result = HTMMotion.Result()
        feedback = HTMMotion.Feedback()

        if len(htm) != 16:
            self.get_logger().error(f'Expected 16 values for HTM, got {len(htm)}')
            goal_handle.abort()
            result.success = False
            result.error = 'HTM must have 16 values'
            return result

        target_pose, target_rot = self.htm_to_pose_and_rot(htm)

        success, err = self.move_to_pose(target_pose, target_rot, v_scale if v_scale > 0.0 else 0.3)

        if not success:
            self.get_logger().error(f'Motion failed: {err}')
            goal_handle.abort()
            result.success = False
            result.error = err
            return result

        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success = True
        result.error = ''
        return result

    # ------------------------------------------------------------------
    # pick_place internals (from pick_place_server)
    # ------------------------------------------------------------------

    def make_htm(self, xyz):
        return [
            0.707, -0.707, 0.0, xyz[0],
            -0.707, -0.707, 0.0, xyz[1],
            0.0, 0.0, -1.0, xyz[2],
            0.0, 0.0, 0.0, 1.0,
        ]

    def do_htm_motion(self, xyz, v_scale=0.2):
        """In-process equivalent of sending an htm_motion goal: builds the
        same HTM the old code sent over the action interface, then calls
        move_to_pose() directly instead of round-tripping through this
        node's own htm_motion action server. Returns (success, error_msg)."""
        htm = self.make_htm(xyz)
        target_pose, target_rot = self.htm_to_pose_and_rot(htm)
        return self.move_to_pose(target_pose, target_rot, v_scale if v_scale > 0.0 else 0.3)

    def send_gripper_move(self, width, speed=0.1):
        goal = Move.Goal()
        goal.width = width
        goal.speed = speed

        if not self._gripper_move_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            return False, 'gripper move action server not available'

        send_future = self._gripper_move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None:
            return False, 'gripper move send_goal_async returned no result'
        if not goal_handle.accepted:
            return False, 'gripper move goal rejected'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, 'gripper move get_result_async returned no result'
        result = wrapped_result.result

        if not result.success:
            return False, f'gripper move failed: {result.error}'
        return True, ''

    def send_gripper_grasp(self, width, force, speed=0.05, eps=0.01):
        goal = Grasp.Goal()
        goal.width = width
        goal.epsilon.inner = eps
        goal.epsilon.outer = eps
        goal.speed = speed
        goal.force = force

        if not self._gripper_grasp_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            return False, 'gripper grasp action server not available'

        send_future = self._gripper_grasp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None:
            return False, 'gripper grasp send_goal_async returned no result'
        if not goal_handle.accepted:
            return False, 'gripper grasp goal rejected'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, 'gripper grasp get_result_async returned no result'
        result = wrapped_result.result

        if not result.success:
            return False, f'gripper grasp failed: {result.error}'
        return True, ''

    def execute_pick_place(self, goal_handle):
        req = goal_handle.request
        result = PickPlace.Result()
        feedback = PickPlace.Feedback()

        pick = list(req.pick_xyz)
        place = list(req.place_xyz)
        z_off = req.z_offset

        pick_above = [pick[0], pick[1], pick[2] + z_off]
        place_above = [place[0], place[1], place[2] + z_off]

        steps = [
            ('opening gripper', lambda: self.send_gripper_move(0.08)),
            ('approaching pick', lambda: self.do_htm_motion(pick_above)),
            ('descending to pick', lambda: self.do_htm_motion(pick)),
            ('grasping', lambda: self.send_gripper_grasp(req.grasp_width, req.grasp_force)),
            ('lifting', lambda: self.do_htm_motion(pick_above)),
            ('approaching place', lambda: self.do_htm_motion(place_above)),
            ('descending to place', lambda: self.do_htm_motion(place)),
            ('releasing', lambda: self.send_gripper_move(0.08)),
            ('retreating', lambda: self.do_htm_motion(place_above)),
        ]

        for i, (stage_name, action) in enumerate(steps):
            feedback.stage = stage_name
            feedback.progress = float(i) / len(steps)
            goal_handle.publish_feedback(feedback)

            ok, err = action()
            if not ok:
                self.get_logger().error(f'Stage "{stage_name}" failed: {err}')
                goal_handle.abort()
                result.success = False
                result.error = f'Failed at stage "{stage_name}": {err}'
                return result

        goal_handle.succeed()
        result.success = True
        result.error = ''
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPickPlaceServer()

    # MultiThreadedExecutor + the two ReentrantCallbackGroups above: lets
    # a directly-issued htm_motion goal be accepted/processed even while a
    # pick_place goal is mid-sequence, instead of queuing behind it.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
