import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import OrientationConstraint
from ament_index_python.packages import get_package_share_directory

from panda_cartesian_control_msgs.action import HTMMotion
from panda_cartesian_control.pinocchio_ik import solver


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


class CartesianMoveitServer(Node):
    def __init__(self):
        super().__init__('cartesian_moveit_server')

        self.base_frame = 'panda_link0'
        self.ee_link = 'panda_link8'
        self.group_name = 'panda_arm'

        self.declare_parameter('joint_states_topic', '/joint_states')
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value

        urdf_path = get_package_share_directory('panda_cartesian_control') + '/urdf/panda_arm.urdf'
        solver.load_model(urdf_path)
        self.joint_names = solver.get_joint_names()
        self.current_q = solver.Q_PREFERRED.copy()

        cb_group = ReentrantCallbackGroup()

        # Tracks the currently in-flight MoveGroup goal handle so a
        # cancel on the HTMMotion goal can be forwarded to it. Only one
        # HTM goal is ever active at a time (single-goal server), so a
        # single attribute is sufficient.
        self._active_move_goal_handle = None
        self._cancel_requested = False

        self._move_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=cb_group)

        self._joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_state_callback, 10,
            callback_group=cb_group)

        self._server = ActionServer(
            self, HTMMotion, 'htm_motion',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group)

        self.get_logger().info(
            f'HTM MoveIt action server ready (Pinocchio IK + CHOMP, fallback to OMPL RRTConnect). '
            f'Listening for joint states on {joint_states_topic}')

    def joint_state_callback(self, msg):
        pos_map = dict(zip(msg.name, msg.position))
        q = list(self.current_q)
        for i, name in enumerate(self.joint_names):
            if name in pos_map:
                q[i] = pos_map[name]
        self.current_q = np.array(q)

    def cancel_callback(self, cancel_request):
        self.get_logger().info('Cancel request received for htm_motion goal.')
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def cancel_active_move_goal(self):
        """Forward the cancel down to whatever MoveGroup goal is currently
        in flight, so the arm actually stops instead of finishing the
        stale trajectory underneath a 'cancelled' HTM goal."""
        if self._active_move_goal_handle is not None:
            self.get_logger().info('Forwarding cancel to active MoveGroup goal.')
            try:
                await self._active_move_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'Failed to cancel MoveGroup goal: {e}')

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

    async def send_move_goal(self, move_goal):
        """Send a MoveGroup goal and await the result.
        Returns (success: bool, error_code: int)."""
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('move_action server not available')
            return False, -2  # server unavailable

        move_goal_handle = await self._move_client.send_goal_async(move_goal)

        if move_goal_handle is None:
            return False, -3  # send_goal_async returned no result
        if not move_goal_handle.accepted:
            return False, -1  # rejected before planning

        # Track so a cancel on the outer HTMMotion goal can be forwarded here.
        self._active_move_goal_handle = move_goal_handle

        wrapped_result = await move_goal_handle.get_result_async()

        self._active_move_goal_handle = None

        if wrapped_result is None:
            return False, -4  # get_result_async returned no result
        move_result = wrapped_result.result

        return (move_result.error_code.val == 1), move_result.error_code.val

    async def move_to_pose(self, pose: PoseStamped, rot, v_scale: float):
        """Solve IK via Pinocchio, then try CHOMP first; fall back to OMPL
        RRTConnect if CHOMP's plan is invalid/fails. Returns (success, error_message)."""
        target_pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        joint_state, err = self.compute_ik_pinocchio(target_pos, rot)
        if joint_state is None:
            return False, err

        if self._cancel_requested:
            return False, 'cancelled before planning'

        # Attempt 1: CHOMP (smooth, consistent, but weaker obstacle avoidance)
        chomp_goal = self.build_joint_goal(joint_state, pose, v_scale, 'chomp', 'CHOMP', use_orientation_constraint=True)
        success, code = await self.send_move_goal(chomp_goal)
        if success:
            self.current_q = np.array(joint_state.position)
            return True, ''

        if self._cancel_requested:
            return False, 'cancelled'

        self.get_logger().warn(
            f'CHOMP failed (error code {code}), falling back to OMPL RRTConnect')

        # Attempt 2: OMPL RRTConnect (reliable global obstacle avoidance)
        ompl_goal = self.build_joint_goal(joint_state, None, v_scale, 'ompl', 'RRTConnectkConfigDefault', use_orientation_constraint=False)
        success, code = await self.send_move_goal(ompl_goal)
        if success:
            self.current_q = np.array(joint_state.position)
            return True, ''

        return False, f'Both CHOMP and OMPL RRTConnect failed, last error code {code}'

    async def execute_callback(self, goal_handle):
        self._cancel_requested = False

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

        success, err = await self.move_to_pose(
            target_pose, target_rot, v_scale if v_scale > 0.0 else 0.3)

        if goal_handle.is_cancel_requested:
            await self.cancel_active_move_goal()
            goal_handle.canceled()
            result.success = False
            result.error = err or 'cancelled'
            return result

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


def main(args=None):
    rclpy.init(args=args)
    node = CartesianMoveitServer()
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