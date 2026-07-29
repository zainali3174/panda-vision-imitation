import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.srv import GetPositionIK

from panda_cartesian_control_msgs.action import HTMMotion


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

        self._move_client = ActionClient(self, MoveGroup, 'move_action')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self._server = ActionServer(
            self, HTMMotion, 'htm_motion', self.execute_callback)

        self.get_logger().info(
            'HTM MoveIt action server ready (KDL IK + CHOMP, fallback to OMPL RRTConnect).')

    def htm_to_pose_stamped(self, htm):
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
        return pose

    def compute_ik(self, pose_stamped):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout.sec = 1

        self._ik_client.wait_for_service()
        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result.error_code.val != 1:
            return None, f'IK failed, error code {result.error_code.val}'

        return result.solution.joint_state, ''

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
        self._move_client.wait_for_server()
        send_future = self._move_client.send_goal_async(move_goal)
        rclpy.spin_until_future_complete(self, send_future)
        move_goal_handle = send_future.result()

        if not move_goal_handle.accepted:
            return False, -1  # rejected before planning

        result_future = move_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        move_result = result_future.result().result

        return (move_result.error_code.val == 1), move_result.error_code.val

    def move_to_pose(self, pose: PoseStamped, v_scale: float):
        """Compute IK once, then try CHOMP first; fall back to OMPL RRTConnect
        if CHOMP's plan is invalid/fails. Returns (success, error_message)."""
        joint_state, err = self.compute_ik(pose)
        if joint_state is None:
            return False, err

        # Attempt 1: CHOMP (smooth, consistent, but weaker obstacle avoidance)
        chomp_goal = self.build_joint_goal(joint_state,pose,v_scale,'chomp','CHOMP',use_orientation_constraint=True)
        success, code = self.send_move_goal(chomp_goal)
        if success:
            return True, ''

        self.get_logger().warn(
            f'CHOMP failed (error code {code}), falling back to OMPL RRTConnect')

        # Attempt 2: OMPL RRTConnect (reliable global obstacle avoidance)
        ompl_goal = self.build_joint_goal(joint_state,None,v_scale,'ompl','RRTConnectkConfigDefault',use_orientation_constraint=False)
        success, code = self.send_move_goal(ompl_goal)
        if success:
            return True, ''

        return False, f'Both CHOMP and OMPL RRTConnect failed, last error code {code}'

    def execute_callback(self, goal_handle):
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

        target = self.htm_to_pose_stamped(htm)

        success, err = self.move_to_pose(target, v_scale if v_scale > 0.0 else 0.3)

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
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
