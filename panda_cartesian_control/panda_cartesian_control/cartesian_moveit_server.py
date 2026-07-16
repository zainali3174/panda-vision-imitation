import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
)
from shape_msgs.msg import SolidPrimitive

from panda_cartesian_control_msgs.action import HTMMotion


def matrix_to_quaternion(m):
    # m is a 3x3 rotation matrix as nested list [[m00,m01,m02],[m10,m11,m12],[m20,m21,m22]]
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
        self.ee_link = 'panda_link8'  # change to panda_hand_tcp if gripper attached
        self.group_name = 'panda_arm'

        self._move_client = ActionClient(self, MoveGroup, 'move_action')

        self._server = ActionServer(
            self, HTMMotion, 'htm_motion', self.execute_callback)

        self.get_logger().info('HTM MoveIt action server ready.')

    def htm_to_pose_stamped(self, htm):
        # htm is a flat list of 16 floats, row-major 4x4
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

    def build_goal(self, pose: PoseStamped, v_scale: float) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = v_scale
        goal.request.max_acceleration_scaling_factor = v_scale

        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.ee_link

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(pose.pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.base_frame
        ori_constraint.link_name = self.ee_link
        ori_constraint.orientation = pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.05
        ori_constraint.absolute_y_axis_tolerance = 0.05
        ori_constraint.absolute_z_axis_tolerance = 0.05
        ori_constraint.weight = 1.0
        constraints.orientation_constraints.append(ori_constraint)

        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        return goal

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
        move_goal = self.build_goal(target, v_scale if v_scale > 0.0 else 0.3)

        self._move_client.wait_for_server()
        send_future = self._move_client.send_goal_async(move_goal)
        rclpy.spin_until_future_complete(self, send_future)
        move_goal_handle = send_future.result()

        if not move_goal_handle.accepted:
            self.get_logger().error('move_group rejected goal')
            goal_handle.abort()
            result.success = False
            result.error = 'move_group rejected goal'
            return result

        result_future = move_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        move_result = result_future.result().result

        if move_result.error_code.val != 1:
            self.get_logger().error(f'Planning/execution failed, error code {move_result.error_code.val}')
            goal_handle.abort()
            result.success = False
            result.error = f'MoveIt error code {move_result.error_code.val}'
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
