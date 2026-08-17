import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from panda_cartesian_control_msgs.srv import ObjectToRobot
from panda_cartesian_control_msgs.action import HTMMotion

# baseTcamera - from eye-to-hand calibration
BASE_T_CAMERA = np.array([
    [ 0.006804, -0.998319, -0.057565,  0.563716],
    [-0.998902, -0.009454,  0.045875,  0.046939],
    [-0.046342,  0.057190, -0.997287,  1.728049],
    [ 0.0,       0.0,       0.0,       1.0],
])
# Known-good fixed orientation (pointing down, 45deg about Z)
EE_ROTATION = [
    0.707, -0.707, 0.0,
    -0.707, -0.707, 0.0,
    0.0, 0.0, -1.0,
]


class CameraToRobotNode(Node):
    def __init__(self):
        super().__init__('camera_to_robot_node')
        self._htm_client = ActionClient(self, HTMMotion, 'htm_motion')
        self._srv = self.create_service(
            ObjectToRobot, 'object_to_robot', self.handle_request)
        self.get_logger().info('camera_to_robot_node ready.')

    def handle_request(self, request, response):
        cam_point = np.array([*request.camera_xyz, 1.0])
        base_point = BASE_T_CAMERA @ cam_point
        base_xyz = base_point[:3] + np.array(request.offset)

        response.base_xyz = base_xyz.tolist()

        htm = [
            EE_ROTATION[0], EE_ROTATION[1], EE_ROTATION[2], base_xyz[0],
            EE_ROTATION[3], EE_ROTATION[4], EE_ROTATION[5], base_xyz[1],
            EE_ROTATION[6], EE_ROTATION[7], EE_ROTATION[8], base_xyz[2],
            0.0, 0.0, 0.0, 1.0,
        ]

        goal = HTMMotion.Goal()
        goal.htm = htm
        goal.v_scale = 0.2

        self._htm_client.wait_for_server()
        send_future = self._htm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            response.success = False
            response.error = 'htm_motion goal rejected'
            return response

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        response.success = result.success
        response.error = result.error
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraToRobotNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
