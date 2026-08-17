import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from panda_cartesian_control_msgs.action import PickPlace
from panda_cartesian_control_msgs.srv import PickFromCamera

DETECTED_OBJECTS_FILE = "/tmp/detected_objects.json"

BASE_T_CAMERA = np.array([
    [ 0.006804, -0.998319, -0.057565,  0.563716],
    [-0.998902, -0.009454,  0.045875,  0.046939],
    [-0.046342,  0.057190, -0.997287,  1.728049],
    [ 0.0,       0.0,       0.0,       1.0],
])

Z_OFFSET = 0.1
GRASP_WIDTH = 0.05
GRASP_FORCE = 20.0


class PickFromCameraNode(Node):
    def __init__(self):
        super().__init__('pick_from_camera_node')
        self._pick_place_client = ActionClient(self, PickPlace, 'pick_place')
        self._srv = self.create_service(
            PickFromCamera, 'pick_from_camera', self.handle_request)
        self.get_logger().info('pick_from_camera_node ready.')

    def handle_request(self, request, response):
        try:
            with open(DETECTED_OBJECTS_FILE, 'r') as f:
                all_tags = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            response.success = False
            response.error = f'Could not read detection file: {e}'
            return response

        tag_key = str(request.tag_id)
        if tag_key not in all_tags:
            response.success = False
            response.error = f'Tag id {request.tag_id} not yet confirmed/detected'
            return response

        det = all_tags[tag_key]
        cam_point = np.array([det['x'], det['y'], det['z'], 1.0])
        base_point = BASE_T_CAMERA @ cam_point
        pick_xyz = (base_point[:3] + np.array(request.offset)).tolist()

        self.get_logger().info(f'Tag {request.tag_id} -> pick_xyz: {pick_xyz}')

        goal = PickPlace.Goal()
        goal.pick_xyz = pick_xyz
        goal.place_xyz = list(request.place_xyz)
        goal.z_offset = Z_OFFSET
        goal.grasp_width = GRASP_WIDTH
        goal.grasp_force = GRASP_FORCE

        self._pick_place_client.wait_for_server()
        send_future = self._pick_place_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            response.success = False
            response.error = 'pick_place goal rejected'
            return response

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        response.success = result.success
        response.error = result.error
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PickFromCameraNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
