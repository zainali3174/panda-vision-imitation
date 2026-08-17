import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from panda_cartesian_control_msgs.action import PickPlace, HTMMotion
from franka_msgs.action import Move, Grasp

ACTION_SERVER_TIMEOUT_SEC = 5.0  # FIX: was unbounded wait_for_server() everywhere


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')

        # FIX: no more silent default. Must be passed explicitly at launch
        # (e.g. -p use_sim:=false), so a missing arg fails loud, not silently.
        self.declare_parameter('use_sim', rclpy.Parameter.Type.BOOL)
        use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value

        gripper_ns = '/panda_gripper_sim_node' if use_sim else '/panda_gripper'
        self.get_logger().info(f'use_sim={use_sim}, gripper namespace: {gripper_ns}')

        self._htm_client = ActionClient(self, HTMMotion, 'htm_motion')
        self._gripper_move_client = ActionClient(self, Move, f'{gripper_ns}/move')
        self._gripper_grasp_client = ActionClient(self, Grasp, f'{gripper_ns}/grasp')

        self._server = ActionServer(
            self, PickPlace, 'pick_place', self.execute_callback)

        self.get_logger().info('Pick-and-place action server ready.')

    def make_htm(self, xyz):
        return [
            0.707, -0.707, 0.0, xyz[0],
            -0.707, -0.707, 0.0, xyz[1],
            0.0, 0.0, -1.0, xyz[2],
            0.0, 0.0, 0.0, 1.0,
        ]

    def send_htm(self, xyz, v_scale=0.2):
        goal = HTMMotion.Goal()
        goal.htm = self.make_htm(xyz)
        goal.v_scale = v_scale

        # FIX: bounded wait, fails loud instead of hanging forever
        if not self._htm_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            return False, 'htm_motion action server not available'

        send_future = self._htm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None:
            return False, 'htm_motion send_goal_async returned no result'
        if not goal_handle.accepted:
            return False, 'htm_motion goal rejected'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, 'htm_motion get_result_async returned no result'
        result = wrapped_result.result

        if not result.success:
            return False, f'htm_motion failed: {result.error}'
        return True, ''

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

    def execute_callback(self, goal_handle):
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
            ('approaching pick', lambda: self.send_htm(pick_above)),
            ('descending to pick', lambda: self.send_htm(pick)),
            ('grasping', lambda: self.send_gripper_grasp(req.grasp_width, req.grasp_force)),
            ('lifting', lambda: self.send_htm(pick_above)),
            ('approaching place', lambda: self.send_htm(place_above)),
            ('descending to place', lambda: self.send_htm(place)),
            ('releasing', lambda: self.send_gripper_move(0.08)),
            ('retreating', lambda: self.send_htm(place_above)),
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
    node = PickPlaceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()