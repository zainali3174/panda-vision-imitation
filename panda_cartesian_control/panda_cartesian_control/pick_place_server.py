import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from panda_cartesian_control_msgs.action import PickPlace, HTMMotion
from franka_msgs.action import Move, Grasp

ACTION_SERVER_TIMEOUT_SEC = 5.0


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')

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

    async def send_htm(self, xyz, v_scale=0.2):
        goal = HTMMotion.Goal()
        goal.htm = self.make_htm(xyz)
        goal.v_scale = v_scale

        if not self._htm_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            return False, 'htm_motion action server not available'

        goal_handle = await self._htm_client.send_goal_async(goal)

        if goal_handle is None:
            return False, 'htm_motion send_goal_async returned no result'
        if not goal_handle.accepted:
            return False, 'htm_motion goal rejected'

        wrapped_result = await goal_handle.get_result_async()
        if wrapped_result is None:
            return False, 'htm_motion get_result_async returned no result'
        result = wrapped_result.result

        if not result.success:
            return False, f'htm_motion failed: {result.error}'
        return True, ''

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

        pick = list(req.pick_xyz)
        place = list(req.place_xyz)
        z_off = req.z_offset

        pick_above = [pick[0], pick[1], pick[2] + z_off]
        place_above = [place[0], place[1], place[2] + z_off]

        # Speeds
        SPEED_FAST = 0.2   # Speed X (High/Travel)
        SPEED_SLOW = 0.05  # Speed Y (Descent/Contact)

        steps = [
            ('opening gripper',     lambda: self.send_gripper_move(0.08)),
            ('approaching pick',    lambda: self.send_htm(pick_above, v_scale=SPEED_FAST)),
            ('descending to pick',  lambda: self.send_htm(pick,       v_scale=SPEED_SLOW)),
            ('grasping',            lambda: self.send_gripper_grasp(req.grasp_width, req.grasp_force)),
            ('lifting',             lambda: self.send_htm(pick_above, v_scale=SPEED_FAST)),
            ('approaching place',   lambda: self.send_htm(place_above, v_scale=SPEED_FAST)),
            ('descending to place', lambda: self.send_htm(place,      v_scale=SPEED_SLOW)),
            ('releasing',           lambda: self.send_gripper_move(0.08)),
            ('retreating',          lambda: self.send_htm(place_above, v_scale=SPEED_FAST)),
        ]

        for i, (stage_name, action) in enumerate(steps):
            feedback.stage = stage_name
            feedback.progress = float(i) / len(steps)
            goal_handle.publish_feedback(feedback)

            ok, err = await action()
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()