#!/usr/bin/env python3
"""
ps_sequencer.py (pick stack sequencer)

Single external trigger for the whole stacking task. Given a base place
location (x, y, z) and a cube height, it:

  1. Takes a snapshot of whatever objects are currently known from
     detected_objects/robot_frame (published by camera_node).
  2. Sends them to pick_place one at a time, in order, each one's place
     target stacked cube_height higher than the last -- so cube 0 lands
     at place_xyz, cube 1 at place_xyz + cube_height, cube 2 at
     place_xyz + 2*cube_height, and so on. This avoids collisions from
     placing every cube at the same spot.
  3. Waits for each pick_place call to fully complete (success or fail)
     before sending the next.
  4. Reports progress ("cube 2 of 4") as PickStack feedback, and the
     final count of cubes actually stacked as the result.

Each pick_place call is given only the tag id to pick -- pick_place_server
tracks that tag's live position itself, so this node doesn't need to
resnapshot or chase positions during execution.

Uses a ReentrantCallbackGroup + MultiThreadedExecutor because this node
blocks synchronously (via rclpy.spin_until_future_complete) inside its
own action-server callback while waiting on the pick_place action. Under
the default MutuallyExclusiveCallbackGroup + single-threaded spin, that
callback lock would prevent pick_place's own response callback from ever
running -> deadlock. Reentrant + multithreaded avoids that.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from panda_cartesian_control_msgs.action import PickStack, PickPlace
from panda_cartesian_control_msgs.msg import DetectedObjects

ACTION_SERVER_TIMEOUT_SEC = 5.0


class PsSequencer(Node):
    def __init__(self):
        super().__init__('ps_sequencer')

        cb_group = ReentrantCallbackGroup()

        self._latest_objects = []  # most recent DetectedObjects.objects snapshot

        self.create_subscription(
            DetectedObjects, 'detected_objects/robot_frame',
            self.objects_callback, 10, callback_group=cb_group)

        self._pick_place_client = ActionClient(
            self, PickPlace, 'pick_place', callback_group=cb_group)

        self._server = ActionServer(
            self, PickStack, 'pick_stack', self.execute_callback,
            callback_group=cb_group)

        self.get_logger().info('ps_sequencer ready.')

    def objects_callback(self, msg):
        self._latest_objects = list(msg.objects)

    def send_pick_place(self, pick_tag_id, place_xyz, z_offset, grasp_width, grasp_force):
        goal = PickPlace.Goal()
        goal.pick_tag_id = pick_tag_id
        goal.place_xyz = list(place_xyz)
        goal.z_offset = z_offset
        goal.grasp_width = grasp_width
        goal.grasp_force = grasp_force

        if not self._pick_place_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            return False, 'pick_place action server not available'

        send_future = self._pick_place_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None:
            return False, 'pick_place send_goal_async returned no result'
        if not goal_handle.accepted:
            return False, 'pick_place goal rejected'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, 'pick_place get_result_async returned no result'
        result = wrapped_result.result

        if not result.success:
            return False, f'pick_place failed: {result.error}'
        return True, ''

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        result = PickStack.Result()
        feedback = PickStack.Feedback()

        # snapshot currently known objects at goal start -- don't chase a
        # list that keeps changing as cubes get picked up mid-execution
        targets = sorted(self._latest_objects, key=lambda o: o.id)
        total = len(targets)

        if total == 0:
            goal_handle.abort()
            result.success = False
            result.cubes_stacked = 0
            result.error = 'no detected objects to pick'
            return result

        base_xyz = list(req.place_xyz)
        cubes_stacked = 0

        for i, obj in enumerate(targets):
            feedback.stage = f'picking tag {obj.id}'
            feedback.current_cube = i + 1
            feedback.total_cubes = total
            feedback.progress = float(i) / total
            goal_handle.publish_feedback(feedback)

            place_xyz = [base_xyz[0], base_xyz[1], base_xyz[2] + i * req.cube_height]

            ok, err = self.send_pick_place(
                obj.id, place_xyz, req.z_offset, req.grasp_width, req.grasp_force)

            if not ok:
                self.get_logger().error(f'Failed on tag {obj.id}: {err}')
                goal_handle.abort()
                result.success = False
                result.cubes_stacked = cubes_stacked
                result.error = f'Failed on tag {obj.id}: {err}'
                return result

            cubes_stacked += 1

        feedback.stage = 'done'
        feedback.current_cube = total
        feedback.total_cubes = total
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success = True
        result.cubes_stacked = cubes_stacked
        result.error = ''
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PsSequencer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()