#!/usr/bin/env python3
"""Send the eight competition poses to Nav2 one at a time."""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


WAYPOINTS = (
    (1.50, -0.30, 90.0, 'detect'),
    (0.95, -3.10, 0.0, 'pick1'),
    (1.28, -0.33, 35.0, 'place'),
    (0.95, -3.10, -180.0, 'pick2'),
    (1.28, -0.33, 35.0, 'place'),
    (1.50, -0.30, 0.0, 'recognize_card_1'),
    (0.95, -3.10, -45.0, 'recognize_card_2'),
    (0.95, -3.10, -180.0, 'recognize_card_3'),
)


class EightPointNavigation(Node):
    def __init__(self):
        super().__init__('eight_point_navigation')
        self.declare_parameter('wait_seconds', 2.0)
        self.declare_parameter('map_frame', 'map')
        self._wait_seconds = float(self.get_parameter('wait_seconds').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_state_client = self.create_client(
            GetState, '/bt_navigator/get_state')

    def wait_for_nav2(self):
        self.get_logger().info('Waiting for Nav2 to become active...')
        while rclpy.ok():
            if not self._nav_state_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info('bt_navigator is not available yet')
                continue
            state_future = self._nav_state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, state_future, timeout_sec=2.0)
            response = state_future.result() if state_future.done() else None
            if (response is not None and
                    response.current_state.id == State.PRIMARY_STATE_ACTIVE):
                if self._client.wait_for_server(timeout_sec=2.0):
                    self.get_logger().info('Nav2 is active and ready')
                    return True
            state_label = ('unknown' if response is None
                           else response.current_state.label)
            self.get_logger().info(
                f'Nav2 state is {state_label}; continuing to wait')
        return False

    def navigate(self, index, x, y, yaw_degrees, label):
        yaw = math.radians(yaw_degrees)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'[{index}/{len(WAYPOINTS)}] Sending {label}: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw_degrees:.1f} deg'
        )
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        if not rclpy.ok():
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'Nav2 rejected waypoint {label}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not rclpy.ok():
            return False

        wrapped_result = result_future.result()
        if wrapped_result is None or wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if wrapped_result is None else wrapped_result.status
            self.get_logger().error(
                f'Waypoint {label} failed (action status: {status}); mission stopped'
            )
            return False

        self.get_logger().info(
            f'Arrived at {label}; waiting {self._wait_seconds:g} seconds'
        )
        time.sleep(self._wait_seconds)
        return True

    def run(self):
        if not self.wait_for_nav2():
            return False
        for index, waypoint in enumerate(WAYPOINTS, start=1):
            if not self.navigate(index, *waypoint):
                return False
        self.get_logger().info('All eight waypoints completed')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = EightPointNavigation()
    succeeded = False
    try:
        succeeded = node.run()
    except KeyboardInterrupt:
        node.get_logger().warning('Eight-point navigation interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if succeeded else 1


if __name__ == '__main__':
    sys.exit(main())
