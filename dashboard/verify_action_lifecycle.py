#!/usr/bin/env python3
"""Exercise real ROS Action callbacks in an isolated domain, never on the robot.

Run after sourcing ROS: ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 python3 verify_action_lifecycle.py
"""

import os
import threading
import time


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "73" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("This check requires isolated ROS_DOMAIN_ID=73 and ROS_LOCALHOST_ONLY=1")

    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    import server

    server.STATE = server.DashboardState()
    gateway = server.RosGateway()
    fake = Node("dashboard_isolated_fake_navigation")
    complete = threading.Event()

    def execute(goal):
        while not complete.is_set():
            if goal.is_cancel_requested:
                goal.canceled()
                return NavigateToPose.Result()
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = 1.0
            goal.publish_feedback(feedback)
            time.sleep(.05)
        goal.succeed()
        return NavigateToPose.Result()

    action = ActionServer(fake, NavigateToPose, "/navigate_to_pose", execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(gateway.node)
    executor.add_node(fake)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def wait(predicate, label):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.03)
        raise AssertionError(f"Timed out: {label}; state={server.STATE.navigation}")

    def start():
        complete.clear()
        for source in ("odom", "pose", "scan", "tf"):
            server.STATE.touch(source)
        code, payload = gateway.start_navigation("start_test", True)
        assert code == 202, payload
        wait(lambda: server.STATE.navigation["status"] == "executing", "goal accepted")
        return server.STATE.navigation["goal_id"]

    try:
        wait(gateway.nav_client.server_is_ready, "isolated server discovered")
        first = start()
        complete.set()
        wait(lambda: server.STATE.navigation["status"] == "succeeded", "first result")
        second = start()
        code, response = gateway.cancel_navigation()
        assert code == 202, response
        wait(lambda: server.STATE.navigation["status"] == "canceled", "cancel confirmed")
        third = start()
        assert len({first, second, third}) == 3
        complete.set()
        wait(lambda: server.STATE.navigation["status"] == "succeeded", "restart after cancel")
        assert gateway.goal_handle is None
        assert gateway.active_request_id is None
        print("PASS: complete -> new goal -> cancel -> new goal -> complete; 3 unique goal IDs; domain 73 only")
    finally:
        complete.set()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        action.destroy()
        fake.destroy_node()
        gateway.close()


if __name__ == "__main__":
    main()
