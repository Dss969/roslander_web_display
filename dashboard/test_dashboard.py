"""Offline regressions for freshness and cancellation; no robot commands."""

import time
import threading
import uuid
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import server


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.state = server.DashboardState()
        self.gateway = server.RosGateway.__new__(server.RosGateway)
        self.gateway.nav_lock = threading.RLock()
        self.gateway.active_request_id = None
        self.gateway.active_goal_uuid = None
        self.gateway.goal_handle = None
        self.gateway.cancel_when_accepted = False
        self.gateway.cancel_sent_at = None
        self.gateway.cancel_sequence = 0
        self.gateway.goal_started_at = time.monotonic()
        self.gateway.last_goal_activity = time.monotonic()
        self.state_patch = patch.object(server, "STATE", self.state)
        self.state_patch.start()
        self.addCleanup(self.state_patch.stop)

    def ready(self):
        self.state.navigation["available"] = True
        for key in ("scan", "pose", "odom"):
            self.state.touch(key)

    def active_goal(self, status="executing"):
        goal_id = uuid.uuid4()
        self.gateway.active_request_id = goal_id.hex
        self.gateway.active_goal_uuid = goal_id.bytes
        self.gateway.goal_handle = SimpleNamespace(status=2)
        self.state.navigation["status"] = status
        return goal_id.hex

    @staticmethod
    def feedback():
        return SimpleNamespace(feedback=SimpleNamespace(
            distance_remaining=2, estimated_time_remaining=None, number_of_recoveries=0))

    def test_stale_odometry_blocks_navigation_and_motion_claim(self):
        self.ready()
        self.state.velocity["linear"] = .3
        self.state.freshness["odom"] = time.monotonic() - 10
        data = self.state.snapshot()
        self.assertFalse(data["navigation"]["safe_to_navigate"])
        self.assertFalse(data["robot"]["moving"])
        self.assertFalse(data["robot"]["velocity_fresh"])

    def test_static_map_does_not_expire_and_stale_battery_is_marked(self):
        self.state.map_meta["source"] = "live"
        self.state.freshness["map"] = time.monotonic() - 600
        with patch.object(server, "BATTERY_PROFILE", {}):
            self.gateway._battery(SimpleNamespace(data=11800))
        self.assertIsNone(self.state.battery["percent"])
        self.assertEqual(self.state.battery["voltage"], 11.8)
        self.state.freshness["battery"] = time.monotonic() - 20
        data = self.state.snapshot()
        self.assertTrue(data["map"]["live"])
        self.assertFalse(data["robot"]["battery"]["fresh"])

    def test_lost_scan_blocks_navigation(self):
        self.ready()
        self.assertTrue(self.state.snapshot()["navigation"]["safe_to_navigate"])
        self.state.freshness["scan"] = time.monotonic() - 10
        data = self.state.snapshot()
        self.assertFalse(data["navigation"]["safe_to_navigate"])
        self.assertNotEqual(data["robot"]["status"], "ready")

    def test_battery_estimate_requires_confirmed_cell_count(self):
        profile = {"confirmed": True, "chemistry": "li-ion", "series_cells": 3}
        self.assertEqual(server.estimate_battery_percent(12.6, profile), 100)
        self.assertEqual(server.estimate_battery_percent(11.4, profile), 50)
        self.assertEqual(server.estimate_battery_percent(9.0, profile), 0)
        self.assertIsNone(server.estimate_battery_percent(11.4, {}))
        self.assertIsNone(server.estimate_battery_percent(None, profile))
        self.assertIsNone(server.estimate_battery_percent(11.4, {**profile, "confirmed": False}))

    def test_diagnostic_from_one_source_cannot_clear_another(self):
        self.gateway.diagnostic_entries = {}
        def report(name, level):
            self.gateway._diagnostics(SimpleNamespace(status=[SimpleNamespace(name=name, level=level, message="test")]))
        report("scan", 2)
        report("camera", 0)
        self.assertEqual([d["name"] for d in self.state.diagnostics], ["scan"])
        report("scan", 0)
        self.assertEqual(self.state.diagnostics, [])

    def test_cancel_rejection_keeps_retry_available(self):
        token = self.active_goal("canceling")
        denied = SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[]))
        self.gateway._cancel_response(denied, token, 0)
        self.assertEqual(self.state.navigation["status"], "stop_failed")

    def test_empty_ekf_statistics_hidden_only_while_real_outputs_arrive(self):
        self.ready()
        self.state.touch("tf")
        self.state.touch("diagnostics")
        self.state.diagnostics = [{"name": "ekf_filter_node: odometry/filtered topic status",
                                  "message": "No events recorded.", "level": 2}]
        self.assertFalse(any(a["message"] == "No events recorded." for a in self.state.snapshot()["alerts"]))
        self.state.freshness["odom"] = time.monotonic() - 10
        self.assertTrue(any(a["message"] == "No events recorded." for a in self.state.snapshot()["alerts"]))
        self.state.touch("odom")
        self.state.diagnostics[0]["message"] = "Unexpected frequency"
        self.assertTrue(any(a["message"] == "Unexpected frequency" for a in self.state.snapshot()["alerts"]))

    def test_feedback_does_not_undo_canceling(self):
        for status in ("canceling", "stop_failed"):
            token = self.active_goal(status)
            self.gateway._navigation_feedback(self.feedback(), token)
            self.assertEqual(self.state.navigation["status"], status)

    def test_late_feedback_cannot_resurrect_completed_goal(self):
        for code in (4, 5, 6):
            token = self.active_goal()
            self.gateway._navigation_result(SimpleNamespace(result=lambda: SimpleNamespace(status=code)), token)
            terminal = self.state.navigation["status"]
            self.gateway._navigation_feedback(self.feedback(), token)
            self.assertEqual(self.state.navigation["status"], terminal)
            self.assertIsNone(self.gateway.goal_handle)
            self.assertNotIn(terminal, server.ACTIVE_NAVIGATION)

    def test_late_old_result_and_cancel_cannot_change_new_goal(self):
        old = self.active_goal()
        self.gateway._finish_navigation(old, 5)
        current = self.active_goal()
        self.gateway._navigation_result(SimpleNamespace(result=lambda: SimpleNamespace(status=4)), old)
        self.gateway._cancel_response(SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[])), old, 0)
        self.assertEqual(self.gateway.active_request_id, current)
        self.assertEqual(self.state.navigation["status"], "executing")

    def test_status_topic_recovers_missing_result_without_releasing_wrong_goal(self):
        token = self.active_goal()
        wrong = SimpleNamespace(goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=list(uuid.uuid4().bytes))), status=5)
        self.gateway._navigation_status(SimpleNamespace(status_list=[wrong]))
        self.assertEqual(self.gateway.active_request_id, token)
        matching = SimpleNamespace(goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=list(self.gateway.active_goal_uuid))), status=5)
        self.gateway._navigation_status(SimpleNamespace(status_list=[matching]))
        self.assertEqual(self.state.navigation["status"], "canceled")
        self.assertIsNone(self.gateway.active_request_id)

    def test_silent_feedback_is_uncertain_not_completed(self):
        self.active_goal()
        self.gateway.last_goal_activity = time.monotonic() - 11
        self.gateway._check_navigation_watchdog()
        self.assertEqual(self.state.navigation["status"], "unconfirmed")
        self.assertIsNotNone(self.gateway.active_request_id)

    def test_silent_cancel_stays_retryable(self):
        self.active_goal("canceling")
        self.gateway.cancel_sent_at = time.monotonic() - 9
        self.gateway._check_navigation_watchdog()
        self.assertEqual(self.state.navigation["status"], "stop_failed")
        self.assertIsNotNone(self.gateway.goal_handle)

    def test_status_terminal_releases_goal_even_when_result_never_arrives(self):
        self.active_goal()
        self.gateway.goal_handle.status = 5
        self.gateway._check_navigation_watchdog()
        self.assertEqual(self.state.navigation["status"], "canceled")

    def test_unknown_status_does_not_reject_running_goal(self):
        self.active_goal()
        self.gateway.goal_handle.status = 0
        self.gateway._check_navigation_watchdog()
        self.assertEqual(self.state.navigation["status"], "executing")


if __name__ == "__main__":
    unittest.main()
