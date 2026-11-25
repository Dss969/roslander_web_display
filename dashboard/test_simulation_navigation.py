"""Fixed-preset navigation contracts; no live ROS or movement in these tests."""
from concurrent.futures import Future
import hashlib
import http.client
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import server
from simulation import SimulationState


class SimulationNavigationTests(unittest.TestCase):
    def setUp(self):
        self.state = SimulationState(Path(__file__).resolve().parents[1] / 'maps/map_01.yaml', navigation_enabled=True)
        self.state.update_clock(30.)
        self.state.update_gazebo(1., False)
        self.state.map_matches_navigation = True
        self.state.navigation['available'] = True
        self.state.pose = dict(x=.7, y=-3.3, yaw=0.)
        for key in ('pose', 'tf', 'odom', 'scan', 'nav_status'):
            self.state.touch(key)
        self.gateway = server.RosGateway.__new__(server.RosGateway)
        g = self.gateway
        g.simulation = True
        g.nav_lock = threading.RLock()
        g.active_request_id = g.active_goal_uuid = g.goal_handle = None
        g.cancel_when_accepted = False
        g.cancel_sent_at = None
        g.cancel_sequence = 0
        g.nav_client = Mock()
        g.nav_client.server_is_ready.return_value = True
        g.nav_client.send_goal_async.return_value = Future()
        g.NavigateToPose = NS(Goal=lambda: NS(pose=NS(header=NS(), pose=NS(position=NS(), orientation=NS()))))
        g.node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=30, nanosec=0))))
        g.UUID = NS
        self.state_patch = patch.object(server, 'STATE', self.state)
        self.state_patch.start()
        self.addCleanup(self.state_patch.stop)

    def test_four_presets_send_exact_positions_and_distinct_yaws(self):
        expected = [('detect', 1.5, -.3, .7071067811865475, .7071067811865476),
                    ('pick1', .95, -3.1, 0., 1.),
                    ('place', 1.28, -.33, .3007057995042731, .9537169507482269),
                    ('pick2', .95, -3.1, -1., 0.)]
        self.assertEqual(list(self.state.preset_goals), [row[0] for row in expected])
        for place, x, y, z, w in expected:
            self.assertEqual(self.gateway.start_navigation(place, False)[0], 202)
            goal = self.gateway.nav_client.send_goal_async.call_args.args[0]
            self.assertEqual(goal.pose.header.frame_id, 'map')
            self.assertEqual(goal.pose.header.stamp.sec, 30)
            self.assertEqual((goal.pose.pose.position.x, goal.pose.pose.position.y), (x, y))
            self.assertAlmostEqual(goal.pose.pose.orientation.z, z)
            self.assertAlmostEqual(goal.pose.pose.orientation.w, w)
            self.gateway._finish_navigation(self.gateway.active_request_id, 4)

    def test_unrecognized_real_car_preset_is_never_sent(self):
        self.assertEqual(self.gateway.start_navigation('start_test', True)[0], 404)
        self.gateway.nav_client.send_goal_async.assert_not_called()

    def test_pause_wrong_map_stale_pose_scan_or_status_block_new_goal(self):
        for key in ('pose', 'tf', 'scan', 'odom'):
            with self.subTest(stale=key):
                self.state.freshness[key] -= 10
                self.assertEqual(self.gateway.start_navigation('detect', False)[0], 409)
                self.state.touch(key)
        self.state.map_matches_navigation = False
        self.assertEqual(self.gateway.start_navigation('detect', False)[0], 409)
        self.state.map_matches_navigation = True
        self.state.update_gazebo(0., True)
        self.assertEqual(self.gateway.start_navigation('detect', False)[0], 409)
        self.gateway.nav_client.send_goal_async.assert_not_called()

    def test_second_click_cannot_send_duplicate_or_replace_goal(self):
        self.assertEqual(self.gateway.start_navigation('detect', False)[0], 202)
        self.assertEqual(self.gateway.start_navigation('pick1', False)[0], 409)
        self.gateway.nav_client.send_goal_async.assert_called_once()

    def test_external_active_goal_blocks_send_and_cannot_be_canceled(self):
        msg = NS(status_list=[NS(status=2, goal_info=NS(stamp=NS(sec=29,nanosec=0), goal_id=NS(uuid=list(range(16)))))])
        self.gateway._observe_nav_status(msg)
        self.assertFalse(self.state.navigation['can_cancel'])
        self.assertEqual(self.gateway.start_navigation('detect', False)[0], 409)
        self.assertEqual(self.gateway.cancel_navigation()[0], 409)
        self.gateway.nav_client.send_goal_async.assert_not_called()

    def test_owned_goal_observer_and_late_messages_do_not_overwrite_result(self):
        self.gateway.start_navigation('detect', False)
        identity = list(self.gateway.active_goal_uuid)
        def status(code):
            return NS(status_list=[NS(status=code, goal_info=NS(stamp=NS(sec=30,nanosec=0), goal_id=NS(uuid=identity)))])
        self.gateway._observe_nav_status(status(4))
        self.gateway._observe_nav_status(status(2))
        self.assertEqual(self.state.navigation['status'], 'succeeded')
        self.assertEqual(self.state.navigation['active_place']['id'], 'detect')
        self.assertFalse(self.state.navigation['can_cancel'])

    def test_cancel_while_waiting_for_acceptance_is_forwarded_on_accept(self):
        self.gateway.start_navigation('pick2', False)
        self.assertEqual(self.gateway.cancel_navigation()[0], 202)
        handle = NS(accepted=True, status=2, get_result_async=lambda: Future(), cancel_goal_async=Mock(return_value=Future()))
        self.gateway.nav_client.send_goal_async.return_value.set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.state.navigation['status'], 'canceling')

    def test_clock_reset_does_not_unlock_or_hide_current_web_goal(self):
        self.gateway.start_navigation('detect', False)
        token = self.gateway.active_request_id
        self.state.update_clock(1.)
        self.assertEqual(self.state.navigation['status'], 'unconfirmed')
        self.assertEqual(self.gateway.active_request_id, token)
        self.assertTrue(self.state.navigation['can_cancel'])

    def test_map_geometry_frame_and_occupancy_must_all_match(self):
        metadata = self.state.expected_map_meta
        cells = [0] * (metadata['width'] * metadata['height'])
        self.state.expected_map_digest = hashlib.sha256(bytes([1]) * len(cells)).digest()
        self.assertTrue(self.state.matches_navigation_map(metadata, cells, 'map'))
        self.assertFalse(self.state.matches_navigation_map(metadata, cells, 'odom'))
        self.assertFalse(self.state.matches_navigation_map(dict(metadata, origin=[0,0,0]), cells, 'map'))
        cells[0] = 100
        self.assertFalse(self.state.matches_navigation_map(metadata, cells, 'map'))

    def test_http_accepts_ids_only_and_blocks_arbitrary_commands(self):
        with patch.object(server, 'MODE', 'simulation'), patch.object(server, 'ROS_GATEWAY', self.gateway):
            http_server = server.ThreadingHTTPServer(('127.0.0.1',0),server.DashboardHandler)
            thread = threading.Thread(target=http_server.serve_forever,daemon=True)
            thread.start()
            try:
                for route, body, expected in [('/api/navigation/start',{'place_id':'detect','x':99},400),
                                              ('/api/navigation/start',{'place_id':'start_test'},404),
                                              ('/api/navigation/cancel',{'goal_id':'another-task'},400),
                                              ('/api/cmd_vel',{},403),
                                              ('/api/navigation/start',{'place_id':'detect'},202)]:
                    connection = http.client.HTTPConnection('127.0.0.1',http_server.server_port)
                    connection.request('POST',route,body=json.dumps(body),headers={'Content-Type':'application/json'})
                    response=connection.getresponse()
                    self.assertEqual(response.status,expected)
                    response.read(); connection.close()
            finally:
                http_server.shutdown(); http_server.server_close(); thread.join()
