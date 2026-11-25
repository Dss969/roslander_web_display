"""Simulation regressions: clock domains, held samples, epochs and API permissions."""
import http.client
import json
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import server
from simulation import SimulationState, parse_world_stats


class SimulationTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.clock = patch('simulation.time.monotonic', side_effect=lambda:self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.state = SimulationState(Path(__file__).resolve().parents[1] / 'maps/map_01.yaml')

    def ready(self):
        self.state.update_clock(30.)
        self.state.update_gazebo(1., False)
        self.state.pose = {'x':.34,'y':-.27,'yaw':0.,'confidence':None}
        self.state.camera_expected = True
        self.state.camera_jpeg = b'frame'
        for key in ('pose','tf','odom','scan','camera','camera_frame'):
            self.state.touch(key)

    def test_fallback_uses_simulation_map_without_real_presets_or_battery(self):
        data=self.state.snapshot()
        self.assertEqual(data['map']['width'],384)
        self.assertAlmostEqual(data['map']['resolution'],.025)
        self.assertEqual(data['map']['source'],'simulation_config')
        self.assertEqual(data['presets']['places'],[])
        self.assertIsNone(data['robot']['battery'])
        self.assertTrue(data['read_only'])

    def test_raw_world_stats_preserve_pause_bit_when_sim_and_real_times_stop(self):
        message='sim_time {\n sec: 30\n nsec: 0\n}\nreal_time {\n sec: 31\n nsec: 250000000\n}\npaused: true\niterations: 30000\n'
        self.assertEqual(parse_world_stats(message),(True,30.,31.25))
        self.assertIsNone(parse_world_stats('partial message'))

    def test_pause_holds_only_previously_valid_samples_without_claiming_live_speed(self):
        self.ready(); self.now=102.; self.state.update_gazebo(0.,True)
        self.now=112.; self.state.update_gazebo(0.,True)
        data=self.state.snapshot()
        self.assertEqual(data['simulation']['state'],'paused')
        self.assertTrue(data['robot']['online'])
        self.assertTrue(data['robot']['pose_held'])
        self.assertFalse(data['robot']['velocity_fresh'])
        self.assertTrue(data['camera']['held'])
        self.assertFalse(data['camera']['online'])
        self.assertFalse(any(s['status']=='error' for s in data['sensors']))

    def test_pause_does_not_revive_samples_already_stale_before_pause(self):
        self.ready(); self.now=110.; self.state.update_gazebo(0.,True)
        data=self.state.snapshot()
        self.assertFalse(data['robot']['localized'])
        self.assertFalse(data['camera']['held'])

    def test_dead_heartbeat_does_not_look_like_permanent_pause(self):
        self.ready(); self.state.update_gazebo(0.,True); self.now=110.
        data=self.state.snapshot()
        self.assertEqual(data['simulation']['state'],'disconnected')
        self.assertFalse(data['robot']['online'])
        self.assertFalse(data['robot']['localized'])

    def test_gazebo_alive_with_missing_ros_clock_is_a_data_disconnect(self):
        self.ready(); self.now=110.; self.state.update_gazebo(1.,False)
        self.assertEqual(self.state.snapshot()['simulation']['state'],'data_disconnected')

    def test_repeated_clock_without_progress_is_not_confirmed_pause(self):
        self.ready(); self.now=103.; self.state.update_clock(30.)
        self.assertEqual(self.state.snapshot()['simulation']['state'],'time_stalled')

    def test_backward_clock_clears_previous_epoch_samples(self):
        self.ready(); self.state.path=[{'x':99,'y':99}]
        self.assertTrue(self.state.update_clock(1.))
        self.assertIsNone(self.state.pose)
        self.assertIsNone(self.state.camera_jpeg)
        self.assertEqual(self.state.path,[])
        self.assertNotIn('pose',self.state.freshness)
        self.assertEqual(self.state.clock_resets,1)

    def test_missing_camera_is_unavailable_without_stream_fallback(self):
        self.ready(); self.state.camera_expected=False; self.state.freshness.pop('camera_frame')
        data=self.state.snapshot()
        self.assertEqual(data['camera']['mode'],'unavailable')
        self.assertIsNone(data['camera']['image_url'])
        self.assertEqual(next(s['status'] for s in data['sensors'] if s['id']=='camera'),'unavailable')

    def test_observed_terminal_goal_cannot_be_resurrected_by_late_feedback(self):
        self.ready(); g=server.RosGateway.__new__(server.RosGateway)
        g.nav_lock=threading.RLock(); g.active_request_id=None
        identity=list(range(16))
        def status(code):
            return NS(status_list=[NS(status=code,goal_info=NS(stamp=NS(sec=29,nanosec=0),goal_id=NS(uuid=identity)))])
        with patch.object(server,'STATE',self.state):
            g._observe_nav_status(status(2)); g._observe_nav_status(status(4))
            g._observe_nav_feedback(NS(goal_id=NS(uuid=identity),feedback=NS(distance_remaining=9,estimated_time_remaining=None,number_of_recoveries=0)))
        self.assertEqual(self.state.navigation['status'],'succeeded')
        self.assertIsNone(self.state.navigation['distance_remaining'])


class SimulationHTTPTests(unittest.TestCase):
    def test_controls_rejected_before_body_parsing_and_real_camera_never_contacted(self):
        state=SimulationState(Path(__file__).resolve().parents[1]/'maps/map_01.yaml')
        with patch.object(server,'STATE',state), patch.object(server,'MODE','simulation'), patch.object(server,'ROS_GATEWAY',None), patch.object(server.urllib.request,'urlopen',side_effect=AssertionError('robot connection attempted')):
            http_server=server.ThreadingHTTPServer(('127.0.0.1',0),server.DashboardHandler)
            thread=threading.Thread(target=http_server.serve_forever,daemon=True);thread.start()
            try:
                for route in ('/api/navigation/start','/api/navigation/cancel','/api/cmd_vel'):
                    connection=http.client.HTTPConnection('127.0.0.1',http_server.server_port)
                    connection.request('POST',route,body=b'not json')
                    response=connection.getresponse(); self.assertEqual(response.status,403)
                    self.assertEqual(json.load(response)['error'],'read_only');connection.close()
                connection=http.client.HTTPConnection('127.0.0.1',http_server.server_port)
                connection.request('GET','/api/camera-stream');response=connection.getresponse()
                self.assertEqual(response.status,404);response.read();connection.close()
            finally:
                http_server.shutdown();http_server.server_close();thread.join()


if __name__=='__main__':
    unittest.main()
