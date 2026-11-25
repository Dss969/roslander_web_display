"""Simulation state and read-only Gazebo heartbeat; no robot connections."""
import json
import hashlib
import math
import os
import re
from pathlib import Path
import subprocess
import threading
import time
from io import BytesIO

from PIL import Image
import yaml


def milliseconds():
    return int(time.time() * 1000)


class SimulationState:
    def __init__(self, map_file, navigation_enabled=False):
        self.lock = threading.RLock()
        self.started_at = time.monotonic()
        self.freshness = {}
        self.scan_times = []
        self.pose = None
        self.velocity = {'linear': 0., 'angular': 0.}
        self.path = []
        self.battery = None
        self.diagnostics = []
        self.map_revision = 0
        self.camera_jpeg = None
        self.last_camera_encode = 0.
        self.camera_expected = None
        self.clock_seconds = None
        self.clock_advanced_at = None
        self.clock_resets = 0
        self.pause_started_at = None
        self.gazebo_paused = None
        self.real_time_factor = None
        self.observed_goal_uuid = None
        self.navigation_enabled = navigation_enabled
        self.map_matches_navigation = False
        self.navigation = dict(available=False, status='idle', status_label='等待 Nav2',
                               active_place=None, message='仅监控导航任务', distance_remaining=None,
                               eta_seconds=None, recoveries=0, updated_at=None, goal_id=None,
                               feedback_at=None)
        self.presets = dict(navigation_enabled=False, places=[],
                            notice='仿真监控模式 · 网页不发送或取消导航任务；实车预设点未加载')
        self.preset_goals = {}
        if navigation_enabled:
            config = json.loads((Path(__file__).parent / 'config/simulation_presets.json').read_text())
            if config['map'] != Path(map_file).name or config['frame_id'] != 'map':
                raise ValueError('仿真预设点与地图配置不匹配')
            if [p['id'] for p in config['places']] != ['detect', 'pick1', 'place', 'pick2']:
                raise ValueError('只开放四个指定的仿真预设点')
            for raw_place in config['places']:
                if not all(math.isfinite(raw_place[k]) for k in ('x', 'y', 'yaw_degrees')):
                    raise ValueError('预设坐标必须为有限数值')
                place = dict(raw_place, yaw=math.radians(raw_place['yaw_degrees']), verified=True, enabled=True)
                self.preset_goals[place['id']] = place
            self.presets = dict(navigation_enabled=True, places=list(self.preset_goals.values()), notice=config['notice'])
        # This workspace's known map is a labelled fallback, never the real-car cache.
        raw = yaml.safe_load(Path(map_file).read_text())
        with Image.open(Path(map_file).parent / raw['image']) as image:
            output = BytesIO()
            image.save(output, format='PNG')
            self.map_png = output.getvalue()
            width, height = image.size
            # Same trinary conversion as map_server; compare the actual live map,
            # not its PNG rendering or just its dimensions.
            pixels = image.convert('L').transpose(Image.FLIP_TOP_BOTTOM).getdata()
            def occupancy(pixel):
                probability = pixel / 255. if raw.get('negate', 0) else (255 - pixel) / 255.
                return 100 if probability > raw['occupied_thresh'] else 0 if probability < raw['free_thresh'] else -1
            self.expected_map_digest = hashlib.sha256(bytes(occupancy(p) + 1 for p in pixels)).digest()
        self.map_meta = dict(width=width, height=height, resolution=float(raw['resolution']),
                             origin=list(raw['origin']), source='simulation_config')
        self.expected_map_meta = dict(self.map_meta)

    def matches_navigation_map(self, metadata, occupancy, frame_id):
        expected = self.expected_map_meta
        geometry = (frame_id == 'map' and metadata['width'] == expected['width']
                    and metadata['height'] == expected['height']
                    and abs(metadata['resolution'] - expected['resolution']) < 1e-6
                    and all(abs(a - b) < 1e-6 for a, b in zip(metadata['origin'], expected['origin'])))
        return geometry and hashlib.sha256(bytes(int(v) + 1 for v in occupancy)).digest() == self.expected_map_digest

    def navigation_blocked_reason(self):
        if not self.navigation_enabled:
            return '仿真监控模式 · 网页导航控制未开放'
        if self.simulation_status()['state'] != 'running':
            return '请先恢复仿真运行并等待数据更新'
        if not self.map_matches_navigation:
            return '等待与预设点匹配的实时 map_01 地图'
        # Action status is event-driven, not a periodic heartbeat. An old idle
        # result is valid until the next transition; use the Action client here.
        if not self.navigation['available']:
            return '等待 Nav2 导航服务'
        if self.pose is None or not self.is_fresh('pose', 5.) or not self.is_fresh('tf', 4.):
            return '等待本机仿真定位与 TF 更新'
        if not self.is_fresh('scan', 4.):
            return '激光雷达无数据，已阻止发送目标'
        if not self.is_fresh('odom', 4.):
            return '里程计数据已中断，已阻止发送目标'
        return None

    def touch(self, key):
        with self.lock:
            self.freshness[key] = time.monotonic()

    def age(self, key):
        stamp = self.freshness.get(key)
        return None if stamp is None else time.monotonic() - stamp

    def is_fresh(self, key, seconds):
        age = self.age(key)
        return age is not None and age < seconds

    def update_clock(self, seconds):
        with self.lock:
            reset = self.clock_seconds is not None and seconds < self.clock_seconds - .001
            if reset:
                self.clock_resets += 1
                self.pose = None
                self.velocity = {'linear': 0., 'angular': 0.}
                self.path = []
                self.camera_jpeg = None
                self.scan_times = []
                self.observed_goal_uuid = None
                for key in ('pose', 'tf', 'odom', 'scan', 'camera', 'camera_frame', 'plan', 'nav_feedback'):
                    self.freshness.pop(key, None)
                pending = self.navigation.get('source') == 'web' and self.navigation['status'] in {'sending', 'executing', 'canceling', 'unconfirmed', 'stop_failed'}
                self.navigation.update(status='unconfirmed' if pending else 'idle',
                                       status_label='仿真时间已重置 · 任务待确认' if pending else '仿真时间已重置',
                                       distance_remaining=None, eta_seconds=None, feedback_at=None)
            if self.clock_seconds is None or reset or seconds > self.clock_seconds:
                self.clock_advanced_at = time.monotonic()
            self.clock_seconds = seconds
            self.touch('clock')
            return reset

    def update_gazebo(self, factor, paused):
        with self.lock:
            if paused and self.gazebo_paused is not True:
                self.pause_started_at = time.monotonic()
            if not paused:
                self.pause_started_at = None
            self.gazebo_paused = paused
            self.real_time_factor = factor
            self.touch('gazebo')

    def simulation_status(self):
        stats_live = self.is_fresh('gazebo', 4.)
        clock_live = self.is_fresh('clock', 4.)
        advancing = self.clock_advanced_at is not None and time.monotonic() - self.clock_advanced_at < 2.
        if stats_live and self.gazebo_paused:
            state, label = 'paused', '仿真已暂停'
        elif clock_live and advancing:
            state, label = 'running', '仿真运行中'
        elif self.clock_seconds is None:
            state, label = 'waiting', '等待仿真数据'
        elif clock_live:
            state, label = 'time_stalled', '仿真时间未推进'
        elif stats_live:
            state, label = 'data_disconnected', 'ROS 时钟数据中断'
        else:
            state, label = 'disconnected', '仿真数据已中断'
        return dict(state=state, label=label, paused=state == 'paused',
                    clock_seconds=self.clock_seconds, clock_resets=self.clock_resets,
                    clock_age_seconds=self.age('clock'), gazebo_age_seconds=self.age('gazebo'),
                    real_time_factor=self.real_time_factor if stats_live else None)

    def held_during_pause(self, key, timeout):
        stamp = self.freshness.get(key)
        return (self.pause_started_at is not None and stamp is not None
                and stamp >= self.pause_started_at - timeout)

    def snapshot(self):
        with self.lock:
            sim = self.simulation_status()
            paused = sim['paused']
            online = sim['state'] in ('running', 'paused', 'time_stalled')
            def usable(key, timeout=4.):
                return online and (self.is_fresh(key, timeout) or (paused and self.held_during_pause(key, timeout)))
            localized = self.pose is not None and usable('pose', 5.)
            velocity_live = sim['state'] == 'running' and self.is_fresh('odom', 4.)
            moving = velocity_live and (abs(self.velocity['linear']) > .015 or abs(self.velocity['angular']) > .04)
            sensors = []
            for key, label in [('scan', '仿真激光雷达'), ('camera', '仿真 RGB 相机'),
                               ('odom', '轮速 / IMU 里程计'), ('pose', '地图定位')]:
                if key == 'camera' and self.camera_expected is False:
                    status, detail = 'unavailable', '当前仿真未提供'
                elif paused and usable(key, 5.):
                    status, detail = 'paused', '仿真暂停，保留暂停前数据'
                elif usable(key, 5. if key == 'pose' else 4.):
                    status, detail = 'ok', '正常'
                elif key not in self.freshness:
                    status, detail = 'unknown', '等待仿真数据'
                else:
                    status, detail = 'error', '数据未更新'
                sensors.append(dict(id=key, label=label, status=status, detail=detail))
            sensors.append(dict(id='battery', label='电池 / 实车硬件', status='not_applicable', detail='纯仿真不适用'))
            alerts = []
            if sim['state'] != 'running':
                alerts.append(dict(level='info' if paused else 'warning', title=sim['label'],
                                   message='画面和位置保留暂停前状态；网页连接仍在更新。' if paused else '网页服务仍可访问，请检查本机仿真和 ROS 数据流。'))
            if self.map_meta.get('source') != 'live':
                alerts.append(dict(level='warning', title='等待实时地图', message='当前显示本工作空间 map_01 配置地图，尚未收到本次 /map。'))
            if sim['state'] == 'running':
                for item in sensors[:4]:
                    if item['status'] == 'error':
                        alerts.append(dict(level='warning', title=item['label'] + '数据中断', message=item['detail']))
                if not localized:
                    alerts.append(dict(level='warning', title='等待地图定位', message='尚未收到当前仿真的 map → base_footprint 变换。'))
            if self.is_fresh('diagnostics', 15.) and not paused:
                for item in self.diagnostics[:4]:
                    if (item['name'] == 'ekf_filter_node: odometry/filtered topic status'
                            and item['message'] == 'No events recorded.' and usable('odom') and usable('tf')):
                        continue
                    alerts.append(dict(level='critical' if item['level'] >= 2 else 'warning',
                                       title=item['name'], message=item['message']))
            camera_live = sim['state'] == 'running' and self.is_fresh('camera_frame', 4.)
            camera_held = paused and self.camera_jpeg is not None and self.held_during_pause('camera_frame', 4.)
            blocked = self.navigation_blocked_reason()
            nav = dict(self.navigation, safe_to_navigate=blocked is None, read_only=not self.navigation_enabled,
                       blocked_reason=blocked,
                       feedback_age_seconds=None if self.navigation['feedback_at'] is None else round((milliseconds()-self.navigation['feedback_at'])/1000., 1))
            if not self.is_fresh('nav_feedback', 5.) and not paused:
                nav['distance_remaining'] = None
                nav['eta_seconds'] = None
            return dict(
                server_time=milliseconds(), mode='simulation', read_only=not self.navigation_enabled,
                simulation=sim,
                robot=dict(name='ROSLander · 仿真', online=online, status='paused' if paused else 'moving' if moving else 'ready' if localized else 'offline' if not online else 'limited',
                           status_label=sim['label'] if sim['state'] != 'running' else '行驶中' if moving else '仿真已就绪' if localized else '仿真在线 · 待定位',
                           moving=moving, pose=self.pose, localized=localized, pose_held=paused and localized,
                           velocity_fresh=velocity_live, velocity_source='wheel_odometry_and_imu', velocity=dict(self.velocity),
                           battery=None, battery_status='not_applicable'),
                network=dict(reachable=online, latency_ms=None, address='ROS_DOMAIN_ID=03 · 本机回环', checked_at=milliseconds(), label='本机仿真数据'),
                map=dict(self.map_meta, live=self.map_meta.get('source') == 'live', revision=self.map_revision, generation=self.started_at, image_url='/api/map.png'),
                camera=dict(online=camera_live, held=camera_held, available=self.camera_expected is not False,
                            mode='snapshot' if camera_live or camera_held else 'unavailable', image_url='/api/camera.jpg' if camera_live or camera_held else None,
                            transport='Local ROS Image', label='仿真 RGB', status='paused' if camera_held else 'live' if camera_live else 'unavailable' if self.camera_expected is False else 'waiting'),
                sensors=sensors, alerts=alerts, path=list(self.path), presets=self.presets, navigation=nav,
                telemetry=dict(scan_hz=(len(self.scan_times)-1)/(self.scan_times[-1]-self.scan_times[0]) if len(self.scan_times)>1 and self.is_fresh('scan',4.) and self.scan_times[-1]>self.scan_times[0] and not paused else None,
                               ages={k:self.age(k) for k in ('scan','camera','odom','pose','battery')}))


def gazebo_stats_loop(state, stop_event):
    """Read raw WorldStatistics; `gz stats` summaries stop updating when paused."""
    previous = None
    while not stop_event.is_set():
        env = os.environ.copy()
        env['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:11345'
        try:
            topics = subprocess.check_output(['gz','topic','-l'],env=env,text=True,timeout=2,start_new_session=True).splitlines()
            topics = [topic for topic in topics if topic.endswith('/world_stats')]
            if len(topics) != 1:
                if stop_event.wait(2.):
                    return
                continue
            if stop_event.is_set():
                return
            process = subprocess.Popen(['stdbuf', '-oL', 'gz', 'topic', '-e', topics[0]], env=env, stdout=subprocess.PIPE,
                                       stderr=subprocess.DEVNULL, text=True, start_new_session=True)
        except (OSError,subprocess.SubprocessError):
            if stop_event.wait(2.):
                return
            continue
        def stop_monitor(process=process):
            while not stop_event.wait(.2):
                if process.poll() is not None:
                    return
            if process.poll() is None:
                process.terminate()
        watcher = threading.Thread(target=stop_monitor, daemon=True)
        watcher.start()
        try:
            message = []
            for line in process.stdout:
                if line.strip():
                    message.append(line)
                elif message:
                    parsed = parse_world_stats(''.join(message))
                    message = []
                    if parsed:
                        paused, sim, real = parsed
                        factor = 0. if paused else None
                        if not paused and previous and real > previous[1] and sim >= previous[0]:
                            factor = round((sim-previous[0])/(real-previous[1]),3)
                        previous = (sim,real)
                        state.update_gazebo(factor,paused)
                if stop_event.is_set():
                    break
        finally:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            process.stdout.close()
        if stop_event.wait(2.):
            break


def parse_world_stats(message):
    paused = re.search(r'^paused:\s*(true|false)\s*$',message,re.MULTILINE)
    if not paused:
        return None
    def seconds(field):
        block = re.search(r'\b'+field+r'\s*\{([^}]+)\}',message)
        if not block:
            return None
        values = dict(re.findall(r'(sec|nsec):\s*(\d+)',block.group(1)))
        return int(values.get('sec',0)) + int(values.get('nsec',0))/1e9
    sim,real = seconds('sim_time'),seconds('real_time')
    return None if sim is None or real is None else (paused.group(1)=='true',sim,real)
