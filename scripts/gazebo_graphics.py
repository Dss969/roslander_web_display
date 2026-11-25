#!/usr/bin/env python3
"""Interactive Gazebo GUI bridge; never owns or stops the simulation server."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import secrets
import shutil
import signal
import socket
import subprocess
import time

WS = Path(__file__).resolve().parents[1]
DEPS = WS / '.deps/remote'
ROOT = DEPS / 'root'
RUNTIME = WS / '.runtime/gazebo_graphics'
STATE = RUNTIME / 'graphics_processes.json'
DISPLAY = ':91'
VNC_PORT = 5991
WEB_PORT = 8091


def graphics_config():
    config = json.loads((WS / 'config/graphics.json').read_text())
    local = WS / 'config/graphics.local.json'
    if local.exists():
        config.update(json.loads(local.read_text()))
    if (not isinstance(config.get('render_device'), str) or not config['render_device']
            or not isinstance(config.get('require_hardware'), bool)
            or not re.fullmatch(r'[1-9]\d{2,3}x[1-9]\d{2,3}', config.get('geometry', ''))
            or type(config.get('fps')) is not int or not 1 <= config['fps'] <= 60):
        raise RuntimeError('config/graphics.json 或 graphics.local.json 配置无效。')
    return config


def starttime(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except (FileNotFoundError, ProcessLookupError):
        return None


def live(item):
    return starttime(item['pid']) == item['starttime']


def save(items):
    temp = STATE.with_suffix('.tmp')
    temp.write_text(json.dumps(items, indent=2) + '\n')
    temp.replace(STATE)


def spawn(name, command, env, items):
    with open(RUNTIME / f'{name}.log', 'ab', buffering=0) as log:
        process = subprocess.Popen(command, cwd=WS, env=env,
                                   stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    item = dict(name=name, pid=process.pid, starttime=starttime(process.pid))
    items.append(item)
    save(items)
    return process


def stop(items):
    for item in reversed(items):
        if live(item):
            os.killpg(item['pid'], signal.SIGINT if item['name'] == 'gzclient' else signal.SIGTERM)
        deadline = time.monotonic() + 6
        while live(item) and time.monotonic() < deadline:
            time.sleep(.1)
        if live(item):
            os.killpg(item['pid'], signal.SIGKILL)
    save([])


def wait_for_gui_render(pid, log_offset):
    """Require master connection and a rendered frame without flooding gz RPC.

    Short-lived `gz topic -i` probes can abort in Gazebo 11 transport cleanup.
    UI subscriptions may also finish only after a viewer exposes the window.
    VirtualGL's own frame statistics verify real rendering before web startup.
    """
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        if starttime(pid) is None:
            raise RuntimeError('专用 gzclient 已退出，见 gzclient.log。')
        data = (RUNTIME / 'gzclient.log').read_bytes()[log_offset:].decode(errors='replace')
        data = re.sub(r'\x1b\[[0-9;]*m', '', data)
        if any(message in data for message in ('Unable to create glx visual', 'GLWidget could not create a scene', 'cannot be preloaded')):
            raise RuntimeError('Gazebo 渲染初始化失败，见 gzclient.log。')
        frames = re.findall(r'Total\s+-\s+[\d.]+\s+Mpixels/sec-\s+([\d.]+)\s+fps', data)
        if 'Connected to gazebo master' in data and frames and float(frames[-1]) > 0:
            (RUNTIME / 'render_ready.json').write_text(json.dumps(dict(
                pid=pid, master_connected=True, rendered_fps=float(frames[-1])), indent=2) + '\n')
            return
        time.sleep(.2)
    raise RuntimeError('未确认 Gazebo 已连接并渲染画面，见 gzclient.log。')


def start(items):
    config = graphics_config()
    if any(live(item) for item in items):
        raise RuntimeError('图形辅助进程已经存在，请先检查 status；不会重复启动。')
    servers = subprocess.run(['pgrep', '-x', 'gzserver'], capture_output=True, text=True).stdout.split()
    if len(servers) != 1:
        raise RuntimeError('需要且只能有一个现有 gzserver；先运行 ./start_nav2.sh gui:=false。')
    if Path(f'/proc/{servers[0]}/cwd').resolve() != WS:
        raise RuntimeError('现有 gzserver 不属于当前工作空间，请核对其来源。')
    if Path('/tmp/.X11-unix/X91').exists() or Path('/tmp/.X91-lock').exists():
        raise RuntimeError('显示 :91 已被占用，不会覆盖其他会话。')
    for port in (VNC_PORT, WEB_PORT):
        with socket.socket() as check:
            check.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            check.bind(('127.0.0.1', port))
    for binary in ('opt/TurboVNC/bin/Xvnc', 'opt/VirtualGL/bin/vglrun', 'usr/bin/fluxbox'):
        if not (ROOT / binary).is_file():
            raise RuntimeError(f'缺少本地依赖: {binary}')
    password = secrets.token_urlsafe(12)[:8]
    password_file = RUNTIME / 'vnc-password.txt'
    password_file.write_text(password + '\n')
    password_file.chmod(0o600)
    encrypted = subprocess.run([str(ROOT / 'opt/TurboVNC/bin/vncpasswd'), '-f'],
                               input=(password + '\n').encode(), capture_output=True,
                               check=True).stdout
    auth = RUNTIME / 'vnc.passwd'
    auth.write_bytes(encrypted)
    auth.chmod(0o600)
    xauth = RUNTIME / 'Xauthority'
    xauth.touch(mode=0o600)
    subprocess.run(['xauth', '-f', str(xauth)],
                   input=f'add {DISPLAY} . {secrets.token_hex(16)}\n', text=True,
                   stdout=subprocess.DEVNULL, check=True)
    env = os.environ.copy()
    env.pop('GAZEBO_HOSTNAME', None)
    env.pop('IGN_RELAY', None)
    env.update(DISPLAY=DISPLAY, XAUTHORITY=str(xauth),
               GAZEBO_IP='127.0.0.1', GAZEBO_MASTER_URI='http://127.0.0.1:11345',
               IGN_IP='127.0.0.1', GZ_IP='127.0.0.1',
               LD_LIBRARY_PATH=str(ROOT / 'usr/lib') + ':' + str(ROOT / 'usr/lib/x86_64-linux-gnu'),
               PYTHONPATH=str(ROOT / 'usr/lib/python3/dist-packages'))
    try:
        items = []
        x = spawn('xvnc', [str(ROOT / 'opt/TurboVNC/bin/Xvnc'), DISPLAY,
                  '-geometry', config['geometry'], '-depth', '24', '-localhost',
                  '-rfbport', str(VNC_PORT), '-rfbauth', str(auth),
                  '-securitytypes', 'VNC', '-noreverse', '-alwaysshared',
                  '-deferupdate', '33', '-auth', str(xauth), '-nolisten', 'tcp',
                  '-noreset', '-desktop', 'Gazebo interactive session'], env, items)
        deadline = time.monotonic() + 15
        while subprocess.run(['xdpyinfo'], env=env, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL).returncode:
            if x.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Xvnc 启动失败，见 xvnc.log')
            time.sleep(.1)
        vgl = ROOT / 'opt/VirtualGL/bin/vglrun'
        libraries = str(ROOT / 'usr/lib') + ':' + str(ROOT / 'usr/lib/x86_64-linux-gnu')
        renderer = subprocess.run([str(vgl), '-ld', libraries, '-d', config['render_device'], '-c', 'proxy',
                                  str(ROOT / 'usr/bin/glxinfo.x86_64-linux-gnu'), '-B'],
                                 env=env, capture_output=True, text=True, timeout=20, check=True)
        (RUNTIME / 'renderer.txt').write_text(renderer.stdout + renderer.stderr)
        if 'OpenGL renderer string:' not in renderer.stdout:
            raise RuntimeError('未取得 OpenGL 渲染器，见 renderer.txt')
        if config['require_hardware'] and any(name in renderer.stdout.lower() for name in ('llvmpipe', 'softpipe', 'software rasterizer')):
            raise RuntimeError('检测到软件渲染，检查显卡驱动和 render_device；见 renderer.txt')
        client_home = RUNTIME / 'client-home'
        client_home.mkdir(exist_ok=True)
        wm_env = env.copy()
        wm_env.update(HOME=str(client_home), XDG_CONFIG_HOME=str(client_home / '.config'))
        wm_config = RUNTIME / 'fluxbox-init'
        shutil.copyfile(WS / 'scripts/graphics/fluxbox-init', wm_config)
        spawn('fluxbox', [str(ROOT / 'usr/bin/fluxbox'), '-rc', str(wm_config)], wm_env, items)
        client_env = env.copy()
        client_env.update(HOME=str(client_home), QT_X11_NO_MITSHM='1',
                          ROS_DOMAIN_ID='03', ROS_LOCALHOST_ONLY='1',
                          GAZEBO_MODEL_DATABASE_URI='',
                          GAZEBO_MODEL_PATH=str(WS / 'install/roslander_description/share') + ':/usr/share/gazebo-11/models',
                          GAZEBO_MASTER_URI='http://127.0.0.1:11345')
        client_log = RUNTIME / 'gzclient.log'
        log_offset = client_log.stat().st_size if client_log.exists() else 0
        client = spawn('gzclient', ['bash', '--noprofile', '--norc', '-c',
              'set -e; source "$1/scripts/workspace_env.sh"; load_workspace; exec "$2" -ld "$5" -d "$3" -c proxy -fps "$4" +pr gzclient --verbose',
              'gazebo-graphics', str(WS), str(vgl), config['render_device'], str(config['fps']), libraries], client_env, items)
        wait_for_gui_render(client.pid, log_offset)
        spawn('websockify', ['python3', '-m', 'websockify',
                            f'127.0.0.1:{WEB_PORT}', f'127.0.0.1:{VNC_PORT}'], env, items)
        time.sleep(1)
        errors = client_log.read_bytes()[log_offset:].decode(errors='replace')
        if any(message in errors for message in ('Unable to create glx visual', 'GLWidget could not create a scene', 'cannot be preloaded')):
            raise RuntimeError('Gazebo 渲染初始化失败，见 .runtime/gazebo_graphics/gzclient.log')
        if not all(live(item) for item in items):
            raise RuntimeError('有图形辅助进程退出，见 .runtime/gazebo_graphics/*.log')
        print(f'内部图形转接已就绪：127.0.0.1:{WEB_PORT}；使用统一网页访问。')
        print(f'凭据文件: {password_file}（仅本机读取，不写入 URL 或日志）')
        print('完整交互模式：键鼠输入已开放；停止图形链路不会停止 Gazebo/Nav2/RViz。')
    except BaseException:
        stop(items)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    args = parser.parse_args()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    RUNTIME.chmod(0o700)
    with open(RUNTIME / 'lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        items = json.loads(STATE.read_text()) if STATE.exists() else []
        if args.action == 'start':
            start(items)
        elif args.action == 'stop':
            stop(items)
            print('仅停止本脚本登记的图形辅助进程；仿真服务保持运行。')
        else:
            print(json.dumps([dict(item, running=live(item)) for item in items], indent=2))


if __name__ == '__main__':
    main()
