#!/usr/bin/env python3
"""Start/stop the unified web entry; never start or stop Gazebo/Nav2."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import urllib.request

import dashboard_service as dashboard
import gazebo_graphics as graphics

WS = Path(__file__).resolve().parents[1]
RUNTIME = WS / '.runtime/remote_web'
RECORD = RUNTIME / 'process.json'


def component(script, action):
    subprocess.run(['python3', str(WS / 'scripts' / script), action], cwd=WS, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'stop', 'status'), nargs='?', default='start')
    args = parser.parse_args()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    RUNTIME.chmod(0o700)
    with open(RUNTIME / 'manager.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        record = json.loads(RECORD.read_text()) if RECORD.exists() else None
        if args.action == 'status':
            backend = json.loads(dashboard.RECORD.read_text()) if dashboard.RECORD.exists() else None
            items = json.loads(graphics.STATE.read_text()) if graphics.STATE.exists() else []
            print(json.dumps(dict(web_running=bool(dashboard.alive(record)),
                                  dashboard_running=bool(dashboard.alive(backend)),
                                  graphics_running=bool(items) and all(graphics.live(i) for i in items),
                                  url='http://127.0.0.1:8088/', process=record), indent=2))
            return
        if args.action == 'stop':
            if dashboard.alive(record):
                if Path(f"/proc/{record['pid']}/cwd").resolve() != WS:
                    raise SystemExit('进程工作目录不匹配，不发送信号。')
                os.kill(record['pid'], signal.SIGTERM)
                deadline = time.monotonic() + 10
                while dashboard.alive(record) and time.monotonic() < deadline:
                    time.sleep(.1)
                if dashboard.alive(record):
                    raise SystemExit('网页尚在清理，请检查 gateway.log')
            component('dashboard_service.py', 'stop')
            print('统一网页已停止；Gazebo/Nav2/RViz 和图形会话保持运行。')
            return
        if dashboard.alive(record):
            print('统一网页已运行：http://127.0.0.1:8088/；可用 status 检查各组件。')
            return
        if not (WS / 'install/local_setup.bash').is_file():
            raise SystemExit('尚未编译，请先执行 ./build_workspace.sh。网页启动不会自动编译。')
        password = RUNTIME / 'password.txt'
        if not password.is_file() or not 8 <= len(password.read_text().strip()) <= 256:
            raise SystemExit('请先运行 python3 scripts/set_web_password.py 设置固定网页密码。')
        if not (WS / '.deps/remote/noVNC-1.7.0/core/rfb.js').is_file():
            raise SystemExit('缺少网页图形依赖，请先运行 ./setup_dependencies.sh。')
        password_file = RUNTIME / 'password.txt'
        if not password_file.exists() or not 8 <= len(password_file.read_text().strip()) <= 256:
            raise SystemExit('请先执行 python3 scripts/set_web_password.py，设置固定网页密码。')
        if not (WS / '.deps/remote/noVNC-1.7.0/core/rfb.js').exists():
            raise SystemExit('缺少网页图形依赖，请先执行 ./setup_dependencies.sh。')
        subprocess.run([str(WS / 'start_nav2.sh'), '--check'], cwd=WS, check=True)
        # Check before starting dependencies, so a port conflict has no side effects.
        with socket.socket() as port:
            port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            port.bind(('127.0.0.1', 8088))
        backend = json.loads(dashboard.RECORD.read_text()) if dashboard.RECORD.exists() else None
        had_backend = dashboard.alive(backend)
        items = json.loads(graphics.STATE.read_text()) if graphics.STATE.exists() else []
        had_graphics = bool(items) and all(graphics.live(i) for i in items)
        process = None
        started_graphics = False
        attempted_backend = False
        try:
            if not had_graphics:
                if any(graphics.live(i) for i in items):
                    raise RuntimeError('图形链路部分进程仍在，请检查 graphics status；不会覆盖现有会话。')
                component('gazebo_graphics.py', 'start')
                started_graphics = True
            attempted_backend = True
            component('dashboard_service.py', 'start')
            with open(RUNTIME / 'gateway.log', 'ab', buffering=0) as log:
                process = subprocess.Popen(['python3', '-u', str(WS / 'scripts/remote_gateway.py')],
                    cwd=WS, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
            record = dict(pid=process.pid, starttime=dashboard.starttime(process.pid))
            RECORD.write_text(json.dumps(record, indent=2) + '\n')
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('统一入口启动失败，检查 .runtime/remote_web/gateway.log')
                try:
                    with urllib.request.urlopen('http://127.0.0.1:8088/login', timeout=1) as response:
                        if response.status == 200:
                            print('统一入口：http://127.0.0.1:8088/')
                            print(f'访问密码文件：{RUNTIME / "password.txt"}（在本机终端读取）')
                            return
                except OSError:
                    pass
                time.sleep(.2)
            raise RuntimeError('统一入口就绪超时')
        except BaseException:
            if process and process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
            if attempted_backend and not had_backend:
                component('dashboard_service.py', 'stop')
            if started_graphics:
                component('gazebo_graphics.py', 'stop')
            raise


if __name__ == '__main__':
    main()
