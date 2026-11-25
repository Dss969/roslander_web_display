#!/usr/bin/env python3
"""Manage only this workspace's simulation Dashboard, never the simulation."""
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

WS = Path(__file__).resolve().parents[1]
RUNTIME = WS / '.runtime/dashboard'
RECORD = RUNTIME / 'process.json'


def starttime(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except FileNotFoundError:
        return None


def alive(record):
    return record and starttime(record['pid']) == record['starttime']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    args = parser.parse_args()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    RUNTIME.chmod(0o700)
    with open(RUNTIME / 'manager.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        record = json.loads(RECORD.read_text()) if RECORD.exists() else None
        if args.action == 'start':
            if alive(record):
                print('仿真 Dashboard 已运行：http://127.0.0.1:8089/')
                return
            with socket.socket() as port:
                port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                port.bind(('127.0.0.1', 8089))
            with open(RUNTIME / 'server.log', 'ab', buffering=0) as log:
                process = subprocess.Popen(['bash', '--noprofile', '--norc', str(WS / 'scripts/start_dashboard_backend.sh')],
                                           cwd=WS, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
            record = dict(pid=process.pid, starttime=starttime(process.pid))
            RECORD.write_text(json.dumps(record, indent=2) + '\n')
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise SystemExit('Dashboard 启动失败，见 .runtime/dashboard/server.log')
                try:
                    with urllib.request.urlopen('http://127.0.0.1:8089/api/health', timeout=1) as response:
                        health = json.load(response)
                    if health.get('mode') == 'simulation' and health.get('ros_connected'):
                        print('仿真 Dashboard：http://127.0.0.1:8089/；Gazebo/Nav2 独立运行。')
                        return
                except OSError:
                    pass
                time.sleep(.2)
            raise SystemExit('未确认仿真 ROS 网关就绪；检查 .runtime/dashboard/server.log')
        elif args.action == 'stop':
            if alive(record):
                if Path(f"/proc/{record['pid']}/cwd").resolve() != WS:
                    raise SystemExit('进程工作目录不匹配，不发送信号。')
                os.kill(record['pid'], signal.SIGTERM)
                deadline = time.monotonic() + 10
                while alive(record) and time.monotonic() < deadline:
                    time.sleep(.1)
                if alive(record):
                    raise SystemExit('Dashboard 尚在清理，见日志；没有停止仿真。')
            print('Dashboard 已停止；Gazebo/Nav2 和远程图形通道保持独立。')
        else:
            print(json.dumps(dict(running=bool(alive(record)), process=record,
                                  url='http://127.0.0.1:8089/'), indent=2))


if __name__ == '__main__':
    main()
