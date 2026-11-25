#!/usr/bin/env python3
"""Prepare local graphical/Python dependencies; optionally install system ROS deps."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import urllib.request

WS = Path(__file__).resolve().parents[1]
DEPS = WS / '.deps/remote'
MANIFEST = WS / 'config/dependencies.json'
SYSTEM_PACKAGES = '''build-essential cmake python3-colcon-common-extensions python3-pip
python3-yaml python3-pil python3-numpy python3-opencv python3-cryptography python3-deprecated
xauth x11-utils iproute2 mesa-utils libimlib2 libglu1-mesa libopengl0 libgl1-mesa-dri
ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2
ros-humble-robot-localization ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
ros-humble-gazebo-ros2-control ros-humble-ros2-controllers ros-humble-xacro
ros-humble-robot-state-publisher ros-humble-cv-bridge ros-humble-ament-cmake-gtest'''.split()
ROS_PACKAGES = ['robot_localization', 'nav2_bringup', 'gazebo_ros', 'gazebo_ros2_control',
                'diff_drive_controller', 'joint_state_broadcaster', 'cv_bridge', 'rviz2']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def python_versions_match():
    installed = {dist.metadata['Name'].lower().replace('_', '-'): dist.version
                 for dist in importlib.metadata.distributions(path=[str(DEPS / 'python')])}
    for line in (WS / 'requirements.txt').read_text().splitlines():
        if line.strip() and not line.lstrip().startswith('#'):
            name, version = line.strip().split('==')
            if installed.get(name.lower().replace('_', '-')) != version:
                return False
    return True


def run(command, **options):
    print('+ ' + ' '.join(map(str, command)), flush=True)
    return subprocess.run(command, check=True, cwd=WS, **options)


def ros_check():
    run(['bash', '--noprofile', '--norc', '-c',
         'set -e; source "$1/scripts/workspace_env.sh"; shift; require_ros_packages "$@"; '
         'python3 -c "import rclpy, cv_bridge, cv2, numpy, PIL, yaml"',
         'dependency-check', str(WS), *ROS_PACKAGES])


def check_local():
    files = ['root/opt/TurboVNC/bin/Xvnc', 'root/opt/TurboVNC/bin/vncpasswd',
             'root/opt/VirtualGL/bin/vglrun', 'root/usr/bin/fluxbox',
             'root/usr/bin/glxinfo.x86_64-linux-gnu', 'noVNC-1.7.0/core/rfb.js']
    missing = [str(DEPS / name) for name in files if not (DEPS / name).is_file()]
    missing += [name for name in ('xauth', 'xdpyinfo', 'ss', 'colcon', 'dpkg-deb') if not shutil.which(name)]
    if missing:
        raise RuntimeError('缺少依赖，请运行 ./setup_dependencies.sh：\n' + '\n'.join(missing))
    if not python_versions_match():
        raise RuntimeError('本地 Python 依赖版本不完整，请运行 ./setup_dependencies.sh。')
    run([sys.executable, '-c',
         'import sys; sys.path[:0]=sys.argv[1:]; import aiohttp, websockify',
         str(DEPS / 'python'), str(DEPS / 'root/usr/lib/python3/dist-packages')])
    ros_check()
    password = WS / '.runtime/remote_web/password.txt'
    if not password.is_file() or not 8 <= len(password.read_text().strip()) <= 256:
        raise RuntimeError('请运行 python3 scripts/set_web_password.py 设置固定网页密码。')
    print('依赖检查通过；显卡渲染将在启动图形会话时实际探测。')


def fetch(item):
    cache = DEPS / 'downloads'
    cache.mkdir(parents=True, exist_ok=True)
    destination = cache / item['file']
    if destination.exists() and digest(destination) == item['sha256']:
        return destination
    temporary = destination.with_suffix(destination.suffix + '.part')
    errors = []
    for url in [item['url'], *item.get('mirrors', [])]:
        print('下载依赖：' + url, flush=True)
        request = urllib.request.Request(url, headers={'User-Agent': 'roslander-web-display-setup'})
        try:
            with urllib.request.urlopen(request, timeout=45) as response, temporary.open('wb') as output:
                shutil.copyfileobj(response, output)
            if digest(temporary) != item['sha256']:
                raise RuntimeError('SHA256 校验失败，未使用下载内容：' + item['file'])
            temporary.replace(destination)
            return destination
        except OSError as error:
            errors.append(str(error))
        finally:
            temporary.unlink(missing_ok=True)
    raise RuntimeError('下载失败：' + item['file'] + '\n' + '\n'.join(errors))


def extract_tar(archive):
    with tarfile.open(archive) as bundle:
        for member in bundle.getmembers():
            target = (DEPS / member.name).resolve()
            if not target.is_relative_to(DEPS.resolve()) or member.issym() or member.islnk() or member.isdev():
                raise RuntimeError('归档含不支持的路径或链接：' + member.name)
        bundle.extractall(DEPS)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--system', action='store_true', help='use sudo apt to install OS/ROS prerequisites first')
    parser.add_argument('--check', action='store_true', help='read-only dependency check, no download/install')
    args = parser.parse_args()
    os_info = Path('/etc/os-release').read_text()
    if 'ID=ubuntu\n' not in os_info or 'VERSION_ID="22.04"' not in os_info or platform.machine() != 'x86_64':
        raise RuntimeError('本部署基线支持 Ubuntu 22.04 x86_64 + ROS 2 Humble。')
    if args.check:
        check_local()
        return
    if args.system:
        prefix = [] if os.geteuid() == 0 else ['sudo']
        run(prefix + ['apt-get', 'update'])
        run(prefix + ['apt-get', 'install', '-y', *SYSTEM_PACKAGES])
    if not Path('/opt/ros/humble/setup.bash').is_file():
        raise RuntimeError('请先安装 ROS 2 Humble 并配置其 APT 源，见 README。')
    ros_check()
    DEPS.mkdir(parents=True, exist_ok=True)
    root = DEPS / 'root'
    root.mkdir(exist_ok=True)
    manifest = json.loads(MANIFEST.read_text())
    marker = DEPS / 'installed.json'
    previous = json.loads(marker.read_text()) if marker.exists() else {}
    signature = digest(MANIFEST)
    if (previous.get('manifest_sha256') != signature
            or not (root / 'opt/TurboVNC/bin/Xvnc').exists()
            or not (DEPS / 'noVNC-1.7.0/core/rfb.js').exists()):
        for item in manifest:
            archive = fetch(item)
            if item['kind'] == 'deb':
                run(['dpkg-deb', '-x', str(archive), str(root)])
            else:
                extract_tar(archive)
    python_signature = digest(WS / 'requirements.txt')
    if not python_versions_match():
        run([sys.executable, '-m', 'pip', 'install', '--disable-pip-version-check',
             '--no-compile', '--upgrade', '--target', str(DEPS / 'python'),
             '--requirement', str(WS / 'requirements.txt')])
    password = WS / '.runtime/remote_web/password.txt'
    if not password.exists():
        if not sys.stdin.isatty():
            raise RuntimeError('依赖已安装。请在交互终端运行 python3 scripts/set_web_password.py。')
        run([sys.executable, str(WS / 'scripts/set_web_password.py')])
    check_local()
    marker.write_text(json.dumps(dict(manifest_sha256=signature,
                                     requirements_sha256=python_signature), indent=2) + '\n')
    print('部署依赖已准备。首次运行请执行 ./build_workspace.sh；以后启动不会编译。')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        raise SystemExit(str(error))
