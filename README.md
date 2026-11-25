# roslander_web_display

用于本地开发和协作的 ROSLander 仿真网页工程。基于 ROS 2 Humble、Gazebo Classic 11 和 Nav2，浏览器可查看地图、相机、导航状态，以及完整可交互的 Gazebo 界面。

当前属于开发中的基线版本。已支持 detect、pick1、place、pick2 四个预设点导航；独立的传感器教学展示 pkg、IMU 加速度积分轨迹及机械臂扩展尚未实现。

本文是本仓库唯一的部署和使用说明。

## 1. 环境要求

- Ubuntu 22.04，x86_64 / amd64，Python 3.10。
- 已安装 ROS 2 Humble，并配置好 ROS 软件源。推荐按 [ROS 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 安装 Humble Desktop。
- 可工作的 OpenGL / EGL 显卡驱动。当前验证环境是 NVIDIA 显卡及 `egl0`；其他显卡、虚拟机和软件渲染环境需要另行验证。
- 首次部署需要联网下载 APT、PyPI 和 GitHub 官方依赖；安装系统包时需要 sudo 权限。

网页里的 Gazebo 依赖 VirtualGL、TurboVNC、websockify 和 noVNC，它们由部署脚本准备。

## 2. 首次部署

### 2.1 获取工程

将仓库克隆或整个源码目录复制到自己的电脑；不要复制其他电脑生成的 `build/`、`install/`、`log/` 和运行记录。

```bash
cd ~/roslander_web_display
```

### 2.2 安装依赖

在已经安装 ROS 2 Humble 的前提下执行：

```bash
./setup_dependencies.sh --system
```

该命令先使用 APT 安装导航、Gazebo、控制器、图像处理及系统工具，再准备工作空间内的图形与 Python 依赖。安装脚本不修改 ROS 软件源，不安装或配置 VPN。

若系统依赖已准备好，只需：

```bash
./setup_dependencies.sh
```

本地依赖下载到 `.deps/`，不会加入 Git。图形依赖使用 `config/dependencies.json` 指定的版本、上游来源及 SHA256 校验值；Ubuntu 包还配置了清华镜像备用地址，下载内容使用同一 SHA256 校验。Python 版本由 `requirements.txt` 固定。重复执行会复用完整依赖，不重新随机配置密码。

本机允许通过私有 `.deps/root/` 补充缺失的 ROS 二进制依赖。这是当前电脑的独立依赖副本，不是 GitHub 源码的一部分；其他电脑应优先使用上面的 `--system` 安装系统 ROS 包。
### 2.3 设置固定网页密码

首次部署如果没有密码，安装脚本会在交互终端提示设置。也可以手动执行：

```bash
python3 scripts/set_web_password.py
```

密码仅保存在 `.runtime/remote_web/password.txt`，权限为 600，不打印、不写入源码、不上传仓库。启动、关闭、重启网页都不会改变密码；密码文件丢失时明确报错。

当前本机迁移已沿用原来的固定网页密码，无需重新设置。

检查依赖：

```bash
./setup_dependencies.sh --check
```

### 2.4 编译一次

```bash
./build_workspace.sh
```

该命令只编译，不启动仿真或网页。首次部署、修改需构建的 ROS 代码、增加 pkg 或调整安装规则后手动执行。

可以把 colcon 参数传进去，例如：

```bash
./build_workspace.sh --packages-up-to roslander_navigation
```

脚本使用 `--symlink-install`。修改已有网页静态文件通常刷新浏览器即可；修改 C++、增加文件或改变包安装规则需要重新构建。运行中的节点仍需重启才能加载代码更新。

## 3. 每次运行

### 3.1 终端 A：启动导航仿真

```bash
cd ~/roslander_web_display
./start_nav2.sh
```

启动 Gazebo、轮速 / IMU 融合、地图、AMCL、Nav2 和 RViz。**该脚本不执行编译。** 未编译时会提示先运行 `./build_workspace.sh`。

地图固定为当前工作空间的 `maps/map_01.yaml`。ROS 域保持 `ROS_DOMAIN_ID=03`，仅本机通信 `ROS_LOCALHOST_ONLY=1`；Gazebo 内部通信也固定为 `127.0.0.1`，避免自动选择 VPN 或其他网卡。

如果不需要桌面 Gazebo 窗口，二选一使用：

```bash
./start_nav2.sh gui:=false
```

如果也不需要桌面 RViz：

```bash
./start_nav2.sh gui:=false rviz:=false
```

`gui:=false` 不影响浏览器专用 Gazebo 界面。不要重复运行多个导航启动命令；检测到已有 gzserver 时，脚本会要求先核对来源。

只检查依赖和当前编译结果、不启动仿真：

```bash
./start_nav2.sh --check
```

### 3.2 终端 B：启动网页

等仿真就绪后：

```bash
cd ~/roslander_web_display
./start_web_dashboard.sh start
./start_web_dashboard.sh status
```

打开 [http://127.0.0.1:8088/](http://127.0.0.1:8088/)，输入固定网页密码。

- “实时监控”：地图、位姿、相机、传感器和导航状态。
- “Gazebo”：真实 Gazebo GUI，支持键鼠、视角、菜单和全屏。
- 四个预设点：点击发送单个 NavigateToPose Action；顶部停止按钮只取消当前网页任务。

网页启动器不编译，也不代替导航脚本启动 gzserver。它管理本工作空间的网页、数据服务和专用图形辅助进程。启动命令返回后，终端 B 可以关闭，终端 A 继续运行。

默认本机 Gazebo 与浏览器 Gazebo 各使用一个 gzclient，视角独立，共享一个仿真世界。多个浏览器共享同一个专用图形会话。

## 4. 停止

先停止网页与数据后端：

```bash
cd ~/roslander_web_display
./start_web_dashboard.sh stop
```

这会保留仿真和专用图形会话。还要关闭浏览器专用图形进程时，再执行：

```bash
python3 scripts/gazebo_graphics.py stop
```

最后在终端 A 按 Ctrl+C 结束导航仿真。不要用广泛的 `killall` 同时停止其他工程。

## 5. 文件与开发分工

```text
roslander_web_display/
├── src/                        五个 ROS 包，后续新增 pkg 也放这里
├── maps/                       固定导航地图
├── dashboard/                  仿真网页前后端与单元测试
├── scripts/                    部署、环境、网页和图形管理
├── config/
│   ├── dependencies.json       图形依赖下载来源、版本及校验值
│   └── graphics.json           渲染设备、分辨率和帧率配置
├── tests/                      本地网页及部署回归检查
├── requirements.txt            Python 依赖版本
├── setup_dependencies.sh        安装 / 检查依赖
├── build_workspace.sh           显式编译
├── start_nav2.sh                启动导航，不编译
├── start_web_dashboard.sh       网页 start / stop / status
├── .gitignore
└── README.md
```

内部 ROS 包名保持不变：`roslander_description`、`roslander_sensors`、`roslander_gazebo_plugins`、`roslander_bt_nodes`、`roslander_navigation`。

重要开发位置：

| 任务 | 位置 |
| --- | --- |
| 导航与行为树 | `src/roslander_navigation/`、`src/roslander_bt_nodes/` |
| 机器人模型与机械扩展 | `src/roslander_description/` |
| 现有仿真传感器插件配置 | `src/roslander_sensors/urdf/roslander_sensors.gazebo.xacro` |
| ROS 数据接入、Action、HTTP API | `dashboard/server.py` |
| 仿真状态、数据时效与地图匹配 | `dashboard/simulation.py` |
| 地图、相机、预设点网页交互 | `dashboard/static/app.js` |
| 网页 Gazebo 连接 | `dashboard/static/gazebo.js` |
| 四个仿真预设点 | `dashboard/config/simulation_presets.json` |
| 固定密码、会话和统一代理 | `scripts/remote_gateway.py` |
| 专用 Gazebo 图形会话 | `scripts/gazebo_graphics.py` |

协作者交付的 ROS 2 pkg 可放入 `src/` 并手动构建；其节点启动、数据 API 和网页接入口仍需双方约定。目前尚未实现任意 pkg 自动注册到网页的机制。

## 6. 显卡与图形配置

默认配置使用 `egl0`、1280×720、30 FPS，要求硬件渲染。启动时会实际运行渲染探测，并保存 `.runtime/gazebo_graphics/renderer.txt`。

机器特定配置可写入 `config/graphics.local.json`，会覆盖默认配置且不进入 Git，例如：

```json
{
  "render_device": "egl0",
  "geometry": "1280x720",
  "fps": 30
}
```

NVIDIA 环境先检查 `nvidia-smi` 和设备访问权限。其他显卡需核对 VirtualGL 可用的 EGL / X 显示设备。配置化不代表已经验证所有显卡；降低分辨率和 FPS 可以减少负载。

## 7. 常见问题

| 现象 | 处理 |
| --- | --- |
| 提示尚未编译 | 手动执行 `./build_workspace.sh`，再启动 |
| 缺少 ROS 包 | 检查 Humble 软件源，执行 `./setup_dependencies.sh --system` |
| 缺少图形或 Python 依赖 | 执行 `./setup_dependencies.sh`；首次下载需要能访问对应站点 |
| PyPI 下载超时 | 核对本机网络；可用 `PIP_INDEX_URL` 指定可信 PyPI 镜像后重新运行安装脚本 |
| 密码未设置 | 执行 `python3 scripts/set_web_password.py` |
| 网页启动要求先运行 gzserver | 先运行本工作空间的 `./start_nav2.sh` |
| 已有 gzserver 或端口 / 显示被占用 | 核对现有进程来源，再用其所属项目的停止方式处理 |
| Gazebo 无法渲染 | 查看 `renderer.txt`、`gzclient.log`，核对驱动和渲染设备 |
| 网页里程计频率诊断提示 | 检查数据是否仍实时更新；不要仅凭诊断文字判断网页故障 |

日志位于 `.runtime/dashboard/`、`.runtime/remote_web/`、`.runtime/gazebo_graphics/`。它们不属于源码，不提交到 GitHub。

端口均只在本机监听：统一网页 8088、内部 Dashboard 8089、图形 WebSocket 8091、VNC 5991。无需开放公网端口。
