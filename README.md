# bxi_teleop_v2

半醒机器人 Elf3 的 ROS 2 遥操作示例工程。本工程集成了强化学习控制示例、Mujoco 仿真、真机启动、手柄/键盘遥控、Pico 4 Ultra 头显图传以及双臂 IK 遥操作能力。

当前示例使用的 `teleop.onnx` 模型输入维度为 540。相较于完整 960 维模型，该模型移除了 `14 * 3 * 10` 维手臂相关输入。模型主要输出 `0-14` 号电机控制量；`15-28` 号电机控制量由 Pico 手柄姿态经 IK 解算得到，并与模型输出拼接后下发至机器人控制链路。相关代码位于src/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py:299

仓库内的 `BxiPicoApp-release.apk` 目前仅在 Pico 4 Ultra 平台完成测试。

## ⚠️ 重要提示

由于当前下半身 locomotion 模型未直接输出手臂关节控制量，真机运行时可能存在站立稳定性风险。进行真机测试时，请安排人员在旁保护，并确保急停装置处于可用状态。

## 功能概览

- 机器人 RL 控制示例：`bxi_example_py_elf3`
- ROS 2 Mujoco 仿真入口：`example_demo.launch.py`、`example_walk.launch.py`
- 真机启动入口：`example_demo_hw.launch.py`、`example_walk_hw.launch.py`
- 手柄/键盘遥控节点：`remote_controller`
- Pico 图传服务与手柄数据接入：`pico_bxi_server`
- Pico 双臂 IK 与头部控制：`elf3_arm_ikpy_control_pico`

## 目录结构

```text
.
├── BxiPicoApp-release.apk              # Pico 4 Ultra 端应用
├── bin/mediamtx                        # 已随仓库提供的 RTSP 服务程序
├── data/                               # 根目录下的双臂 URDF 与 mesh
├── mediamtx.yml                        # MediaMTX RTSP 配置，默认端口 2212
├── build.sh                            # colcon Release 构建脚本
├── clean.sh                            # 清理 build/install/log
├── push_rtsp.sh                        # 手动启动 MediaMTX + ffmpeg 推流脚本
└── src/
    ├── bxi_rl_controller_ros2_example/ # BXI ROS 2 控制示例、遥控器、策略模型与文档
    ├── elf3_arm_bringup/               # Pico/手臂/灵巧手相关 launch
    ├── elf3_arm_ikpy_control_pico/     # Pico 手柄姿态到双臂 IK 的控制节点
    ├── hand_control/                   # 灵巧手触发器到关节目标的控制节点
    ├── fake_trigger/                   # 调试用虚拟触发器
    ├── pico_bxi_server/                # Pico TCP 数据接收与 RTSP 推流服务
    └── aero-hand-open/                 # 预留目录；当前仓库未包含实现
```

> 注意：`elf3_arm_bringup.launch.py` 会启动 `aero_hand_open`。当前仓库中的 `src/aero-hand-open/` 为空目录；如果未额外安装灵巧手驱动，请使用 `elf3_arm_bringup_nohand.launch.py`。

## 环境要求

推荐运行环境：

- Ubuntu 22.04
- ROS 2 Humble
- `colcon` 构建工具
- BXI ROS 2 基础包：提供 `communication`、`mujoco`、`hardware_elf3` 等依赖包
- `ffmpeg`：用于 Pico 图传推流，仓库未集成
- `libglfw3-dev`：用于 Mujoco 仿真
- `libyaml-cpp-dev`：用于构建 `remote_controller`
- Python 依赖：`numpy`、`scipy`、`matplotlib`、`ikpy`、`onnx`、`onnxruntime`、`PyYAML`
- 可选依赖：`python3-pyqt5`，用于运行 `fake_trigger`

示例安装命令：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ffmpeg \
  libglfw3-dev \
  libyaml-cpp-dev \
  python3-pip \
  python3-pyqt5

python3 -m pip install numpy scipy matplotlib ikpy onnx onnxruntime PyYAML
```

BXI ROS 2 基础包通常部署在 `/opt/bxi/bxi_ros2_pkg`。启动本工程前，需要先加载基础环境：

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
```

真机环境下，请在 root 用户中执行基础包和本工程的环境加载及启动命令。

## 构建

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
bash build.sh
source install/setup.bash
```

清理构建产物：

```bash
bash clean.sh
```

## Pico 图传检查

1. 将 `BxiPicoApp-release.apk` 安装到 Pico 4 Ultra。
2. 确保 Pico 和运行本工程的电脑处于同一局域网。
3. 将摄像头接到电脑的 USB 3.0 口，当前代码默认读取 `/dev/video4`。
4. 启动 Pico 接入和图传服务：

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch elf3_arm_bringup elf3_arm_bringup_nohand.launch.py
```

若出现类似以下日志，表示摄像头已成功推流：

```text
[pico_bxi_server-1] INF [RTSP] [session ...] is publishing to path 'video', 1 track (H264)
```

Pico 应用会扫描局域网内开放 `2212` 端口的设备，并列出可能的 RTSP 服务 IP。使用手柄选择对应 IP 后，即可接入图传画面。需要关闭图传画面时，按右手手柄 `A` 键进入透传模式。

如果未出现推流日志，请优先检查：

- 摄像头是否接入 USB 3.0 端口
- 摄像头设备号是否是 `/dev/video4`
- 当前用户是否具备摄像头访问权限，可临时执行 `sudo chmod 777 /dev/video4`
- `ffmpeg` 是否已安装

也可以使用根目录脚本单独测试推流：

```bash
bash push_rtsp.sh
```

## 启动仿真示例

建议分别打开 3 个终端。每个终端均需先加载运行环境：

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
```

终端 1：启动遥控器。默认使用手柄输入：

```bash
ros2 launch remote_controller remote_controller.launch.py
```

没有手柄时，可以使用键盘输入：

```bash
ros2 launch remote_controller remote_controller_keyboard.launch.py
```

终端 2：启动 Pico、手臂接入和图传服务：

```bash
ros2 launch elf3_arm_bringup elf3_arm_bringup_nohand.launch.py
```

终端 3：启动 Mujoco 仿真和控制策略：

```bash
ros2 launch bxi_example_py_elf3 example_demo.launch.py
```

注意：`example_demo.launch.py` 会同时启动仿真和控制程序。启动后请勿再次按下遥控器上的启动按键，以免重复触发启动流程。

## 启动真机示例

真机操作存在较高风险。部署到真机前，必须先在仿真中验证控制流程。启动前请确认急停、供电、网络、工作空间以及周围环境均处于安全状态。

真机环境建议在 root 用户中运行：

```bash
sudo su
cd /path/to/this/repo
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
```

终端 1：启动遥控器：

```bash
ros2 launch remote_controller remote_controller.launch.py
```

终端 2：启动 Pico、手臂接入和图传服务：

```bash
ros2 launch elf3_arm_bringup elf3_arm_bringup_nohand.launch.py
```

终端 3：启动真机硬件节点和控制策略：

```bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py
```

注意：`example_demo_hw.launch.py` 内部使用 `/tmp/bxi_example_hw.lock` 锁文件避免重复启动硬件控制实例。如果提示已有实例正在运行，请先确认并停止旧实例。

## Pico 校准与遥操流程

1. 启动 `elf3_arm_ikpy_control_pico` 后，将双手举过头顶。
2. 节点提示开始校准后，以两侧肩关节为圆心，尽量伸直双臂并覆盖完整球面轨迹。
3. `elf3_arm_ikpy_control_pico` 显示校准成功后，进入下一步。
4. 使用遥控器让机器人进入 `normal` 站立状态。
5. 按下 `RT + A` 进入遥操状态。
6. 握紧 Pico 手柄抓握键，接入 Pico 遥操作控制。

## 常用 ROS 话题

| 话题 | 方向 | 说明 |
| --- | --- | --- |
| `motion_commands` | `remote_controller` 发布，控制策略订阅 | 手柄或键盘遥控命令 |
| `pico/data` | `pico_bxi_server` 发布 | Pico 原始数据包 |
| `pico/left_trigger`、`pico/right_trigger` | `pico_bxi_server` 发布 | 左右扳机值 |
| `pico/left_grip`、`pico/right_grip` | `pico_bxi_server` 发布 | 左右抓握值 |
| `pico_control_joint_states` | 控制策略发布，Pico IK 订阅 | 当前双臂关节状态 |
| `pico_control_joint_commands` | Pico IK 发布，控制策略订阅 | 双臂 IK 目标关节 |
| `pico_control_head` | Pico IK 发布 | 头部姿态 |
| `pico_motion_commands` | Pico IK 发布 | Pico 手柄映射得到的运动命令 |

## 遥控器配置

默认遥控器配置文件：

```text
src/bxi_rl_controller_ros2_example/src/remote_controller/config/xbox_default.yaml
```

手柄按键说明见：

```text
src/bxi_rl_controller_ros2_example/src/remote_controller/README.md
```

如需扩展输入控制器、添加机器人业务状态、配置状态转移或过渡时间，请参考：

```text
src/bxi_rl_controller_ros2_example/docs/controller_and_state_extension_zh.md
```
<!-- 
## 灵巧手说明

默认示例使用 `elf3_arm_bringup_nohand.launch.py`，不会启动灵巧手驱动。

如果需要启用灵巧手：

1. 补齐并构建 `aero_hand_open`、`aero_hand_open_msgs` 等外部依赖包。
2. 根据实际连接方式修改 `src/elf3_arm_bringup/launch/elf3_arm_bringup.launch.py`：

```python
{'left_port': '/dev/ttyACM0'},
{'right_port': '/dev/ttyACM1'},
{'bluetooth': False}
```

使用蓝牙连接时，请将端口配置改为对应蓝牙地址，并设置：

```python
{'bluetooth': True}
```

3. 启动完整 bringup：

```bash
ros2 launch elf3_arm_bringup elf3_arm_bringup.launch.py
```

`hand_control` 会订阅 Pico 扳机值，并发布左右手 `/left/joint_control`、`/right/joint_control`。 -->

## 常见问题

### 找不到 `communication`、`mujoco` 或 `hardware_elf3`

通常是因为未正确加载 BXI ROS 2 基础包。请先执行：

```bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
```

然后重新构建或启动。

### Pico 端找不到图传 IP

请确认电脑和 Pico 位于同一局域网，且电脑防火墙未拦截 `2212` 端口。也可以在电脑上确认 MediaMTX 是否正在监听：

```bash
ss -lntup | grep 2212
```

### 摄像头无法打开

当前代码默认使用 `/dev/video4`。如果实际设备号不同，请修改以下文件：

```text
src/pico_bxi_server/src/pico_bxi_server.cpp
push_rtsp.sh
```

### 启用 `elf3_arm_bringup.launch.py` 后找不到 `aero_hand_open`

当前仓库未包含 `aero_hand_open` 的实现。未安装灵巧手外部驱动时，请使用：

```bash
ros2 launch elf3_arm_bringup elf3_arm_bringup_nohand.launch.py
```

## 安全提醒

- 真机运行前必须先在仿真中验证控制程序。
- 真机启动时确保急停可用，并保持人员远离机器人活动范围。
- 硬件控制命令丢失、超速、过扭矩或位置越界都可能触发保护。
- 一旦出现异常，立即急停并停止相关 ROS 2 节点。
