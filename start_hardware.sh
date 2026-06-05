#!/bin/bash
SESSION="teleop"
# ============================================================
# 检查硬件进程和服务
# ============================================================

if [ $EUID -eq 0 ]; then
    echo "当前是 root 用户"
else
    echo "当前不是 root 用户，请先执行sudo su"
    exit 1
fi

if ffmpeg -f v4l2 -i /dev/video4 -frames:v 1 -f image2 -y /tmp/cam_test.jpg 2>/dev/null; then
    echo "✓ 摄像头存在且可用"
    rm -f /tmp/cam_test.jpg
else
    echo "✗ 摄像头不存在或无法访问，请检查是否插入深度相机"
    exit 1
fi

# 检查 hardware_elf3 进程是否存在
if pgrep -x "hardware_elf3" > /dev/null; then
    echo "错误：检测到 hardware_elf3 进程正在运行！"
    echo "请先关闭机器人（停止硬件驱动），然后重新运行此脚本。"
    exit 1
fi

# 检查 ros_elf_launch.service 是否正在运行
if systemctl is-active --quiet ros_elf_launch.service; then
    echo "检测到 ros_elf_launch.service 正在运行，正在停止 remote_controller..."
    sudo killall -9 remote_controller 2>/dev/null
    echo "已停止 remote_controller"
fi

# 检查并安装 tmux
if ! command -v tmux &>/dev/null; then
    echo "tmux not found, installing..."
    sudo apt update && sudo apt install tmux -y
fi

if tmux has-session -t $SESSION 2>/dev/null; then
    echo "错误：tmux 会话 '$SESSION' 已经存在！"
    echo "请先手动关闭现有会话：tmux kill-session -t $SESSION"
    echo "或者手动附加到现有会话：tmux attach-session -t $SESSION"
    exit 1
fi

# -L temp 创建临时套接字，启动新服务器，会继承当前 Shell 的所有环境变量
tmux -L temp new-session -d -s $SESSION

tmux -L temp set-option -g mouse on
tmux -L temp set-option -g pane-border-status top
tmux -L temp set-option -g pane-border-format " #[fg=black,bg=green] #T #[default] "

# ---------------------------------------------------------------
# 核心布局划分
# ---------------------------------------------------------------

tmux -L temp split-window -h -p 45 -t $SESSION

tmux -L temp select-pane -t $SESSION:0.0
tmux -L temp select-pane -T "遥控器"

tmux -L temp select-pane -t $SESSION:0.1
tmux -L temp select-pane -T "Pico解算"


tmux -L temp send-keys -t $SESSION:0.0 "source install/setup.bash&&ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch remote_controller remote_controller.launch.py" C-m
tmux -L temp send-keys -t $SESSION:0.1 "source install/setup.bash&&ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch elf3_arm_bringup elf3_arm_bringup_nohand.launch.py" C-m

tmux -L temp select-pane -t $SESSION:0.0
tmux -L temp attach-session -t $SESSION