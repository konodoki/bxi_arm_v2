#!/bin/bash

SESSION="teleop"

if ffmpeg -f v4l2 -i /dev/video4 -frames:v 1 -f image2 -y /tmp/cam_test.jpg 2>/dev/null; then
    echo "✓ 摄像头存在且可用"
    rm -f /tmp/cam_test.jpg
else
    echo "✗ 摄像头不存在或无法访问，请检查是否插入深度相机"
    exit 1
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

# 使用临时服务器继承当前完整环境
# -L temp 创建临时套接字，启动新服务器，会继承当前 Shell 的所有环境变量
tmux -L temp new-session -d -s $SESSION

# 设置全局选项
tmux -L temp set-option -g mouse on
tmux -L temp set-option -g pane-border-status top
tmux -L temp set-option -g pane-border-format " #[fg=black,bg=green] #T #[default] "

# ---------------------------------------------------------------
# 核心布局划分
# ---------------------------------------------------------------

# 在第一个窗口（窗口0）中水平分割右侧45%的面板
tmux -L temp split-window -h -p 45 -t $SESSION

tmux -L temp select-pane -t $SESSION:0.0
tmux -L temp split-window -v -p 50 -t $SESSION:0.0

tmux -L temp select-pane -t $SESSION:0.0
tmux -L temp select-pane -T "遥控器"

tmux -L temp select-pane -t $SESSION:0.1
tmux -L temp select-pane -T "仿真程序"

tmux -L temp select-pane -t $SESSION:0.2
tmux -L temp select-pane -T "Pico解算"

# 发送命令到各个面板
tmux -L temp send-keys -t $SESSION:0.0 "source install/setup.bash&&ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch remote_controller remote_controller.launch.py" C-m
tmux -L temp send-keys -t $SESSION:0.1 "source install/setup.bash&&ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch bxi_example_py_elf3 example_demo.launch.py" C-m
tmux -L temp send-keys -t $SESSION:0.2 "source install/setup.bash&&ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch elf3_arm_bringup elf3_arm_bringup_nohand.launch.py" C-m

# 最后，选择默认面板并附加会话
tmux -L temp select-pane -t $SESSION:0.0
tmux -L temp attach-session -t $SESSION