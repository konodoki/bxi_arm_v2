#!/bin/bash
###
 # @Author: konodoki 1326898804@qq.com
 # @Date: 2026-06-05 13:29:17
 # @LastEditors: konodoki 1326898804@qq.com
 # @LastEditTime: 2026-06-05 13:51:30
 # @FilePath: /bxi_arm_v2/stop_all.sh
 # @Description: 
 # 
 # Copyright (c) 2026 by konodoki, All Rights Reserved. 
### 

if pgrep -x "hardware_elf3" > /dev/null; then
    echo "错误：检测到 hardware_elf3 进程正在运行！"
    echo "请先关闭机器人（停止硬件驱动），然后重新运行此脚本。"
    exit 1
fi

if systemctl list-unit-files | grep -q "^ros_elf_launch.service"; then
    echo ""
    read -p "是否重启机器人本体遥控服务？(y/n): " -n 1 -r
    echo

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "正在重启 ros_elf_launch.service..."
        sudo systemctl restart ros_elf_launch.service
        sleep 2
        
        # 检查重启后的状态
        if systemctl is-active --quiet ros_elf_launch.service; then
            echo "✓ ros_elf_launch.service 已成功重启"
        else
            echo "✗ ros_elf_launch.service 重启失败，请检查服务状态"
        fi
    else
        echo "跳过重启 ros_elf_launch.service 遥控器将无法控制机器人"
        echo "若需要则需手动重启sudo systemctl restart ros_elf_launch.service"
        sleep 3
    fi
fi

tmux kill-server