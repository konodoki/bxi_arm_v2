# bxi_arm_v2

此仓库为半醒机器人遥操的示例代码。除双臂部分的控制已取消，运行后机器人除双臂部分的电机全为零力矩。

对应的apk程序仅仅只在pico4 ultra平台测试过

# 运行demo
## 运行前先测试图传服务

运行push_rtsp.sh脚本启动图传服务。可自行更改其中的/dev/video4为你自己的摄像头。

将BxiPicoApp-release.apk安装至pico4 ultra。

保证pico4和电脑处于同一局域网。程序会不间断扫描局域网内的2212端口，并将有可能为rtsp服务器的ip列出来，只需点击ip即可连接至图传服务接入画面

若能接受到画面则将src/pico_bxi_server/src/pico_bxi_server.cpp:125行的dev/video4同步修改成你的摄像头并编译

## 连接灵巧手并测试

若为usb连接则将src/elf3_arm_bringup/launch/elf3_arm_bringup.launch.py改为如下
```
{'left_port':'/dev/ttyACM0'},
{'right_port':'/dev/ttyACM1'},
{'bluetooth':False}
```
若为蓝牙连接则改为
```
{'left_port':'xx:xx:xx:xx:xx'},
{'right_port':'xx:xx:xx:xx:xx'},
{'bluetooth':True}
```

```
#测试灵巧手是否可用
#启动手部驱动(注意修改为你自己的连接方式)
ros2 run aero_hand_open aero_hand_node --ros-args -p left_port:="" -p right_port:="" -p bluetooth:=True
#启动动作控制
ros2 run hand_control hand_control
#启动虚拟扳机
ros2 run fake_trigger fake_trigger
```

## 启动demo
```
sudo su #只有进root下才能操控机器人
ros2 launch bxi_example_py_elf3 example_launch_mjlab_hw.py #启动机器人
ros2 launch elf3_arm_bringup elf3_arm_bringup.launch.py #启动手臂控制
```


<!-- ```
src/
├── aero-hand-open #半醒手臂的驱动程序
├── bxi_rl_controller_ros2_example #半醒机器人的控制示例
├── elf3_arm_bringup #launch文件
├── elf3_arm_ikpy_control_pico #
├── fake_trigger #虚拟pico的扳机，仅仅用于测试
├── hand_control #灵巧手部分的控制
└── pico_bxi_server #pico的ros2驱动
```

# 各子目录详细介绍
## pico_bxi_server
此仓库会启动位于bin/mediamtx程序（已集成无需安装）从而启动一个rtsp服务器，具体的配置在mediamtx.yml。

同时也会启动ffmpeg(未集成，需要apt安装),将摄像头推流至rtsp服务器。

当用户启动pico上的程序时，程序会不间断扫描局域网内的2212端口，并将有可能为rtsp服务器的ip列出来，只需点击ip即可连接至pico_bxi_server。并将画面传入pico -->

