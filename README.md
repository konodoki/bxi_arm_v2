# bxi_arm_v2

此仓库为半醒机器人遥操的示例代码。模型输出仅仅只有0～14号电机，15～28号电机为手臂。

对应的apk程序仅仅只在pico4 ultra平台测试过

```
src/
├── aero-hand-open #半醒手臂的驱动程序
├── bxi_rl_controller_ros2_example #半醒机器人的控制示例
├── elf3_arm_bringup #launch文件
├── elf3_arm_ikpy_control_pico #
├── hand_control #灵巧手部分的控制，示例里并未使用
└── pico_bxi_server #pico的ros2驱动
```

# 运行demo
## 运行前先测试图传服务

将BxiPicoApp-release.apk安装至pico4 ultra。

```
#运行图传服务
ros2 launch elf3_arm_bringup elf3_arm_bringup_nohand.launch.py
#出现以下输出代表摄像头打开成功
#[pico_bxi_server-1] 2026/05/14 17:22:02 INF [RTSP] [session de574919] is publishing to path 'video', 1 track (H264)
#若未出现检查摄像头是否插在usb3.0的口上，即蓝色的口。或者输入sudo chmod 777 /dev/video4 赋予权限试试
```

保证pico4和电脑处于同一局域网。程序会不间断扫描局域网内的2212端口，并将有可能为rtsp服务器的ip列出来，只需用手柄点击ip即可连接至图传服务接入画面

若想将画面关闭，则按下右手手柄A键进入透传

<!-- ## 连接灵巧手并测试

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
ros2 run fake_trigger fake_trigger -->

## 启动仿真

```
ros2 launch remote_controller remote_controller.launch.py #启动遥控器
ros2 launch elf3_arm_bringup elf3_arm_bringup_nohand.launch.py #启动手臂控制
ros2 launch bxi_example_py_elf3 example_demo.launch.py #启动机器人
#注意此时千万不要按遥控器上的启动按键，因为机器人已经通过命令启动了。正常操作即可
```

## 启动真机

```
sudo su #只有进root下才能操控机器人
ros2 launch remote_controller remote_controller.launch.py #启动遥控器
ros2 launch elf3_arm_bringup elf3_arm_bringup_nohand.launch.py #启动手臂控制
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py #启动机器人
#注意此时千万不要按遥控器上的启动按键，因为机器人已经通过命令启动了。正常操作即可
```

## 校准pico

将双手举过头顶，elf3_arm_ikpy_control_pico节点会提示开始校准

随后双手以两肩关节为圆心手臂伸直比划，尽量让手柄画出一个完整球面

elf3_arm_ikpy_control_pico节点显示校准成功则可进行下一步

## 操作

先使用遥控器正常让机器人进入normal站立状态

然后按下RT+A机器人进入遥操状态

此时操作手握紧pico手柄上的抓握按键则遥操接入

<!-- # 各子目录详细介绍
## pico_bxi_server
此仓库会启动位于bin/mediamtx程序（已集成无需安装）从而启动一个rtsp服务器，具体的配置在mediamtx.yml。

同时也会启动ffmpeg(未集成，需要apt安装),将摄像头推流至rtsp服务器。

当用户启动pico上的程序时，程序会不间断扫描局域网内的2212端口，并将有可能为rtsp服务器的ip列出来，只需点击ip即可连接至pico_bxi_server。并将画面传入pico -->

