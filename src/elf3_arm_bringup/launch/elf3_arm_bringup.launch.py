import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pico_bxi_server",
                executable="pico_bxi_server",
                name="pico_bxi_server",
                output="screen",
                
            ),

            Node(
                package="elf3_arm_ikpy_control_pico",
                executable="elf3_arm_ikpy_control_pico",
                output="screen",
            ),
            
            Node(
                package="hand_control",
                executable="hand_control",
                name="hand_control",
                output="screen",
            ),
                        
            Node(
                package="aero_hand_open",
                executable="aero_hand_node",
                name="aero_hand_open",
                output="screen",
                parameters=[
                    {'left_port':'/dev/ttyACM0'},
                    {'right_port':'/dev/ttyACM1'},
                    {'bluetooth':False}
                ]
            ),
        ]
    )
