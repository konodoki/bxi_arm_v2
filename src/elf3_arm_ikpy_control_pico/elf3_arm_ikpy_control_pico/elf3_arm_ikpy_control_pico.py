import signal
import sys
import threading
import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import sensor_msgs.msg
import geometry_msgs.msg
from sphere_leastlq import *
from filter import *
from pico_hand import PicoHand
import communication.msg as bxiMsg
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
# --- 常量配置 ---
ROBOT_ARM_LENGTH = 0.6
JOINT_NAMES = [
    'l_shoulder_y_joint', 'l_shoulder_x_joint', 'l_shoulder_z_joint',
    'l_elbow_y_joint', 'l_wrist_x_joint', 'l_wrist_y_joint', 'l_wrist_z_joint',
    'r_shoulder_y_joint', 'r_shoulder_x_joint', 'r_shoulder_z_joint',
    'r_elbow_y_joint', 'r_wrist_x_joint', 'r_wrist_y_joint', 'r_wrist_z_joint'
]

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_ik_control_node')
        
        # 1. 运动学加载
        self.l_chain = Chain.from_urdf_file("data/elf3_arm_l.urdf", active_links_mask=[False] + [True]*7)
        self.r_chain = Chain.from_urdf_file("data/elf3_arm_r.urdf", active_links_mask=[False] + [True]*7)
        
        # 2. 状态变量
        self.ik_result = [0.0] * 14
        self.l_hand_filter = MultiChannelLowPassFilter(num_channels=8, alpha=0.8)
        self.right_hand_filter = MultiChannelLowPassFilter(num_channels=8, alpha=0.8)
        self.head_ori = [1,0,0,0]
        self.joy_data = [0.0,0.0,0.0,0.0]
        # 3. ROS 通信
        cb_group = MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(depth=1, durability=qos_profile_sensor_data.durability, reliability=qos_profile_sensor_data.reliability)
        self.create_subscription(sensor_msgs.msg.JointState,'pico_control_joint_states',self.joint_callback,qos)
        self.joint_pub = self.create_publisher(sensor_msgs.msg.JointState, 'pico_control_joint_commands', qos)
        self.head_pub = self.create_publisher(geometry_msgs.msg.Vector3, 'pico_control_head', qos)
        self.joy_pub = self.create_publisher(bxiMsg.MotionCommands, 'pico_motion_commands', qos)
        self.timer = self.create_timer(0.01, self.publish_joints, callback_group=cb_group) # 100Hz 足够

    @staticmethod
    def quat_to_matrix(w, x, y, z):
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
    @staticmethod
    def quaternion_to_euler(w,x, y, z, epsilon=1e-10):
        """
        安全地转换四元数到欧拉角，处理万向节锁附近的情况
        """
        # 归一化
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # 计算pitch
        sinp = 2 * (w * y - z * x)
        
        # 处理万向节锁（pitch接近±90度）
        if abs(sinp) >= 1 - epsilon:
            # 在万向节锁处，yaw和roll合并
            pitch = np.copysign(np.pi/2, sinp)
            
            # 计算yaw+roll
            yaw_plus_roll = 2 * np.arctan2(x, w)
            # 任意分配，这里设roll为0
            yaw = yaw_plus_roll
            roll = 0.0
        else:
            pitch = np.arcsin(sinp)
            
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x*x + y*y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)
            
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y*y + z*z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    def joint_callback(self,msg):
        self.last_joint_state_s = time.time()
        joints = msg.position
        self.left_real_joints = [0] + list(joints[0:7])
        self.right_real_joints = [0] + list(joints[7:])
    def solve_ik(self, side, pos, ori):
        chain = self.l_chain if side == 'l' else self.r_chain
        initial_angles = None
        
        if side == 'l' and hasattr(self,"left_real_joints"):
            if time.time() - self.last_joint_state_s < 2:
                initial_angles = self.left_real_joints
                
        if side == 'r' and hasattr(self,"right_real_joints"):
            if time.time() - self.last_joint_state_s < 2:
                initial_angles = self.right_real_joints
                
        if initial_angles is not None:
            lower_bounds = [link.bounds[0] for link in chain.links]
            upper_bounds = [link.bounds[1] for link in chain.links]
            initial_angles = np.clip(initial_angles, lower_bounds, upper_bounds)

        try:
            raw_ik = chain.inverse_kinematics(
                target_position=pos,
                target_orientation=ori,
                orientation_mode="all",
                initial_position=initial_angles
            )
            return raw_ik
        except ValueError as e:
            self.get_logger().error(f"IK Error: {e}")
            return initial_angles if initial_angles is not None else np.zeros(len(chain.links))

    def publish_joints(self):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.ik_result
        self.joint_pub.publish(msg)
        
        #head
        # head = RobotControlNode.quaternion_to_euler_scipy(self.head_ori[0],self.head_ori[1],self.head_ori[2],self.head_ori[3])
        roll,pitch,yaw = RobotControlNode.quaternion_to_euler(*self.head_ori)
        m = geometry_msgs.msg.Vector3()
        m.x = roll
        m.y = pitch
        m.z = yaw
        self.head_pub.publish(m)
        
        #joy
        m = bxiMsg.MotionCommands()
        from geometry_msgs.msg import Vector3
        v = Vector3()
        v.x = self.joy_data[1]
        v.y = -self.joy_data[0]
        v.z = 0.0
        m.vel_des=v
        m.yawdot_des=-self.joy_data[2]
        self.joy_pub.publish(m)

def run_pico_logic(node, stop_event):
    """主逻辑循环：处理手柄输入和IK计算"""
    # 初始化手柄
    left_pico = PicoHand('left')
    right_pico = PicoHand('right')
    head_pico = PicoHand('head')

    # 启动底层 ROS 线程支持 PicoHand
    shared_node = PicoHand.get_node()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(shared_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    #滤波参数

    left_hand_filter = MultiChannelLowPassFilter(num_channels=8, alpha=0.2)

    right_hand_filter = MultiChannelLowPassFilter(num_channels=8, alpha=0.2)

    # 校准
    l_arm_ori, r_arm_ori, l_k, r_k = fit_pico_method2(
        left_pico, right_pico, head_pico, ROBOT_ARM_LENGTH, stop_event
    )

    p_offset = np.array([0.1, 0.0, 0.0]) # 头部到躯干的偏置
    while rclpy.ok() and not stop_event.is_set():
        h_cur = head_pico.get_pos()
        # 获取头显四元数 (WXYZ)
        h_w, h_x, h_y, h_z = head_pico.get_quaternion()
        node.head_ori = [h_w, h_x, h_y, h_z]
        
        # 构造头显的旋转对象
        # 注意：scipy 默认输入顺序是 XYZW，所以我们要调整一下
        r_head = R.from_quat([h_x, h_y, h_z, h_w])
        r_head_inv = r_head.inv()

        node.joy_data = list(left_pico.get_joy() + right_pico.get_joy())

        # 处理左手
        l_cur = left_pico.get_pos()
        _, l_g = left_pico.get_button_values()
        if l_g > 0.5:
            # 1. 位置偏移：将全局位置转为相对于头部的局部位置
            # 先减去头部位置，再通过头部旋转的逆矩阵进行旋转对齐
            l_rel_pos = np.array(l_cur) - (np.array(h_cur) + p_offset)
            l_pos_local = r_head_inv.apply(l_rel_pos) # 这一步让手的位置随头转动
            
            # 映射到机器人手臂坐标系
            l_pos = (l_pos_local - l_arm_ori) * l_k

            # 2. 姿态转换
            lw, lx, ly, lz = left_pico.get_quaternion()
            r_l_hand = R.from_quat([lx, ly, lz, lw])
            # 计算相对于头的局部旋转: q_local = q_head^-1 * q_hand
            r_l_local = r_head_inv * r_l_hand
            
            # 转换为矩阵供 IK 使用
            l_ori = r_l_local.as_matrix()

            l_ang = left_hand_filter.filter(node.solve_ik('l', l_pos, l_ori))
            node.ik_result[0:7] = l_ang[1:].tolist()

        # 处理右手
        r_cur = right_pico.get_pos()
        _, r_g = right_pico.get_button_values()
        if r_g > 0.5:
            # 1. 位置偏移同步旋转
            r_rel_pos = np.array(r_cur) - (np.array(h_cur) + p_offset)
            r_pos_local = r_head_inv.apply(r_rel_pos)
            r_pos = (r_pos_local - r_arm_ori) * r_k

            # 2. 姿态转换
            rw, rx, ry, rz = right_pico.get_quaternion()
            r_r_hand = R.from_quat([rx, ry, rz, rw])
            r_r_local = r_head_inv * r_r_hand
            
            r_ori = r_r_local.as_matrix()

            r_ang = right_hand_filter.filter(node.solve_ik('r', r_pos, r_ori))
            node.ik_result[7:] = r_ang[1:].tolist()

def main():
    rclpy.init()
    
    node = RobotControlNode()
    stop_event = threading.Event()

    # 启动计算线程
    pico_thread = threading.Thread(target=run_pico_logic, args=(node, stop_event))
    pico_thread.start()

    try:
        # 主线程负责 ROS 事件轮询
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        pico_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
