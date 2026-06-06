"""Pico pose to ELF3 arm IK command bridge."""

from pathlib import Path
import threading
import time
from typing import Iterable, Optional

import communication.msg as bxi_msg
import geometry_msgs.msg
from ikpy.chain import Chain
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import sensor_msgs.msg
from scipy.spatial.transform import Rotation

from .filter import MultiChannelLowPassFilter
from .pico_hand import PicoHand
from .sphere_leastlq import CalibrationStopped, fit_pico_method2


DEFAULT_ROBOT_ARM_LENGTH = 0.6
DEFAULT_HEAD_TO_TORSO_OFFSET = [0.1, 0.0, 0.0]
JOINT_NAMES = [
    'l_shoulder_y_joint',
    'l_shoulder_x_joint',
    'l_shoulder_z_joint',
    'l_elbow_y_joint',
    'l_wrist_x_joint',
    'l_wrist_y_joint',
    'l_wrist_z_joint',
    'r_shoulder_y_joint',
    'r_shoulder_x_joint',
    'r_shoulder_z_joint',
    'r_elbow_y_joint',
    'r_wrist_x_joint',
    'r_wrist_y_joint',
    'r_wrist_z_joint',
]


class RobotControlNode(Node):
    """Publishes robot arm IK commands from Pico controller poses."""

    def __init__(self) -> None:
        super().__init__('robot_ik_control_node')

        self._declare_parameters()
        self._load_parameters()

        self.l_chain = Chain.from_urdf_file(
            self._resolve_urdf_path('elf3_arm_l.urdf'),
            active_links_mask=[False] + [True] * 7,
        )
        self.r_chain = Chain.from_urdf_file(
            self._resolve_urdf_path('elf3_arm_r.urdf'),
            active_links_mask=[False] + [True] * 7,
        )

        self._state_lock = threading.Lock()
        self.ik_result = [0.0] * len(JOINT_NAMES)
        self.head_ori = [1.0, 0.0, 0.0, 0.0]
        self.joy_data = [0.0, 0.0, 0.0, 0.0]
        self.left_real_joints: Optional[list[float]] = None
        self.right_real_joints: Optional[list[float]] = None
        self.last_joint_state_time = 0.0

        qos = QoSProfile(
            depth=1,
            durability=qos_profile_sensor_data.durability,
            reliability=qos_profile_sensor_data.reliability,
        )
        callback_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            sensor_msgs.msg.JointState,
            self.joint_state_topic,
            self.joint_callback,
            qos,
        )
        self.joint_pub = self.create_publisher(
            sensor_msgs.msg.JointState,
            self.joint_command_topic,
            qos,
        )
        self.head_pub = self.create_publisher(
            geometry_msgs.msg.Vector3,
            self.head_topic,
            qos,
        )
        self.joy_pub = self.create_publisher(
            bxi_msg.MotionCommands,
            self.motion_command_topic,
            qos,
        )
        self.timer = self.create_timer(
            self.publish_period_sec,
            self.publish_joints,
            callback_group=callback_group,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('urdf_dir', 'data')
        self.declare_parameter(
            'joint_state_topic',
            'pico_control_joint_states',
        )
        self.declare_parameter(
            'joint_command_topic',
            'pico_control_joint_commands',
        )
        self.declare_parameter('head_topic', 'pico_control_head')
        self.declare_parameter(
            'motion_command_topic',
            'pico_motion_commands',
        )
        self.declare_parameter('publish_period_sec', 0.01)
        self.declare_parameter(
            'joint_state_timeout_sec',
            2.0,
        )
        self.declare_parameter(
            'robot_arm_length',
            DEFAULT_ROBOT_ARM_LENGTH,
        )
        self.declare_parameter('grip_threshold', 0.5)
        self.declare_parameter('ik_filter_alpha', 0.2)
        self.declare_parameter(
            'head_to_torso_offset',
            DEFAULT_HEAD_TO_TORSO_OFFSET,
        )

    def _load_parameters(self) -> None:
        self.urdf_dir = str(self.get_parameter('urdf_dir').value)
        self.joint_state_topic = str(
            self.get_parameter('joint_state_topic').value
        )
        self.joint_command_topic = str(
            self.get_parameter('joint_command_topic').value
        )
        self.head_topic = str(self.get_parameter('head_topic').value)
        self.motion_command_topic = str(
            self.get_parameter('motion_command_topic').value
        )
        self.publish_period_sec = self._positive_float(
            'publish_period_sec'
        )
        self.joint_state_timeout_sec = self._positive_float(
            'joint_state_timeout_sec'
        )
        self.robot_arm_length = self._positive_float('robot_arm_length')
        self.grip_threshold = self._non_negative_float('grip_threshold')
        self.ik_filter_alpha = self._positive_float('ik_filter_alpha')
        if self.ik_filter_alpha > 1.0:
            raise ValueError('ik_filter_alpha must be <= 1.0')
        self.head_to_torso_offset = self._float_list_parameter(
            'head_to_torso_offset',
            3,
        )

    def _positive_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    def _non_negative_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f'{name} must be non-negative')
        return value

    def _float_list_parameter(self, name: str, length: int) -> list[float]:
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != length:
            raise ValueError(f'{name} must contain {length} values')
        return values

    def _resolve_urdf_path(self, filename: str) -> str:
        urdf_dir = Path(self.urdf_dir).expanduser()
        candidates = [urdf_dir / filename]

        if not urdf_dir.is_absolute():
            package_file = Path(__file__).resolve()
            candidates.extend(
                parent / self.urdf_dir / filename
                for parent in package_file.parents
            )

        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        searched = ', '.join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(
            f'URDF not found: {filename}; tried {searched}'
        )

    @staticmethod
    def quaternion_to_euler(
        w: float,
        x: float,
        y: float,
        z: float,
        epsilon: float = 1e-10,
    ) -> tuple[float, float, float]:
        """Convert a WXYZ quaternion to roll, pitch, yaw."""
        norm = np.sqrt(x * x + y * y + z * z + w * w)
        if norm <= epsilon:
            return 0.0, 0.0, 0.0

        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        sinp = 2 * (w * y - z * x)

        if abs(sinp) >= 1 - epsilon:
            pitch = np.copysign(np.pi / 2, sinp)
            yaw = 2 * np.arctan2(x, w)
            roll = 0.0
        else:
            pitch = np.arcsin(sinp)
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

        return float(roll), float(pitch), float(yaw)

    def joint_callback(self, msg: sensor_msgs.msg.JointState) -> None:
        if len(msg.position) < len(JOINT_NAMES):
            self.get_logger().warn(
                'received incomplete joint state: '
                f'{len(msg.position)} positions'
            )
            return

        with self._state_lock:
            self.last_joint_state_time = time.monotonic()
            self.left_real_joints = [0.0] + list(msg.position[0:7])
            self.right_real_joints = [0.0] + list(msg.position[7:14])

    def solve_ik(
        self,
        side: str,
        pos: Iterable[float],
        ori: np.ndarray,
    ) -> np.ndarray:
        chain = self.l_chain if side == 'l' else self.r_chain
        initial_angles = self._initial_angles(side)

        if initial_angles is not None:
            lower_bounds = [link.bounds[0] for link in chain.links]
            upper_bounds = [link.bounds[1] for link in chain.links]
            initial_angles = np.clip(
                initial_angles,
                lower_bounds,
                upper_bounds,
            )

        try:
            return chain.inverse_kinematics(
                target_position=pos,
                target_orientation=ori,
                orientation_mode='all',
                initial_position=initial_angles,
            )
        except ValueError as exc:
            self.get_logger().error(f'IK error: {exc}')
            if initial_angles is not None:
                return np.asarray(initial_angles)
            return np.zeros(len(chain.links))

    def _initial_angles(self, side: str) -> Optional[np.ndarray]:
        with self._state_lock:
            if (
                time.monotonic() - self.last_joint_state_time
                > self.joint_state_timeout_sec
            ):
                return None
            if side == 'l' and self.left_real_joints is not None:
                return np.asarray(self.left_real_joints)
            if side == 'r' and self.right_real_joints is not None:
                return np.asarray(self.right_real_joints)
        return None

    def set_head_orientation(self, orientation: Iterable[float]) -> None:
        with self._state_lock:
            self.head_ori = [float(value) for value in orientation]

    def set_joy_data(self, joy_data: Iterable[float]) -> None:
        with self._state_lock:
            self.joy_data = [float(value) for value in joy_data]

    def set_arm_solution(self, side: str, joints: Iterable[float]) -> None:
        joints = [float(value) for value in joints]
        if len(joints) != 7:
            raise ValueError('arm solution must contain 7 joints')

        with self._state_lock:
            if side == 'l':
                self.ik_result[0:7] = joints
            else:
                self.ik_result[7:14] = joints

    def _publish_snapshot(
        self,
    ) -> tuple[list[float], list[float], list[float]]:
        with self._state_lock:
            return (
                list(self.ik_result),
                list(self.head_ori),
                list(self.joy_data),
            )

    def publish_joints(self) -> None:
        ik_result, head_ori, joy_data = self._publish_snapshot()

        joint_msg = sensor_msgs.msg.JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = JOINT_NAMES
        joint_msg.position = ik_result
        self.joint_pub.publish(joint_msg)

        roll, pitch, yaw = self.quaternion_to_euler(*head_ori)
        head_msg = geometry_msgs.msg.Vector3()
        head_msg.x = roll
        head_msg.y = pitch
        head_msg.z = yaw
        self.head_pub.publish(head_msg)

        motion_msg = bxi_msg.MotionCommands()
        motion_msg.vel_des = geometry_msgs.msg.Vector3()
        motion_msg.vel_des.x = joy_data[1]
        motion_msg.vel_des.y = -joy_data[0]
        motion_msg.vel_des.z = 0.0
        motion_msg.yawdot_des = -joy_data[2]
        self.joy_pub.publish(motion_msg)


def _start_pico_executor() -> tuple[SingleThreadedExecutor, threading.Thread]:
    shared_node = PicoHand.get_node()
    executor = SingleThreadedExecutor()
    executor.add_node(shared_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    return executor, executor_thread


def _solve_arm_from_pico(
    node: RobotControlNode,
    side: str,
    pico: PicoHand,
    head_pos: np.ndarray,
    head_rotation_inv: Rotation,
    arm_origin: np.ndarray,
    arm_scale: float,
    ik_filter: MultiChannelLowPassFilter,
) -> None:
    _, grip = pico.get_button_values()
    if grip <= node.grip_threshold:
        return

    hand_pos = np.asarray(pico.get_pos(), dtype=float)
    rel_pos = hand_pos - (head_pos + np.asarray(node.head_to_torso_offset))
    pos_local = head_rotation_inv.apply(rel_pos)
    target_pos = (pos_local - arm_origin) * arm_scale

    qw, qx, qy, qz = pico.get_quaternion()
    hand_rotation = Rotation.from_quat([qx, qy, qz, qw])
    target_orientation = (head_rotation_inv * hand_rotation).as_matrix()

    joint_angles = ik_filter.filter(
        node.solve_ik(side, target_pos, target_orientation)
    )
    node.set_arm_solution(side, joint_angles[1:].tolist())


def run_pico_logic(
    node: RobotControlNode,
    stop_event: threading.Event,
) -> None:
    """Process Pico poses and continuously update IK targets."""
    left_pico = PicoHand('left')
    right_pico = PicoHand('right')
    head_pico = PicoHand('head')

    executor, executor_thread = _start_pico_executor()
    left_filter = MultiChannelLowPassFilter(
        num_channels=8,
        alpha=node.ik_filter_alpha,
    )
    right_filter = MultiChannelLowPassFilter(
        num_channels=8,
        alpha=node.ik_filter_alpha,
    )

    try:
        l_arm_origin, r_arm_origin, l_scale, r_scale = fit_pico_method2(
            left_pico,
            right_pico,
            head_pico,
            node.robot_arm_length,
            stop_event,
        )
    except CalibrationStopped:
        return
    except Exception as exc:
        node.get_logger().error(f'Pico calibration failed: {exc}')
        return

    l_arm_origin = np.asarray(l_arm_origin, dtype=float)
    r_arm_origin = np.asarray(r_arm_origin, dtype=float)

    try:
        while rclpy.ok() and not stop_event.is_set():
            head_pos = np.asarray(head_pico.get_pos(), dtype=float)
            h_w, h_x, h_y, h_z = head_pico.get_quaternion()
            node.set_head_orientation([h_w, h_x, h_y, h_z])

            head_rotation = Rotation.from_quat([h_x, h_y, h_z, h_w])
            head_rotation_inv = head_rotation.inv()

            left_joy = left_pico.get_joy()
            right_joy = right_pico.get_joy()
            node.set_joy_data([*left_joy, *right_joy])

            _solve_arm_from_pico(
                node,
                'l',
                left_pico,
                head_pos,
                head_rotation_inv,
                l_arm_origin,
                l_scale,
                left_filter,
            )
            _solve_arm_from_pico(
                node,
                'r',
                right_pico,
                head_pos,
                head_rotation_inv,
                r_arm_origin,
                r_scale,
                right_filter,
            )

            time.sleep(node.publish_period_sec)
    finally:
        executor.shutdown()
        executor_thread.join(timeout=1.0)
        PicoHand.destroy_shared_node()


def main(args=None) -> None:
    """Run the Pico IK bridge node."""
    rclpy.init(args=args)
    node = RobotControlNode()
    stop_event = threading.Event()
    pico_thread = threading.Thread(
        target=run_pico_logic,
        args=(node, stop_event),
    )
    pico_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        pico_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
