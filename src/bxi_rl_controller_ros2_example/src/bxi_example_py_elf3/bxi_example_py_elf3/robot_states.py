import math
import os
import pickle
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

from ament_index_python.packages import get_package_share_path
from bxi_example_py_elf3.utils.robot_state_base import MotorFrame, RobotControlState
from bxi_example_py_elf3.utils.state_machine import StateBehavior, TransitionProfile
from bxi_example_py_elf3.utils.tfs import quaternion_to_euler_array

if TYPE_CHECKING:
    from bxi_example_py_elf3.bxi_example_demo import BxiExample
else:
    BxiExample = Any


class NormalState(RobotControlState):
    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(
            ctx.normal,
            with_cmd_vel=True,
            cmd_vel=self.get_cmd_vel(ctx),
        )

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.normal.target_dof_pos, ctx.normal.kps, ctx.normal.kds
        )

    def get_motor_frame(self, ctx: BxiExample, dt: float) -> Optional[MotorFrame]:
        cmd_vel = self.get_cmd_vel(ctx)
        qpos, vel = ctx.normal.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            cmd_vel,
        )
        return self._motor_frame(qpos, ctx.normal.kps, ctx.normal.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        frame = self.get_motor_frame(ctx, dt)
        if frame is not None:
            ctx.set_motor_target(*frame)


class ZeroTorqueState(RobotControlState):
    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.joint_nominal_pos,
            np.zeros(ctx.dof_num, dtype=np.float32),
            np.zeros(ctx.dof_num, dtype=np.float32),
        )

    def get_motor_frame(self, ctx: BxiExample, dt: float) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.joint_nominal_pos,
            np.zeros(ctx.dof_num, dtype=np.float32),
            np.zeros(ctx.dof_num, dtype=np.float32),
        )


class PdBrakeState(RobotControlState):
    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.pd_pos, ctx.normal.kps, ctx.normal.kds)

    def get_motor_frame(self, ctx: BxiExample, dt: float) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.pd_pos, ctx.normal.kps, ctx.normal.kds)


class InitialPosState(RobotControlState):
    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.initial_pos, ctx.joint_kp, ctx.joint_kd)

    def get_motor_frame(self, ctx: BxiExample, dt: float) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.initial_pos, ctx.joint_kp, ctx.joint_kd)


class DanceState(RobotControlState):
    def __init__(self, name: str, state_id: int, start_frame: int = 100):
        super().__init__(name, state_id)
        self.start_frame = start_frame
        self.playing = True

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.dance.timestep = self.start_frame
        if hasattr(ctx.dance, "timeinit"):
            ctx.dance.timeinit = 0.0
        ctx.preheat_model(ctx.dance)

    def on_enter(self, ctx: BxiExample) -> None:
        self.playing = True
        ctx.dance.timestep = self.start_frame

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.dance.target_dof_pos,
            ctx.dance.kps,
            ctx.dance.kds,
        )

    def get_motor_frame(self, ctx: BxiExample, dt: float) -> Optional[MotorFrame]:
        if ctx.dance.timestep >= ctx.dance.motionpos.shape[0]:
            return None

        qpos = ctx.dance.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
        )

        if self.playing:
            ctx.dance.timestep += 50 * dt  # 模型动画是50hz播放的，dt是推理间隔

        return self._motor_frame(
            qpos,
            ctx.dance.kps,
            ctx.dance.kds,
        )

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.dance.timestep >= ctx.dance.motionpos.shape[0]:
            print("Motion replay finished, resetting simulation.")
            ctx.dance.timestep = self.start_frame
            ctx.request_state(
                "normal",
                trigger="motion_finished",
                transition={
                    "base": "dual_running_blend",
                    "duration": 0.5,
                    "data": {"run_from": False},
                },
            )
            return

        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        frame = self.get_motor_frame(ctx, dt)
        if frame is not None:
            ctx.set_motor_target(*frame)

    def on_action(self, ctx: BxiExample, action_name: str) -> bool:
        if action_name != "toggle_dance_pause":
            return False

        self.playing = not self.playing
        return True


class MotionState(RobotControlState):
    policy_attr = ""
    finish_trigger = "flip_finished"
    end_frame_trim = 0
    end_transition = {}

    def __init__(self, name: str, state_id: int):
        super().__init__(name, state_id)
        self.playing = True

    def _policy(self, ctx: BxiExample) -> Any:
        return getattr(ctx, self.policy_attr)

    def on_enter_transition(self, ctx, from_state, progress, transition):
        policy = self._policy(ctx)
        policy.timestep = policy.start_frame
        return super().on_enter_transition(ctx, from_state, progress, transition)

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        policy = self._policy(ctx)
        if hasattr(policy, "timeinit"):
            policy.timeinit = 0.0
        ctx.preheat_model(policy)

    def on_enter(self, ctx: BxiExample) -> None:
        self.playing = True
        policy = self._policy(ctx)
        policy.timestep = policy.start_frame
        if hasattr(policy, "timeinit"):
            policy.timeinit = 0.0

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        policy = self._policy(ctx)
        qpos = getattr(policy, "target_dof_pos", None)
        if qpos is None:
            qpos = getattr(policy, "default_dof_pos", None)
        if qpos is None:
            return None
        return self._motor_frame(qpos, policy.kps, policy.kds)

    def get_motor_frame(self, ctx: BxiExample, dt: float) -> Optional[MotorFrame]:
        policy = self._policy(ctx)

        qpos = policy.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
        )

        if self.playing:
            policy.timestep += 50 * dt  # 模型动画是50hz播放的，dt是推理间隔

        return self._motor_frame(qpos, policy.kps, policy.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        policy = self._policy(ctx)

        frame = self.get_motor_frame(ctx, dt)
        if frame is not None:
            ctx.set_motor_target(*frame)

        if policy.timestep > policy.end_frame - self.end_frame_trim:
            print("Motion replay finished, resetting simulation.")
            ctx.request_state(
                "normal", trigger=self.finish_trigger, transition=self.end_transition
            )


class ForwardFlipState(MotionState):
    policy_attr = "forward_flip"
    finish_trigger = "forward_flip_finished"
    end_frame_trim = 125
    end_transition = {
        "base": "dual_running_blend",
        "duration": 1.0,
        "data": {
            "curve": "smootherstep",
            "run_from": True,
        },  # 过渡的时候模型继续推理，同时推理下一个模型
    }


class HandPlayBackState(RobotControlState):
    start_frame = 0
    tail_trim_frames = 0
    return_time = 0.5
    file_name = "applause.pkl"

    def __init__(self, name, state_id):
        super().__init__(name, state_id)
        self.frame = 0.0
        self.applause_data, self.fps = self._load_applause_data()

    def _load_applause_data(self) -> tuple[np.ndarray, float]:
        data_path = os.path.join(
            get_package_share_path("bxi_example_py_elf3"),
            "data",
            self.file_name,
        )
        with open(data_path, "rb") as data_file:
            data = pickle.load(data_file)

        dof_pos = np.asarray(data["dof_pos"], dtype=np.float32)[:, -14:]
        start = min(self.start_frame, dof_pos.shape[0])
        end = max(start, dof_pos.shape[0] - self.tail_trim_frames)
        applause_data = dof_pos[start:end]
        if applause_data.shape[0] == 0:
            raise ValueError(
                f"HandPlayBack data is empty after frame trim: {data_path}"
            )

        return applause_data, float(data["fps"])

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(
            ctx.teleop,
            with_cmd_vel=True,
            cmd_vel=self.get_cmd_vel(ctx),
        )

    def on_enter(self, ctx: BxiExample) -> None:
        self.frame = 0.0
        self.playing = True

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        qpos = ctx.teleop.target_dof_pos.copy()
        qpos[-14:] = self.applause_data[0]
        return self._motor_frame(qpos, ctx.teleop.kps, ctx.teleop.kds)

    def get_motor_frame(self, ctx, dt):
        cmd_vel = self.get_cmd_vel(ctx)
        qpos, vel = ctx.teleop.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            cmd_vel,
        )
        if self.frame < self.applause_data.shape[0]:
            qpos[-14:] = self.applause_data[int(self.frame)]
        else:
            qpos[-14:] = self.applause_data[-1]
        if self.playing:
            self.frame += self.fps * dt
        return self._motor_frame(qpos, ctx.teleop.kps, ctx.teleop.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            ctx.request_state("zero_torque", trigger="safety")
            return
        if self.frame >= self.applause_data.shape[0]:
            ctx.request_state(
                "normal",
                trigger="applause_finished",
                transition={
                    "base": "dual_running_blend",
                    "duration": 1.0,
                },
            )
            return
        frame = self.get_motor_frame(ctx, dt)
        if frame is not None:
            ctx.set_motor_target(*frame)

    def on_action(self, ctx: BxiExample, action_name: str) -> bool:
        if action_name != "toggle_dance_pause":
            return False

        self.playing = not self.playing
        return True


class ApplauseState(HandPlayBackState):
    start_frame = 600
    tail_trim_frames = 600
    file_name = "isaaclab_model/applause.pkl"


import sensor_msgs
import std_msgs
from datetime import datetime
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import communication.msg as bxiMsg
from bxi_example_py_elf3.utils.bxi_motor import *

class TeleopState(RobotControlState):
    def __init__(self, name, state_id):
        super().__init__(name, state_id)
        self._recording = False
        self._record_fps = 0
        self._record_root_pos = []
        self._record_root_rot = []
        self._record_dof_pos = []

    def on_bind(self, ctx):
        self.l_arm = [0.5, 0.3, -0.1, -0.2, 0.0, 0.0, 0.0]
        self.r_arm = [0.5, -0.3, 0.1, -0.2, 0.0, 0.0, 0.0]
        self.l_grip = 0.0
        self.r_grip = 0.0
        self.l_trigger = 0.0
        self.r_trigger = 0.0
        qos = QoSProfile(
            depth=1,
            durability=qos_profile_sensor_data.durability,
            reliability=qos_profile_sensor_data.reliability,
        )
        self.arm_joint_state_pub = ctx.create_publisher(
            sensor_msgs.msg.JointState, "pico_control_joint_states", qos
        )
        self.pico_control_sub = ctx.create_subscription(
            sensor_msgs.msg.JointState,
            "pico_control_joint_commands",
            self.arm_joint_callback,
            qos,
        )
        self.left_grip_sub = ctx.create_subscription(
            std_msgs.msg.Float32, "pico/left_grip", self.left_grip_callback, qos
        )
        self.right_grip_sub = ctx.create_subscription(
            std_msgs.msg.Float32, "pico/right_grip", self.right_grip_callback, qos
        )
        self.left_trigger_sub = ctx.create_subscription(
            std_msgs.msg.Float32, "pico/left_trigger", self.left_trigger_callback, qos
        )
        self.right_trigger_sub = ctx.create_subscription(
            std_msgs.msg.Float32, "pico/right_trigger", self.right_trigger_callback, qos
        )
        qos_hand=QoSProfile(depth=100)
        self.gripper_control_pub = ctx.create_publisher(bxiMsg.CANFDPacket, "canfd_packet/tx", qos_hand)
        self.gripper_control_pub.publish(BxiMotor.build_motor_packet(5,1,BxiMotor.enter_motor_mode()))
        self.gripper_control_pub.publish(BxiMotor.build_motor_packet(6,1,BxiMotor.enter_motor_mode()))
        print("clamp enable!")
        self.gripper_control_pub.publish(BxiMotor.build_motor_packet(5,1,BxiMotor.zero()))
        self.gripper_control_pub.publish(BxiMotor.build_motor_packet(6,1,BxiMotor.zero()))
        print("clamp set zero!")
        

    def arm_joint_callback(self, msg):
        joint_pos = msg.position
        self.l_arm = joint_pos[0:7]
        self.r_arm = joint_pos[7:]

    def left_grip_callback(self, msg):
        self.l_grip = msg.data

    def right_grip_callback(self, msg):
        self.r_grip = msg.data

    def left_trigger_callback(self, msg):
        self.l_trigger = msg.data

    def right_trigger_callback(self, msg):
        self.r_trigger = msg.data

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(ctx.teleop, with_cmd_vel=True)

    def on_enter(self, ctx: BxiExample) -> None:
        self.playing = True

    def on_exit(self, ctx: BxiExample) -> None:
        if self._recording:
            self._stop_recording(ctx, reason="state_exit")
        super().on_exit(ctx)

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        qpos = ctx.teleop.target_dof_pos.copy()
        return self._motor_frame(qpos, ctx.teleop.kps, ctx.teleop.kds)

    def _reset_recording_buffers(self) -> None:
        self._record_root_pos = []
        self._record_root_rot = []
        self._record_dof_pos = []

    def _start_recording(self, ctx: BxiExample, dt: float) -> None:
        self._reset_recording_buffers()
        self._recording = True
        self._record_fps = max(1, int(round(1.0 / dt))) if dt > 0.0 else 0
        print(f"teleop recording started, fps={self._record_fps}")

    def _stop_recording(self, ctx: BxiExample, reason: str = "trigger_release") -> None:
        self._recording = False
        frame_count = len(self._record_dof_pos)
        if frame_count == 0:
            print(f"teleop recording stopped without frames ({reason})")
            self._reset_recording_buffers()
            return

        data = {
            "fps": int(self._record_fps),
            "root_pos": np.asarray(self._record_root_pos, dtype=np.float64),
            "root_rot": np.asarray(self._record_root_rot, dtype=np.float64),
            "dof_pos": np.asarray(self._record_dof_pos, dtype=np.float64),
            "local_body_pos": None,
            "link_body_list": None,
        }

        record_dir = self._record_output_dir()
        os.makedirs(record_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        record_path = os.path.join(record_dir, f"teleop_record_{timestamp}.pkl")
        with open(record_path, "wb") as record_file:
            pickle.dump(data, record_file, protocol=pickle.HIGHEST_PROTOCOL)

        print(
            f"teleop recording saved: {record_path}, "
            f"frames={frame_count}, dof_pos={data['dof_pos'].shape}"
        )
        self._reset_recording_buffers()

    def _record_output_dir(self) -> str:
        env_dir = os.environ.get("BXI_TELEOP_RECORD_DIR")
        if env_dir:
            return env_dir

        try:
            package_root = get_package_share_path("bxi_example_py_elf3")
        except Exception:
            package_root = os.path.dirname(os.path.dirname(__file__))
        return os.path.join(package_root, "data", "teleop_records")

    def _record_frame(self, ctx: BxiExample, qpos: np.ndarray) -> None:
        self._record_root_pos.append(self._current_root_pos(ctx))
        self._record_root_rot.append(self._current_root_rot(ctx))
        self._record_dof_pos.append(self._current_dof_pos(ctx, qpos))

    def _current_root_pos(self, ctx: BxiExample) -> np.ndarray:
        return np.zeros(3, dtype=np.float64)

    def _current_root_rot(self, ctx: BxiExample) -> np.ndarray:
        root_rot = getattr(ctx, "current_quat_wxyz", None)
        root_rot = np.asarray(
            root_rot if root_rot is not None else [1.0, 0.0, 0.0, 0.0],
            dtype=np.float64,
        ).reshape(4)
        norm = np.linalg.norm(root_rot)
        if norm <= 1e-8:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return (root_rot / norm).copy()

    def _current_dof_pos(self, ctx: BxiExample, qpos: np.ndarray) -> np.ndarray:
        dof_pos = getattr(ctx, "current_q", qpos)
        dof_pos = np.asarray(dof_pos, dtype=np.float64)
        if dof_pos.shape != np.asarray(qpos).shape:
            dof_pos = np.asarray(qpos, dtype=np.float64)
        return dof_pos.copy()

    def _update_recording(self, ctx: BxiExample, dt: float, qpos: np.ndarray) -> None:
        l_trigger = float(getattr(self, "l_trigger", 0.0))
        r_trigger = float(getattr(self, "r_trigger", 0.0))

        if self._recording and l_trigger < 0.5 and r_trigger < 0.5:
            self._stop_recording(ctx)
            return

        if not self._recording and l_trigger > 0.5 and r_trigger > 0.5:
            self._start_recording(ctx, dt)

        if self._recording:
            self._record_frame(ctx, qpos)
    def _update_clamp(self, ctx: BxiExample, dt: float):
        l_trigger = float(getattr(self, "l_trigger", 0.0))
        r_trigger = float(getattr(self, "r_trigger", 0.0))
        self.gripper_control_pub.publish(BxiMotor.build_motor_packet(5,1,BxiMotor.pack_cmd(
            joint=JointControl(
                p_des=float((1-l_trigger)*0.1),
                v_des=0.0,
                kp=float(20),
                kd=float(1),
                t_ff=0.0,
            ),
            p_range=(-12.5,12.5),
            v_range=(-45.0,45.0),
            t_range=(-40,40),
            kp_range=(0,500),
            kd_range=(0,5),
        )))
        self.gripper_control_pub.publish(BxiMotor.build_motor_packet(6,1,BxiMotor.pack_cmd(
            joint=JointControl(
                p_des=float((1-r_trigger)*0.1),
                v_des=0.0,
                kp=float(20),
                kd=float(1),
                t_ff=0.0,
            ),
            p_range=(-12.5,12.5),
            v_range=(-45.0,45.0),
            t_range=(-40,40),
            kp_range=(0,500),
            kd_range=(0,5),
        )))
        
    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            if self._recording:
                self._stop_recording(ctx, reason="safety")
            ctx.request_state("zero_torque", trigger="safety")
            return
        cmd_vel = self.get_cmd_vel(ctx)
        qpos, vel = ctx.teleop.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            cmd_vel,
        )
        
        arm_joint_state = JointState()
        arm_joint_state.name = ['l_shoulder_y_joint',
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
                            'r_wrist_z_joint']
        arm_joint_state.position = ctx.current_q.tolist()[-14:]
        arm_joint_state.velocity = ctx.current_dq.tolist()[-14:]
        self.arm_joint_state_pub.publish(arm_joint_state)
        
        left_arm_range = slice(3 + 12, 3 + 12 + 7)
        right_arm_range = slice(3 + 12 + 7, 3 + 12 + 14)

        # 初始化手部状态
        if not hasattr(self, "_hand"):
            self._hand = {
                "left": {"kp": None, "target": None, "last_grip": 0},
                "right": {"kp": None, "target": None, "last_grip": 0},
            }

        growth_rate = 0.2  # 缓启动速度

        def update_hand(hand, grip, arm_range):
            state = self._hand[hand]
            current_grip_state = grip > 0.5
            last_grip_state = state["last_grip"] > 0.5

            # 检测状态切换
            if current_grip_state != last_grip_state:
                # 状态切换，触发缓启动/缓关闭
                state["target"] = ctx.teleop.kps[arm_range].copy()
                if state["kp"] is None:
                    state["kp"] = state["target"].copy()
                else:
                    # 状态切换，从当前值的10%开始缓启动
                    state["kp"] = [kp * 0.1 for kp in state["target"]]

            # 如果没有目标值，初始化
            if state["target"] is None:
                state["target"] = ctx.teleop.kps[arm_range].copy()
            if state["kp"] is None:
                state["kp"] = state["target"].copy()

            # 缓慢调整到目标值（使用 any 检查是否有差距）
            need_update = any(
                abs(state["kp"][i] - state["target"][i]) > 0.001
                for i in range(len(state["kp"]))
            )

            if need_update:
                for i in range(len(state["kp"])):
                    diff = state["target"][i] - state["kp"][i]
                    if abs(diff) > 0.001:
                        state["kp"][i] += diff * growth_rate * dt
                        # 防止过冲
                        if (diff > 0 and state["kp"][i] > state["target"][i]) or (
                            diff < 0 and state["kp"][i] < state["target"][i]
                        ):
                            state["kp"][i] = state["target"][i]

            # 保存当前grip状态
            state["last_grip"] = grip

        # 更新左右手
        update_hand("left", self.l_grip, left_arm_range)
        update_hand("right", self.r_grip, right_arm_range)

        # 构建完整的 kp 列表
        kp_to_use = ctx.teleop.kps.copy()
        kp_to_use[left_arm_range] = self._hand["left"]["kp"]
        kp_to_use[right_arm_range] = self._hand["right"]["kp"]

        # 控制手臂位置（仅在 grip > 0.5 时）
        if self.l_grip > 0.5:
            qpos[left_arm_range] = self.l_arm
        if self.r_grip > 0.5:
            qpos[right_arm_range] = self.r_arm

        #录制 或者 控制夹爪 二选一
        self._update_recording(ctx, dt, qpos) #录制
        # self._update_clamp(ctx,dt) #夹爪
        
        ctx.set_motor_target(qpos, kp_to_use, ctx.teleop.kds)

    def on_action(self, ctx: BxiExample, action_name: str) -> bool:
        if action_name != "toggle_dance_pause":
            return False

        self.playing = not self.playing
        return True


class RecoverState(RobotControlState):
    end_frame_trim = 0

    def __init__(self, name: str, state_id: int):
        super().__init__(name, state_id)
        self.playing = True
        self.motion_selected = False

    def on_enter_transition(self, ctx, from_state, progress, transition):
        ctx.recover.timestep = ctx.recover.start_frame
        return super().on_enter_transition(ctx, from_state, progress, transition)

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        if self._configure_recover_motion(ctx):
            ctx.preheat_model(ctx.recover)

    def on_enter(self, ctx: BxiExample) -> None:
        self.playing = True
        if not self._configure_recover_motion(ctx):
            ctx.request_state("zero_torque", trigger="recover_pose_rejected")

    def _configure_recover_motion(self, ctx: BxiExample) -> bool:
        eu_ang = quaternion_to_euler_array(ctx.quat_xyzw)
        eu_ang[eu_ang > math.pi] -= 2 * math.pi

        if eu_ang[1] < -(math.pi / 4.0):
            # 躺地上
            ctx.recover.end_frame = 880
            ctx.recover.timestep = 600
            ctx.recover.start_frame = 600
            self.end_frame_trim = 20
            self.motion_selected = True
            return True
        elif eu_ang[1] > (math.pi / 4.0):
            # 趴地上
            ctx.recover.end_frame = 1690
            ctx.recover.timestep = 1350
            ctx.recover.start_frame = 1350
            self.end_frame_trim = 0
            self.motion_selected = True
            return True

        self.motion_selected = False
        return False

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        if not self.motion_selected:
            return None
        return self._motor_frame(
            ctx.recover.target_dof_pos, ctx.recover.kps, ctx.recover.kds
        )

    def get_motor_frame(self, ctx: BxiExample, dt: float) -> Optional[MotorFrame]:
        if ctx.recover.timestep > ctx.recover.end_frame:
            return None

        qpos = ctx.recover.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
        )

        if self.playing:
            ctx.recover.timestep += 50 * dt  # 模型动画是50hz播放的，dt是推理间隔
        return self._motor_frame(qpos, ctx.recover.kps, ctx.recover.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.recover.timestep > ctx.recover.end_frame - self.end_frame_trim:
            ctx.request_state(
                "normal",
                trigger="recover_finished",
                transition={
                    "base": "dual_running_blend",
                    "duration": 0.5,
                    "data": {"run_from": True},  #
                },
            )
            return

        frame = self.get_motor_frame(ctx, dt)
        if frame is not None:
            ctx.set_motor_target(*frame)


class AmpRunState(RobotControlState):
    def __init__(self, name: str, state_id: int):
        super().__init__(name, state_id)
        self.max_vel = 0.0
        self.pre_cmd_vel_run = np.array([0.0, 0.0, 0.0])
        self.cmd_vel_run = np.array([0.0, 0.0, 0.0])

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(
            ctx.amp_run,
            with_cmd_vel=True,
            cmd_vel=self.get_cmd_vel(ctx),
        )

    def on_enter(self, ctx: BxiExample) -> None:
        self.max_vel = 0.0
        self.pre_cmd_vel_run = np.array([0.0, 0.0, 0.0])
        self.cmd_vel_run = np.array([0.0, 0.0, 0.0])

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.amp_run.target_dof_pos, ctx.amp_run.kps, ctx.amp_run.kds
        )

    def process_cmd_vel(
        self,
        ctx: BxiExample,
        cmd_vel: np.ndarray,
    ) -> Optional[np.ndarray]:
        self.cmd_vel_run[:2] = 0.98 * self.pre_cmd_vel_run[:2] + 0.02 * cmd_vel[:2]
        self.cmd_vel_run[2] = cmd_vel[2]
        self.pre_cmd_vel_run = self.cmd_vel_run.copy()
        return self.cmd_vel_run

    def get_motor_frame(self, ctx: BxiExample, dt: float) -> Optional[MotorFrame]:
        cmd_vel = self.get_cmd_vel(ctx)
        qpos, vel = ctx.amp_run.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            cmd_vel,
        )

        if vel[0] > self.max_vel:
            self.max_vel = vel[0]
        if ctx.loop_count >= 100 + int(0.3 / ctx.dt):
            print(self.max_vel)
            ctx.loop_count = int(0.3 / ctx.dt)
            self.max_vel = 0.0

        return self._motor_frame(qpos, ctx.amp_run.kps, ctx.amp_run.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        frame = self.get_motor_frame(ctx, dt)
        if frame is not None:
            ctx.set_motor_target(*frame)


class NormalRunState(RobotControlState):
    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(
            ctx.normal_run,
            with_cmd_vel=True,
            cmd_vel=self.get_cmd_vel(ctx),
        )

    def on_enter(self, ctx: BxiExample) -> None:
        if hasattr(ctx.normal_run, "action"):
            ctx.normal_run.action = np.zeros_like(ctx.normal_run.action)

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        qpos = ctx.normal_run.default_joint_pos.copy()
        if hasattr(ctx.normal_run, "target_q"):
            qpos += ctx.normal_run.target_q
        return self._motor_frame(
            qpos,
            ctx.normal_run.joint_stiffness,
            ctx.normal_run.joint_damping,
        )

    def get_motor_frame(self, ctx: BxiExample, dt: float) -> Optional[MotorFrame]:
        cmd_vel = self.get_cmd_vel(ctx)
        qpos = ctx.normal_run.infer_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_xyzw,
            ctx.current_omega,
            cmd_vel,
        )
        return self._motor_frame(
            qpos,
            ctx.normal_run.joint_stiffness,
            ctx.normal_run.joint_damping,
        )

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        frame = self.get_motor_frame(ctx, dt)
        if frame is not None:
            ctx.set_motor_target(*frame)
