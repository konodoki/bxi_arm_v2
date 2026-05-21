import math
import os
import pickle
from datetime import datetime
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

from ament_index_python.packages import get_package_share_path
from bxi_example_py_elf3.robot_state_base import MotorFrame, RobotControlState
from bxi_example_py_elf3.state_machine import StateBehavior, TransitionProfile
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
        ctx.preheat_model(ctx.normal, with_cmd_vel=True)

    def on_enter(self, ctx: BxiExample) -> None:
        self.reset_loop(ctx)

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.normal.target_dof_pos, ctx.normal.kps, ctx.normal.kds
        )

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        qpos, vel = ctx.normal.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            ctx.current_cmd_vel,
        )
        ctx.set_motor_target(qpos, ctx.normal.kps, ctx.normal.kds)


class ZeroTorqueState(RobotControlState):
    def on_enter(self, ctx: BxiExample) -> None:
        self.reset_loop(ctx)

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.joint_nominal_pos,
            np.zeros(ctx.dof_num, dtype=np.float32),
            np.zeros(ctx.dof_num, dtype=np.float32),
        )

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        ctx.set_motor_target(
            ctx.joint_nominal_pos,
            np.zeros(ctx.dof_num, dtype=np.float32),
            np.zeros(ctx.dof_num, dtype=np.float32),
        )


class PdBrakeState(RobotControlState):
    def on_enter(self, ctx: BxiExample) -> None:
        self.reset_loop(ctx)

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.pd_pos, ctx.normal.kps, ctx.normal.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        soft_start = min(ctx.loop_count / (2.0 / ctx.dt), 1.0)
        qpos = ctx.pos_last_state + (ctx.pd_pos - ctx.pos_last_state) * soft_start
        kp = ctx.kp_last + (ctx.normal.kps - ctx.kp_last) * soft_start
        kd = ctx.kd_last + (ctx.normal.kds - ctx.kd_last) * soft_start
        ctx.set_motor_target(qpos, kp, kd)


class InitialPosState(RobotControlState):
    def on_enter(self, ctx: BxiExample) -> None:
        self.reset_loop(ctx)

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.initial_pos, ctx.joint_kp, ctx.joint_kd)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        soft_start = min(ctx.loop_count / (2.0 / ctx.dt), 1.0)
        qpos = ctx.pos_last_state + (ctx.initial_pos - ctx.pos_last_state) * soft_start
        kp = ctx.kp_last + (ctx.joint_kp - ctx.kp_last) * soft_start
        kd = ctx.kd_last + (ctx.joint_kd - ctx.kd_last) * soft_start
        ctx.set_motor_target(qpos, kp, kd)


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
        self.reset_loop(ctx)
        self.playing = True
        ctx.dance.timestep = self.start_frame

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.dance.target_dof_pos,
            ctx.dance.stiffness_array,
            ctx.dance.damping_array,
        )

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.dance.timestep >= ctx.dance.motionpos.shape[0]:
            print("Motion replay finished, resetting simulation.")
            ctx.dance.timestep = self.start_frame
            ctx.request_state("normal", trigger="motion_finished")
            return

        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        qpos = ctx.dance.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
        )
        ctx.set_motor_target(qpos, ctx.dance.stiffness_array, ctx.dance.damping_array)

        if self.playing:
            ctx.dance.timestep += 1

    def on_action(self, ctx: BxiExample, action_name: str) -> bool:
        if action_name != "toggle_dance_pause":
            return False

        self.playing = not self.playing
        return True


class ApplauseState(RobotControlState):
    def __init__(self, name, state_id):
        super().__init__(name, state_id)
        self.start_frame = 600
        self.tail_trim_frames = 600
        self.return_time = 0.5
        self.frame = 0.0
        self.return_elapsed = 0.0
        self.returning = False
        self.return_start_pos = np.zeros(14, dtype=np.float32)
        self.applause_data, self.fps = self._load_applause_data()

    def _load_applause_data(self) -> tuple[np.ndarray, float]:
        try:
            data_path = os.path.join(
                get_package_share_path("bxi_example_py_elf3"),
                "data",
                "applause.pkl",
            )
        except Exception:
            data_path = ""

        if not data_path or not os.path.exists(data_path):
            package_root = os.path.dirname(os.path.dirname(__file__))
            data_path = os.path.join(package_root, "data", "applause.pkl")

        with open(data_path, "rb") as data_file:
            data = pickle.load(data_file)

        dof_pos = np.asarray(data["dof_pos"], dtype=np.float32)[:, -14:]
        start = min(self.start_frame, dof_pos.shape[0])
        end = max(start, dof_pos.shape[0] - self.tail_trim_frames)
        applause_data = dof_pos[start:end]
        if applause_data.shape[0] == 0:
            raise ValueError(f"applause data is empty after frame trim: {data_path}")

        return applause_data, float(data["fps"])

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(ctx.teleop, with_cmd_vel=True)

    def on_enter(self, ctx: BxiExample) -> None:
        self.reset_loop(ctx)
        self.frame = 0.0
        self.return_elapsed = 0.0
        self.returning = False
        self.return_start_pos = self.applause_data[0].copy()
        self.playing = True

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        qpos = ctx.teleop.target_dof_pos.copy()
        qpos[-14:] = self.applause_data[0]
        return self._motor_frame(qpos, ctx.teleop.kps, ctx.teleop.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        qpos, vel = ctx.teleop.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            ctx.current_cmd_vel,
        )

        if self.returning:
            alpha = min(1.0, self.return_elapsed / self.return_time)
            qpos[-14:] = (
                self.return_start_pos + (qpos[-14:] - self.return_start_pos) * alpha
            )
            self.return_elapsed += dt
            if self.return_elapsed >= self.return_time:
                ctx.set_motor_target(qpos, ctx.teleop.kps, ctx.teleop.kds)
                ctx.request_state(
                    "normal",
                    trigger="applause_finished",
                    transition={"base": "first_frame_switch", "enter_duration": 0.1},
                )
                return
        else:
            frame_index = int(self.frame)
            if frame_index >= self.applause_data.shape[0]:
                self.returning = True
                self.return_elapsed = 0.0
                self.return_start_pos = self.applause_data[-1].copy()
                qpos[-14:] = self.return_start_pos
            else:
                qpos[-14:] = self.applause_data[frame_index]
                if self.playing:
                    self.frame += self.fps * dt

        ctx.set_motor_target(qpos, ctx.teleop.kps, ctx.teleop.kds)

    def on_action(self, ctx: BxiExample, action_name: str) -> bool:
        if action_name != "toggle_dance_pause":
            return False

        self.playing = not self.playing
        return True


class TeleopState(RobotControlState):
    def __init__(self, name, state_id):
        super().__init__(name, state_id)
        self._recording = False
        self._record_fps = 0
        self._record_root_pos = []
        self._record_root_rot = []
        self._record_dof_pos = []

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(ctx.teleop, with_cmd_vel=True)

    def on_enter(self, ctx: BxiExample) -> None:
        self.reset_loop(ctx)
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
        l_trigger = float(getattr(ctx, "l_trigger", 0.0))
        r_trigger = float(getattr(ctx, "r_trigger", 0.0))

        if self._recording and l_trigger < 0.5 and r_trigger < 0.5:
            self._stop_recording(ctx)
            return

        if (
            not self._recording
            and l_trigger > 0.5
            and r_trigger > 0.5
        ):
            self._start_recording(ctx, dt)

        if self._recording:
            self._record_frame(ctx, qpos)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            if self._recording:
                self._stop_recording(ctx, reason="safety")
            ctx.request_state("zero_torque", trigger="safety")
            return
            
        qpos, vel = ctx.teleop.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            ctx.current_cmd_vel,
        )
        
        left_arm_range = slice(3+12, 3+12+7)
        right_arm_range = slice(3+12+7, 3+12+14)
        
        # 初始化手部状态
        if not hasattr(self, '_hand'):
            self._hand = {
                'left': {'kp': None, 'target': None, 'last_grip': 0},
                'right': {'kp': None, 'target': None, 'last_grip': 0}
            }
        
        growth_rate = 0.2  # 缓启动速度
        
        def update_hand(hand, grip, arm_range):
            state = self._hand[hand]
            current_grip_state = grip > 0.5
            last_grip_state = state['last_grip'] > 0.5
            
            # 检测状态切换
            if current_grip_state != last_grip_state:
                # 状态切换，触发缓启动/缓关闭
                state['target'] = ctx.teleop.kps[arm_range].copy()
                if state['kp'] is None:
                    state['kp'] = state['target'].copy()
                else:
                    # 状态切换，从当前值的10%开始缓启动
                    state['kp'] = [kp * 0.1 for kp in state['target']]
            
            # 如果没有目标值，初始化
            if state['target'] is None:
                state['target'] = ctx.teleop.kps[arm_range].copy()
            if state['kp'] is None:
                state['kp'] = state['target'].copy()
            
            # 缓慢调整到目标值（使用 any 检查是否有差距）
            need_update = any(abs(state['kp'][i] - state['target'][i]) > 0.001 
                            for i in range(len(state['kp'])))
            
            if need_update:
                for i in range(len(state['kp'])):
                    diff = state['target'][i] - state['kp'][i]
                    if abs(diff) > 0.001:
                        state['kp'][i] += diff * growth_rate * dt
                        # 防止过冲
                        if (diff > 0 and state['kp'][i] > state['target'][i]) or \
                        (diff < 0 and state['kp'][i] < state['target'][i]):
                            state['kp'][i] = state['target'][i]
            
            # 保存当前grip状态
            state['last_grip'] = grip
        
        # 更新左右手
        update_hand('left', ctx.l_grip, left_arm_range)
        update_hand('right', ctx.r_grip, right_arm_range)
        
        # 构建完整的 kp 列表
        kp_to_use = ctx.teleop.kps.copy()
        kp_to_use[left_arm_range] = self._hand['left']['kp']
        kp_to_use[right_arm_range] = self._hand['right']['kp']
        
        # 控制手臂位置（仅在 grip > 0.5 时）
        if ctx.l_grip > 0.5:
            qpos[left_arm_range] = ctx.l_arm
        if ctx.r_grip > 0.5:
            qpos[right_arm_range] = ctx.r_arm

        self._update_recording(ctx, dt, qpos)
        ctx.set_motor_target(qpos, kp_to_use, ctx.teleop.kds)

    def on_action(self, ctx: BxiExample, action_name: str) -> bool:
        if action_name != "toggle_dance_pause":
            return False

        self.playing = not self.playing
        return True


class RecoverState(RobotControlState):
    def __init__(self, name: str, state_id: int):
        super().__init__(name, state_id)
        self.playing = True
        self.motion_selected = False

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
        self.reset_loop(ctx)
        self.playing = True
        if not self._configure_recover_motion(ctx):
            ctx.request_state("zero_torque", trigger="recover_pose_rejected")

    def _configure_recover_motion(self, ctx: BxiExample) -> bool:
        eu_ang = quaternion_to_euler_array(ctx.quat_xyzw)
        eu_ang[eu_ang > math.pi] -= 2 * math.pi

        if eu_ang[1] < -(math.pi / 4.0):
            ctx.recover.end_frame = 880
            ctx.recover.timestep = 600
            ctx.recover.start_frame = 600
            self.motion_selected = True
            return True
        elif eu_ang[1] > (math.pi / 4.0):
            ctx.recover.end_frame = 1690
            ctx.recover.timestep = 1350
            ctx.recover.start_frame = 1350
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

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.recover.timestep > ctx.recover.end_frame:
            ctx.recover.timestep = ctx.recover.start_frame
            ctx.request_state("normal", trigger="recover_finished",transition="first_frame_switch")
            return

        qpos = ctx.recover.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
        )
        ctx.set_motor_target(qpos, ctx.recover.kps, ctx.recover.kds)

        if self.playing:
            ctx.recover.timestep += 1


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
        ctx.preheat_model(ctx.amp_run, with_cmd_vel=True)

    def on_enter(self, ctx: BxiExample) -> None:
        self.reset_loop(ctx)
        self.max_vel = 0.0
        self.pre_cmd_vel_run = np.array([0.0, 0.0, 0.0])
        self.cmd_vel_run = np.array([0.0, 0.0, 0.0])

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.amp_run.target_dof_pos, ctx.amp_run.kps, ctx.amp_run.kds
        )

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        self.cmd_vel_run[:2] = (
            0.98 * self.pre_cmd_vel_run[:2] + 0.02 * ctx.current_cmd_vel[:2]
        )
        self.cmd_vel_run[2] = ctx.current_cmd_vel[2]
        qpos, vel = ctx.amp_run.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            self.cmd_vel_run,
        )

        if vel[0] > self.max_vel:
            self.max_vel = vel[0]
        if ctx.loop_count >= 100 + int(0.3 / ctx.dt):
            print(self.max_vel)
            ctx.loop_count = int(0.3 / ctx.dt)
            self.max_vel = 0.0

        self.pre_cmd_vel_run = self.cmd_vel_run.copy()
        ctx.set_motor_target(qpos, ctx.amp_run.kps, ctx.amp_run.kds)


class NormalRunState(RobotControlState):
    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(ctx.normal_run, with_cmd_vel=True)

    def on_enter(self, ctx: BxiExample) -> None:
        self.reset_loop(ctx)
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

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        qpos = ctx.normal_run.infer_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_xyzw,
            ctx.current_omega,
            ctx.current_cmd_vel,
        )
        ctx.set_motor_target(
            qpos,
            ctx.normal_run.joint_stiffness,
            ctx.normal_run.joint_damping,
        )


def _state_behavior_classes():
    def walk_subclasses(base_class):
        for subclass in base_class.__subclasses__():
            yield subclass
            yield from walk_subclasses(subclass)

    return {cls.__name__: cls for cls in walk_subclasses(RobotControlState)}


def _allocate_state_id(state_name, state_config, used_ids, next_id):
    configured_id = (state_config or {}).get("id")
    if configured_id is not None:
        state_id = int(configured_id)
        if state_id in used_ids:
            raise ValueError(f"duplicate state id {state_id} for state: {state_name}")
        used_ids.add(state_id)
        next_id = max(next_id, state_id + 1)
        return state_id, next_id

    while next_id in used_ids:
        next_id += 1
    state_id = next_id
    used_ids.add(state_id)
    return state_id, next_id + 1


def build_robot_states(config):
    states_config = config.get("states", {})
    if not states_config:
        raise ValueError("state machine config must define states")

    behavior_classes = _state_behavior_classes()
    states = {}
    used_ids = set()
    next_id = 0

    for state_name, state_config in states_config.items():
        state_config = state_config or {}
        behavior_name = state_config.get("behavior")
        if not behavior_name:
            raise ValueError(f"state '{state_name}' must define behavior")

        behavior_class = behavior_classes.get(behavior_name)
        if behavior_class is None:
            raise ValueError(
                f"unknown state behavior '{behavior_name}' for state '{state_name}'"
            )

        state_id, next_id = _allocate_state_id(
            state_name, state_config, used_ids, next_id
        )
        params = state_config.get("params", {}) or {}
        states[state_name] = behavior_class(state_name, state_id, **params)

    return states
