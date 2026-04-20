#!/usr/bin/env python3
"""DualSense 手柄遥操 IsaacSim 中 Franka 机械臂。

实时读取 DualSense 手柄信号，映射到末端位姿增量，
通过 IK 求解关节角，再经 ROS2 发布到 IsaacSim。

运行方式 (conda activate tele):
    sudo python teleoperation_manipulator.py --device /dev/input/event6
    sudo python teleoperation_manipulator.py --device /dev/input/event6 --rate 20
"""

from __future__ import annotations

import argparse
import os
import signal
import select
import sys
import threading
import time

import numpy as np
from evdev import InputDevice, ecodes

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

MOTION_GEN_EXT = os.path.join(
    SCRIPT_DIR,
    "app/exts/isaacsim.robot_motion.motion_generation",
)
FRANKA_CONFIG_DIR = os.path.join(MOTION_GEN_EXT, "motion_policy_configs/franka")
URDF_PATH = os.path.join(FRANKA_CONFIG_DIR, "lula_franka_gen.urdf")

JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7",
]
DEFAULT_Q = np.array([0.00, -1.3, 0.00, -2.87, 0.00, 2.00, 0.75])
EE_FRAME = "panda_hand"
LOCK_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]

# 末端位姿工作空间范围 (世界坐标系下的安全边界)
# 注意 z 下限 > 0 避免撞到桌面/基座
POS_RANGE = {
    "x": (0.15, 0.75),
    "y": (-0.55, 0.55),
    "z": (0.10, 0.90),
}
# Franka 基座到末端的可达球壳 (用于 IK 前的粗筛)
REACH_MIN = 0.18
REACH_MAX = 0.85

STICK_CENTER = 127
STICK_DEADZONE = 10
SENSITIVITY_POS = 0.002
SENSITIVITY_ORI = 0.02
GRIPPER_MAX = 0.04


# ---------------------------------------------------------------------------
# IK 求解器 (来自 franka_ik_noisaacsim.py)
# ---------------------------------------------------------------------------

import pinocchio as pin
import pink
from pink.tasks import FrameTask, DampingTask, LowAccelerationTask, PostureTask
from pink.limits import ConfigurationLimit, VelocityLimit


def euler_to_rot_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def rot_matrix_to_euler(R: np.ndarray) -> tuple[float, float, float]:
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _build_reduced_model(
    urdf_path: str,
    lock_joint_names: list[str],
    base_placement: pin.SE3 | None = None,
) -> pin.Model:
    full_model = pin.buildModelFromUrdf(urdf_path)
    seen: dict[str, int] = {}
    for i in range(full_model.nframes):
        name = full_model.frames[i].name
        if name in seen:
            full_model.frames[i].name = f"{name}__joint"
        else:
            seen[name] = i

    lock_ids = []
    for name in lock_joint_names:
        for i in range(full_model.njoints):
            if full_model.names[i] == name:
                lock_ids.append(i)
                break

    q_ref = pin.neutral(full_model)
    model = pin.buildReducedModel(full_model, lock_ids, q_ref)
    if base_placement is not None:
        model.jointPlacements[1] = base_placement * model.jointPlacements[1]
    return model


class FrankaIKSolver:
    def __init__(
        self,
        urdf_path: str = URDF_PATH,
        ee_frame: str = EE_FRAME,
        lock_joints: list[str] | None = None,
    ):
        assert os.path.isfile(urdf_path), f"URDF 文件不存在: {urdf_path}"
        if lock_joints is None:
            lock_joints = LOCK_JOINTS

        self._model = _build_reduced_model(urdf_path, lock_joints)
        self._data = self._model.createData()
        self._ee_frame = ee_frame
        self._ee_frame_id = self._model.getFrameId(ee_frame)

        self._dt = 0.02
        self._max_iters = 60
        self._pos_threshold = 0.005
        self._ori_threshold = 0.05
        # 早停: 连续若干次误差不再下降就认为收敛/卡住
        self._stall_tolerance = 1e-5
        self._stall_patience = 8

        # 增大 lm_damping 让接近奇异时更稳 (代价: 收敛稍慢, 但更不易发散)
        self._ee_task = FrameTask(
            ee_frame, position_cost=1.0, orientation_cost=1.0, lm_damping=0.01,
        )
        self._damping_task = DampingTask(cost=0.05)
        self._low_acc_task = LowAccelerationTask(cost=0.01)
        # 给姿态任务一个弱拉向 DEFAULT_Q 的姿态约束,
        # 避免长时间漂移到关节极限附近
        self._posture_task = PostureTask(cost=0.005)
        self._posture_task.set_target(DEFAULT_Q.astype(np.float64))

        self._config_limit = ConfigurationLimit(self._model)
        self._velocity_limit = VelocityLimit(self._model)
        self._model.velocityLimit[:] = 2.0

        self._solver = "daqp"
        try:
            import daqp  # noqa: F401
        except ImportError:
            self._solver = "quadprog"
            try:
                import quadprog  # noqa: F401
            except ImportError:
                self._solver = "proxqp"

    def forward_kinematics(
        self, joint_positions: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        pin.forwardKinematics(self._model, self._data, joint_positions[:self._model.nq])
        pin.updateFramePlacements(self._model, self._data)
        oMf = self._data.oMf[self._ee_frame_id]
        return np.array(oMf.translation), np.array(oMf.rotation)

    def solve_ik(
        self,
        target_position: np.ndarray,
        target_rot_matrix: np.ndarray,
        warm_start: np.ndarray | None = None,
    ) -> tuple[np.ndarray, bool]:
        # 可达性粗筛: 目标离基座太近或太远直接放弃, 省得陷入奇异
        reach = float(np.linalg.norm(target_position))
        if reach < REACH_MIN or reach > REACH_MAX:
            q0 = (warm_start if warm_start is not None else DEFAULT_Q).astype(np.float64)
            return q0[:self._model.nq].copy(), False

        target_se3 = pin.SE3(
            target_rot_matrix.astype(np.float64),
            target_position.astype(np.float64),
        )
        self._ee_task.set_target(target_se3)

        current_q = (warm_start if warm_start is not None else DEFAULT_Q).astype(np.float64)
        if len(current_q) > self._model.nq:
            current_q = current_q[:self._model.nq]

        configuration = pink.Configuration(self._model, self._data, current_q)
        tasks = [self._ee_task, self._damping_task, self._low_acc_task, self._posture_task]
        limits = [self._config_limit, self._velocity_limit]

        best_q = current_q.copy()
        best_pos_err = float("inf")
        best_ori_err = float("inf")
        prev_err = float("inf")
        stall_count = 0

        for _ in range(self._max_iters):
            try:
                velocity = pink.solve_ik(
                    configuration, tasks, self._dt,
                    solver=self._solver, limits=limits, safety_break=False,
                )
            except Exception:
                break
            configuration.integrate_inplace(velocity, self._dt)
            current_q = configuration.q.copy()

            fk_pos, fk_rot = self.forward_kinematics(current_q)
            pos_err = np.linalg.norm(fk_pos - target_position)
            rot_err_mat = target_rot_matrix.T @ fk_rot
            ori_err = float(np.linalg.norm(pin.log3(rot_err_mat)))

            total_err = pos_err + 0.1 * ori_err
            if total_err < best_pos_err + 0.1 * best_ori_err:
                best_pos_err = pos_err
                best_ori_err = ori_err
                best_q = current_q.copy()

            if pos_err < self._pos_threshold and ori_err < self._ori_threshold:
                return current_q, True

            # 误差不再下降 -> 陷入局部最优, 提前结束省时间
            if abs(prev_err - total_err) < self._stall_tolerance:
                stall_count += 1
                if stall_count >= self._stall_patience:
                    break
            else:
                stall_count = 0
            prev_err = total_err

        success = (
            best_pos_err < self._pos_threshold
            and best_ori_err < self._ori_threshold
        )
        return best_q, success

    def solve_ik_from_euler(
        self, x: float, y: float, z: float,
        roll: float, pitch: float, yaw: float,
        warm_start: np.ndarray | None = None,
    ) -> tuple[np.ndarray, bool]:
        pos = np.array([x, y, z], dtype=np.float64)
        rot = euler_to_rot_matrix(roll, pitch, yaw)
        return self.solve_ik(pos, rot, warm_start=warm_start)


# ---------------------------------------------------------------------------
# DualSense 手柄读取
# ---------------------------------------------------------------------------

class DualSenseReader:
    """非阻塞读取 DualSense 手柄状态。"""

    # 摇杆轴 (中心值 127) 与触发器轴 (中心值 0) 分开初始化
    _STICK_AXES = (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY)
    _TRIGGER_AXES = (ecodes.ABS_Z, ecodes.ABS_RZ)
    _HAT_AXES = (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y)

    def __init__(self, device_path: str):
        self._device = InputDevice(device_path)
        self._lock = threading.Lock()
        self._running = True
        self._initialized = False

        self._values: dict[tuple[int, int], int] = {}
        for code in (ecodes.BTN_SOUTH, ecodes.BTN_EAST, ecodes.BTN_NORTH,
                     ecodes.BTN_WEST, ecodes.BTN_TL, ecodes.BTN_TR,
                     ecodes.BTN_TL2, ecodes.BTN_TR2, ecodes.BTN_SELECT,
                     ecodes.BTN_START, ecodes.BTN_MODE, ecodes.BTN_THUMBL,
                     ecodes.BTN_THUMBR):
            self._values[(ecodes.EV_KEY, code)] = 0
        for code in self._STICK_AXES:
            self._values[(ecodes.EV_ABS, code)] = STICK_CENTER
        for code in self._TRIGGER_AXES:
            self._values[(ecodes.EV_ABS, code)] = 0
        for code in self._HAT_AXES:
            self._values[(ecodes.EV_ABS, code)] = 0

        # 从设备读取当前真实值作为初始状态
        abs_info = self._device.capabilities(absinfo=True).get(ecodes.EV_ABS, [])
        for code, info in abs_info:
            self._values[(ecodes.EV_ABS, code)] = info.value

        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self) -> None:
        while self._running:
            ready, _, _ = select.select([self._device.fd], [], [], 0.005)
            if ready:
                for event in self._device.read():
                    if event.type in (ecodes.EV_KEY, ecodes.EV_ABS):
                        with self._lock:
                            self._values[(event.type, event.code)] = event.value

    def get(self, event_type: int, code: int) -> int:
        with self._lock:
            return self._values.get((event_type, code), 0)

    @property
    def l3_pressed(self) -> bool:
        return self.get(ecodes.EV_KEY, ecodes.BTN_THUMBL) == 1

    @property
    def left_stick_x(self) -> int:
        return self.get(ecodes.EV_ABS, ecodes.ABS_X)

    @property
    def left_stick_y(self) -> int:
        return self.get(ecodes.EV_ABS, ecodes.ABS_Y)

    @property
    def right_stick_x(self) -> int:
        return self.get(ecodes.EV_ABS, ecodes.ABS_RX)

    @property
    def right_stick_y(self) -> int:
        return self.get(ecodes.EV_ABS, ecodes.ABS_RY)

    @property
    def r2_analog(self) -> int:
        return self.get(ecodes.EV_ABS, ecodes.ABS_RZ)

    @property
    def btn_triangle(self) -> bool:
        return self.get(ecodes.EV_KEY, ecodes.BTN_NORTH) == 1

    @property
    def btn_cross(self) -> bool:
        return self.get(ecodes.EV_KEY, ecodes.BTN_SOUTH) == 1

    @property
    def dpad_y(self) -> int:
        return self.get(ecodes.EV_ABS, ecodes.ABS_HAT0Y)

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)


# ---------------------------------------------------------------------------
# 摇杆信号 -> 末端位姿增量
# ---------------------------------------------------------------------------

def stick_to_delta(value: int, center: int = STICK_CENTER, deadzone: int = STICK_DEADZONE) -> float:
    """将摇杆原始值 [0, 255] 映射到 [-1.0, 1.0], 含死区。"""
    diff = value - center
    if abs(diff) < deadzone:
        return 0.0
    sign = 1.0 if diff > 0 else -1.0
    magnitude = (abs(diff) - deadzone) / (center - deadzone)
    return sign * min(magnitude, 1.0)


def compute_target_delta(
    reader: DualSenseReader,
    sensitivity_pos: float,
    sensitivity_ori: float,
) -> tuple[float, float, float, float, float, float, float]:
    """根据手柄状态计算末端位姿增量和夹爪值。

    返回的 (dx,dy,dz) 与 (droll,dpitch,dyaw) 都定义在末端局部坐标系中,
    由主循环再叠加到世界坐标系下的末端位姿上。

    映射规则:
        左摇杆 左右      -> 末端 y 轴平移
        左摇杆 上下      -> 末端 x 轴平移 (上=沿末端x正向)
        △ (三角键)      -> 末端 z 轴正向平移
        X (交叉键)      -> 末端 z 轴负向平移
        右摇杆 左右      -> 绕末端 x 轴旋转 (roll)
        右摇杆 上下      -> 绕末端 y 轴旋转 (pitch)
        方向键 上        -> 绕末端 z 轴旋转 (yaw 增大)
        方向键 下        -> 绕末端 z 轴旋转 (yaw 减小)
        R2              -> 夹爪 (0=开, 255=关)
    """
    lx = stick_to_delta(reader.left_stick_x)
    ly = stick_to_delta(reader.left_stick_y)
    rx = stick_to_delta(reader.right_stick_x)
    ry = stick_to_delta(reader.right_stick_y)

    dx = -ly * sensitivity_pos            # 左摇杆上下 -> x (上=ly<0 -> x增大)
    dy = lx * sensitivity_pos             # 左摇杆左右 -> y (右=lx>0 -> y增大)

    # 末端 panda_hand 的 +z 朝向夹爪外侧 (一般朝下), 因此 △ 取负、X 取正,
    # 这样在世界系视觉上 △=向上抬, X=向下压, 与按键直觉一致
    dz = 0.0
    if reader.btn_triangle:               # △ -> 沿末端 -z (世界视觉上向上)
        dz = -sensitivity_pos
    elif reader.btn_cross:                # X -> 沿末端 +z (世界视觉上向下)
        dz = sensitivity_pos

    droll = rx * sensitivity_ori          # 右 -> roll增大
    dpitch = -ry * sensitivity_ori        # 上(ry<0) -> pitch增大

    dyaw = 0.0
    dpad = reader.dpad_y                  # 方向键上=-1, 下=1
    if dpad != 0:
        dyaw = -dpad * sensitivity_ori    # 上(-1) -> yaw增大

    gripper_ratio = reader.r2_analog / 255.0
    gripper = (1.0 - gripper_ratio) * GRIPPER_MAX  # R2=0 -> 开(max), R2=255 -> 关(0)

    return dx, dy, dz, droll, dpitch, dyaw, gripper


# ---------------------------------------------------------------------------
# ROS2 发布
# ---------------------------------------------------------------------------

def create_ros2_publisher():
    """初始化 ROS2 节点并创建 /joint_command 发布者。"""
    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("teleoperation_manipulator")
    publisher = node.create_publisher(JointState, "/joint_command", 10)
    return node, publisher, JointState


def publish_joint_command(node, publisher, JointState, joint_positions: np.ndarray, gripper: float) -> None:
    """发布 JointState 消息到 /joint_command。

    关节顺序: [panda_finger_joint1, panda_joint1, ..., panda_joint7]
    """
    import rclpy

    msg = JointState()
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = ""
    msg.name = [
        "panda_finger_joint1",
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
    ]
    msg.position = [float(gripper)] + [float(q) for q in joint_positions]
    msg.velocity = []
    msg.effort = []
    publisher.publish(msg)
    rclpy.spin_once(node, timeout_sec=0)


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="DualSense 遥操 IsaacSim Franka 机械臂",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "运行示例:\n"
            "  sudo python teleoperation_manipulator.py --device /dev/input/event6\n"
            "  sudo python teleoperation_manipulator.py --device /dev/input/event6 --rate 30\n"
        ),
    )
    parser.add_argument("--device", default="/dev/input/event6", help="DualSense 设备路径")
    parser.add_argument("--rate", type=float, default=20.0, help="控制频率 (Hz)")
    parser.add_argument("--sensitivity-pos", type=float, default=SENSITIVITY_POS,
                        help="位置灵敏度 (米/tick)")
    parser.add_argument("--sensitivity-ori", type=float, default=SENSITIVITY_ORI,
                        help="姿态灵敏度 (rad/tick)")
    parser.add_argument("--urdf", type=str, default=URDF_PATH, help="URDF 文件路径")
    parser.add_argument("--no-ros", action="store_true",
                        help="不使用 ROS2, 仅打印结果 (调试用)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    running = True

    def _handle_stop(_sig: int, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    # --- 初始化 IK 求解器 ---
    print("正在初始化 IK 求解器 (pinocchio + pink) ...")
    solver = FrankaIKSolver(urdf_path=args.urdf)
    fk_pos, fk_rot = solver.forward_kinematics(DEFAULT_Q)
    fk_rpy = rot_matrix_to_euler(fk_rot)
    print(f"初始末端位置: ({fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}) m")
    print(f"初始末端姿态: ({fk_rpy[0]:.4f}, {fk_rpy[1]:.4f}, {fk_rpy[2]:.4f}) rad")

    # --- 初始化 ROS2 ---
    node = publisher = JointState = None
    if not args.no_ros:
        print("正在初始化 ROS2 ...")
        node, publisher, JointState = create_ros2_publisher()
        print("ROS2 节点 [teleoperation_manipulator] 已创建, 发布 /joint_command")
    else:
        print("[调试模式] 不使用 ROS2, 仅打印关节角")

    # --- 初始化 DualSense ---
    print(f"正在连接 DualSense 手柄: {args.device}")
    ds = DualSenseReader(args.device)
    print("DualSense 已连接, 开始遥操控制!")

    print("=" * 60)
    print("操控映射 (末端局部坐标系):")
    print("  左摇杆 上下       -> 末端 x 轴平移 (上=沿末端x正向)")
    print("  左摇杆 左右       -> 末端 y 轴平移 (右=沿末端y正向)")
    print("  △ (三角键)       -> 末端 z 轴正向平移")
    print("  X (交叉键)       -> 末端 z 轴负向平移")
    print("  右摇杆 左右       -> 绕末端 x 轴旋转 (roll)")
    print("  右摇杆 上下       -> 绕末端 y 轴旋转 (pitch)")
    print("  方向键 上         -> 绕末端 z 轴旋转 (yaw 增大)")
    print("  方向键 下         -> 绕末端 z 轴旋转 (yaw 减小)")
    print("  R2               -> 夹爪 (松开=开, 按下=关)")
    print("  Ctrl+C           -> 退出")
    print("=" * 60)

    # 末端在世界坐标系中的位姿 (位置 + 旋转矩阵), 用矩阵避免欧拉角奇异
    # cur_pos / cur_rot 保存 "上一次 IK 成功的可行位姿"
    cur_pos = np.array([fk_pos[0], fk_pos[1], fk_pos[2]], dtype=np.float64)
    cur_rot = fk_rot.astype(np.float64).copy()
    last_q = DEFAULT_Q.copy()
    gripper = GRIPPER_MAX
    period = 1.0 / args.rate

    ik_fail_streak = 0

    # 启动时先发一次初始关节角, 确认 ROS2 通信正常
    if publisher is not None:
        publish_joint_command(node, publisher, JointState, last_q, gripper)
        print("已发送初始关节角到 /joint_command")
        time.sleep(0.5)

    while running:
        t0 = time.monotonic()

        dx, dy, dz, droll, dpitch, dyaw, gripper = compute_target_delta(
            ds, args.sensitivity_pos, args.sensitivity_ori,
        )

        # 候选目标 = 当前可行位姿 + 本帧末端局部增量
        # (不直接修改 cur_pos/cur_rot, 失败时整帧丢弃, 不会漂移)
        local_translation = np.array([dx, dy, dz], dtype=np.float64)
        cand_pos = cur_pos + cur_rot @ local_translation
        cand_pos[0] = np.clip(cand_pos[0], *POS_RANGE["x"])
        cand_pos[1] = np.clip(cand_pos[1], *POS_RANGE["y"])
        cand_pos[2] = np.clip(cand_pos[2], *POS_RANGE["z"])
        cand_rot = cur_rot @ euler_to_rot_matrix(droll, dpitch, dyaw)

        has_motion = (
            abs(dx) + abs(dy) + abs(dz) + abs(droll) + abs(dpitch) + abs(dyaw) > 1e-9
        )

        if has_motion:
            joint_positions, success = solver.solve_ik(
                cand_pos, cand_rot, warm_start=last_q,
            )
        else:
            # 无操作时不必再跑 IK, 保持上一帧关节角
            joint_positions, success = last_q, True

        if success:
            last_q = joint_positions.copy()
            cur_pos = cand_pos
            cur_rot = cand_rot
            ik_fail_streak = 0
        else:
            # 候选目标不可达: 丢弃本帧增量, 保留上一次成功位姿
            # 避免目标漂到不可达区域后越陷越深
            ik_fail_streak += 1

        if publisher is not None:
            publish_joint_command(node, publisher, JointState, last_q, gripper)

        ik_ms = (time.monotonic() - t0) * 1000
        status = "OK" if success else f"FAIL({ik_fail_streak})"
        cur_rpy = rot_matrix_to_euler(cur_rot)
        sys.stdout.write(
            f"\r[{status} {ik_ms:4.0f}ms] "
            f"pos=({cur_pos[0]:.3f},{cur_pos[1]:.3f},{cur_pos[2]:.3f}) "
            f"rpy=({cur_rpy[0]:.2f},{cur_rpy[1]:.2f},{cur_rpy[2]:.2f}) "
            f"grip={gripper:.3f} "
            f"d_local=({dx:.4f},{dy:.4f},{dz:.4f},{droll:.3f},{dpitch:.3f},{dyaw:.3f})"
            f"    "
        )
        sys.stdout.flush()

        elapsed = time.monotonic() - t0
        sleep_time = period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    # --- 清理 ---
    print("\n正在退出 ...")
    ds.stop()
    if node is not None:
        node.destroy_node()
        import rclpy
        rclpy.shutdown()
    print("已退出。")


if __name__ == "__main__":
    main()
