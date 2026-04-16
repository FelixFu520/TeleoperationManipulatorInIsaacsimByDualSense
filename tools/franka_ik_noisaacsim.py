#!/usr/bin/env python3
"""Franka Panda 逆运动学(IK)求解工具 —— 无 Isaac Sim 依赖版本。

使用 pinocchio + pink 替代 lula，可在独立 conda 环境中运行。
依赖: pip install pin pin-pink daqp

运行方式:
    python tools/franka_ik_noisaacsim.py
    python tools/franka_ik_noisaacsim.py --x 0.4 --y 0.0 --z 0.5 --roll 0 --pitch 3.14 --yaw 0
    python tools/franka_ik_noisaacsim.py --interactive
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask, DampingTask, LowAccelerationTask, PostureTask
from pink.limits import ConfigurationLimit, VelocityLimit

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

MOTION_GEN_EXT = os.path.join(
    PROJECT_DIR,
    "app/exts/isaacsim.robot_motion.motion_generation",
)
FRANKA_CONFIG_DIR = os.path.join(MOTION_GEN_EXT, "motion_policy_configs/franka")
URDF_PATH = os.path.join(FRANKA_CONFIG_DIR, "lula_franka_gen.urdf")

JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7",
]
JOINT_LIMITS_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_LIMITS_UPPER = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

DEFAULT_Q = np.array([0.00, -1.3, 0.00, -2.87, 0.00, 2.00, 0.75])

EE_FRAME = "panda_hand"

LOCK_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]

IK_MAX_ITERS = 300
IK_DT = 0.01
IK_POS_THRESHOLD = 0.005
IK_ORI_THRESHOLD = 0.05


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------

def euler_to_rot_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """将 (roll, pitch, yaw) 欧拉角 (ZYX 内旋) 转为 3x3 旋转矩阵。"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    return Rz @ Ry @ Rx


def rot_matrix_to_euler(R: np.ndarray) -> tuple[float, float, float]:
    """从 3x3 旋转矩阵提取 (roll, pitch, yaw) 欧拉角 (ZYX 内旋)。"""
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


def check_joint_limits(joint_positions: np.ndarray, margin: float = 0.0) -> list[str]:
    """检查关节值是否超出限位，返回告警信息列表。"""
    warnings = []
    for i, (name, q, lo, hi) in enumerate(
        zip(JOINT_NAMES, joint_positions, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
    ):
        if q < lo + margin:
            warnings.append(f"  {name}: {q:.4f} rad 接近/超出下限 {lo:.4f} rad")
        elif q > hi - margin:
            warnings.append(f"  {name}: {q:.4f} rad 接近/超出上限 {hi:.4f} rad")
    return warnings


# ---------------------------------------------------------------------------
# Pinocchio 模型构建 (处理重名帧兼容性)
# ---------------------------------------------------------------------------

def _build_reduced_model(
    urdf_path: str,
    lock_joint_names: list[str],
    base_placement: pin.SE3 | None = None,
) -> pin.Model:
    """从 URDF 构建简化模型，锁定指定关节。

    新版 pinocchio (>=3) 在 buildReducedModel 时若 URDF 存在同名帧
    (FIXED_JOINT + BODY) 会报错。此处通过先去重帧名来绕过该问题。
    """
    full_model = pin.buildModelFromUrdf(urdf_path)

    # --- 处理重名帧: 给 FIXED_JOINT 类型的重名帧追加后缀 ---
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


def _compute_fk(
    model: pin.Model, data: pin.Data, q: np.ndarray, frame_id: int
) -> pin.SE3:
    """正运动学：返回指定帧的 SE3 位姿。"""
    pin.forwardKinematics(model, data, q[:model.nq])
    pin.updateFramePlacements(model, data)
    return data.oMf[frame_id]


# ---------------------------------------------------------------------------
# IK 求解器
# ---------------------------------------------------------------------------

class FrankaIKSolver:
    """基于 pinocchio + pink 的 Franka IK 求解器封装。

    接口与原 lula 版本 (franka_ik.py) 兼容。
    """

    def __init__(
        self,
        urdf_path: str = URDF_PATH,
        ee_frame: str = EE_FRAME,
        lock_joints: list[str] | None = None,
        ik_config: dict | None = None,
    ):
        assert os.path.isfile(urdf_path), f"URDF 文件不存在: {urdf_path}"

        if lock_joints is None:
            lock_joints = LOCK_JOINTS

        self._model = _build_reduced_model(urdf_path, lock_joints)
        self._data = self._model.createData()
        self._ee_frame = ee_frame
        self._ee_frame_id = self._model.getFrameId(ee_frame)

        cfg = {
            "position_cost": 1.0,
            "orientation_cost": 1.0,
            "damping_cost": 0.01,
            "low_acc_cost": 0.01,
            "velocity_limit": 2.0,
            "lm_damping": 0.001,
            "posture_cost": 0.0,
        }
        if ik_config:
            cfg.update(ik_config)

        self._dt = IK_DT
        self._max_iters = IK_MAX_ITERS
        self._pos_threshold = IK_POS_THRESHOLD
        self._ori_threshold = IK_ORI_THRESHOLD

        # Pink 任务 & 限制
        self._ee_task = FrameTask(
            ee_frame,
            position_cost=cfg["position_cost"],
            orientation_cost=cfg["orientation_cost"],
            lm_damping=cfg["lm_damping"],
        )
        self._damping_task = DampingTask(cost=cfg["damping_cost"])
        self._low_acc_task = LowAccelerationTask(cost=cfg["low_acc_cost"])
        self._posture_task = PostureTask(cost=cfg["posture_cost"])

        self._config_limit = ConfigurationLimit(self._model)
        self._velocity_limit = VelocityLimit(self._model)
        self._model.velocityLimit[:] = cfg["velocity_limit"]

        # QP 求解器选择
        self._solver = "daqp"
        try:
            import daqp  # noqa: F401
        except ImportError:
            self._solver = "quadprog"
            try:
                import quadprog  # noqa: F401
            except ImportError:
                self._solver = "proxqp"

    @property
    def joint_names(self) -> list[str]:
        names = []
        for i in range(1, self._model.njoints):
            name = self._model.names[i]
            if name != "universe":
                names.append(name)
        return names

    @property
    def frame_names(self) -> list[str]:
        return [self._model.frames[i].name for i in range(self._model.nframes)]

    def forward_kinematics(
        self, joint_positions: np.ndarray, frame: str | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """正运动学：关节角 -> (position, rotation_matrix)。"""
        frame = frame or self._ee_frame
        fid = self._model.getFrameId(frame)
        oMf = _compute_fk(self._model, self._data, joint_positions, fid)
        return np.array(oMf.translation), np.array(oMf.rotation)

    def solve_ik(
        self,
        target_position: np.ndarray,
        target_rot_matrix: np.ndarray,
        warm_start: np.ndarray | None = None,
        position_tolerance: float | None = None,
        orientation_tolerance: float | None = None,
    ) -> tuple[np.ndarray, bool]:
        """逆运动学求解。

        Args:
            target_position: (3,) 目标位置 [x, y, z]，单位: 米
            target_rot_matrix: (3,3) 目标旋转矩阵
            warm_start: 初始关节角猜测值
            position_tolerance: 位置误差阈值 (米)
            orientation_tolerance: 姿态误差阈值 (rad)

        Returns:
            (joint_positions, success)
        """
        pos_tol = position_tolerance or self._pos_threshold
        ori_tol = orientation_tolerance or self._ori_threshold

        target_se3 = pin.SE3(
            target_rot_matrix.astype(np.float64),
            target_position.astype(np.float64),
        )
        self._ee_task.set_target(target_se3)

        current_q = (warm_start if warm_start is not None else DEFAULT_Q).astype(np.float64)
        if len(current_q) > self._model.nq:
            current_q = current_q[:self._model.nq]

        configuration = pink.Configuration(
            self._model, self._data, current_q
        )

        tasks = [self._ee_task, self._damping_task, self._low_acc_task]
        if self._posture_task.cost > 0:
            tasks.append(self._posture_task)
        limits = [self._config_limit, self._velocity_limit]

        best_q = current_q.copy()
        best_pos_err = float("inf")

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
            ori_err = np.linalg.norm(pin.log3(rot_err_mat))

            if pos_err < best_pos_err:
                best_pos_err = pos_err
                best_q = current_q.copy()

            if pos_err < pos_tol and ori_err < ori_tol:
                return current_q, True

        return best_q, (best_pos_err < pos_tol)

    def solve_ik_from_euler(
        self,
        x: float, y: float, z: float,
        roll: float, pitch: float, yaw: float,
        warm_start: np.ndarray | None = None,
    ) -> tuple[np.ndarray, bool]:
        """从欧拉角输入求解 IK。"""
        pos = np.array([x, y, z], dtype=np.float64)
        rot = euler_to_rot_matrix(roll, pitch, yaw)
        return self.solve_ik(pos, rot, warm_start=warm_start)


# ---------------------------------------------------------------------------
# 打印 & 交互
# ---------------------------------------------------------------------------

def print_result(
    solver: FrankaIKSolver,
    joint_positions: np.ndarray,
    success: bool,
    target_pos: np.ndarray,
    target_rpy: tuple[float, float, float],
    limit_margin: float = 0.1,
) -> None:
    """格式化打印 IK 求解结果。"""
    print("=" * 65)
    print("目标位姿:")
    print(f"  位置 (x,y,z)       : ({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}) m")
    print(f"  姿态 (roll,pitch,yaw): ({target_rpy[0]:.4f}, {target_rpy[1]:.4f}, {target_rpy[2]:.4f}) rad")
    print("-" * 65)

    if success:
        print("IK 求解: 成功 ✓")
    else:
        print("IK 求解: 失败 ✗  (未收敛到目标精度，以下为近似解)")

    print("-" * 65)
    print("关节结果:")
    print(f"  {'关节名称':<18} {'角度(rad)':>10} {'角度(deg)':>10} {'下限':>10} {'上限':>10}")
    print(f"  {'─' * 18} {'─' * 10} {'─' * 10} {'─' * 10} {'─' * 10}")
    for i, (name, q, lo, hi) in enumerate(
        zip(JOINT_NAMES, joint_positions, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
    ):
        print(f"  {name:<18} {q:>10.4f} {np.degrees(q):>10.2f} {lo:>10.4f} {hi:>10.4f}")

    warnings = check_joint_limits(joint_positions, margin=limit_margin)
    if warnings:
        print("-" * 65)
        print(f"⚠ 关节限位告警 (margin={limit_margin:.2f} rad):")
        for w in warnings:
            print(w)

    fk_pos, fk_rot = solver.forward_kinematics(joint_positions)
    fk_rpy = rot_matrix_to_euler(fk_rot)
    pos_err = np.linalg.norm(fk_pos - target_pos)
    print("-" * 65)
    print("正运动学验证 (FK):")
    print(f"  实际位置 : ({fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}) m")
    print(f"  实际姿态 : ({fk_rpy[0]:.4f}, {fk_rpy[1]:.4f}, {fk_rpy[2]:.4f}) rad")
    print(f"  位置误差 : {pos_err:.6f} m")
    print("=" * 65)


def run_interactive(solver: FrankaIKSolver, limit_margin: float) -> None:
    """交互模式：循环输入目标位姿并求解。"""
    print("\n进入交互模式 (输入 q 退出)")
    print("默认初始位姿通过 FK 计算自 default_q")

    fk_pos, fk_rot = solver.forward_kinematics(DEFAULT_Q)
    fk_rpy = rot_matrix_to_euler(fk_rot)
    print(f"默认关节角: {DEFAULT_Q}")
    print(f"对应末端位置: ({fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}) m")
    print(f"对应末端姿态: ({fk_rpy[0]:.4f}, {fk_rpy[1]:.4f}, {fk_rpy[2]:.4f}) rad\n")

    last_q = DEFAULT_Q.copy()

    while True:
        try:
            raw = input("输入 x y z roll pitch yaw (空格分隔, q退出): ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出。")
            break

        if raw.lower() in ("q", "quit", "exit"):
            print("退出。")
            break
        if not raw:
            continue

        parts = raw.replace(",", " ").split()
        if len(parts) != 6:
            print("错误: 请输入6个数值 (x y z roll pitch yaw)")
            continue

        try:
            vals = [float(v) for v in parts]
        except ValueError:
            print("错误: 无法解析数值")
            continue

        x, y, z, roll, pitch, yaw = vals
        target_pos = np.array([x, y, z])

        joint_positions, success = solver.solve_ik_from_euler(
            x, y, z, roll, pitch, yaw, warm_start=last_q
        )
        print_result(solver, joint_positions, success, target_pos, (roll, pitch, yaw), limit_margin)

        if success:
            last_q = joint_positions.copy()
        print()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Franka Panda 逆运动学(IK)求解工具 (无 Isaac Sim 依赖)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  python tools/franka_ik_noisaacsim.py\n"
            "  python tools/franka_ik_noisaacsim.py --x 0.4 --y 0.0 --z 0.5 --roll 0 --pitch 3.14 --yaw 0\n"
            "  python tools/franka_ik_noisaacsim.py --interactive\n"
            "  python tools/franka_ik_noisaacsim.py --interactive --margin 0.2\n"
        ),
    )
    parser.add_argument("--x", type=float, default=None, help="目标 x 位置 (米)")
    parser.add_argument("--y", type=float, default=None, help="目标 y 位置 (米)")
    parser.add_argument("--z", type=float, default=None, help="目标 z 位置 (米)")
    parser.add_argument("--roll", type=float, default=None, help="目标 roll (弧度)")
    parser.add_argument("--pitch", type=float, default=None, help="目标 pitch (弧度)")
    parser.add_argument("--yaw", type=float, default=None, help="目标 yaw (弧度)")
    parser.add_argument("--interactive", action="store_true", help="进入交互模式")
    parser.add_argument(
        "--margin", type=float, default=0.1,
        help="关节限位告警余量 (弧度), 默认 0.1",
    )
    parser.add_argument(
        "--ee-frame", type=str, default=EE_FRAME,
        help=f"末端执行器参考坐标系, 默认: {EE_FRAME}",
    )
    parser.add_argument("--list-frames", action="store_true", help="列出所有可用坐标系后退出")
    parser.add_argument(
        "--urdf", type=str, default=URDF_PATH,
        help=f"URDF 文件路径, 默认: {URDF_PATH}",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    print("正在初始化 IK 求解器 (pinocchio + pink) ...")
    solver = FrankaIKSolver(urdf_path=args.urdf, ee_frame=args.ee_frame)
    print(f"求解器初始化完成。末端坐标系: {args.ee_frame}")
    print(f"关节: {solver.joint_names}")

    if args.list_frames:
        print("\n可用坐标系:")
        for name in solver.frame_names:
            print(f"  - {name}")
        return

    fk_pos, fk_rot = solver.forward_kinematics(DEFAULT_Q)
    fk_rpy = rot_matrix_to_euler(fk_rot)
    print(f"\n默认关节角 default_q = {DEFAULT_Q.tolist()}")
    print(f"  -> 末端位置 : ({fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}) m")
    print(f"  -> 末端姿态 : ({fk_rpy[0]:.4f}, {fk_rpy[1]:.4f}, {fk_rpy[2]:.4f}) rad")

    if args.interactive:
        run_interactive(solver, args.margin)
        return

    if args.x is not None and args.y is not None and args.z is not None:
        x, y, z = args.x, args.y, args.z
        roll = args.roll if args.roll is not None else 0.0
        pitch = args.pitch if args.pitch is not None else np.pi
        yaw = args.yaw if args.yaw is not None else 0.0
    else:
        x, y, z = fk_pos[0], fk_pos[1], fk_pos[2]
        roll, pitch, yaw = fk_rpy
        print("\n未指定目标位姿，使用默认关节角的 FK 位姿进行自验证测试:")

    target_pos = np.array([x, y, z])
    print()
    joint_positions, success = solver.solve_ik_from_euler(
        x, y, z, roll, pitch, yaw, warm_start=DEFAULT_Q
    )
    print_result(solver, joint_positions, success, target_pos, (roll, pitch, yaw), args.margin)


if __name__ == "__main__":
    main()
