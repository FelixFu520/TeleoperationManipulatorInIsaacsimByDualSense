#!/usr/bin/env python3
"""Franka Panda 逆运动学(IK)求解工具。

通过输入末端执行器的目标位姿 (x,y,z) + (roll,pitch,yaw)，
使用 Lula IK 求解器反算出 Franka 7 个关节的角度值。

运行方式:
    ./app/python.sh tools/franka_ik.py
    ./app/python.sh tools/franka_ik.py --x 0.4 --y 0.0 --z 0.5 --roll 0 --pitch 3.14 --yaw 0
    ./app/python.sh tools/franka_ik.py --interactive
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

MOTION_GEN_EXT = os.path.join(
    PROJECT_DIR,
    "app/exts/isaacsim.robot_motion.motion_generation",
)
FRANKA_CONFIG_DIR = os.path.join(MOTION_GEN_EXT, "motion_policy_configs/franka")
ROBOT_DESCRIPTOR_YAML = os.path.join(FRANKA_CONFIG_DIR, "rmpflow/robot_descriptor.yaml")
URDF_PATH = os.path.join(FRANKA_CONFIG_DIR, "lula_franka_gen.urdf")

LULA_EXT = os.path.join(PROJECT_DIR, "app/exts/isaacsim.robot_motion.lula")
LULA_PREBUNDLE = os.path.join(LULA_EXT, "pip_prebundle")
if os.path.isdir(LULA_PREBUNDLE) and LULA_PREBUNDLE not in sys.path:
    sys.path.insert(0, LULA_PREBUNDLE)

import lula

# Franka Panda 7个关节的限位 (弧度), 来源于 URDF
JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7",
]
JOINT_LIMITS_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_LIMITS_UPPER = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

DEFAULT_Q = np.array([0.00, -1.3, 0.00, -2.87, 0.00, 2.00, 0.75])

EE_FRAME = "panda_hand"


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


class FrankaIKSolver:
    """基于 Lula 的 Franka IK 求解器封装。"""

    def __init__(
        self,
        robot_description_path: str = ROBOT_DESCRIPTOR_YAML,
        urdf_path: str = URDF_PATH,
        ee_frame: str = EE_FRAME,
    ):
        assert os.path.isfile(robot_description_path), (
            f"robot_description 文件不存在: {robot_description_path}"
        )
        assert os.path.isfile(urdf_path), f"URDF 文件不存在: {urdf_path}"

        self._robot_desc = lula.load_robot(robot_description_path, urdf_path)
        self._kinematics = self._robot_desc.kinematics()
        self._ik_config = lula.CyclicCoordDescentIkConfig()
        self._ee_frame = ee_frame

        self._ik_config.position_tolerance = 0.005
        self._ik_config.orientation_tolerance = 0.05
        self._ik_config.max_num_descents = 256

    @property
    def joint_names(self) -> list[str]:
        return [
            self._robot_desc.c_space_coord_name(i)
            for i in range(self._robot_desc.num_c_space_coords())
        ]

    @property
    def frame_names(self) -> list[str]:
        return self._kinematics.frame_names()

    def forward_kinematics(
        self, joint_positions: np.ndarray, frame: str | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """正运动学：关节角 -> (position, rotation_matrix)。"""
        frame = frame or self._ee_frame
        pose = self._kinematics.pose(np.expand_dims(joint_positions, 1), frame)
        return np.array(pose.translation), np.array(pose.rotation.matrix())

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
            orientation_tolerance: 姿态误差阈值 (lula 内部格式)

        Returns:
            (joint_positions, success)
        """
        if position_tolerance is not None:
            self._ik_config.position_tolerance = position_tolerance
        if orientation_tolerance is not None:
            self._ik_config.orientation_tolerance = orientation_tolerance

        target_pose = lula.Pose3(
            lula.Rotation3(target_rot_matrix.astype(np.float64)),
            target_position.astype(np.float64),
        )

        if warm_start is not None:
            self._ik_config.cspace_seeds = [warm_start.astype(np.float64)]
        else:
            self._ik_config.cspace_seeds = [DEFAULT_Q.astype(np.float64)]

        result = lula.compute_ik_ccd(
            self._kinematics, target_pose, self._ee_frame, self._ik_config
        )
        return np.array(result.cspace_position), result.success

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

    # 关节限位检查
    warnings = check_joint_limits(joint_positions, margin=limit_margin)
    if warnings:
        print("-" * 65)
        print(f"⚠ 关节限位告警 (margin={limit_margin:.2f} rad):")
        for w in warnings:
            print(w)

    # 正运动学验证
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Franka Panda 逆运动学(IK)求解工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  ./app/python.sh tools/test_ik.py\n"
            "  ./app/python.sh tools/test_ik.py --x 0.4 --y 0.0 --z 0.5 --roll 0 --pitch 3.14 --yaw 0\n"
            "  ./app/python.sh tools/test_ik.py --interactive\n"
            "  ./app/python.sh tools/test_ik.py --interactive --margin 0.2\n"
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
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    print("正在初始化 Lula IK 求解器 ...")
    solver = FrankaIKSolver(ee_frame=args.ee_frame)
    print(f"求解器初始化完成。末端坐标系: {args.ee_frame}")
    print(f"关节: {solver.joint_names}")

    if args.list_frames:
        print("\n可用坐标系:")
        for name in solver.frame_names:
            print(f"  - {name}")
        return

    # 先用默认关节角做一次 FK，展示默认位姿
    fk_pos, fk_rot = solver.forward_kinematics(DEFAULT_Q)
    fk_rpy = rot_matrix_to_euler(fk_rot)
    print(f"\n默认关节角 default_q = {DEFAULT_Q.tolist()}")
    print(f"  -> 末端位置 : ({fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}) m")
    print(f"  -> 末端姿态 : ({fk_rpy[0]:.4f}, {fk_rpy[1]:.4f}, {fk_rpy[2]:.4f}) rad")

    if args.interactive:
        run_interactive(solver, args.margin)
        return

    # 单次求解模式
    if args.x is not None and args.y is not None and args.z is not None:
        x, y, z = args.x, args.y, args.z
        roll = args.roll if args.roll is not None else 0.0
        pitch = args.pitch if args.pitch is not None else np.pi
        yaw = args.yaw if args.yaw is not None else 0.0
    else:
        # 没有指定目标，使用默认关节角的 FK 位姿作为测试
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
