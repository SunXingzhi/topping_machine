"""Delta 机械臂几何参数与派生坐标系。

约定：
- 基座坐标系原点在静平台中心，Z 轴向上；
- 主动臂转角 theta 从水平径向量起，向上为正；
- 长度单位米，角度单位弧度。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class DeltaGeometry:
    """静平台、动平台和连杆的几何参数。"""

    R: float = 0.139
    r: float = 0.039
    L: float = 0.140
    l: float = 0.200
    phi: tuple[float, float, float] = (
        0.0,
        2.0 * math.pi / 3.0,
        4.0 * math.pi / 3.0,
    )
    theta_min: float = -math.pi
    theta_max: float = math.pi
    # 平台端球铰偏移；当前可视化先用 0，后续做 C2/C3 时可打开。
    platform_offset: float = 0.0
    ball_tilt: float = 0.0

    def __post_init__(self) -> None:
        if self.R <= 0.0 or self.r <= 0.0:
            raise ValueError("R 和 r 必须为正数")
        if self.R <= self.r:
            raise ValueError("静平台半径 R 必须大于动平台半径 r")
        if self.L <= 0.0 or self.l <= 0.0:
            raise ValueError("L 和 l 必须为正数")
        if len(self.phi) != 3:
            raise ValueError("phi 必须包含三条腿的角度")
        if self.theta_min >= self.theta_max:
            raise ValueError("theta_min 必须小于 theta_max")
        if self.platform_offset < 0.0:
            raise ValueError("platform_offset 不能为负")
        if not (-math.pi / 2.0 <= self.ball_tilt <= math.pi / 2.0):
            raise ValueError("ball_tilt 必须在 [-pi/2, pi/2] 内")

    @property
    def u_dir(self) -> np.ndarray:
        """三条腿的径向单位向量，形状 (3, 3)。"""
        phi = np.asarray(self.phi, dtype=float)
        return np.stack(
            [np.cos(phi), np.sin(phi), np.zeros(3, dtype=float)],
            axis=-1,
        )

    @property
    def t_dir(self) -> np.ndarray:
        """三条腿的切向单位向量，形状 (3, 3)。"""
        phi = np.asarray(self.phi, dtype=float)
        return np.stack(
            [-np.sin(phi), np.cos(phi), np.zeros(3, dtype=float)],
            axis=-1,
        )

    @property
    def z_axis(self) -> np.ndarray:
        return np.array([0.0, 0.0, 1.0], dtype=float)

    @property
    def ball_axis(self) -> np.ndarray:
        """平台端球窝轴，形状 (3, 3)。"""
        return np.sin(self.ball_tilt) * self.u_dir + np.cos(
            self.ball_tilt
        ) * self.z_axis

    @property
    def platform_offset_vec(self) -> np.ndarray:
        """动平台铰点相对平台中心的偏移，形状 (3, 3)。"""
        return self.r * self.u_dir + self.platform_offset * self.ball_axis

    @property
    def platform_offset_z(self) -> float:
        return self.platform_offset * math.cos(self.ball_tilt)

    @property
    def platform_offset_r(self) -> float:
        return self.platform_offset * math.sin(self.ball_tilt)
