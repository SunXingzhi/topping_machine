"""Delta 机械臂正逆运动学。

实现来自项目文档 01_运动学建模.md 的闭式解，不依赖 scipy。
所有函数同时支持单点和批量输入。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import DeltaGeometry


@dataclass(frozen=True)
class IKResult:
    theta: np.ndarray
    ok: np.ndarray
    ratio: np.ndarray
    branch_gap: np.ndarray


@dataclass(frozen=True)
class FKResult:
    P: np.ndarray
    B: np.ndarray
    A: np.ndarray
    ok: np.ndarray
    branch: str


def wrap(x: np.ndarray) -> np.ndarray:
    """把角度折叠到 (-pi, pi]。"""
    return np.remainder(x + np.pi, 2.0 * np.pi) - np.pi


def elbow_points(theta: np.ndarray, geom: DeltaGeometry) -> tuple[np.ndarray, np.ndarray]:
    """返回肘点 B_i 与等效球心 E_i，形状 (..., 3 腿, 3 分量)。"""
    theta = np.asarray(theta, dtype=float)
    c, s = np.cos(theta), np.sin(theta)
    B = (geom.R + geom.L * c)[..., None] * geom.u_dir + (
        geom.L * s
    )[..., None] * geom.z_axis
    E = B - geom.platform_offset_vec
    return B, E


def platform_attachment_points(P: np.ndarray, geom: DeltaGeometry) -> np.ndarray:
    """动平台铰点 A_i = P + r*u_i + L1*ball_axis_i。"""
    P = np.asarray(P, dtype=float)
    return P[..., None, :] + geom.platform_offset_vec


def forward_kinematics(
    theta: np.ndarray,
    geom: DeltaGeometry,
    lower: bool = True,
) -> FKResult:
    """由三个主动臂角度求动平台中心。

    lower=True 取 z 较小的一支（悬挂式 Delta 的常规装配）；
    轨迹跟踪建议使用 forward_kinematics_continuous。
    """
    theta = np.asarray(theta, dtype=float)
    B, E = elbow_points(theta, geom)
    E1, E2, E3 = E[..., 0, :], E[..., 1, :], E[..., 2, :]

    v1, v2 = E2 - E1, E3 - E1
    n_raw = np.cross(v1, v2)
    nn = np.linalg.norm(n_raw, axis=-1)
    ok = nn > 1e-9
    n = n_raw / np.where(ok, nn, 1.0)[..., None]

    a11 = np.sum(v1 * v1, axis=-1)
    a12 = np.sum(v1 * v2, axis=-1)
    a22 = np.sum(v2 * v2, axis=-1)
    beta1 = 0.5 * (np.sum(E2 * E2, -1) - np.sum(E1 * E1, -1))
    beta2 = 0.5 * (np.sum(E3 * E3, -1) - np.sum(E1 * E1, -1))
    b1 = beta1 - np.sum(E1 * v1, axis=-1)
    b2 = beta2 - np.sum(E1 * v2, axis=-1)

    det = a11 * a22 - a12 * a12
    ok = ok & (np.abs(det) > 1e-9)
    ds = np.where(np.abs(det) > 1e-9, det, 1.0)
    alpha = (b1 * a22 - b2 * a12) / ds
    beta = (a11 * b2 - a12 * b1) / ds

    W2 = (
        alpha * alpha * a11
        + beta * beta * a22
        + 2.0 * alpha * beta * a12
    )
    g2 = geom.l * geom.l - W2
    ok = ok & (g2 >= 0.0)
    g = np.sqrt(np.maximum(g2, 0.0))

    mid = E1 + alpha[..., None] * v1 + beta[..., None] * v2
    p_lo = mid - g[..., None] * n
    p_hi = mid + g[..., None] * n
    if lower:
        P = np.where((p_lo[..., 2] <= p_hi[..., 2])[..., None], p_lo, p_hi)
        branch = "lower"
    else:
        P = np.where((p_lo[..., 2] <= p_hi[..., 2])[..., None], p_hi, p_lo)
        branch = "upper"

    return FKResult(
        P=P,
        B=B,
        A=platform_attachment_points(P, geom),
        ok=ok,
        branch=branch,
    )


def forward_kinematics_continuous(
    theta: np.ndarray,
    geom: DeltaGeometry,
    prev_P: np.ndarray | None = None,
) -> FKResult:
    """按上一周期平台位置选择 FK 分支，避免轨迹中跳支。"""
    lower = forward_kinematics(theta, geom, lower=True)
    upper = forward_kinematics(theta, geom, lower=False)
    if prev_P is None:
        return lower

    prev_P = np.asarray(prev_P, dtype=float)
    lower_dist = np.linalg.norm(lower.P - prev_P, axis=-1)
    upper_dist = np.linalg.norm(upper.P - prev_P, axis=-1)
    use_lower = lower_dist <= upper_dist

    P = np.where(use_lower[..., None], lower.P, upper.P)
    B = lower.B
    A = platform_attachment_points(P, geom)
    ok = np.where(use_lower, lower.ok, upper.ok)
    branch = "lower" if bool(np.all(use_lower)) else "upper"
    return FKResult(P=P, B=B, A=A, ok=ok, branch=branch)


def inverse_kinematics(
    P: np.ndarray,
    geom: DeltaGeometry,
    prev: np.ndarray | None = None,
    branch: str = "plus",
) -> IKResult:
    """由动平台中心求三个主动臂角度。

    prev 给定时按角度连续性选支；否则按 branch 选 plus/minus。
    """
    P = np.asarray(P, dtype=float)
    a = np.einsum("...k,ik->...i", P, geom.u_dir)
    b = np.einsum("...k,ik->...i", P, geom.t_dir)
    z = P[..., 2][..., None] + geom.platform_offset_z

    ap = a + geom.r + geom.platform_offset_r - geom.R
    rho = np.hypot(ap, z)
    safe = rho > 1e-9

    d = (
        ap * ap + b * b + z * z + geom.L * geom.L - geom.l * geom.l
    ) / (2.0 * geom.L)
    ratio = np.where(safe, d / np.where(safe, rho, 1.0), np.inf)

    ok = np.all(safe, axis=-1) & np.all(np.abs(ratio) <= 1.0, axis=-1)

    psi = np.arctan2(z, ap)
    delta = np.arccos(np.clip(ratio, -1.0, 1.0))
    th_p = psi + delta
    th_m = psi - delta

    if prev is None:
        if branch not in ("plus", "minus"):
            raise ValueError("branch 必须是 'plus' 或 'minus'")
        theta = th_p if branch == "plus" else th_m
    else:
        prev = np.asarray(prev, dtype=float)
        use_p = np.abs(wrap(th_p - prev)) <= np.abs(wrap(th_m - prev))
        theta = np.where(use_p, th_p, th_m)

    return IKResult(
        theta=theta,
        ok=ok,
        ratio=ratio,
        branch_gap=2.0 * delta,
    )


def leg_solvable(P: np.ndarray, geom: DeltaGeometry) -> np.ndarray:
    """单腿环面判据，返回 (..., 3) 布尔数组。"""
    P = np.asarray(P, dtype=float)
    a = np.einsum("...k,ik->...i", P, geom.u_dir)
    b = np.einsum("...k,ik->...i", P, geom.t_dir)
    z = P[..., 2][..., None] + geom.platform_offset_z
    ap = a + geom.r + geom.platform_offset_r - geom.R
    rho = np.hypot(ap, z)
    return ((rho - geom.L) ** 2 + b * b <= geom.l * geom.l) & (
        geom.l * geom.l <= (rho + geom.L) ** 2 + b * b
    )


def reachable(P: np.ndarray, geom: DeltaGeometry) -> np.ndarray:
    """三条腿同时可解才算 P 可达。"""
    return np.all(leg_solvable(P, geom), axis=-1)
