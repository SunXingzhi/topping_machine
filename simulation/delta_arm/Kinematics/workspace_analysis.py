"""
Delta 机械臂正运动学 + 约束工作空间 / Z 轴行程分析

用途
  1) 正解 FK：输入三个主动臂关节角 θ = [θ1, θ2, θ3]，输出动平台中心 P = (x, y, z)
  2) 关节空间正扫：遍历 θ ∈ [MIN_THETA, MAX_THETA]，得到同时满足关节限位、
     臂端关节约束、平台端球铰锥角、杆间干涉的真实工作空间
     （正扫不依赖逆解分支选择，可规避 pose_viewer.py 中 branch=-1 取到"肘内"解的问题）
  3) 输出：全局 Z 行程、不同 XY 半径下的 Z 可用区间、不同 Z 高度下的最大可达半径

几何参数与 pose_viewer.py 保持一致，单位 mm。
"""

import numpy as np
from pathlib import Path

# ==========================================
# 1. 几何参数与物理约束（与 pose_viewer.py 一致，单位 mm）
# ==========================================
R = 139.0  # 静平台半径（固定铰点 C_i 分布圆）
r = 39.0  # 动平台半径（动铰点 A_i 分布圆）
L = 140.0  # 主动臂长度
l = 200.0  # 从动臂长度
H = 0.0  # 静平台高度（Z = 0 平面）

MAX_THETA = np.pi / 3.0  # 主动臂最高 +60°（相对水平径向）
MIN_THETA = -np.pi / 2.0  # 主动臂最低 -90°

# ---- 关节配置：平行四边形从动臂（每腿 2 杆）----
# 臂端关节类型："ball" = 球头铰链（锥角约束）；"ujoint" = 十字万向节（双轴摆角约束）
ARM_JOINT = "ball"
# 球铰允许的锥角：从动臂相对球窝轴的最大偏摆角（ARM_JOINT="ball" 时生效）
MAX_BALL_CONE = np.deg2rad(40.0)
# 万向节单方向摆角上限（ARM_JOINT="ujoint" 时生效）；面内/面外相互独立
UJ_ALPHA_MAX = np.deg2rad(110.0)
UJ_BETA_MAX = np.deg2rad(90.0)
# 球窝轴的安装偏置角：按"主动臂下垂"位姿的从动臂方向确定，
# 使锥角围绕行程中心对称展开（= asin((R-r)/l)，本例为 30°）
MOUNT_ANGLE = np.arcsin(np.clip((R - r) / l, -1.0, 1.0))
# 平行四边形叉口间距 d（两根杆中心距，需 > ROD_DIAMETER）
FORK_WIDTH = 30.0
ROD_DIAMETER = 15.0  # 连杆直径，用于干涉判定

# 三条腿的方位角、径向单位向量 u_i 与切向单位向量 t_i
PHI = np.array([0.0, 2.0 * np.pi / 3.0, 4.0 * np.pi / 3.0])
UX, UY = np.cos(PHI), np.sin(PHI)
U_DIR = np.stack([UX, UY, np.zeros(3)], axis=-1)  # (3,3) 径向
T_DIR = np.stack([-UY, UX, np.zeros(3)], axis=-1)  # (3,3) 切向，叉口间距沿此方向
U_OFFSET = r * U_DIR  # 动平台铰点相对中心偏移 (3,3)

# 平台端球窝轴：在方位角平面内，相对竖直向外倾斜 MOUNT_ANGLE
BALL_SOCKET = np.sin(MOUNT_ANGLE) * U_DIR + np.cos(MOUNT_ANGLE) * np.array(
    [0.0, 0.0, 1.0]
)

# 6 根从动臂的编号：0/1/2 = 腿 0/1/2 的第一根，3/4/5 = 腿 0/1/2 的第二根
# 同腿两根杆间距恒为 FORK_WIDTH，仍需按线段距离校核（杆倾斜时垂直间距会缩小）
ROD_PAIRS = [(i, j) for i in range(6) for j in range(i + 1, 6)]


# ==========================================
# 2. 正运动学
# ==========================================
def _leg_geometry(thetas):
    """由关节角求 B_i（主动臂末端）与等效球心 E_i = B_i - r·u_i

    A_i = P + r·u_i，|A_i - B_i| = l  <=>  |P - E_i| = l
    于是正解等价于三个半径 l、球心 E_i 的球面求交。
    """
    c, s = np.cos(thetas), np.sin(thetas)
    B = np.stack(
        [R * UX + L * c * UX, R * UY + L * c * UY, H + L * s * np.ones(3)], axis=-1
    )
    E = B - U_OFFSET
    return B, E


def forward_kinematics(thetas):
    """正解（批量）

    thetas: (..., 3) 三个主动臂关节角（rad）
    return: P (..., 3) 动平台中心坐标；B (..., 3, 3) 各腿 B 点；valid (...,) 是否有解
    """
    thetas = np.asarray(thetas, dtype=float)
    B, E = _leg_geometry(thetas)
    E1, E2, E3 = E[..., 0, :], E[..., 1, :], E[..., 2, :]

    v1, v2 = E2 - E1, E3 - E1
    n = np.cross(v1, v2)
    nn = np.linalg.norm(n, axis=-1)
    valid = nn > 1e-9
    n = n / np.where(valid, nn, 1.0)[..., None]

    # 两球面相减得到两个平面方程：P·v1 = b1, P·v2 = b2
    # 令 P = E1 + W，W = α·v1 + β·v2 + γ·n（n ⟂ v1、v2），则 W·v1 = b1 - E1·v1
    a11 = np.sum(v1 * v1, -1)
    a12 = np.sum(v1 * v2, -1)
    a22 = np.sum(v2 * v2, -1)
    b1 = 0.5 * (np.sum(E2 * E2, -1) - np.sum(E1 * E1, -1)) - np.sum(E1 * v1, -1)
    b2 = 0.5 * (np.sum(E3 * E3, -1) - np.sum(E1 * E1, -1)) - np.sum(E1 * v2, -1)

    det = a11 * a22 - a12**2
    valid &= np.abs(det) > 1e-9
    ds = np.where(np.abs(det) > 1e-9, det, 1.0)
    alpha = (b1 * a22 - b2 * a12) / ds
    beta = (a11 * b2 - a12 * b1) / ds

    # n 同时垂直于 v1、v2，故 gamma 由球方程单独解出
    base = alpha**2 * a11 + beta**2 * a22 + 2.0 * alpha * beta * a12
    g2 = l * l - base
    valid &= g2 >= 0.0
    g = np.sqrt(np.maximum(g2, 0.0))

    mid = E1 + alpha[..., None] * v1 + beta[..., None] * v2
    p_lo = mid - g[..., None] * n
    p_hi = mid + g[..., None] * n
    # 取 z 较小的一支（动平台在静平台下方）
    P = np.where((p_lo[..., 2] <= p_hi[..., 2])[..., None], p_lo, p_hi)
    return P, B, valid


def seg_seg_dist(p1, q1, p2, q2, eps=1e-12):
    """批量线段-线段最短距离（无分支写法）"""
    d1, d2 = q1 - p1, q2 - p2
    w = p1 - p2
    a = np.sum(d1 * d1, -1)
    e = np.sum(d2 * d2, -1)
    b = np.sum(d1 * d2, -1)
    c = np.sum(d1 * w, -1)
    f = np.sum(d2 * w, -1)

    denom = a * e - b * b
    has_denom = denom > eps
    s = np.where(has_denom, (b * f - c * e) / np.where(has_denom, denom, 1.0), 0.0)
    s = np.clip(s, 0.0, 1.0)

    t = np.where(e > eps, (b * s + f) / np.where(e > eps, e, 1.0), 0.0)
    s_low = np.clip(-c / np.where(a > eps, a, 1.0), 0.0, 1.0)
    s_high = np.clip((b - c) / np.where(a > eps, a, 1.0), 0.0, 1.0)
    s = np.where(t < 0.0, s_low, s)
    s = np.where(t > 1.0, s_high, s)
    t = np.clip(t, 0.0, 1.0)

    c1 = p1 + s[..., None] * d1
    c2 = p2 + t[..., None] * d2
    return np.linalg.norm(c1 - c2, axis=-1)


def _leg_vectors(thetas, P, B):
    """返回主动臂方向向量 a_i = B_i - C_i 与从动臂方向向量 b_i = A_i - B_i"""
    C = np.stack([R * UX, R * UY, H * np.ones(3)], axis=-1)  # (3,3) 静平台铰点
    v_arm = B - C
    A = P[..., None, :] + U_OFFSET  # (...,3,3) 动平台铰点
    v_leg = A - B
    return v_arm, v_leg


def _arm_socket(thetas):
    """臂端球窝轴：位于腿的径向平面内，把主动臂方向绕切向轴反旋 MOUNT_ANGLE

    初始位姿（主动臂下垂）时该轴与从动臂方向重合，锥角行程从此居中展开。
    """
    ang = thetas - MOUNT_ANGLE  # (...,3)
    return (
        np.cos(ang)[..., None] * U_DIR
        + np.sin(ang)[..., None] * np.array([0.0, 0.0, 1.0])
    )


def _local_components(vec):
    """把世界系方向向量分解到各腿的（径向, 切向, 竖直）"""
    vr = np.sum(vec * U_DIR, axis=-1)
    vt = np.sum(vec * T_DIR, axis=-1)
    vz = vec[..., 2]
    return vr, vt, vz


def _arm_joint_mask(thetas, b_hat):
    """臂端关节约束

    ARM_JOINT="ball"   -> 球铰锥角：b_i 与臂端球窝轴 s_i(θ) 的夹角 ≤ MAX_BALL_CONE
    ARM_JOINT="ujoint" -> 万向节双轴摆角：面内 α ≤ UJ_ALPHA_MAX，面外 β ≤ UJ_BETA_MAX
    """
    if ARM_JOINT == "ball":
        s_arm = _arm_socket(thetas)
        return np.all(
            np.sum(b_hat * s_arm, axis=-1) >= np.cos(MAX_BALL_CONE), axis=-1
        )

    br, bt, bz = _local_components(b_hat)
    alpha = np.abs(np.remainder(np.arctan2(bz, br) - thetas + np.pi, 2 * np.pi) - np.pi)
    beta = np.abs(np.arctan2(bt, np.hypot(br, bz)))
    return np.all(alpha <= UJ_ALPHA_MAX, axis=-1) & np.all(beta <= UJ_BETA_MAX, axis=-1)


def constraint_masks(thetas, P, B):
    """返回各物理约束的通过掩码

    关节配置：平行四边形从动臂，每腿两根平行杆。
      臂端：球头铰链（锥角）或十字万向节（双轴摆角），由 ARM_JOINT 选择
      平台端：球头铰链，从动臂方向 -b_i 与球窝轴 h_i 的夹角 ≤ MAX_BALL_CONE

    平行四边形保证两杆始终平行、两叉口也平行，两根杆相对各自支座的摆角完全相同，
    故关节约束按一根杆判定即可；两根杆只影响承载分配。
    """
    m_joint = np.all((thetas >= MIN_THETA) & (thetas <= MAX_THETA), axis=-1)

    _, v_leg = _leg_vectors(thetas, P, B)
    b_hat = v_leg / l  # 从动臂单位方向，B → A

    # 臂端关节
    m_arm = _arm_joint_mask(thetas, b_hat)

    # 平台端球铰：球窝轴固定在动平台上，参考方向取 A → B
    m_plat = np.all(
        np.sum(-b_hat * BALL_SOCKET, axis=-1) >= np.cos(MAX_BALL_CONE), axis=-1
    )

    # 6 根从动臂（每腿两根，沿切向错开 FORK_WIDTH）两两干涉
    A = P[..., None, :] + U_OFFSET  # (...,3,3)
    off = FORK_WIDTH * T_DIR  # (3,3)
    starts = np.concatenate([B, B + off], axis=-2)  # (...,6,3)
    ends = np.concatenate([A, A + off], axis=-2)  # (...,6,3)
    m_rod = np.ones(P.shape[:-1], dtype=bool)
    for i, j in ROD_PAIRS:
        dmin = seg_seg_dist(
            starts[..., i, :], ends[..., i, :], starts[..., j, :], ends[..., j, :]
        )
        m_rod &= dmin > ROD_DIAMETER

    return m_joint, m_arm, m_plat, m_rod


# ==========================================
# 3. 关节空间正扫
# ==========================================
def iter_sweep_chunks(n=61, chunk=200_000):
    """按块生成 n³ 个关节组合，避免一次性分配 n³×3 的大数组"""
    g = np.linspace(MIN_THETA, MAX_THETA, n)
    total = n * n * n
    for s in range(0, total, chunk):
        idx = np.arange(s, min(s + chunk, total))
        k = idx % n
        j = (idx // n) % n
        i = idx // (n * n)
        yield np.stack([g[i], g[j], g[k]], axis=-1)


def build_workspace(n=61, chunk=200_000):
    """分块正扫关节空间，返回有效采样点与各约束的通过计数"""
    counts = {
        "正解有解": 0,
        "关节限位": 0,
        "臂端关节": 0,
        "平台端球铰": 0,
        "杆间干涉": 0,
    }
    total = n * n * n
    kept_t, kept_p, kept_b = [], [], []
    for thetas in iter_sweep_chunks(n, chunk):
        P, B, ok = forward_kinematics(thetas)
        m_joint, m_arm, m_plat, m_rod = constraint_masks(thetas, P, B)
        counts["正解有解"] += int(ok.sum())
        counts["关节限位"] += int(m_joint.sum())
        counts["臂端关节"] += int(m_arm.sum())
        counts["平台端球铰"] += int(m_plat.sum())
        counts["杆间干涉"] += int(m_rod.sum())
        valid = ok & m_joint & m_arm & m_plat & m_rod
        if np.any(valid):
            kept_t.append(thetas[valid])
            kept_p.append(P[valid])
            kept_b.append(B[valid])

    if kept_t:
        thetas = np.concatenate(kept_t)
        P = np.concatenate(kept_p)
        B = np.concatenate(kept_b)
    else:
        thetas = np.empty((0, 3))
        P = np.empty((0, 3))
        B = np.empty((0, 3, 3))
    return thetas, P, B, counts, total


# ==========================================
# 4. 逆解（仅用于正解-逆解闭环自检）
# ==========================================
def inverse_kinematics_leg(p, i, branch):
    phi = PHI[i]
    a = p[0] * np.cos(phi) + p[1] * np.sin(phi) + r - R
    b = -p[0] * np.sin(phi) + p[1] * np.cos(phi)
    rho = np.hypot(a, p[2])
    if rho < 1e-9:
        return np.nan
    d = (a * a + b * b + p[2] ** 2 + L * L - l * l) / (2.0 * L)
    ratio = d / rho
    if abs(ratio) > 1:
        return np.nan
    return np.arctan2(p[2], a) + branch * np.arccos(ratio)


def verify_roundtrip(thetas, P, n_sample=2000, seed=0):
    """正解 -> 逆解 闭环校验，确认 FK 与 IK 描述同一构型

    注意：逆解的 ± 分支是逐腿而言的，非对称位姿下三条腿取到的分支号可以不同，
    因此这里对每条腿分别挑最接近的分支再取误差，而不是要求三条腿共用同一分支号。
    """
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(P), size=min(n_sample, len(P)), replace=False)
    err = 0.0
    for k in idx:
        p = P[k]
        leg_err = []
        for i in range(3):
            best = np.inf
            for br in (+1, -1):
                t = inverse_kinematics_leg(p, i, br)
                if np.isfinite(t):
                    d = abs(np.remainder(t - thetas[k, i] + np.pi, 2 * np.pi) - np.pi)
                    best = min(best, d)
            leg_err.append(best)
        err = max(err, max(leg_err))
    return np.degrees(err)


# ==========================================
# 5. 报告
# ==========================================
def z_profile(P, rad, step=5.0):
    """按 Z 分层，返回每层中心与最大可达 XY 半径"""
    z = P[:, 2]
    centers, rmax = [], []
    for e in np.arange(z.min(), z.max(), step):
        m = (z >= e) & (z < e + step)
        if np.any(m):
            centers.append(e + step / 2.0)
            rmax.append(rad[m].max())
    return np.array(centers), np.array(rmax)


def report(thetas, P):
    if len(P) == 0:
        print(
            "\n[无有效采样点] 当前几何与关节参数组合下工作空间为空，请放宽约束或调整参数。"
        )
        return
    rad = np.hypot(P[:, 0], P[:, 1])
    zmin, zmax = P[:, 2].min(), P[:, 2].max()

    print("=" * 62)
    print(
        "Delta 机械臂工作空间分析（R=%.0f r=%.0f L=%.0f l=%.0f, 单位 mm）"
        % (R, r, L, l)
    )
    print("=" * 62)
    joint_desc = (
        "球头铰链，锥角 ≤ %.0f°" % np.degrees(MAX_BALL_CONE)
        if ARM_JOINT == "ball"
        else "十字万向节，面内 ≤ %.0f°，面外 ≤ %.0f°"
        % (np.degrees(UJ_ALPHA_MAX), np.degrees(UJ_BETA_MAX))
    )
    print(
        "关节：主动臂 θ ∈ [%.0f°, %+.0f°]；臂端 %s；平台端球铰 ≤ %.0f°；"
        "球窝安装偏置 %.0f°；叉口间距 %.0f，杆径 %.0f"
        % (
            np.degrees(MIN_THETA),
            np.degrees(MAX_THETA),
            joint_desc,
            np.degrees(MAX_BALL_CONE),
            np.degrees(MOUNT_ANGLE),
            FORK_WIDTH,
            ROD_DIAMETER,
        )
    )
    print("有效采样点：%d" % len(P))
    print(
        "X 范围：[%.1f, %.1f]   Y 范围：[%.1f, %.1f]"
        % (P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max())
    )
    print("Z 范围：[%.1f, %.1f]   全局 Z 行程 = %.1f mm" % (zmin, zmax, zmax - zmin))

    print("\n--- 固定半径环上的 Z 可达区间（环宽 ±12.5mm）---")
    print("  半径ρ(mm)      Z_min      Z_max     Z行程(mm)")
    for rho in (0, 25, 50, 75, 100, 125, 150):
        m = np.abs(rad - rho) <= 12.5
        label = "中心附近" if rho == 0 else "%8.0f" % rho
        if not np.any(m):
            print("  %s          无采样点" % label)
            continue
        print(
            "  %s  %10.1f %10.1f %12.1f"
            % (label, P[m, 2].min(), P[m, 2].max(), P[m, 2].max() - P[m, 2].min())
        )

    # Z 分层步长随区间高度自适应，工作空间很窄时也能分辨
    step_z = max((zmax - zmin) / 30.0, 0.05)

    print("\n--- 不同 Z 高度下的最大可达半径 ---")
    print("     Z(mm)    最大半径(mm)")
    zc, rmax = z_profile(P, rad, step=step_z)
    for a, b in zip(zc, rmax):
        print("  %8.0f %14.1f" % (a, b))

    # 按半径反查：给定半径 ρ 内可连续覆盖的 Z 区间（这才是"作业包络"的可用行程）
    print("\n--- 半径 ρ 内可用的 Z 区间（要求该区间内每层都达到 ρ）---")
    zc_f, rmax_f = z_profile(P, rad, step=max(step_z * 0.5, 0.02))
    print("  半径ρ(mm)      Z_min      Z_max     Z行程(mm)")
    for rho in (0, 25, 50, 75, 100, 125, 150, 200, 250):
        ok = rmax_f >= rho
        if not np.any(ok):
            print("  %8.0f          不可达" % rho)
            continue
        # 取最长连续段
        best_len, best_start, start = 0, None, None
        for j, flag in enumerate(ok):
            if flag and start is None:
                start = j
            if (not flag or j == len(ok) - 1) and start is not None:
                end = j if not flag else j + 1
                if end - start > best_len:
                    best_len, best_start = end - start, start
                start = None
        z_a = zc_f[best_start]
        z_b = zc_f[best_start + best_len - 1]
        print("  %8.0f  %10.1f %10.1f %12.1f" % (rho, z_a, z_b, z_b - z_a))

    # 中心轴线上的解析对照，便于手算复核，同时看关节摆角余量
    if ARM_JOINT == "ball":
        print("\n--- 中心轴线解析对照（对称构型，含球铰锥角）---")
        print("  位姿             z(mm)   臂端球铰   平台端球铰")
    else:
        print("\n--- 中心轴线解析对照（对称构型，含万向节摆角）---")
        print("  位姿             z(mm)   面内 α     面外 β    平台端球铰")
    for name, th in (
        ("θ=+60°(上限)", MAX_THETA),
        ("θ=0°", 0.0),
        ("θ=-90°(下限)", MIN_THETA),
    ):
        th_arr = np.array([th, th, th])
        P_fk, B_fk, ok = forward_kinematics(th_arr)
        if not ok:
            print("  %-12s    无解" % name)
            continue
        _, vl = _leg_vectors(th_arr, P_fk, B_fk)
        b_hat = vl[0] / l
        plat_cone = np.degrees(
            np.arccos(np.clip(np.sum(-b_hat * BALL_SOCKET[0]), -1, 1))
        )
        if ARM_JOINT == "ball":
            s_arm = _arm_socket(th_arr)[0]
            arm_cone = np.degrees(np.arccos(np.clip(np.sum(b_hat * s_arm), -1, 1)))
            print("  %-12s %8.1f %10.1f %11.1f" % (name, P_fk[2], arm_cone, plat_cone))
        else:
            br, bt, bz = _local_components(b_hat)
            alpha = np.degrees(
                np.abs(np.remainder(np.arctan2(bz, br) - th + np.pi, 2 * np.pi) - np.pi)
            )
            beta = np.degrees(np.abs(np.arctan2(bt, np.hypot(br, bz))))
            print(
                "  %-12s %8.1f %10.1f %10.1f %11.1f"
                % (name, P_fk[2], alpha, beta, plat_cone)
            )


def plot(thetas, P):
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        print("\n[跳过绘图] 未安装 plotly，执行 `pip install plotly` 后可输出 HTML。")
        return None

    rad = np.hypot(P[:, 0], P[:, 1])
    fig = make_subplots(
        rows=1,
        cols=2,
        specs=[[{"type": "scatter3d"}, {"type": "xy"}]],
        subplot_titles=("约束内工作空间", "Z 行程 - 半径包络"),
        column_widths=[0.62, 0.38],
    )
    fig.add_trace(
        go.Scatter3d(
            x=P[:, 0],
            y=P[:, 1],
            z=P[:, 2],
            mode="markers",
            marker=dict(size=1.2, color=P[:, 2], colorscale="Viridis", opacity=0.25),
            name="可达点",
        ),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=rad,
            y=P[:, 2],
            mode="markers",
            marker=dict(size=1.5, color=P[:, 2], colorscale="Viridis", opacity=0.25),
            showlegend=False,
        ),
        row=1,
        col=2,
    )
    fig.update_layout(
        title="Delta 机械臂：正解扫描工作空间与 Z 轴行程",
        margin=dict(l=0, r=0, b=0, t=50),
        showlegend=False,
    )
    fig.update_scenes(
        aspectmode="data",
        xaxis_title="X (mm)",
        yaxis_title="Y (mm)",
        zaxis_title="Z (mm)",
    )
    fig.update_xaxes(title_text="XY 半径 (mm)", row=1, col=2)
    fig.update_yaxes(title_text="Z (mm)", row=1, col=2)

    out_dir = Path(__file__).resolve().parent / "output"
    out_dir.mkdir(exist_ok=True)
    out = out_dir / ("workspace_%s.html" % ARM_JOINT)
    fig.write_html(str(out), include_plotlyjs="cdn")
    return str(out)


if __name__ == "__main__":
    import sys

    n = int(sys.argv[1]) if len(sys.argv) > 1 else 121
    thetas, P, _, counts, total = build_workspace(n=n)
    print("\n--- 各约束单独通过点数（采样 n=%d，共 %d 个关节组合）---" % (n, total))
    for name, c in counts.items():
        print("  %-6s %10d  (%.2f%%)" % (name, c, 100.0 * c / total))
    print("  %-6s %10d" % ("全部满足", len(P)))
    print()

    err_deg = verify_roundtrip(thetas, P)
    print("正解-逆解闭环校验：最大角度残差 = %.2e 度" % err_deg)
    report(thetas, P)
    html = plot(thetas, P)
    if html:
        print("\n3D 文件已输出：%s" % html)
