import numpy as np
import plotly.graph_objects as go

# ==========================================
# 1. 机械臂几何参数与物理约束配置
# ==========================================
R = 150.0  # 顶部固定平台半径
r = 50.0  # 动平台(末端)半径
L = 200.0  # 主动臂长度
l = 200.0  # 从动臂(副臂)长度
H = 0.0  # 顶部平台高度 (Z=0)

# 物理约束
MAX_THETA = np.pi / 3.0  # 主动臂向上最高 60°
MIN_THETA = -np.pi / 2.0  # 主动臂向下最低 -90°
MAX_U_JOINT = np.pi / 3.0  # 万向节最大偏角 60° (副臂与垂直方向夹角)
ROD_DIAMETER = 15.0  # 连杆物理直径(mm)，用于防干涉检测


def get_spindle_azimuth(i):
    """获取三个主动臂的方位角 (0, 120, 240度)"""
    return i * 2 * np.pi / 3


def get_C_points():
    """获取顶部三个固定铰点 C_i"""
    C = []
    for i in range(3):
        phi = get_spindle_azimuth(i)
        C.append([R * np.cos(phi), R * np.sin(phi), H])
    return np.array(C)


def get_A_points(p):
    """获取动平台三个铰点 A_i，p为动平台中心 [x,y,z]"""
    A = []
    for i in range(3):
        phi = get_spindle_azimuth(i)
        A.append([p[0] + r * np.cos(phi), p[1] + r * np.sin(phi), p[2]])
    return np.array(A)


def inverse_kinematics(p, branch=1):
    """解析逆解，返回三个关节角 theta"""
    thetas = []
    for i in range(3):
        phi = get_spindle_azimuth(i)
        x, y, z = p
        a_i = x * np.cos(phi) + y * np.sin(phi) + r - R
        b_i = -x * np.sin(phi) + y * np.cos(phi)
        d_i = (a_i**2 + b_i**2 + z**2 + L**2 - l**2) / (2 * L)
        rho_i = np.sqrt(a_i**2 + z**2)

        # 防止除零或不可达
        if rho_i < 1e-6 or abs(d_i / rho_i) > 1:
            return None

        psi_i = np.arctan2(z, a_i)
        acos_term = np.arccos(np.clip(d_i / rho_i, -1.0, 1.0))
        theta_i = psi_i + branch * acos_term
        thetas.append(theta_i)
    return thetas


def get_B_points(C, thetas):
    """根据关节角计算主动臂末端 B_i"""
    B = []
    for i in range(3):
        phi = get_spindle_azimuth(i)
        theta = thetas[i]
        bx = C[i][0] + L * np.cos(theta) * np.cos(phi)
        by = C[i][1] + L * np.cos(theta) * np.sin(phi)
        bz = C[i][2] + L * np.sin(theta)
        B.append([bx, by, bz])
    return np.array(B)


# ==========================================
# 2. 带物理约束的向量化工作空间计算
# ==========================================
def compute_workspace_with_constraints(x_range, y_range, z_range, resolution=35):
    x = np.linspace(x_range[0], x_range[1], resolution)
    y = np.linspace(y_range[0], y_range[1], resolution)
    z = np.linspace(z_range[0], z_range[1], resolution)

    X, Y, Z = np.meshgrid(x, y, z)
    x_flat, y_flat, z_flat = X.flatten(), Y.flatten(), Z.flatten()

    valid = np.ones_like(x_flat, dtype=bool)

    # 存储用于干涉检测的 B 点和 A 点坐标
    B_all = np.zeros((3, len(x_flat), 3))
    A_all = np.zeros((3, len(x_flat), 3))

    for i in range(3):
        phi = get_spindle_azimuth(i)
        a_i = x_flat * np.cos(phi) + y_flat * np.sin(phi) + r - R
        b_i = -x_flat * np.sin(phi) + y_flat * np.cos(phi)
        rho_i = np.sqrt(a_i**2 + z_flat**2)
        d_i = (a_i**2 + b_i**2 + z_flat**2 + L**2 - l**2) / (2 * L)

        with np.errstate(divide="ignore", invalid="ignore"):
            ratio = d_i / rho_i

        # 基础几何可达性
        geom_valid = (np.abs(ratio) <= 1) & (~np.isnan(ratio))
        valid = valid & geom_valid

        # 计算 theta_i (默认 branch=-1)
        psi_i = np.arctan2(z_flat, a_i)
        acos_term = np.arccos(np.clip(ratio, -1.0, 1.0))
        theta_i = psi_i - acos_term

        # 【约束 1】主动臂转动限位：向上最高 60°，向下最低 -90°
        limit_valid = (theta_i <= MAX_THETA) & (theta_i >= MIN_THETA)

        # 【约束 2】万向节限位：副臂与垂直方向夹角 <= 60°
        B_z = L * np.sin(theta_i)
        cos_u_joint = np.abs(z_flat - B_z) / l
        u_joint_valid = cos_u_joint >= np.cos(MAX_U_JOINT)

        valid = valid & limit_valid & u_joint_valid

        # 计算三维坐标保存供干涉检测
        B_x = R * np.cos(phi) + L * np.cos(theta_i) * np.cos(phi)
        B_y = R * np.sin(phi) + L * np.cos(theta_i) * np.sin(phi)
        B_all[i] = np.column_stack((B_x, B_y, B_z))

        A_all[i] = np.column_stack(
            (x_flat + r * np.cos(phi), y_flat + r * np.sin(phi), z_flat)
        )

    # 【约束 3】物理杆干涉检测 (简化版：检测相邻两根副臂 B/A 的最近距离)
    def seg_dist(p1, p2, p3, p4):
        dist1 = np.linalg.norm(p1 - p3, axis=1)
        dist2 = np.linalg.norm(p2 - p4, axis=1)
        return np.minimum(dist1, dist2)

    dist_12 = seg_dist(B_all[0], A_all[0], B_all[1], A_all[1])
    dist_23 = seg_dist(B_all[1], A_all[1], B_all[2], A_all[2])
    dist_31 = seg_dist(B_all[2], A_all[2], B_all[0], A_all[0])

    interference_valid = (
        (dist_12 > ROD_DIAMETER) & (dist_23 > ROD_DIAMETER) & (dist_31 > ROD_DIAMETER)
    )
    valid = valid & interference_valid

    return x_flat[valid], y_flat[valid], z_flat[valid]


# ==========================================
# 3. 生成数据
# ==========================================
print("正在计算带物理约束的工作空间... (请稍候)")
X_range = (-350, 350)
Y_range = (-350, 350)
Z_range = (-550, -100)

ws_x, ws_y, ws_z = compute_workspace_with_constraints(
    X_range, Y_range, Z_range, resolution=40
)
print(f"计算完成！满足所有物理约束的点数: {len(ws_x)}")

# 生成一条测试轨迹 (圆形螺旋下降)
t = np.linspace(0, 2 * np.pi, 100)
traj_x = 60 * np.cos(t)
traj_y = 60 * np.sin(t)
traj_z = -350 + 40 * np.sin(3 * t)

# 获取轨迹起点的机械臂形态
p_start = np.array([traj_x[0], traj_y[0], traj_z[0]])
thetas_start = inverse_kinematics(p_start)
C_pts = get_C_points()
B_pts = get_B_points(C_pts, thetas_start)
A_pts = get_A_points(p_start)

# ==========================================
# 4. 绘图
# ==========================================
fig = go.Figure()

# A. 绘制工作空间点云
fig.add_trace(
    go.Scatter3d(
        x=ws_x,
        y=ws_y,
        z=ws_z,
        mode="markers",
        marker=dict(size=1.5, color=ws_z, colorscale="Viridis", opacity=0.15),
        name="约束内工作空间",
    )
)

# B. 绘制测试轨迹线
fig.add_trace(
    go.Scatter3d(
        x=traj_x,
        y=traj_y,
        z=traj_z,
        mode="lines",
        line=dict(color="magenta", width=4),
        name="运动轨迹",
    )
)

# C. 绘制固定平台 (黑色)
fig.add_trace(
    go.Scatter3d(
        x=[C_pts[0][0], C_pts[1][0], C_pts[2][0], C_pts[0][0]],
        y=[C_pts[0][1], C_pts[1][1], C_pts[2][1], C_pts[0][1]],
        z=[C_pts[0][2], C_pts[1][2], C_pts[2][2], C_pts[0][2]],
        mode="lines",
        line=dict(color="black", width=6),
        name="顶部固定平台",
    )
)

# D. 绘制动平台 (绿色)
fig.add_trace(
    go.Scatter3d(
        x=[A_pts[0][0], A_pts[1][0], A_pts[2][0], A_pts[0][0]],
        y=[A_pts[0][1], A_pts[1][1], A_pts[2][1], A_pts[0][1]],
        z=[A_pts[0][2], A_pts[1][2], A_pts[2][2], A_pts[0][2]],
        mode="lines",
        line=dict(color="green", width=6),
        name="动平台",
    )
)

# E. 绘制主动臂(红)和从动臂(灰)
for i in range(3):
    fig.add_trace(
        go.Scatter3d(
            x=[C_pts[i][0], B_pts[i][0]],
            y=[C_pts[i][1], B_pts[i][1]],
            z=[C_pts[i][2], B_pts[i][2]],
            mode="lines",
            line=dict(color="red", width=8),
            showlegend=False,
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=[B_pts[i][0], A_pts[i][0]],
            y=[B_pts[i][1], A_pts[i][1]],
            z=[B_pts[i][2], A_pts[i][2]],
            mode="lines",
            line=dict(color="gray", width=4),
            showlegend=False,
        )
    )

# F. 绘制物理极限参考线 (红色虚线，显示主动臂最高 60° 的极限位置)
for i in range(3):
    phi = get_spindle_azimuth(i)
    bx_60 = R * np.cos(phi) + L * np.cos(MAX_THETA) * np.cos(phi)
    by_60 = R * np.sin(phi) + L * np.cos(MAX_THETA) * np.sin(phi)
    bz_60 = H + L * np.sin(MAX_THETA)

    fig.add_trace(
        go.Scatter3d(
            x=[C_pts[i][0], bx_60],
            y=[C_pts[i][1], by_60],
            z=[C_pts[i][2], bz_60],
            mode="lines",
            line=dict(color="red", width=2, dash="dash"),
            showlegend=False,
        )
    )

# G. 绘制关节点 (黑色小球)
joint_x = (
    [C_pts[i][0] for i in range(3)]
    + [B_pts[i][0] for i in range(3)]
    + [A_pts[i][0] for i in range(3)]
)
joint_y = (
    [C_pts[i][1] for i in range(3)]
    + [B_pts[i][1] for i in range(3)]
    + [A_pts[i][1] for i in range(3)]
)
joint_z = (
    [C_pts[i][2] for i in range(3)]
    + [B_pts[i][2] for i in range(3)]
    + [A_pts[i][2] for i in range(3)]
)

fig.add_trace(
    go.Scatter3d(
        x=joint_x,
        y=joint_y,
        z=joint_z,
        mode="markers",
        marker=dict(size=6, color="black"),
        showlegend=False,
    )
)

# 页面布局配置
fig.update_layout(
    title="带物理约束的 Delta 机械臂 3D 交互可视化",
    scene=dict(
        xaxis_title="X (mm)",
        yaxis_title="Y (mm)",
        zaxis_title="Z (mm)",
        aspectmode="data",  # 强制保持真实物理比例，不变形
        camera=dict(eye=dict(x=1.5, y=1.5, z=1.0)),  # 俯视视角
    ),
    margin=dict(l=0, r=0, b=0, t=40),
    legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01),
)

# 在浏览器中打开
fig.show()
