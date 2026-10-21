# import math
from math import sin, cos, atan, sqrt, pi, e, acos, asin
import numpy as np

# define delta arm params

MAIN_LINK_LENGTH = 0.2  # length of main link in m
AUXILIARY_LINK_LENGTH = 0.5  # length of auxiliary link in m
MAIN_FIXED_POINT_AZIMUTH = 2 / 3 * pi  # azimuth of main fixed point in rad
MAX_MAIN_LINK_ANGLE = 0.5  # max angle of main link refer to x-axis in rad
MIN_MAIN_LINK_ANGLE = -0.5
MAX_AUXILIARY_LINK_ANGLE = 0.5  # max angle of auxiliary link in rad
SPINDLE_FIXED_POINT_RADIUS = (
    0.1  # distance from the triangle's center to the main axis fixed point
)
TARGET_PANNEL_RADIUS = 0.1  # distance from the target pannel's triangle's center to the auxiliary link's center

# spindel-axis coordinate system unit vector
SPINDEL_AXIS_UNIT = (sin(MAIN_FIXED_POINT_AZIMUTH), cos(MAIN_FIXED_POINT_AZIMUTH), 0)
print(f"MAIN_FIXED_POINT_AZIMUTH's angle: {MAIN_FIXED_POINT_AZIMUTH}")
print(f"SPINDEL_AXIS_UNIT: {SPINDEL_AXIS_UNIT}")

# # the radial vector of the target point
# ai = np.array([0, 0, 0])

# # the tangent vector of the target point
# bi = np.array([0, 0, 0])


def get_spindle_azimuth(i):
    """
    get spindle azimuth of delta arm
    i: index of spindle azimuth in [0, 1, 2]
    """
    return (i - 1) * 2 * pi / 3


def is_valid_position(x, y, z):
    """
    check if the position is valid
    """
    a = []
    b = []
    d = []
    rho = []  # 建议把 p 改为 rho，避免和坐标 p(x,y,z) 混淆

    is_valid = True  # 初始化为 True

    # 假设你要传入的两个连杆长度为 L 和 l
    # L = MAIN_LINK_LENGTH
    # l = FOREARM_LINK_LENGTH

    for i in range(3):
        phi = get_spindle_azimuth(i)

        a_i = (
            x * cos(phi)
            + y * sin(phi)
            + TARGET_PANNEL_RADIUS
            - SPINDLE_FIXED_POINT_RADIUS
        )
        a.append(a_i)

        b_i = -x * sin(phi) + y * cos(phi)
        b.append(b_i)

        d_i = (
            a_i**2 + b_i**2 + z**2 + MAIN_LINK_LENGTH**2 - AUXILIARY_LINK_LENGTH**2
        ) / (2 * MAIN_LINK_LENGTH)
        d.append(d_i)

        # 计算 rho_i
        rho_i = sqrt(a_i**2 + z**2)
        rho.append(rho_i)

        # 防止除以 0 的保护
        if rho_i < 1e-8:
            return False

        # 判断存在性条件 |d_i / rho_i| <= 1
        if abs(d_i / rho_i) > 1:
            is_valid = False
            break  # 只要有一条腿够不着，就判定为无效，提前退出循环

    return is_valid


import math


def delta_inverse_kinematics(x, y, z, i, branch=-1):
    """
    inverse kinematics of delta arm
    :param x, y, z: 末端执行器（动平台中心）的坐标
    :param i: 第 i 条腿 (0, 1, 2)
    :param branch: 分支选择。1 代表 '+' 号分支，-1 代表 '-' 号分支（通常打顶机取 -1）
    :return: 第 i 条腿的主动关节角度 theta (弧度)
    """
    # 1. 判断位置是否在工作空间内
    if not is_valid_position(x, y, z):
        print(f"Position ({x}, {y}, {z}) is not valid for leg {i}.")
        return None

    # 2. 获取第 i 条腿的方位角 (0, 120, 240 度转化为弧度)
    phi = get_spindle_azimuth(i)

    # 3. 计算 ai 和 bi
    a_i = (
        x * math.cos(phi)
        + y * math.sin(phi)
        + TARGET_PANNEL_RADIUS
        - SPINDLE_FIXED_POINT_RADIUS
    )
    b_i = -x * math.sin(phi) + y * math.cos(phi)

    # 4. 计算 di 和 rhoi
    # 注意：请确保你的系统里定义了从动臂长度（副臂长度），这里假设为 FOREARM_LINK_LENGTH
    # L 是主动臂长度，l 是从动臂长度
    d_i = (
        a_i**2 + b_i**2 + z**2 + MAIN_LINK_LENGTH**2 - FOREARM_LINK_LENGTH**2
    ) / (2 * MAIN_LINK_LENGTH)
    rho_i = math.sqrt(a_i**2 + z**2)

    # 5. 计算中间相位角 psi
    # atan2 的范围是 [-pi, pi]，能自动处理 a_i 为 0 或负数的情况
    psi_i = math.atan2(z, a_i)

    # 6. 计算反余弦项
    # 因为有 is_valid_position 做拦截，这里 d_i / rho_i 理论上不会超出 [-1, 1]
    # 但为了浮点数安全，可以加一个 clip
    ratio = max(-1.0, min(1.0, d_i / rho_i))
    acos_term = math.acos(ratio)

    # 7. 根据分支选择计算最终角度
    theta_i = psi_i + branch * acos_term

    return theta_i


def delta_forward_kinematics(theta1, theta2, theta3):
    """
    forward kinematics of delta arm
    """
    pass
