/**
  ******************************************************************************
  * @file    motor.h
  * @brief   PD42S1 三轴 CAN 闭环步进电机驱动模块公共接口
  *
  * 目标硬件：
  *   STM32F103C8T6 + SN65HVD230 + 3 x ATK-PD42S1（CAN 光耦版）
  *
  * 设计约束：
  *   - CAN1，500 kbps，扩展帧，正点原子自定义协议
  *   - 从机地址固定为 1 / 2 / 3
  *   - 位置单位：51200 counts / 圈
  *   - 运动接口只等待驱动器 ACK，不等待机械到位
  *   - Motor_Init() 不自动使能、不自动运动
  *
  * 详细需求、协议、时序、测试和风险见：
  *   docs/PD42S1_Motor_Driver_Requirements_Design_v1.0.md
  ******************************************************************************
  */

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * 公共常量
 *============================================================================*/

/** 每圈位置计数；通信位置模式固定 51200 counts/圈。 */
#define MOTOR_COUNTS_PER_REV        (51200)

/** 绝对位置目标默认允许范围：|target| <= 51200 counts。 */
#define MOTOR_ABS_LIMIT_COUNTS      (51200)

/** 位置/速度模式允许的最大转速。 */
#define MOTOR_MAX_SPEED_RPM         (6000U)

/** 加减速度参数最大值；0 表示直接启动。 */
#define MOTOR_MAX_ACCEL             (200U)

/** 单次指令等待 ACK 的超时时间（ms）。 */
#define MOTOR_ACK_TIMEOUT_MS        (50U)

/** 两条完整指令之间的保护间隔（ms）；协议要求至少 2 ms。 */
#define MOTOR_CMD_GAP_MS            (3U)

/** 多帧应答的空闲判定时间（ms）。 */
#define MOTOR_RX_GAP_MS             (5U)

/** 到位等待的轮询周期（ms）。 */
#define MOTOR_WAIT_POLL_MS          (50U)

/** Motor_WaitInPosition() 的默认总超时（ms）。 */
#define MOTOR_DEFAULT_WAIT_TIMEOUT_MS (30000U)

/** 幂等指令的自动重试次数。相对运动不使用该重试。 */
#define MOTOR_RETRY_COUNT           (1U)

/** 协议应答重组缓冲区大小（字节）。 */
#define MOTOR_RX_BUF_SIZE           (64U)

/** 协议指令组包缓冲区大小（字节）。 */
#define MOTOR_TX_BUF_SIZE           (32U)

/*==============================================================================
 * 公共类型
 *============================================================================*/

/**
 * @brief 电机编号。
 *
 * 电机编号与驱动器从机地址一一对应：
 *   MOTOR_1 -> addr 1
 *   MOTOR_2 -> addr 2
 *   MOTOR_3 -> addr 3
 */
typedef enum
{
    MOTOR_1 = 0,        /**< 1 号电机，从机地址 1 */
    MOTOR_2 = 1,        /**< 2 号电机，从机地址 2 */
    MOTOR_3 = 2,        /**< 3 号电机，从机地址 3 */
    MOTOR_COUNT = 3     /**< 电机数量，仅用于数组边界检查 */
} motor_id_t;

/**
 * @brief 公共 API 返回状态。
 */
typedef enum
{
    MOTOR_OK = 0,           /**< 成功 */
    MOTOR_ERR_PARAM,        /**< 参数非法：空指针、速度/加减速度越界等 */
    MOTOR_ERR_RANGE,        /**< 绝对目标超出 ±MOTOR_ABS_LIMIT_COUNTS */
    MOTOR_ERR_NOT_INIT,     /**< 模块尚未初始化 */
    MOTOR_ERR_BUSY,         /**< 重入或总线忙 */
    MOTOR_ERR_TIMEOUT,      /**< ACK 等待或到位等待超时 */
    MOTOR_ERR_CAN,          /**< HAL CAN 启动、发送或状态错误 */
    MOTOR_ERR_FRAME,        /**< 应答帧结构错误 */
    MOTOR_ERR_CHECKSUM,     /**< 应答校验和错误 */
    MOTOR_ERR_DEVICE,       /**< 驱动器返回 0xE? 错误码 */
    MOTOR_ERR_STALL,        /**< 堵转 */
    MOTOR_ERR_STATE         /**< 过载、欠压或当前状态不允许操作 */
} motor_status_t;

/**
 * @brief 驱动器 0x2C 运行状态编码。
 */
typedef enum
{
    MOTOR_STATE_STOPPED = 0,      /**< 停止 */
    MOTOR_STATE_DONE = 1,         /**< 任务完成 */
    MOTOR_STATE_RUNNING = 2,      /**< 正在运行 */
    MOTOR_STATE_OVERLOAD = 3,     /**< 过载 */
    MOTOR_STATE_STALLED = 4,      /**< 堵转 */
    MOTOR_STATE_UNDERVOLTAGE = 5  /**< 欠压 */
} motor_run_state_t;

/*==============================================================================
 * 初始化与诊断
 *============================================================================*/

/**
 * @brief 初始化电机驱动模块。
 *
 * 前置条件：应用已经调用 CubeMX 生成的 MX_CAN_Init()，CAN 配置为 500 kbps。
 *
 * 本函数只配置 CAN 过滤器、启动 CAN 并清空 RX FIFO：
 *   - 不发送使能命令；
 *   - 不发送运动命令；
 *   - 不改变电机当前状态。
 *
 * @return MOTOR_OK 成功；MOTOR_ERR_CAN 启动或配置失败。
 */
motor_status_t Motor_Init(void);

/**
 * @brief 探测指定电机是否在线。
 *
 * 发送 0x20 读取软硬件版本指令，并校验应答地址、功能码、帧尾和校验和。
 * 该函数不使能电机、不产生运动。
 *
 * @param[in] id 电机编号（MOTOR_1 ~ MOTOR_3）。
 * @return MOTOR_OK 在线；MOTOR_ERR_PARAM 编号非法；
 *         MOTOR_ERR_NOT_INIT 未初始化；MOTOR_ERR_TIMEOUT 无应答；
 *         其他错误见 motor_status_t。
 */
motor_status_t Motor_Ping(motor_id_t id);

/*==============================================================================
 * 使能控制
 *============================================================================*/

/**
 * @brief 使能或失能指定电机。
 *
 * 对应驱动器 0xFA 指令。该函数是幂等的，ACK 超时可安全重试。
 *
 * @param[in] id     电机编号。
 * @param[in] enable true = 使能；false = 失能。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_Enable(motor_id_t id, bool enable);

/**
 * @brief 读取指定电机的使能状态。
 *
 * 对应驱动器 0x2F 指令。返回的 enabled 已转换为正逻辑：
 *   true  = 电机使能
 *   false = 电机失能
 *
 * @param[in]  id      电机编号。
 * @param[out] enabled 使能状态输出，不能为 NULL。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_ReadEnabled(motor_id_t id, bool *enabled);

/*==============================================================================
 * 位置运动
 *============================================================================*/

/**
 * @brief 相对当前位置运动指定 counts。
 *
 * 对应驱动器 0xF3 指令。函数只等待驱动器 ACK，不等待机械到位。
 * 本函数不自动重试：ACK 丢失后重发可能导致多走一段。
 * delta_counts 为 INT32_MIN 时取绝对值会溢出，返回 MOTOR_ERR_PARAM。
 *
 * @param[in] id          电机编号。
 * @param[in] delta_counts 相对位置，带符号；正数正转，负数反转。
 * @param[in] speed_rpm   最大速度，1 ~ MOTOR_MAX_SPEED_RPM。
 * @param[in] accel       加减速度参数，0 ~ MOTOR_MAX_ACCEL；0 表示直接启动。
 * @return MOTOR_OK 成功；MOTOR_ERR_PARAM 参数非法；其他错误见 motor_status_t。
 */
motor_status_t Motor_MoveRelative(motor_id_t id, int32_t delta_counts,
                                  uint16_t speed_rpm, uint8_t accel);

/**
 * @brief 运动到指定绝对位置。
 *
 * 对应驱动器 0xF2 指令。函数只等待驱动器 ACK，不等待机械到位。
 * 发送前检查 |target_counts| <= MOTOR_ABS_LIMIT_COUNTS；超出时返回
 * MOTOR_ERR_RANGE，且不发送任何 CAN 报文。
 * target_counts 为 INT32_MIN 时返回 MOTOR_ERR_PARAM。
 *
 * 注意：PD42S1 为单圈绝对值编码器，本函数按单圈绝对定位使用。
 *
 * @param[in] id           电机编号。
 * @param[in] target_counts 目标绝对位置，带符号。
 * @param[in] speed_rpm    最大速度，1 ~ MOTOR_MAX_SPEED_RPM。
 * @param[in] accel        加减速度参数，0 ~ MOTOR_MAX_ACCEL。
 * @return MOTOR_OK 成功；MOTOR_ERR_RANGE 绝对目标越界；
 *         MOTOR_ERR_PARAM 其他参数非法；其他错误见 motor_status_t。
 */
motor_status_t Motor_MoveAbsolute(motor_id_t id, int32_t target_counts,
                                  uint16_t speed_rpm, uint8_t accel);

/**
 * @brief 相对运动指定圈数。
 *
 * @param[in] id        电机编号。
 * @param[in] rev       相对圈数，带符号；0.5 表示半圈。
 * @param[in] speed_rpm 最大速度，1 ~ MOTOR_MAX_SPEED_RPM。
 * @param[in] accel     加减速度参数，0 ~ MOTOR_MAX_ACCEL。
 * @return 见 Motor_MoveRelative()。
 */
motor_status_t Motor_MoveRelativeRev(motor_id_t id, float rev,
                                     uint16_t speed_rpm, uint8_t accel);

/**
 * @brief 相对运动指定角度。
 *
 * @param[in] id        电机编号。
 * @param[in] deg       相对角度，带符号；90.0f 表示正转 90 度。
 * @param[in] speed_rpm 最大速度，1 ~ MOTOR_MAX_SPEED_RPM。
 * @param[in] accel     加减速度参数，0 ~ MOTOR_MAX_ACCEL。
 * @return 见 Motor_MoveRelative()。
 */
motor_status_t Motor_MoveRelativeDeg(motor_id_t id, float deg,
                                     uint16_t speed_rpm, uint8_t accel);

/**
 * @brief 绝对运动到指定圈数对应的位置。
 *
 * 换算后仍受 ±MOTOR_ABS_LIMIT_COUNTS 限制。
 *
 * @param[in] id        电机编号。
 * @param[in] rev       目标绝对圈数，带符号。
 * @param[in] speed_rpm 最大速度，1 ~ MOTOR_MAX_SPEED_RPM。
 * @param[in] accel     加减速度参数，0 ~ MOTOR_MAX_ACCEL。
 * @return 见 Motor_MoveAbsolute()。
 */
motor_status_t Motor_MoveAbsoluteRev(motor_id_t id, float rev,
                                     uint16_t speed_rpm, uint8_t accel);

/**
 * @brief 绝对运动到指定角度对应的位置。
 *
 * 换算后仍受 ±MOTOR_ABS_LIMIT_COUNTS 限制。
 *
 * @param[in] id        电机编号。
 * @param[in] deg       目标绝对角度，带符号。
 * @param[in] speed_rpm 最大速度，1 ~ MOTOR_MAX_SPEED_RPM。
 * @param[in] accel     加减速度参数，0 ~ MOTOR_MAX_ACCEL。
 * @return 见 Motor_MoveAbsolute()。
 */
motor_status_t Motor_MoveAbsoluteDeg(motor_id_t id, float deg,
                                     uint16_t speed_rpm, uint8_t accel);

/*==============================================================================
 * 速度与停止
 *============================================================================*/

/**
 * @brief 进入速度模式。
 *
 * 对应驱动器 0xF1 指令。函数只等待驱动器 ACK，不等待电机达到目标速度。
 * 该函数是幂等的，ACK 超时可安全重试。
 *
 * @param[in] id    电机编号。
 * @param[in] rpm   目标转速，带符号；正数正转，负数反转；
 *                  |rpm| 范围 1 ~ MOTOR_MAX_SPEED_RPM。
 *                  停止请使用 Motor_Stop()，不要用 rpm = 0。
 *                  rpm 为 INT32_MIN 时返回 MOTOR_ERR_PARAM。
 * @param[in] accel 加减速度参数，0 ~ MOTOR_MAX_ACCEL。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_SetSpeed(motor_id_t id, int32_t rpm, uint8_t accel);

/**
 * @brief 单轴立即停止（刹车）。
 *
 * 对应驱动器 0xFC 指令，等待 ACK。该函数是幂等的，可安全重试。
 *
 * @param[in] id 电机编号。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_Stop(motor_id_t id);

/**
 * @brief 广播立即停止所有电机。
 *
 * 使用广播地址 0 发送 0xFC。所有驱动器执行但不应答。
 * 返回 MOTOR_OK 仅表示 CAN 报文发送成功，不代表每个驱动器都已确认。
 * 广播会影响总线上所有从机，只能在专用总线使用。
 *
 * @return MOTOR_OK 报文发送成功；MOTOR_ERR_NOT_INIT 未初始化；
 *         MOTOR_ERR_BUSY 总线忙；MOTOR_ERR_TIMEOUT 发送超时；
 *         MOTOR_ERR_CAN 发送失败。
 */
motor_status_t Motor_StopAll(void);

/*==============================================================================
 * 状态读取
 *============================================================================*/

/**
 * @brief 读取实时位置。
 *
 * 对应驱动器 0x2A 指令。返回值为带符号 counts，51200 counts = 1 圈。
 *
 * @param[in]  id     电机编号。
 * @param[out] counts 位置输出，不能为 NULL。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_ReadPosition(motor_id_t id, int32_t *counts);

/**
 * @brief 读取实时转速。
 *
 * 对应驱动器 0x29 指令。返回值单位 RPM，带符号。
 *
 * @param[in]  id  电机编号。
 * @param[out] rpm 转速输出，不能为 NULL。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_ReadSpeed(motor_id_t id, int16_t *rpm);

/**
 * @brief 读取运行状态。
 *
 * 对应驱动器 0x2C 指令。
 *
 * @param[in]  id    电机编号。
 * @param[out] state 状态输出，不能为 NULL。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_ReadState(motor_id_t id, motor_run_state_t *state);

/**
 * @brief 读取到位标志。
 *
 * 对应驱动器 0x30 指令。
 *
 * @param[in]  id          电机编号。
 * @param[out] in_position true = 已到位；false = 未到位。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_ReadInPosition(motor_id_t id, bool *in_position);

/**
 * @brief 读取堵转标志。
 *
 * 对应驱动器 0x2D 指令。
 *
 * @param[in]  id      电机编号。
 * @param[out] stalled true = 堵转；false = 未堵转。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_ReadStall(motor_id_t id, bool *stalled);

/*==============================================================================
 * 到位等待
 *============================================================================*/

/**
 * @brief 等待指定电机到位。
 *
 * 周期读取 0x30 到位标志，并检查 0x2D 堵转和 0x2C 故障状态。
 * 本函数会阻塞，不能在中断中调用。
 *
 * 超时后只返回错误，不自动刹车。需要安全停机时，调用者应显式调用
 * Motor_Stop() 或 Motor_StopAll()。
 *
 * @param[in] id         电机编号。
 * @param[in] timeout_ms 总超时（ms）；0 表示使用默认值
 *                       MOTOR_DEFAULT_WAIT_TIMEOUT_MS。
 * @return MOTOR_OK 到位；MOTOR_ERR_TIMEOUT 超时；
 *         MOTOR_ERR_STALL 堵转；MOTOR_ERR_STATE 过载/欠压等；
 *         其他错误见 motor_status_t。
 */
motor_status_t Motor_WaitInPosition(motor_id_t id, uint32_t timeout_ms);

/*==============================================================================
 * 零点与错误诊断
 *============================================================================*/

/**
 * @brief 将当前位置、位置误差和累计脉冲数清零。
 *
 * 对应驱动器 0xF8 指令。属于调试/标定接口，不自动重试。
 * 手册未明确保证该零点跨上电保持；v1 不依赖它掉电保存。
 *
 * @param[in] id 电机编号。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
motor_status_t Motor_ZeroPosition(motor_id_t id);

/**
 * @brief 读取最近一次驱动器返回的原始错误码。
 *
 * 返回值来自驱动器应答的 Byte1：
 *   0x01 成功；0xE1 ~ 0xE6 为驱动器错误。
 * 若尚未收到过错误码，返回 0x00。
 *
 * 本函数不发起 CAN 通信，可在错误处理后调用。
 *
 * @param[in] id 电机编号。
 * @return 原始错误码；编号非法时返回 0x00。
 */
uint8_t Motor_GetLastDeviceError(motor_id_t id);

/*==============================================================================
 * 单位换算
 *
 * 注意：以下函数是纯换算辅助，不替代运动接口的范围检查。
 * 输入超出 int32_t 可表示范围时返回 0。
 *============================================================================*/

/**
 * @brief counts 转圈数。
 * @param[in] counts 位置计数。
 * @return 圈数（float）。
 */
float Motor_CountsToRev(int32_t counts);

/**
 * @brief counts 转角度。
 * @param[in] counts 位置计数。
 * @return 角度（度，float）。
 */
float Motor_CountsToDeg(int32_t counts);

/**
 * @brief 圈数转 counts，四舍五入。
 * @param[in] rev 圈数。
 * @return counts；溢出时返回 0。
 */
int32_t Motor_RevToCounts(float rev);

/**
 * @brief 角度转 counts，四舍五入。
 * @param[in] deg 角度（度）。
 * @return counts；溢出时返回 0。
 */
int32_t Motor_DegToCounts(float deg);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */