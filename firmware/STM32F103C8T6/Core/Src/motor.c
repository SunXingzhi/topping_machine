/**
  ******************************************************************************
  * @file    motor.c
  * @brief   PD42S1 三轴 CAN 闭环步进电机驱动模块
  *
  * P2 范围：
  *   - CAN 过滤器、启动、RX FIFO 清理
  *   - 协议帧组包、8 位校验和、8 字节拆分发送
  *   - 多帧应答接收与重组、5 ms 空闲判定
  *   - 2 ms 指令间隔（实现为 3 ms 保护间隔）
  *   - 发送/接收超时、幂等指令重试
  *   - Motor_Init / Motor_Ping / Motor_GetLastDeviceError
  *
  * 尚未实现（P3）：
  *   - 使能、相对/绝对位置、速度、停止
  *   - 位置/速度/状态/到位/堵转读取
  *   - 到位等待、设零
  ******************************************************************************
  */

#include "main.h"
#include "can.h"
#include "motor.h"

#include <stddef.h>

/*==============================================================================
 * 私有定义
 *============================================================================*/

#define MOTOR_FRAME_HEAD            (0xC5U)
#define MOTOR_FRAME_TAIL            (0x5CU)
#define MOTOR_CAN_MAX_DLC           (8U)
#define MOTOR_DEVICE_OK             (0x01U)

/* 功能码 */
#define MOTOR_FUNC_READ_VERSION     (0x20U)
#define MOTOR_FUNC_READ_SPEED       (0x29U)
#define MOTOR_FUNC_READ_POSITION    (0x2AU)
#define MOTOR_FUNC_READ_STATE       (0x2CU)
#define MOTOR_FUNC_READ_STALL       (0x2DU)
#define MOTOR_FUNC_READ_ENABLED     (0x2FU)
#define MOTOR_FUNC_READ_IN_POSITION (0x30U)
#define MOTOR_FUNC_SET_ENABLE       (0xFAU)
#define MOTOR_FUNC_SPEED_MODE       (0xF1U)
#define MOTOR_FUNC_ABS_MODE         (0xF2U)
#define MOTOR_FUNC_REL_MODE         (0xF3U)
#define MOTOR_FUNC_STOP             (0xFCU)
#define MOTOR_FUNC_ZERO             (0xF8U)

/* 指令数据长度 */
#define MOTOR_MOVE_DATA_LEN         (8U)
#define MOTOR_SPEED_DATA_LEN        (6U)

/* 应答总长度（包含帧头、地址、功能码、数据、校验和、帧尾） */
#define MOTOR_RESP_SIMPLE_LEN       (6U)
#define MOTOR_RESP_ENABLE_LEN       (7U)
#define MOTOR_RESP_PING_LEN         (8U)
#define MOTOR_RESP_SPEED_LEN        (8U)
#define MOTOR_RESP_STATE_LEN        (7U)
#define MOTOR_RESP_POS_LEN          (10U)

/* 到位等待策略 */
#define MOTOR_WAIT_STATE_EVERY      (5U)
#define MOTOR_WAIT_COMM_FAIL_MAX    (2U)

/*==============================================================================
 * 私有类型
 *============================================================================*/

typedef struct
{
    uint8_t addr;       /**< 驱动器从机地址 */
    uint32_t tx_id;     /**< 主机发送使用的扩展帧 ID */
} motor_can_cfg_t;

/*==============================================================================
 * 私有变量
 *============================================================================*/

/** 地址与发送 ID 配置；地址固定为 1/2/3。 */
static const motor_can_cfg_t s_motor_cfg[MOTOR_COUNT] =
{
    { 1U, 0x1000U },
    { 2U, 0x1000U },
    { 3U, 0x1000U }
};

static bool s_initialized = false;                          /**< 模块是否已初始化 */
static bool s_busy = false;                                 /**< 事务重入保护 */
static uint32_t s_last_cmd_end_tick = 0U;                   /**< 上一条指令结束时刻 */
static uint8_t s_last_device_error[MOTOR_COUNT] = { 0U };   /**< 最近一次驱动器结果码 */

/*==============================================================================
 * 私有函数声明
 *============================================================================*/

static const motor_can_cfg_t *motor_get_cfg(motor_id_t id);
static void motor_wait_cmd_gap(void);
static void motor_flush_fifo(uint32_t fifo);
static void motor_flush_rx(void);
static motor_status_t motor_wait_tx_mailbox_free(void);
static motor_status_t motor_wait_tx_done(uint32_t mailbox);
static motor_status_t motor_send_frame(motor_id_t id, uint8_t code,
                                       const uint8_t *data, uint8_t data_len);
static motor_status_t motor_send_broadcast_stop(void);
static motor_status_t motor_receive_frame(motor_id_t id, uint8_t expected_code,
                                          uint8_t *resp, uint8_t resp_cap,
                                          uint8_t *resp_len);
static motor_status_t motor_can_transact(motor_id_t id, uint8_t code,
                                         const uint8_t *data, uint8_t data_len,
                                         uint8_t *resp, uint8_t resp_cap,
                                         uint8_t *resp_len, bool allow_retry);
static motor_status_t motor_map_device_error(uint8_t code);
static motor_status_t motor_handle_response(motor_id_t id, const uint8_t *resp,
                                            uint8_t len);
static motor_status_t motor_simple_command(motor_id_t id, uint8_t code,
                                           const uint8_t *data, uint8_t data_len,
                                           bool allow_retry);
static motor_status_t motor_validate_motion(uint16_t speed_rpm, uint8_t accel);
static bool motor_is_transient_error(motor_status_t status);
static void motor_put_u16_be(uint8_t *p, uint16_t value);
static void motor_put_u32_be(uint8_t *p, uint32_t value);
static int16_t motor_get_i16_be(const uint8_t *p);
static int32_t motor_get_i32_be(const uint8_t *p);
static uint32_t motor_float_to_u32(float value);
static bool motor_convert_float_to_counts(float value, double factor,
                                          int32_t *counts);
static void motor_build_move_data(int32_t value, uint16_t speed_rpm,
                                  uint8_t accel, uint8_t *data);
static void motor_build_speed_data(int32_t rpm, uint8_t accel, uint8_t *data);

/*==============================================================================
 * 私有函数实现
 *============================================================================*/

/**
 * @brief 获取指定电机的配置。
 * @param[in] id 电机编号。
 * @return 配置指针；编号非法时返回 NULL。
 */
static const motor_can_cfg_t *motor_get_cfg(motor_id_t id)
{
    if ((uint32_t)id >= (uint32_t)MOTOR_COUNT)
    {
        return NULL;
    }

    return &s_motor_cfg[(uint32_t)id];
}

/**
 * @brief 等待两条完整指令之间的最小间隔。
 */
static void motor_wait_cmd_gap(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_last_cmd_end_tick;

    if (elapsed < MOTOR_CMD_GAP_MS)
    {
        HAL_Delay(MOTOR_CMD_GAP_MS - elapsed);
    }
}

/**
 * @brief 清空一个 RX FIFO。
 * @param[in] fifo CAN_RX_FIFO0 或 CAN_RX_FIFO1。
 */
static void motor_flush_fifo(uint32_t fifo)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[MOTOR_CAN_MAX_DLC];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan, fifo) > 0U)
    {
        if (HAL_CAN_GetRxMessage(&hcan, fifo, &header, data) != HAL_OK)
        {
            break;
        }
    }
}

/**
 * @brief 清空两个 RX FIFO，丢弃上一事务的残留应答。
 */
static void motor_flush_rx(void)
{
    motor_flush_fifo(CAN_RX_FIFO0);
    motor_flush_fifo(CAN_RX_FIFO1);
}

/**
 * @brief 等待至少一个空闲发送邮箱。
 * @return MOTOR_OK 有空闲邮箱；MOTOR_ERR_TIMEOUT 超时。
 */
static motor_status_t motor_wait_tx_mailbox_free(void)
{
    uint32_t start = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
    {
        if ((HAL_GetTick() - start) >= MOTOR_ACK_TIMEOUT_MS)
        {
            return MOTOR_ERR_TIMEOUT;
        }
    }

    return MOTOR_OK;
}

/**
 * @brief 等待指定邮箱发送完成。
 * @param[in] mailbox HAL 返回的发送邮箱编号。
 * @return MOTOR_OK 发送完成；MOTOR_ERR_TIMEOUT 超时并尝试中止。
 */
static motor_status_t motor_wait_tx_done(uint32_t mailbox)
{
    uint32_t start = HAL_GetTick();

    while (HAL_CAN_IsTxMessagePending(&hcan, mailbox) != 0U)
    {
        if ((HAL_GetTick() - start) >= MOTOR_ACK_TIMEOUT_MS)
        {
            (void)HAL_CAN_AbortTxRequest(&hcan, mailbox);
            return MOTOR_ERR_TIMEOUT;
        }
    }

    return MOTOR_OK;
}

/**
 * @brief 组协议帧并按 8 字节拆分发送。
 *
 * 帧格式：0xC5 | addr | code | data... | checksum | 0x5C
 * checksum = 前面所有字节的 8 位累加和。
 *
 * @param[in] id       电机编号。
 * @param[in] code     功能码。
 * @param[in] data     数据域，可为 NULL（data_len = 0）。
 * @param[in] data_len 数据域长度。
 * @return MOTOR_OK 发送成功；MOTOR_ERR_PARAM 参数非法；
 *         MOTOR_ERR_TIMEOUT 邮箱/发送超时；MOTOR_ERR_CAN HAL 发送失败。
 */
static motor_status_t motor_send_frame(motor_id_t id, uint8_t code,
                                       const uint8_t *data, uint8_t data_len)
{
    const motor_can_cfg_t *cfg = motor_get_cfg(id);
    uint8_t frame[MOTOR_TX_BUF_SIZE];
    uint8_t frame_len = 0U;
    uint8_t checksum = 0U;
    uint8_t offset = 0U;
    uint8_t i;
    motor_status_t result = MOTOR_OK;

    if (cfg == NULL)
    {
        return MOTOR_ERR_PARAM;
    }
    if (data_len > (uint8_t)(MOTOR_TX_BUF_SIZE - 5U))
    {
        return MOTOR_ERR_PARAM;
    }
    if ((data_len > 0U) && (data == NULL))
    {
        return MOTOR_ERR_PARAM;
    }

    frame[frame_len] = MOTOR_FRAME_HEAD;
    frame_len++;
    frame[frame_len] = cfg->addr;
    frame_len++;
    frame[frame_len] = code;
    frame_len++;

    for (i = 0U; i < data_len; i++)
    {
        frame[frame_len] = data[i];
        frame_len++;
    }

    for (i = 0U; i < frame_len; i++)
    {
        checksum = (uint8_t)(checksum + frame[i]);
    }

    frame[frame_len] = checksum;
    frame_len++;
    frame[frame_len] = MOTOR_FRAME_TAIL;
    frame_len++;

    while (offset < frame_len)
    {
        CAN_TxHeaderTypeDef header = { 0 };
        uint32_t mailbox = 0U;
        uint8_t chunk = (uint8_t)(frame_len - offset);

        if (chunk > MOTOR_CAN_MAX_DLC)
        {
            chunk = (uint8_t)MOTOR_CAN_MAX_DLC;
        }

        result = motor_wait_tx_mailbox_free();
        if (result != MOTOR_OK)
        {
            break;
        }

        header.ExtId = cfg->tx_id;
        header.IDE = CAN_ID_EXT;
        header.RTR = CAN_RTR_DATA;
        header.DLC = (uint32_t)chunk;
        header.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(&hcan, &header, &frame[offset], &mailbox) != HAL_OK)
        {
            result = MOTOR_ERR_CAN;
            break;
        }

        result = motor_wait_tx_done(mailbox);
        if (result != MOTOR_OK)
        {
            break;
        }

        offset = (uint8_t)(offset + chunk);
    }

    s_last_cmd_end_tick = HAL_GetTick();
    return result;
}

/**
 * @brief 接收并重组一条完整协议应答。
 *
 * 接收所有扩展帧，按 5 ms 总线空闲判断一帧结束；随后校验：
 *   - 帧头 0xC5、帧尾 0x5C；
 *   - 地址和功能码与请求匹配；
 *   - 8 位校验和正确。
 *
 * @param[in]  id            期望应答的电机编号。
 * @param[in]  expected_code 期望功能码。
 * @param[out] resp          应答缓冲区。
 * @param[in]  resp_cap      缓冲区容量。
 * @param[out] resp_len      实际应答长度。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
static motor_status_t motor_receive_frame(motor_id_t id, uint8_t expected_code,
                                          uint8_t *resp, uint8_t resp_cap,
                                          uint8_t *resp_len)
{
    const motor_can_cfg_t *cfg = motor_get_cfg(id);
    uint32_t start;
    uint32_t last_rx;
    uint8_t len = 0U;
    bool got = false;

    if ((cfg == NULL) || (resp == NULL) || (resp_len == NULL) || (resp_cap < 5U))
    {
        return MOTOR_ERR_PARAM;
    }

    start = HAL_GetTick();
    last_rx = start;

    for (;;)
    {
        uint32_t fill;

        if ((HAL_GetTick() - start) >= MOTOR_ACK_TIMEOUT_MS)
        {
            return MOTOR_ERR_TIMEOUT;
        }

        fill = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);

        while (fill > 0U)
        {
            CAN_RxHeaderTypeDef header;
            uint8_t data[MOTOR_CAN_MAX_DLC];
            uint8_t dlc;
            uint8_t i;

            if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
            {
                return MOTOR_ERR_CAN;
            }

            fill = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);

            if ((header.IDE != CAN_ID_EXT) || (header.RTR != CAN_RTR_DATA))
            {
                continue;
            }

            dlc = (uint8_t)header.DLC;
            if ((dlc == 0U) || (dlc > MOTOR_CAN_MAX_DLC))
            {
                continue;
            }

            if (((uint16_t)len + (uint16_t)dlc) > (uint16_t)resp_cap)
            {
                return MOTOR_ERR_FRAME;
            }

            for (i = 0U; i < dlc; i++)
            {
                resp[len] = data[i];
                len++;
            }

            got = true;
            last_rx = HAL_GetTick();
        }

        if (!got)
        {
            continue;
        }

        if ((HAL_GetTick() - last_rx) < MOTOR_RX_GAP_MS)
        {
            continue;
        }

        if (len < 5U)
        {
            return MOTOR_ERR_FRAME;
        }
        if ((resp[0] != MOTOR_FRAME_HEAD) || (resp[len - 1U] != MOTOR_FRAME_TAIL))
        {
            return MOTOR_ERR_FRAME;
        }
        if ((resp[1] != cfg->addr) || (resp[2] != expected_code))
        {
            len = 0U;
            got = false;
            last_rx = HAL_GetTick();
            continue;
        }

        {
            uint8_t checksum = 0U;
            uint8_t i;

            for (i = 0U; i < (uint8_t)(len - 2U); i++)
            {
                checksum = (uint8_t)(checksum + resp[i]);
            }

            if (checksum != resp[len - 2U])
            {
                return MOTOR_ERR_CHECKSUM;
            }
        }

        *resp_len = len;
        return MOTOR_OK;
    }
}

/**
 * @brief 执行一次“发送指令 + 接收应答”事务。
 *
 * @param[in]  id           电机编号。
 * @param[in]  code         功能码。
 * @param[in]  data         数据域，可为 NULL。
 * @param[in]  data_len     数据域长度。
 * @param[out] resp         应答缓冲区。
 * @param[in]  resp_cap     缓冲区容量。
 * @param[out] resp_len     应答长度。
 * @param[in]  allow_retry  是否允许对超时/总线/帧错误重试一次。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
static motor_status_t motor_can_transact(motor_id_t id, uint8_t code,
                                         const uint8_t *data, uint8_t data_len,
                                         uint8_t *resp, uint8_t resp_cap,
                                         uint8_t *resp_len, bool allow_retry)
{
    motor_status_t result = MOTOR_ERR_TIMEOUT;
    uint8_t attempts;
    uint8_t attempt;

    if (!s_initialized)
    {
        return MOTOR_ERR_NOT_INIT;
    }
    if (motor_get_cfg(id) == NULL)
    {
        return MOTOR_ERR_PARAM;
    }
    if ((resp == NULL) || (resp_len == NULL) || (resp_cap < 5U))
    {
        return MOTOR_ERR_PARAM;
    }
    if ((data_len > 0U) && (data == NULL))
    {
        return MOTOR_ERR_PARAM;
    }
    if (data_len > (uint8_t)(MOTOR_TX_BUF_SIZE - 5U))
    {
        return MOTOR_ERR_PARAM;
    }
    if (s_busy)
    {
        return MOTOR_ERR_BUSY;
    }

    s_busy = true;

    attempts = allow_retry ? (uint8_t)(MOTOR_RETRY_COUNT + 1U) : 1U;

    for (attempt = 0U; attempt < attempts; attempt++)
    {
        motor_wait_cmd_gap();
        motor_flush_rx();

        result = motor_send_frame(id, code, data, data_len);
        if (result == MOTOR_OK)
        {
            result = motor_receive_frame(id, code, resp, resp_cap, resp_len);
        }

        if (result == MOTOR_OK)
        {
            break;
        }
        if (result == MOTOR_ERR_PARAM)
        {
            break;
        }
    }

    s_busy = false;
    return result;
}

/**
 * @brief 把驱动器原始结果码映射为公共状态。
 * @param[in] code 驱动器应答 Byte1。
 * @return 公共状态。
 */
static motor_status_t motor_map_device_error(uint8_t code)
{
    switch (code)
    {
        case MOTOR_DEVICE_OK:
            return MOTOR_OK;

        case 0xE1U:
        case 0xE2U:
        case 0xE3U:
            return MOTOR_ERR_FRAME;

        case 0xE4U:
            return MOTOR_ERR_CHECKSUM;

        case 0xE5U:
        case 0xE6U:
        default:
            return MOTOR_ERR_DEVICE;
    }
}

/**
 * @brief 校验并解析公共应答中的驱动器结果码。
 *
 * 所有应答至少包含 6 字节：帧头、地址、功能码、结果码、校验和、帧尾。
 * 成功时结果码为 0x01；0xE1~0xE6 为驱动器错误。
 *
 * @param[in] id   电机编号。
 * @param[in] resp 完整应答帧。
 * @param[in] len  应答长度。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
static motor_status_t motor_handle_response(motor_id_t id, const uint8_t *resp,
                                            uint8_t len)
{
    uint8_t code;

    if ((resp == NULL) || (len < MOTOR_RESP_SIMPLE_LEN))
    {
        return MOTOR_ERR_FRAME;
    }

    code = resp[3];

    if ((uint32_t)id < (uint32_t)MOTOR_COUNT)
    {
        s_last_device_error[(uint32_t)id] = code;
    }

    if (code != MOTOR_DEVICE_OK)
    {
        return motor_map_device_error(code);
    }

    return MOTOR_OK;
}

/**
 * @brief 执行一条“成功应答只有结果码”的简单指令。
 *
 * 适用于使能、位置运动、速度、停止、设零。
 *
 * @param[in] id          电机编号。
 * @param[in] code        功能码。
 * @param[in] data        数据域。
 * @param[in] data_len    数据域长度。
 * @param[in] allow_retry 是否允许重试。
 * @return MOTOR_OK 成功；其他错误见 motor_status_t。
 */
static motor_status_t motor_simple_command(motor_id_t id, uint8_t code,
                                           const uint8_t *data, uint8_t data_len,
                                           bool allow_retry)
{
    uint8_t resp[16];
    uint8_t len = 0U;
    motor_status_t result;

    result = motor_can_transact(id, code, data, data_len,
                                resp, (uint8_t)sizeof(resp), &len, allow_retry);
    if (result != MOTOR_OK)
    {
        return result;
    }

    result = motor_handle_response(id, resp, len);
    if (result != MOTOR_OK)
    {
        return result;
    }

    if (len != MOTOR_RESP_SIMPLE_LEN)
    {
        return MOTOR_ERR_FRAME;
    }

    return MOTOR_OK;
}

/**
 * @brief 校验位置/速度运动的公共参数。
 * @param[in] speed_rpm 速度，1~6000 RPM。
 * @param[in] accel     加减速度，0~200。
 * @return MOTOR_OK 合法；MOTOR_ERR_PARAM 非法。
 */
static motor_status_t motor_validate_motion(uint16_t speed_rpm, uint8_t accel)
{
    if ((speed_rpm == 0U) || (speed_rpm > MOTOR_MAX_SPEED_RPM) ||
        (accel > MOTOR_MAX_ACCEL))
    {
        return MOTOR_ERR_PARAM;
    }

    return MOTOR_OK;
}

/**
 * @brief 判断错误是否适合在到位等待中短暂重试。
 * @param[in] status 状态。
 * @return true 表示通信类瞬时错误。
 */
static bool motor_is_transient_error(motor_status_t status)
{
    return (status == MOTOR_ERR_TIMEOUT) ||
           (status == MOTOR_ERR_CAN) ||
           (status == MOTOR_ERR_FRAME) ||
           (status == MOTOR_ERR_CHECKSUM);
}

/**
 * @brief 写入大端 uint16_t。
 */
static void motor_put_u16_be(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)(value & 0xFFU);
}

/**
 * @brief 写入大端 uint32_t。
 */
static void motor_put_u32_be(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)((value >> 24) & 0xFFU);
    p[1] = (uint8_t)((value >> 16) & 0xFFU);
    p[2] = (uint8_t)((value >> 8) & 0xFFU);
    p[3] = (uint8_t)(value & 0xFFU);
}

/**
 * @brief 读取大端 int16_t。
 */
static int16_t motor_get_i16_be(const uint8_t *p)
{
    union
    {
        uint16_t u;
        int16_t i;
    } value;

    value.u = (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
    return value.i;
}

/**
 * @brief 读取大端 int32_t。
 */
static int32_t motor_get_i32_be(const uint8_t *p)
{
    union
    {
        uint32_t u;
        int32_t i;
    } value;

    value.u = ((uint32_t)p[0] << 24) |
              ((uint32_t)p[1] << 16) |
              ((uint32_t)p[2] << 8) |
              ((uint32_t)p[3]);
    return value.i;
}

/**
 * @brief 将 float 的 IEEE-754 位模式转为 uint32_t。
 */
static uint32_t motor_float_to_u32(float value)
{
    union
    {
        float f;
        uint32_t u;
    } data;

    data.f = value;
    return data.u;
}

/**
 * @brief 将浮点圈数/角度换算成 counts，并检查范围和 NaN。
 *
 * @param[in]  value  输入值（圈或度）。
 * @param[in]  factor 每单位对应的 counts。
 * @param[out] counts 换算结果。
 * @return true 成功；false 输入无效或超出 int32_t 范围。
 */
static bool motor_convert_float_to_counts(float value, double factor,
                                          int32_t *counts)
{
    double scaled;
    double rounded;

    if ((counts == NULL) || (value != value))
    {
        return false;
    }

    scaled = (double)value * factor;
    rounded = (scaled >= 0.0) ? (scaled + 0.5) : (scaled - 0.5);

    if ((rounded > (double)INT32_MAX) || (rounded < (double)INT32_MIN))
    {
        return false;
    }

    *counts = (int32_t)rounded;
    return true;
}

/**
 * @brief 构造 0xF2 / 0xF3 的 8 字节数据域。
 *
 * 调用者必须保证 value != INT32_MIN。
 */
static void motor_build_move_data(int32_t value, uint16_t speed_rpm,
                                  uint8_t accel, uint8_t *data)
{
    uint32_t magnitude;

    if (value < 0)
    {
        data[0] = 1U;
        magnitude = (uint32_t)(-value);
    }
    else
    {
        data[0] = 0U;
        magnitude = (uint32_t)value;
    }

    data[1] = accel;
    motor_put_u16_be(&data[2], speed_rpm);
    motor_put_u32_be(&data[4], magnitude);
}

/**
 * @brief 构造 0xF1 的 6 字节数据域。
 *
 * 调用者必须保证 rpm != 0、rpm != INT32_MIN、|rpm| <= 6000。
 */
static void motor_build_speed_data(int32_t rpm, uint8_t accel, uint8_t *data)
{
    uint32_t magnitude;
    uint32_t raw_float;

    if (rpm < 0)
    {
        data[0] = 1U;
        magnitude = (uint32_t)(-rpm);
    }
    else
    {
        data[0] = 0U;
        magnitude = (uint32_t)rpm;
    }

    data[1] = accel;
    raw_float = motor_float_to_u32((float)magnitude);
    motor_put_u32_be(&data[2], raw_float);
}

/**
 * @brief 广播立即停止，不等待应答。
 * @return MOTOR_OK 发送成功；其他错误见 motor_status_t。
 */
static motor_status_t motor_send_broadcast_stop(void)
{
    uint8_t frame[5];
    CAN_TxHeaderTypeDef header = { 0 };
    uint32_t mailbox = 0U;
    motor_status_t result;

    frame[0] = MOTOR_FRAME_HEAD;
    frame[1] = 0x00U;
    frame[2] = MOTOR_FUNC_STOP;
    frame[3] = (uint8_t)(frame[0] + frame[1] + frame[2]);
    frame[4] = MOTOR_FRAME_TAIL;

    result = motor_wait_tx_mailbox_free();
    if (result != MOTOR_OK)
    {
        s_last_cmd_end_tick = HAL_GetTick();
        return result;
    }

    header.ExtId = 0x1000U;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 5U;
    header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &header, frame, &mailbox) != HAL_OK)
    {
        s_last_cmd_end_tick = HAL_GetTick();
        return MOTOR_ERR_CAN;
    }

    result = motor_wait_tx_done(mailbox);
    s_last_cmd_end_tick = HAL_GetTick();

    return result;
}

/*==============================================================================
 * 公共函数实现
 *============================================================================*/

motor_status_t Motor_Init(void)
{
    CAN_FilterTypeDef filter = { 0 };
    uint8_t i;

    if (s_initialized)
    {
        return MOTOR_OK;
    }
    if (hcan.Instance != CAN1)
    {
        return MOTOR_ERR_CAN;
    }
    if ((hcan.State != HAL_CAN_STATE_READY) &&
        (hcan.State != HAL_CAN_STATE_LISTENING))
    {
        return MOTOR_ERR_CAN;
    }

    /* 接收所有扩展帧，具体地址/功能码在软件层匹配。 */
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
    {
        return MOTOR_ERR_CAN;
    }

    if (hcan.State == HAL_CAN_STATE_READY)
    {
        if (HAL_CAN_Start(&hcan) != HAL_OK)
        {
            return MOTOR_ERR_CAN;
        }
    }

    motor_flush_rx();

    for (i = 0U; i < (uint8_t)MOTOR_COUNT; i++)
    {
        s_last_device_error[i] = 0U;
    }

    s_last_cmd_end_tick = HAL_GetTick() - MOTOR_CMD_GAP_MS;
    s_busy = false;
    s_initialized = true;

    return MOTOR_OK;
}

motor_status_t Motor_Ping(motor_id_t id)
{
    uint8_t resp[MOTOR_RX_BUF_SIZE];
    uint8_t len = 0U;
    motor_status_t result;

    result = motor_can_transact(id, MOTOR_FUNC_READ_VERSION, NULL, 0U,
                                resp, (uint8_t)sizeof(resp), &len, true);
    if (result != MOTOR_OK)
    {
        return result;
    }

    result = motor_handle_response(id, resp, len);
    if (result != MOTOR_OK)
    {
        return result;
    }

    if (len != MOTOR_RESP_PING_LEN)
    {
        return MOTOR_ERR_FRAME;
    }

    return MOTOR_OK;
}

motor_status_t Motor_Enable(motor_id_t id, bool enable)
{
    uint8_t data[1];
    uint8_t resp[16];
    uint8_t len = 0U;
    motor_status_t result;

    data[0] = (uint8_t)(enable ? 0x00U : 0x01U);

    result = motor_can_transact(id, MOTOR_FUNC_SET_ENABLE, data, 1U,
                                resp, (uint8_t)sizeof(resp), &len, true);
    if (result != MOTOR_OK)
    {
        return result;
    }

    result = motor_handle_response(id, resp, len);
    if (result != MOTOR_OK)
    {
        return result;
    }

    if ((len != MOTOR_RESP_ENABLE_LEN) || (resp[4] > 1U))
    {
        return MOTOR_ERR_FRAME;
    }

    return MOTOR_OK;
}

motor_status_t Motor_ReadEnabled(motor_id_t id, bool *enabled)
{
    uint8_t resp[16];
    uint8_t len = 0U;
    motor_status_t result;

    if (enabled == NULL)
    {
        return MOTOR_ERR_PARAM;
    }

    result = motor_can_transact(id, MOTOR_FUNC_READ_ENABLED, NULL, 0U,
                                resp, (uint8_t)sizeof(resp), &len, true);
    if (result != MOTOR_OK)
    {
        return result;
    }

    result = motor_handle_response(id, resp, len);
    if (result != MOTOR_OK)
    {
        return result;
    }

    if (len != MOTOR_RESP_STATE_LEN)
    {
        return MOTOR_ERR_FRAME;
    }
    if (resp[4] > 1U)
    {
        return MOTOR_ERR_FRAME;
    }

    /* 0x2F：0 = 使能，1 = 失能。 */
    *enabled = (resp[4] == 0U);
    return MOTOR_OK;
}

motor_status_t Motor_MoveRelative(motor_id_t id, int32_t delta_counts,
                                  uint16_t speed_rpm, uint8_t accel)
{
    uint8_t data[MOTOR_MOVE_DATA_LEN];
    motor_status_t result;

    if (delta_counts == INT32_MIN)
    {
        return MOTOR_ERR_PARAM;
    }

    result = motor_validate_motion(speed_rpm, accel);
    if (result != MOTOR_OK)
    {
        return result;
    }

    motor_build_move_data(delta_counts, speed_rpm, accel, data);
    return motor_simple_command(id, MOTOR_FUNC_REL_MODE, data,
                                MOTOR_MOVE_DATA_LEN, false);
}

motor_status_t Motor_MoveAbsolute(motor_id_t id, int32_t target_counts,
                                  uint16_t speed_rpm, uint8_t accel)
{
    uint8_t data[MOTOR_MOVE_DATA_LEN];
    motor_status_t result;

    if (target_counts == INT32_MIN)
    {
        return MOTOR_ERR_PARAM;
    }
    if ((target_counts < -MOTOR_ABS_LIMIT_COUNTS) ||
        (target_counts > MOTOR_ABS_LIMIT_COUNTS))
    {
        return MOTOR_ERR_RANGE;
    }

    result = motor_validate_motion(speed_rpm, accel);
    if (result != MOTOR_OK)
    {
        return result;
    }

    motor_build_move_data(target_counts, speed_rpm, accel, data);
    return motor_simple_command(id, MOTOR_FUNC_ABS_MODE, data,
                                MOTOR_MOVE_DATA_LEN, true);
}

motor_status_t Motor_MoveRelativeRev(motor_id_t id, float rev,
                                     uint16_t speed_rpm, uint8_t accel)
{
    int32_t counts;

    if (!motor_convert_float_to_counts(rev, (double)MOTOR_COUNTS_PER_REV, &counts))
    {
        return MOTOR_ERR_PARAM;
    }

    return Motor_MoveRelative(id, counts, speed_rpm, accel);
}

motor_status_t Motor_MoveRelativeDeg(motor_id_t id, float deg,
                                     uint16_t speed_rpm, uint8_t accel)
{
    int32_t counts;

    if (!motor_convert_float_to_counts(deg,
                                       (double)MOTOR_COUNTS_PER_REV / 360.0, &counts))
    {
        return MOTOR_ERR_PARAM;
    }

    return Motor_MoveRelative(id, counts, speed_rpm, accel);
}

motor_status_t Motor_MoveAbsoluteRev(motor_id_t id, float rev,
                                     uint16_t speed_rpm, uint8_t accel)
{
    int32_t counts;

    if (!motor_convert_float_to_counts(rev, (double)MOTOR_COUNTS_PER_REV, &counts))
    {
        return MOTOR_ERR_PARAM;
    }

    return Motor_MoveAbsolute(id, counts, speed_rpm, accel);
}

motor_status_t Motor_MoveAbsoluteDeg(motor_id_t id, float deg,
                                     uint16_t speed_rpm, uint8_t accel)
{
    int32_t counts;

    if (!motor_convert_float_to_counts(deg,
                                       (double)MOTOR_COUNTS_PER_REV / 360.0, &counts))
    {
        return MOTOR_ERR_PARAM;
    }

    return Motor_MoveAbsolute(id, counts, speed_rpm, accel);
}

motor_status_t Motor_SetSpeed(motor_id_t id, int32_t rpm, uint8_t accel)
{
    uint8_t data[MOTOR_SPEED_DATA_LEN];

    if ((rpm == 0) || (rpm == INT32_MIN) ||
        (rpm > (int32_t)MOTOR_MAX_SPEED_RPM) ||
        (rpm < -(int32_t)MOTOR_MAX_SPEED_RPM) ||
        (accel > MOTOR_MAX_ACCEL))
    {
        return MOTOR_ERR_PARAM;
    }

    motor_build_speed_data(rpm, accel, data);
    return motor_simple_command(id, MOTOR_FUNC_SPEED_MODE, data,
                                MOTOR_SPEED_DATA_LEN, true);
}

motor_status_t Motor_Stop(motor_id_t id)
{
    return motor_simple_command(id, MOTOR_FUNC_STOP, NULL, 0U, true);
}

motor_status_t Motor_StopAll(void)
{
    motor_status_t result;

    if (!s_initialized)
    {
        return MOTOR_ERR_NOT_INIT;
    }
    if (s_busy)
    {
        return MOTOR_ERR_BUSY;
    }

    s_busy = true;

    motor_wait_cmd_gap();
    motor_flush_rx();
    result = motor_send_broadcast_stop();

    s_busy = false;
    return result;
}

motor_status_t Motor_ReadPosition(motor_id_t id, int32_t *counts)
{
    uint8_t resp[16];
    uint8_t len = 0U;
    motor_status_t result;

    if (counts == NULL)
    {
        return MOTOR_ERR_PARAM;
    }

    result = motor_can_transact(id, MOTOR_FUNC_READ_POSITION, NULL, 0U,
                                resp, (uint8_t)sizeof(resp), &len, true);
    if (result != MOTOR_OK)
    {
        return result;
    }

    result = motor_handle_response(id, resp, len);
    if (result != MOTOR_OK)
    {
        return result;
    }

    if (len != MOTOR_RESP_POS_LEN)
    {
        return MOTOR_ERR_FRAME;
    }

    *counts = motor_get_i32_be(&resp[4]);
    return MOTOR_OK;
}

motor_status_t Motor_ReadSpeed(motor_id_t id, int16_t *rpm)
{
    uint8_t resp[16];
    uint8_t len = 0U;
    motor_status_t result;

    if (rpm == NULL)
    {
        return MOTOR_ERR_PARAM;
    }

    result = motor_can_transact(id, MOTOR_FUNC_READ_SPEED, NULL, 0U,
                                resp, (uint8_t)sizeof(resp), &len, true);
    if (result != MOTOR_OK)
    {
        return result;
    }

    result = motor_handle_response(id, resp, len);
    if (result != MOTOR_OK)
    {
        return result;
    }

    if (len != MOTOR_RESP_SPEED_LEN)
    {
        return MOTOR_ERR_FRAME;
    }

    *rpm = motor_get_i16_be(&resp[4]);
    return MOTOR_OK;
}

motor_status_t Motor_ReadState(motor_id_t id, motor_run_state_t *state)
{
    uint8_t resp[16];
    uint8_t len = 0U;
    motor_status_t result;

    if (state == NULL)
    {
        return MOTOR_ERR_PARAM;
    }

    result = motor_can_transact(id, MOTOR_FUNC_READ_STATE, NULL, 0U,
                                resp, (uint8_t)sizeof(resp), &len, true);
    if (result != MOTOR_OK)
    {
        return result;
    }

    result = motor_handle_response(id, resp, len);
    if (result != MOTOR_OK)
    {
        return result;
    }

    if ((len != MOTOR_RESP_STATE_LEN) || (resp[4] > 5U))
    {
        return MOTOR_ERR_FRAME;
    }

    *state = (motor_run_state_t)resp[4];
    return MOTOR_OK;
}

motor_status_t Motor_ReadInPosition(motor_id_t id, bool *in_position)
{
    uint8_t resp[16];
    uint8_t len = 0U;
    motor_status_t result;

    if (in_position == NULL)
    {
        return MOTOR_ERR_PARAM;
    }

    result = motor_can_transact(id, MOTOR_FUNC_READ_IN_POSITION, NULL, 0U,
                                resp, (uint8_t)sizeof(resp), &len, true);
    if (result != MOTOR_OK)
    {
        return result;
    }

    result = motor_handle_response(id, resp, len);
    if (result != MOTOR_OK)
    {
        return result;
    }

    if ((len != MOTOR_RESP_STATE_LEN) || (resp[4] > 1U))
    {
        return MOTOR_ERR_FRAME;
    }

    *in_position = (resp[4] != 0U);
    return MOTOR_OK;
}

motor_status_t Motor_ReadStall(motor_id_t id, bool *stalled)
{
    uint8_t resp[16];
    uint8_t len = 0U;
    motor_status_t result;

    if (stalled == NULL)
    {
        return MOTOR_ERR_PARAM;
    }

    result = motor_can_transact(id, MOTOR_FUNC_READ_STALL, NULL, 0U,
                                resp, (uint8_t)sizeof(resp), &len, true);
    if (result != MOTOR_OK)
    {
        return result;
    }

    result = motor_handle_response(id, resp, len);
    if (result != MOTOR_OK)
    {
        return result;
    }

    if ((len != MOTOR_RESP_STATE_LEN) || (resp[4] > 1U))
    {
        return MOTOR_ERR_FRAME;
    }

    *stalled = (resp[4] != 0U);
    return MOTOR_OK;
}

motor_status_t Motor_WaitInPosition(motor_id_t id, uint32_t timeout_ms)
{
    uint32_t start;
    uint32_t effective_timeout;
    uint32_t polls = 0U;
    uint8_t comm_fail = 0U;
    motor_status_t result;
    bool in_position = false;
    bool stalled = false;

    if ((uint32_t)id >= (uint32_t)MOTOR_COUNT)
    {
        return MOTOR_ERR_PARAM;
    }

    effective_timeout = (timeout_ms == 0U) ?
                        MOTOR_DEFAULT_WAIT_TIMEOUT_MS : timeout_ms;
    start = HAL_GetTick();

    for (;;)
    {
        if ((HAL_GetTick() - start) >= effective_timeout)
        {
            return MOTOR_ERR_TIMEOUT;
        }

        result = Motor_ReadInPosition(id, &in_position);
        if (result == MOTOR_OK)
        {
            comm_fail = 0U;

            if (in_position)
            {
                return MOTOR_OK;
            }

            result = Motor_ReadStall(id, &stalled);
            if (result == MOTOR_OK)
            {
                comm_fail = 0U;

                if (stalled)
                {
                    return MOTOR_ERR_STALL;
                }
            }
            else if (!motor_is_transient_error(result))
            {
                return result;
            }
            else
            {
                comm_fail++;
            }
        }
        else if (!motor_is_transient_error(result))
        {
            return result;
        }
        else
        {
            comm_fail++;
        }

        if (comm_fail >= MOTOR_WAIT_COMM_FAIL_MAX)
        {
            return result;
        }

        polls++;
        if ((polls % MOTOR_WAIT_STATE_EVERY) == 0U)
        {
            motor_run_state_t state;

            result = Motor_ReadState(id, &state);
            if (result == MOTOR_OK)
            {
                comm_fail = 0U;

                if ((state == MOTOR_STATE_OVERLOAD) ||
                    (state == MOTOR_STATE_STALLED) ||
                    (state == MOTOR_STATE_UNDERVOLTAGE))
                {
                    return MOTOR_ERR_STATE;
                }
            }
            else if (!motor_is_transient_error(result))
            {
                return result;
            }
            else
            {
                comm_fail++;
            }

            if (comm_fail >= MOTOR_WAIT_COMM_FAIL_MAX)
            {
                return result;
            }
        }

        HAL_Delay(MOTOR_WAIT_POLL_MS);
    }
}

motor_status_t Motor_ZeroPosition(motor_id_t id)
{
    return motor_simple_command(id, MOTOR_FUNC_ZERO, NULL, 0U, false);
}

float Motor_CountsToRev(int32_t counts)
{
    return (float)counts / (float)MOTOR_COUNTS_PER_REV;
}

float Motor_CountsToDeg(int32_t counts)
{
    return ((float)counts * 360.0f) / (float)MOTOR_COUNTS_PER_REV;
}

int32_t Motor_RevToCounts(float rev)
{
    int32_t counts;

    if (!motor_convert_float_to_counts(rev, (double)MOTOR_COUNTS_PER_REV, &counts))
    {
        return 0;
    }

    return counts;
}

int32_t Motor_DegToCounts(float deg)
{
    int32_t counts;

    if (!motor_convert_float_to_counts(deg,
                                       (double)MOTOR_COUNTS_PER_REV / 360.0, &counts))
    {
        return 0;
    }

    return counts;
}

uint8_t Motor_GetLastDeviceError(motor_id_t id)
{
    if ((uint32_t)id >= (uint32_t)MOTOR_COUNT)
    {
        return 0U;
    }

    return s_last_device_error[(uint32_t)id];
}