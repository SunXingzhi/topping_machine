#include "main.h"
#include "can.h"
#include "motor.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan;

#define RXQ_SIZE 32
typedef struct
{
    uint32_t id;
    uint32_t ide;
    uint32_t rtr;
    uint32_t dlc;
    uint8_t data[8];
} rx_msg_t;

static rx_msg_t g_rxq[RXQ_SIZE];
static unsigned g_rx_head = 0U;
static unsigned g_rx_tail = 0U;

static uint32_t g_tick = 1000U;
static uint8_t g_last_tx[64];
static unsigned g_last_tx_len = 0U;
static unsigned g_tx_frames = 0U;

static uint8_t g_response[32];
static unsigned g_response_len = 0U;
static unsigned g_response_chunk = 8U;
static unsigned g_respond_after_frames = 1U;
static unsigned g_auto_respond = 0U;

static CAN_FilterTypeDef g_filter;

uint32_t HAL_GetTick(void)
{
    return g_tick++;
}

void HAL_Delay(uint32_t Delay)
{
    g_tick += Delay;
}

HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *handle, const CAN_FilterTypeDef *filter)
{
    (void)handle;
    g_filter = *filter;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *handle)
{
    handle->State = HAL_CAN_STATE_LISTENING;
    return HAL_OK;
}

uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *handle)
{
    (void)handle;
    return 3U;
}

uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *handle, uint32_t mailbox)
{
    (void)handle;
    (void)mailbox;
    return 0U;
}

HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *handle, uint32_t mailbox)
{
    (void)handle;
    (void)mailbox;
    return HAL_OK;
}

uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *handle, uint32_t fifo)
{
    (void)handle;
    (void)fifo;
    return (uint32_t)(g_rx_tail - g_rx_head);
}

HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *handle, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *header, uint8_t data[])
{
    rx_msg_t *msg;
    unsigned i;

    (void)handle;
    (void)fifo;

    if (g_rx_head == g_rx_tail)
    {
        return HAL_ERROR;
    }

    msg = &g_rxq[g_rx_head % RXQ_SIZE];
    g_rx_head++;

    header->StdId = 0U;
    header->ExtId = msg->id;
    header->IDE = msg->ide;
    header->RTR = msg->rtr;
    header->DLC = msg->dlc;
    header->Timestamp = 0U;
    header->FilterMatchIndex = 0U;

    for (i = 0U; i < msg->dlc; i++)
    {
        data[i] = msg->data[i];
    }

    return HAL_OK;
}

static void queue_response(void)
{
    unsigned offset = 0U;

    while (offset < g_response_len)
    {
        rx_msg_t *msg;
        unsigned chunk = g_response_chunk;
        unsigned i;

        if ((chunk == 0U) || (chunk > 8U))
        {
            chunk = 8U;
        }
        if (chunk > (g_response_len - offset))
        {
            chunk = g_response_len - offset;
        }

        msg = &g_rxq[g_rx_tail % RXQ_SIZE];
        g_rx_tail++;

        msg->id = 0x1000U;
        msg->ide = CAN_ID_EXT;
        msg->rtr = CAN_RTR_DATA;
        msg->dlc = chunk;
        for (i = 0U; i < chunk; i++)
        {
            msg->data[i] = g_response[offset + i];
        }

        offset += chunk;
    }
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *handle,
                                       const CAN_TxHeaderTypeDef *header,
                                       const uint8_t data[], uint32_t *mailbox)
{
    unsigned i;

    (void)handle;

    for (i = 0U; i < header->DLC; i++)
    {
        assert(g_last_tx_len < sizeof(g_last_tx));
        g_last_tx[g_last_tx_len] = data[i];
        g_last_tx_len++;
    }

    g_tx_frames++;

    if ((g_auto_respond != 0U) && (g_tx_frames == g_respond_after_frames))
    {
        queue_response();
    }

    *mailbox = 1U;
    return HAL_OK;
}

static void reset_capture(void)
{
    g_last_tx_len = 0U;
    g_tx_frames = 0U;
    g_response_len = 0U;
    g_response_chunk = 8U;
    g_respond_after_frames = 1U;
    g_auto_respond = 0U;
    g_rx_head = 0U;
    g_rx_tail = 0U;
}

static void set_response(const uint8_t *data, unsigned len,
                         unsigned chunk, unsigned after_frames)
{
    assert(len <= sizeof(g_response));
    memcpy(g_response, data, len);
    g_response_len = len;
    g_response_chunk = chunk;
    g_respond_after_frames = after_frames;
    g_auto_respond = 1U;
}

static void expect_tx(const uint8_t *expected, unsigned len)
{
    assert(g_last_tx_len == len);
    assert(memcmp(g_last_tx, expected, len) == 0);
}

static void test_init(void)
{
    hcan.Instance = CAN1;
    hcan.State = HAL_CAN_STATE_READY;

    assert(Motor_Init() == MOTOR_OK);
    assert(hcan.State == HAL_CAN_STATE_LISTENING);
    assert(g_filter.FilterActivation == CAN_FILTER_ENABLE);
    assert(g_filter.FilterScale == CAN_FILTERSCALE_32BIT);
    assert(g_filter.FilterMode == CAN_FILTERMODE_IDMASK);
    assert(g_filter.FilterFIFOAssignment == CAN_FILTER_FIFO0);
}

static void test_ping_success(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x20, 0x01, 0x0A, 0x0B, 0xFC, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x20, 0xE6, 0x5C };

    reset_capture();
    set_response(response, sizeof(response), 3U, 1U);

    assert(Motor_Ping(MOTOR_1) == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
    assert(g_tx_frames == 1U);
    assert(Motor_GetLastDeviceError(MOTOR_1) == 0x01U);
}

static void test_ping_error(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x20, 0xE6, 0xCC, 0x5C };

    reset_capture();
    set_response(response, sizeof(response), 6U, 1U);

    assert(Motor_Ping(MOTOR_1) == MOTOR_ERR_DEVICE);
    assert(Motor_GetLastDeviceError(MOTOR_1) == 0xE6U);
}

static void test_ping_timeout(void)
{
    reset_capture();

    assert(Motor_Ping(MOTOR_1) == MOTOR_ERR_TIMEOUT);
    assert(g_tx_frames == 2U);
}

static void test_enable(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0xFA, 0x01, 0x00, 0xC1, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0xFA, 0x00, 0xC0, 0x5C };

    reset_capture();
    set_response(response, sizeof(response), 7U, 1U);

    assert(Motor_Enable(MOTOR_1, true) == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
    assert(g_tx_frames == 1U);
}

static void test_read_enabled(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x2F, 0x01, 0x00, 0xF6, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x2F, 0xF5, 0x5C };
    bool enabled = false;

    reset_capture();
    set_response(response, sizeof(response), 7U, 1U);

    assert(Motor_ReadEnabled(MOTOR_1, &enabled) == MOTOR_OK);
    assert(enabled == true);
    expect_tx(expected, sizeof(expected));
}

static void test_read_position(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x2A, 0x01, 0x00, 0x00, 0x32, 0x00, 0x23, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x2A, 0xF0, 0x5C };
    int32_t counts = 0;

    reset_capture();
    set_response(response, sizeof(response), 4U, 1U);

    assert(Motor_ReadPosition(MOTOR_1, &counts) == MOTOR_OK);
    assert(counts == 12800);
    expect_tx(expected, sizeof(expected));
}

static void test_read_speed(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x29, 0x01, 0x00, 0x64, 0x54, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x29, 0xEF, 0x5C };
    int16_t rpm = 0;

    reset_capture();
    set_response(response, sizeof(response), 8U, 1U);

    assert(Motor_ReadSpeed(MOTOR_1, &rpm) == MOTOR_OK);
    assert(rpm == 100);
    expect_tx(expected, sizeof(expected));
}

static void test_read_state(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x2C, 0x01, 0x02, 0xF5, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x2C, 0xF2, 0x5C };
    motor_run_state_t state = MOTOR_STATE_STOPPED;

    reset_capture();
    set_response(response, sizeof(response), 7U, 1U);

    assert(Motor_ReadState(MOTOR_1, &state) == MOTOR_OK);
    assert(state == MOTOR_STATE_RUNNING);
    expect_tx(expected, sizeof(expected));
}

static void test_read_in_position(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x30, 0x01, 0x01, 0xF8, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x30, 0xF6, 0x5C };
    bool in_position = false;

    reset_capture();
    set_response(response, sizeof(response), 7U, 1U);

    assert(Motor_ReadInPosition(MOTOR_1, &in_position) == MOTOR_OK);
    assert(in_position == true);
    expect_tx(expected, sizeof(expected));
}

static void test_read_stall(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x2D, 0x01, 0x01, 0xF5, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x2D, 0xF3, 0x5C };
    bool stalled = false;

    reset_capture();
    set_response(response, sizeof(response), 7U, 1U);

    assert(Motor_ReadStall(MOTOR_1, &stalled) == MOTOR_OK);
    assert(stalled == true);
    expect_tx(expected, sizeof(expected));
}

static void test_move_absolute(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0xF2, 0x01, 0xB9, 0x5C };
    const uint8_t expected[] = {
        0xC5, 0x01, 0xF2, 0x00, 0x64, 0x07, 0xD0,
        0x00, 0x00, 0x64, 0x00, 0x57, 0x5C
    };

    reset_capture();
    set_response(response, sizeof(response), 6U, 2U);

    assert(Motor_MoveAbsolute(MOTOR_1, 25600, 2000, 100) == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
    assert(g_tx_frames == 2U);
}

static void test_move_relative_no_retry(void)
{
    const uint8_t expected[] = {
        0xC5, 0x01, 0xF3, 0x00, 0x64, 0x07, 0xD0,
        0x00, 0x00, 0x32, 0x00, 0x26, 0x5C
    };

    reset_capture();

    assert(Motor_MoveRelative(MOTOR_1, 12800, 2000, 100) == MOTOR_ERR_TIMEOUT);
    expect_tx(expected, sizeof(expected));
    assert(g_tx_frames == 2U);
}

static void test_absolute_range(void)
{
    reset_capture();

    assert(Motor_MoveAbsolute(MOTOR_1, 76800, 1000, 100) == MOTOR_ERR_RANGE);
    assert(g_tx_frames == 0U);
    assert(g_last_tx_len == 0U);
}

static void test_set_speed(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0xF1, 0x01, 0xB8, 0x5C };
    const uint8_t expected[] = {
        0xC5, 0x01, 0xF1, 0x00, 0x64, 0x44, 0x16, 0x00, 0x00, 0x75, 0x5C
    };

    reset_capture();
    set_response(response, sizeof(response), 6U, 1U);

    assert(Motor_SetSpeed(MOTOR_1, 600, 100) == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
}

static void test_stop(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0xFC, 0x01, 0xC3, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0xFC, 0xC2, 0x5C };

    reset_capture();
    set_response(response, sizeof(response), 6U, 1U);

    assert(Motor_Stop(MOTOR_1) == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
}

static void test_stop_all(void)
{
    const uint8_t expected[] = { 0xC5, 0x00, 0xFC, 0xC1, 0x5C };

    reset_capture();

    assert(Motor_StopAll() == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
    assert(g_tx_frames == 1U);
}

static void test_zero(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0xF8, 0x01, 0xBF, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0xF8, 0xBE, 0x5C };

    reset_capture();
    set_response(response, sizeof(response), 6U, 1U);

    assert(Motor_ZeroPosition(MOTOR_1) == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
}

static void test_wait_in_position(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x30, 0x01, 0x01, 0xF8, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x30, 0xF6, 0x5C };

    reset_capture();
    set_response(response, sizeof(response), 7U, 1U);

    assert(Motor_WaitInPosition(MOTOR_1, 1000U) == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
}

static void test_enable_disable(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0xFA, 0x01, 0x01, 0xC2, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0xFA, 0x01, 0xC1, 0x5C };

    reset_capture();
    set_response(response, sizeof(response), 7U, 1U);

    assert(Motor_Enable(MOTOR_1, false) == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
}

static void test_read_enabled_false(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x2F, 0x01, 0x01, 0xF7, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x2F, 0xF5, 0x5C };
    bool enabled = true;

    reset_capture();
    set_response(response, sizeof(response), 7U, 1U);

    assert(Motor_ReadEnabled(MOTOR_1, &enabled) == MOTOR_OK);
    assert(enabled == false);
    expect_tx(expected, sizeof(expected));
}

static void test_read_position_negative(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x2A, 0x01, 0xFF, 0xFF, 0xCE, 0x00, 0xBD, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x2A, 0xF0, 0x5C };
    int32_t counts = 0;

    reset_capture();
    set_response(response, sizeof(response), 10U, 1U);

    assert(Motor_ReadPosition(MOTOR_1, &counts) == MOTOR_OK);
    assert(counts == -12800);
    expect_tx(expected, sizeof(expected));
}

static void test_read_speed_negative(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0x29, 0x01, 0xFF, 0x9C, 0x8B, 0x5C };
    const uint8_t expected[] = { 0xC5, 0x01, 0x29, 0xEF, 0x5C };
    int16_t rpm = 0;

    reset_capture();
    set_response(response, sizeof(response), 8U, 1U);

    assert(Motor_ReadSpeed(MOTOR_1, &rpm) == MOTOR_OK);
    assert(rpm == -100);
    expect_tx(expected, sizeof(expected));
}

static void test_set_speed_negative(void)
{
    const uint8_t response[] = { 0xC5, 0x01, 0xF1, 0x01, 0xB8, 0x5C };
    const uint8_t expected[] = {
        0xC5, 0x01, 0xF1, 0x01, 0x64, 0x44, 0x16, 0x00, 0x00, 0x76, 0x5C
    };

    reset_capture();
    set_response(response, sizeof(response), 6U, 1U);

    assert(Motor_SetSpeed(MOTOR_1, -600, 100) == MOTOR_OK);
    expect_tx(expected, sizeof(expected));
}

static void test_move_absolute_deg_range(void)
{
    reset_capture();

    assert(Motor_MoveAbsoluteDeg(MOTOR_1, 360.1f, 1000U, 100U) == MOTOR_ERR_RANGE);
    assert(g_tx_frames == 0U);
}

static void test_conversions(void)
{
    assert(Motor_RevToCounts(1.0f) == 51200);
    assert(Motor_DegToCounts(90.0f) == 12800);
    assert(Motor_CountsToRev(25600) == 0.5f);
    assert(Motor_CountsToDeg(12800) == 90.0f);
}

int main(void)
{
    test_init();
    test_ping_success();
    test_ping_error();
    test_ping_timeout();
    test_enable();
    test_enable_disable();
    test_read_enabled();
    test_read_enabled_false();
    test_read_position();
    test_read_position_negative();
    test_read_speed();
    test_read_speed_negative();
    test_read_state();
    test_read_in_position();
    test_read_stall();
    test_move_absolute();
    test_move_relative_no_retry();
    test_absolute_range();
    test_move_absolute_deg_range();
    test_set_speed();
    test_set_speed_negative();
    test_stop();
    test_stop_all();
    test_zero();
    test_wait_in_position();
    test_conversions();

    printf("ALL MOTOR HOST TESTS PASSED\n");
    return 0;
}