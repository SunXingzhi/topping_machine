/**
  ******************************************************************************
  * @file    motor_validation_test.c
  * @brief   PD42S1 三轴 CAN 驱动接口实机验证测试
  *
  * 该文件只在 MOTOR_BUILD_TESTS=ON 时参与构建。
  * 生产固件不包含本测试，main.c 不会自动使能或运动。
  ******************************************************************************
  */

#include "main.h"
#include "motor.h"
#include "motor_validation_test.h"

/*
 * 接口验证测试参数：
 *   速度为正 = 方向 0（手册定义为顺时针）
 *   每台独立测试，单台失败不会阻塞其它电机
 */
#define MOTOR_TEST_SPEED_RPM         100
#define MOTOR_TEST_ACCEL             100U
#define MOTOR_TEST_RUN_MS            500U
#define MOTOR_TEST_STOP_SETTLE_MS    300U
#define MOTOR_TEST_PING_TRIES        3U
#define MOTOR_TEST_ROUNDS            3U
#define MOTOR_TEST_START_DELAY_MS    1000U
#define MOTOR_TEST_ROUND_PAUSE_MS    3000U
#define MOTOR_TEST_GROUP_SPEED_RPM   50
#define MOTOR_TEST_GROUP_RUN_MS      500U

static volatile motor_status_t g_test_status[MOTOR_COUNT];
static volatile int32_t g_test_pos_before[MOTOR_COUNT];
static volatile int32_t g_test_pos_after[MOTOR_COUNT];
static volatile int16_t g_test_speed_run[MOTOR_COUNT];
static volatile int16_t g_test_speed_stop[MOTOR_COUNT];
static volatile int16_t g_test_group_speed[MOTOR_COUNT];
static volatile motor_status_t g_test_group_status[MOTOR_COUNT];
static volatile motor_run_state_t g_test_state_run[MOTOR_COUNT];
static volatile motor_run_state_t g_test_state_stop[MOTOR_COUNT];
static volatile bool g_test_enabled_on[MOTOR_COUNT];
static volatile bool g_test_enabled_off[MOTOR_COUNT];
static volatile bool g_test_inpos[MOTOR_COUNT];
static volatile bool g_test_stall[MOTOR_COUNT];
static volatile uint8_t g_test_device_error[MOTOR_COUNT];
static volatile uint8_t g_test_step;
static volatile uint8_t g_test_round;

static void Motor_TestSafeStop(motor_id_t id)
{
  (void)Motor_Stop(id);
  (void)Motor_Enable(id, false);
}

static motor_status_t Motor_TestPing(motor_id_t id)
{
  uint8_t tries = 0U;
  motor_status_t st = MOTOR_ERR_TIMEOUT;

  while (tries < MOTOR_TEST_PING_TRIES)
  {
    st = Motor_Ping(id);
    if (st == MOTOR_OK)
    {
      return MOTOR_OK;
    }

    tries++;
    HAL_Delay(100U);
  }

  return st;
}

static void Motor_TestOne(motor_id_t id)
{
  uint8_t idx = (uint8_t)id;
  motor_status_t st;
  int32_t pos;
  int16_t rpm;
  motor_run_state_t state;
  bool flag;

  g_test_status[idx] = MOTOR_OK;
  g_test_step = 1U;
  st = Motor_TestPing(id);
  if (st != MOTOR_OK)
  {
    g_test_status[idx] = st;
    return;
  }

  g_test_step = 2U;
  st = Motor_Enable(id, true);
  if (st != MOTOR_OK)
  {
    goto fail;
  }

  g_test_step = 3U;
  st = Motor_ReadEnabled(id, &flag);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_enabled_on[idx] = flag;
  if (!flag)
  {
    st = MOTOR_ERR_STATE;
    goto fail;
  }

  g_test_step = 4U;
  st = Motor_ReadPosition(id, &pos);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_pos_before[idx] = pos;

  g_test_step = 5U;
  st = Motor_ReadInPosition(id, &flag);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_inpos[idx] = flag;

  g_test_step = 6U;
  st = Motor_ReadStall(id, &flag);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_stall[idx] = flag;
  if (flag)
  {
    st = MOTOR_ERR_STALL;
    goto fail;
  }

  g_test_step = 7U;
  st = Motor_SetSpeed(id, MOTOR_TEST_SPEED_RPM,
                      (uint8_t)MOTOR_TEST_ACCEL);
  if (st != MOTOR_OK)
  {
    goto fail;
  }

  HAL_Delay(MOTOR_TEST_RUN_MS);

  g_test_step = 8U;
  st = Motor_ReadSpeed(id, &rpm);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_speed_run[idx] = rpm;
  if (rpm <= 0)
  {
    st = MOTOR_ERR_STATE;
    goto fail;
  }

  g_test_step = 9U;
  st = Motor_ReadPosition(id, &pos);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_pos_after[idx] = pos;
  if (g_test_pos_after[idx] == g_test_pos_before[idx])
  {
    st = MOTOR_ERR_STATE;
    goto fail;
  }

  g_test_step = 10U;
  st = Motor_ReadState(id, &state);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_state_run[idx] = state;

  g_test_step = 11U;
  st = Motor_ReadInPosition(id, &flag);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_inpos[idx] = flag;

  g_test_step = 12U;
  st = Motor_ReadStall(id, &flag);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_stall[idx] = flag;
  if (flag)
  {
    st = MOTOR_ERR_STALL;
    goto fail;
  }

  g_test_step = 13U;
  st = Motor_Stop(id);
  if (st != MOTOR_OK)
  {
    goto fail;
  }

  HAL_Delay(MOTOR_TEST_STOP_SETTLE_MS);

  g_test_step = 14U;
  st = Motor_ReadSpeed(id, &rpm);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_speed_stop[idx] = rpm;
  if ((rpm > 5) || (rpm < -5))
  {
    st = MOTOR_ERR_STATE;
    goto fail;
  }

  g_test_step = 15U;
  st = Motor_ReadState(id, &state);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_state_stop[idx] = state;

  g_test_step = 16U;
  st = Motor_Enable(id, false);
  if (st != MOTOR_OK)
  {
    goto fail;
  }

  g_test_step = 17U;
  st = Motor_ReadEnabled(id, &flag);
  if (st != MOTOR_OK)
  {
    goto fail;
  }
  g_test_enabled_off[idx] = flag;
  if (flag)
  {
    st = MOTOR_ERR_STATE;
    goto fail;
  }

  g_test_step = 18U;
  g_test_device_error[idx] = Motor_GetLastDeviceError(id);
  g_test_status[idx] = MOTOR_OK;
  g_test_step = 0U;
  return;

fail:
  g_test_status[idx] = st;
  Motor_TestSafeStop(id);
}

static void Motor_TestGroupStop(void)
{
  uint8_t i;
  int16_t rpm;
  motor_status_t st;

  for (i = 0U; i < (uint8_t)MOTOR_COUNT; i++)
  {
    g_test_group_status[i] = g_test_status[i];
    if (g_test_status[i] != MOTOR_OK)
    {
      continue;
    }

    g_test_step = 30U;
    st = Motor_Enable((motor_id_t)i, true);
    if (st != MOTOR_OK)
    {
      g_test_group_status[i] = st;
      continue;
    }

    g_test_step = 31U;
    st = Motor_SetSpeed((motor_id_t)i, MOTOR_TEST_GROUP_SPEED_RPM,
                        (uint8_t)MOTOR_TEST_ACCEL);
    if (st != MOTOR_OK)
    {
      g_test_group_status[i] = st;
    }
  }

  HAL_Delay(MOTOR_TEST_GROUP_RUN_MS);

  g_test_step = 32U;
  (void)Motor_StopAll();
  HAL_Delay(MOTOR_TEST_STOP_SETTLE_MS);

  for (i = 0U; i < (uint8_t)MOTOR_COUNT; i++)
  {
    g_test_step = 33U;
    if (Motor_ReadSpeed((motor_id_t)i, &rpm) == MOTOR_OK)
    {
      g_test_group_speed[i] = rpm;
    }

    g_test_step = 34U;
    (void)Motor_Enable((motor_id_t)i, false);
  }

  g_test_step = 0U;
}

void MotorTest_RunValidation(void)
{
  uint8_t round = 0U;
  uint8_t i;

  HAL_Delay(MOTOR_TEST_START_DELAY_MS);

  while (round < MOTOR_TEST_ROUNDS)
  {
    g_test_round = (uint8_t)(round + 1U);

    /* 每台独立测试；单台 Ping/命令失败只跳过该台，不阻塞其它电机。 */
    for (i = 0U; i < (uint8_t)MOTOR_COUNT; i++)
    {
      Motor_TestOne((motor_id_t)i);
    }

    Motor_TestGroupStop();
    HAL_Delay(MOTOR_TEST_ROUND_PAUSE_MS);
    round++;
  }

  (void)Motor_StopAll();
  for (i = 0U; i < (uint8_t)MOTOR_COUNT; i++)
  {
    (void)Motor_Enable((motor_id_t)i, false);
  }
}
