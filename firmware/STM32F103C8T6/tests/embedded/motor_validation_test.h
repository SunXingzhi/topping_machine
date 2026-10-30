/**
  ******************************************************************************
  * @file    motor_validation_test.h
  * @brief   PD42S1 三轴接口实机验证测试入口
  ******************************************************************************
  */

#ifndef MOTOR_VALIDATION_TEST_H
#define MOTOR_VALIDATION_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运行三轴接口验证测试。
 *
 * 仅在 MOTOR_BUILD_TESTS=ON 的测试构建中调用。
 * 测试会逐台 Ping、使能、短时旋转、读取状态、停止和失能；
 * 单台失败不会阻塞其它电机。
 */
void MotorTest_RunValidation(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_VALIDATION_TEST_H */
