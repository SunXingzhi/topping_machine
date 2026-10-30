# Firmware tests

本目录保存 PD42S1 三轴 CAN 驱动固件的测试代码。生产固件默认**不编译**这些测试。

## 目录

| 路径 | 说明 |
|---|---|
| `embedded/motor_validation_test.c/.h` | 运行在 STM32 上的实机接口验证 |
| `host/test_motor.c` | 使用 fake HAL 的 PC 侧协议测试 |
| `host/fake_hal/` | PC 测试用的最小 HAL 头文件 |
| `host/run_host_tests.ps1` | Windows/MSVC 下编译并运行 host 测试 |

## 生产构建

默认 `MOTOR_BUILD_TESTS=OFF`，`main.c` 只做初始化并进入空主循环，不会自动使能或运动。

```powershell
cmake --preset Debug
cmake --build --preset Debug

cmake --preset Release
cmake --build --preset Release
```

## 实机验证构建

`Test` preset 会打开 `MOTOR_BUILD_TESTS=ON`，把测试代码加入固件，并定义 `MOTOR_TEST_ENABLE`：

```powershell
cmake --preset Test
cmake --build --preset Test
```

测试固件在 `build/Test/`，上电 1 秒后自动执行。

### 测试流程

- 逐台测试 M1/M2/M3；
- 每台 Ping 最多 3 次，失败只跳过该台，不阻塞其它电机；
- 每台：使能 → 读使能/位置/到位/堵转 → 100 RPM 运行 0.5 s → 读速度/位置/状态 → 停止 → 读停止后速度/状态 → 失能 → 读失能状态/原始结果码；
- 三台分别完成后，对测试成功的电机再做一次 50 RPM 群组运行，然后 `Motor_StopAll()`；
- 共 3 轮，每轮间隔 3 s；结束后停止并失能。

> 测试固件会自动运动。首次测试应空载或确认机械行程安全，并准备随时断电。

### 验证的接口

`Motor_Init`、`Motor_Ping`、`Motor_Enable`、`Motor_ReadEnabled`、`Motor_ReadPosition`、`Motor_ReadSpeed`、`Motor_ReadState`、`Motor_ReadInPosition`、`Motor_ReadStall`、`Motor_SetSpeed`、`Motor_Stop`、`Motor_StopAll`、`Motor_GetLastDeviceError`。

本次测试不覆盖：

`Motor_MoveRelative`、`Motor_MoveAbsolute`、`Motor_MoveRelativeRev/Deg`、`Motor_MoveAbsoluteRev/Deg`、`Motor_WaitInPosition`、`Motor_ZeroPosition`、四个单位换算函数。

### GDB 观察变量

```text
g_test_round
g_test_step
g_test_status[3]
g_test_group_status[3]
g_test_pos_before[3]
g_test_pos_after[3]
g_test_speed_run[3]
g_test_speed_stop[3]
g_test_group_speed[3]
g_test_state_run[3]
g_test_state_stop[3]
g_test_enabled_on[3]
g_test_enabled_off[3]
g_test_inpos[3]
g_test_stall[3]
g_test_device_error[3]
```

`g_test_step`：

```text
1  Ping
2  使能
3  读使能
4  读位置
5  读到到位
6  读堵转
7  设速度
8  读运行速度
9  读运动后位置
10 读运行状态
11 读到到位
12 读堵转
13 单轴停止
14 读停止后速度
15 读停止后状态
16 失能
17 读失能状态
18 读原始结果码
30 群组使能
31 群组设速度
32 StopAll
33 读群组速度
34 群组失能
0  当前阶段完成
```

## Host 协议测试

要求 Windows + Visual Studio C++ 工具链。

```powershell
powershell -ExecutionPolicy Bypass -File tests/host/run_host_tests.ps1
```

预期输出：

```text
ALL MOTOR HOST TESTS PASSED
```

Host 测试使用 fake HAL，不连接真实 CAN 总线；它验证帧组包、校验和、应答解析、重试和接口逻辑，不代表实机 CAN 时序已经通过。

## 测试代码与生产代码的边界

- `main.c` 只保留 `Motor_Init()` 和 `#ifdef MOTOR_TEST_ENABLE` 的测试入口；
- 测试宏、测试全局变量和测试函数全部在 `tests/embedded/`；
- 生产构建不会编译 `tests/`；
- 测试构建使用独立的 `build/Test/` 目录，避免覆盖生产固件。