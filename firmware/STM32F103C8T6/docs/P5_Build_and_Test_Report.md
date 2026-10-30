# P5 构建与测试报告

## PD42S1 三轴 CAN 电机驱动模块

| 项目 | 内容 |
|---|---|
| 报告编号 | FW-MOTOR-PD42S1-P5-001 |
| 日期 | 2026-10-27（2026-10-30 更新） |
| 状态 | P5 完成；P6 实机联调中 |
| 适用工程 | `F:\topping\topping_machine\firmware\STM32F103C8T6` |
| 设计文档 | `docs/PD42S1_Motor_Driver_Requirements_Design_v1.0.md` |
| 结论 | 构建、链接、协议逻辑测试通过；硬件通信与运动尚未验证 |

> **2026-10-30 更新**：测试代码已从 `main.c` 移至 `tests/embedded/`，生产 `main.c` 只保留初始化并进入空主循环；测试构建通过 `MOTOR_BUILD_TESTS=ON` / `Test` preset 打开。旧的“三轴连续旋转测试”内容作为历史记录保留，当前测试行为见 §6.3 和附录 A。

---

## 1. 验证范围

本阶段验证：

- 根 CMake 是否包含 `motor.c`；
- Debug / Release 是否能从干净状态完整编译、链接；
- HEX / BIN 是否生成且地址正确；
- 25 个公共 API 是否有定义；
- 协议帧、校验和、拆分/重组、错误处理、重试策略；
- 测试构建（`MOTOR_BUILD_TESTS=ON`）是否能编译链接；
- 生产构建是否不包含测试代码、不会自动运动。

本阶段不验证：

- 真实 CAN 总线波形和时序；
- 真实 PD42S1 应答；
- 地址 1/2/3 的实际配置；
- 终端电阻、24 V 供电、共地；
- 电机实际运动、闭环到位和堵转。

上述项目属于 P6 实机联调。

---

## 2. 工具链

| 工具 | 版本/路径 |
|---|---|
| CMake | 3.28.1 |
| Ninja | 1.11.1 |
| arm-none-eabi-gcc | 13.3.1（STM32CubeCLT 1.18.0） |
| 主机 C 编译器 | MSVC 19.16.27054（Visual Studio 2017） |
| 目标 MCU | STM32F103C8T6，64 KB Flash / 20 KB RAM |

---

## 3. 干净构建结果

### 3.1 命令

```powershell
cmake --preset Debug
cmake --build --preset Debug --clean-first

cmake --preset Release
cmake --build --preset Release --clean-first
```

### 3.2 结果

| 构建 | RAM | FLASH | 结果 |
|---|---:|---:|---|
| Debug | 1632 B / 20 KB（7.97%） | 7972 B / 64 KB（12.16%） | 通过 |
| Release | 1632 B / 20 KB（7.97%） | 4628 B / 64 KB（7.06%） | 通过 |
| Test | 1720 B / 20 KB（8.40%） | 14480 B / 64 KB（22.09%） | 通过 |

- 链接阶段同时执行 `objcopy`，生成 HEX / BIN。
- 构建输出未出现编译或链接错误。
- 2026-10-30 重新构建：生产 Debug/Release 不包含测试代码；`Test` preset 包含 `tests/embedded/motor_validation_test.c`，见附录 A。

### 3.3 Demo 编译验证

临时将 `MOTOR_DEMO_ENABLE` 改为 1 后重新编译：

| 项目 | 结果 |
|---|---|
| 编译/链接 | 通过 |
| RAM | 1648 B |
| FLASH | 15520 B |
| 源码状态 | P5 时点已恢复 `MOTOR_DEMO_ENABLE = 0`；2026-10-30 起测试移至 `tests/embedded/` |

Demo 打开版本能链接通过，说明 P3 的 25 个公共 API 均有定义；默认固件不包含自动运动逻辑。

---

## 4. 构建产物

### 4.1 文件

| 构建 | 文件 | 大小 |
|---|---|---:|
| Debug | `build/Debug/STM32F103C8T6.elf` | 709296 B |
| Debug | `build/Debug/STM32F103C8T6.hex` | 22508 B |
| Debug | `build/Debug/STM32F103C8T6.bin` | 7972 B |
| Release | `build/Release/STM32F103C8T6.elf` | 23776 B |
| Release | `build/Release/STM32F103C8T6.hex` | 13103 B |
| Release | `build/Release/STM32F103C8T6.bin` | 4628 B |

### 4.2 SHA256

| 文件 | SHA256 |
|---|---|
| `build/Release/STM32F103C8T6.hex` | `7F5D3BF02A4D89C65BD660F8DFD741152C4E28231AEF51F4D1FA236F3DFBB7CE` |
| `build/Release/STM32F103C8T6.bin` | `7E0F386F1740F6061962B171AB9A20BD1C247D717F5C1ED1DFB73CDD43944536` |
| `build/Debug/STM32F103C8T6.hex` | `5D5E79113ED485A7D50094E1839EC973AABFDE94A7F06969475C253C529BB761` |
| `build/Debug/STM32F103C8T6.bin` | `D3446D90F604C8BF83239F2BBDC19A7A116C1EF6E041886498A9FECEFE01AE4F` |

> 推荐烧录 Release 版本；需要调试符号时使用 Debug 版本。

---

## 5. 主机侧协议测试

### 5.1 方法

- 临时 fake HAL 头文件模拟 `CAN_HandleTypeDef`、发送邮箱、RX FIFO 和 HAL 函数。
- 使用 MSVC 在本机编译 `motor.c` + 测试用例。
- 测试文件不进入固件仓库，仅作为验证工具。

### 5.2 结果

```text
ALL P3 HOST TESTS PASSED
```

共 26 组测试，覆盖：

- Ping 成功、设备错误、超时重试；
- 使能/失能、使能状态；
- 正负位置、正负速度、运行状态、到位、堵转；
- 多帧应答重组；
- 绝对位置多帧发送；
- 相对运动不自动重试；
- 绝对目标越界、角度越界；
- 正负速度模式；
- 单轴停止、广播停止、设零；
- 到位等待成功；
- counts / 圈 / 度换算。

### 5.3 静态检查

`motor.c` 使用 ARM GCC 严格编译通过：

```text
-Wall -Wextra -Werror -pedantic
-Wmissing-prototypes -Wstrict-prototypes -Wshadow
-Wconversion -Wsign-conversion
```

目标文件尺寸（P3 记录）：

```text
text 7084, data 0, bss 15
```

---

## 6. 烧录说明

### 6.1 烧录前

- [ ] ST-Link 的 SWDIO / SWCLK / GND / 3.3V 正确连接。
- [ ] 目标板 `BOOT0 = 0`，从 Flash 启动。
- [ ] 24 V 电机电源先断开，只给 MCU 逻辑供电。
- [ ] 确认烧录文件与目标板一致（STM32F103C8T6）。

### 6.2 STM32CubeProgrammer

1. 打开 STM32CubeProgrammer，选择 ST-Link。
2. 连接方式选 SWD；必要时使用 “Connect Under Reset”。
3. 加载 `build/Release/STM32F103C8T6.hex`。
4. 起始地址使用 HEX 内嵌地址；若使用 BIN，起始地址填 `0x08000000`。
5. 执行下载并校验。
6. 复位目标板。

> 当前 CMake 已同时生成 ELF / HEX / BIN；ELF 可用于调试，HEX/BIN 可用于量产烧录。

### 6.3 烧录后

**生产固件（默认构建）**烧录后只执行：

- 系统时钟、GPIO、CAN1 初始化；
- `Motor_Init()` 配置过滤器并启动 CAN；
- 进入空主循环；
- **不使能、不运动、不自动测试**。

**测试固件（`Test` preset / `MOTOR_BUILD_TESTS=ON`）**烧录后执行：

- 上电等待 1 秒；
- 逐台测试 M1/M2/M3；每台 Ping 最多 3 次，单台失败只跳过该台；
- 每台执行使能、读状态/位置、100 RPM 短时旋转、停止、失能；
- 三台分别完成后，对测试成功的电机做 50 RPM 群组运行，再调用 `Motor_StopAll()`；
- 共 3 轮，结束后停止并失能。

> 测试固件会自动使能并短时运动；测试时必须确保机械安全。生产固件不含测试代码。

---

## 7. 上电前硬件检查

- [ ] 三台驱动器地址分别设为 1、2、3。
- [ ] 三台驱动器 CAN 速率均为 500 kbps。
- [ ] 三台驱动器 MODBUS 均为失能（使用正点原子自定义协议）。
- [ ] 总线只保留两个 120 Ω 终端；断电测 `CAN_H-CAN_L` 约 60 Ω。
- [ ] STM32、SN65HVD230、三台驱动器共地。
- [ ] 24 V 电源容量足够，极性正确。
- [ ] 电机 A+ / A− / B− / B+ 接线正确。
- [ ] 首次上电不带机械负载，机构处于安全位置。
- [ ] 准备急停/断电手段。

---

## 8. P6 实机联调建议顺序

1. 只给 MCU 和收发器上电，确认无异常发热。
2. 烧录默认固件，确认不自动动作。
3. 用调试器或临时 Demo 执行 `Motor_Ping(1/2/3)`。
4. 单轴小角度、低速相对运动（例如 10°、100 RPM）。
5. 单轴绝对 90°，读取位置和到位标志。
6. 速度模式低速运行，执行停止。
7. 测试到位等待、堵转标志、故障状态。
8. 三轴分别测试后，再做三轴顺序控制。
9. 记录 H-01~H-15 结果。
10. 通过后再接入整机机械负载。

---

## 9. 验收状态

| 项目 | 状态 |
|---|---|
| 需求与设计评审 | 已确认 |
| 25 个公共 API 定义 | 已实现 |
| ARM 严格编译 | 通过 |
| Debug 整机编译 | 通过 |
| Release 整机编译 | 通过 |
| HEX/BIN 生成 | 通过 |
| 主机侧协议测试 | 26 组通过 |
| 实机 CAN 通信 | 待 P6 |
| 实机电机运动/闭环 | 待 P6 |
| 终端电阻/供电/地址 | 待 P6 |

---

## 10. 遗留风险

| 编号 | 风险 | 处理 |
|---|---|---|
| R-03 | 厂商例程 0xF2/0xF3 为 14 字节，手册为 13 字节 | 首次位置指令用单轴低速验证；必要时兼容两种长度 |
| R-04 | 驱动器回传 CAN ID 未确认 | 软件按 addr/code 匹配，不依赖唯一 ID |
| R-05 | 绝对零点不等于机械零点 | 装配对齐或上层加偏移 |
| R-06 | ±51200 不是机械限位 | 增加硬限位/限位开关，或上层缩小范围 |
| R-10 | 无硬件，未实机验证 | P6 按 H-01~H-15 执行 |

---

## 11. 结论

P5 的构建、链接、HEX/BIN 生成和主机侧协议逻辑测试全部通过。固件已具备进入实机联调的条件。

由于尚未接线，P5 **不能**证明真实 CAN 通信、驱动器应答、地址配置、终端电阻、供电和机械运动正常；这些必须在 P6 按检查清单逐项验证。

---

## 附录 A：2026-10-30 接口验证测试（`tests/embedded`）

为保持生产 `main.c` 干净，原 `main.c` 中的测试代码已移至：

- `tests/embedded/motor_validation_test.c`
- `tests/embedded/motor_validation_test.h`

测试构建通过 `MOTOR_BUILD_TESTS=ON` 或 `Test` preset 打开；生产构建默认不编译测试。

| 项目 | 内容 |
|---|---|
| 入口 | `main()` 在 `Motor_Init()` 后按 `MOTOR_TEST_ENABLE` 调用 `MotorTest_RunValidation()` |
| 测试方式 | 逐台测试 M1/M2/M3；每台 Ping 最多 3 次 |
| 单台失败 | 记录 `g_test_status[]`，停止/失能该台，继续下一台，不阻塞其它电机 |
| 速度 | 单台 100 RPM、500 ms；群组 50 RPM、500 ms |
| 停止 | 单台 `Motor_Stop()`；群组 `Motor_StopAll()` |
| 轮次 | 3 轮，每轮间隔 3 秒；结束后停止并失能 |
| 覆盖接口 | `Motor_Init`、`Motor_Ping`、`Motor_Enable`、`Motor_ReadEnabled`、`Motor_ReadPosition`、`Motor_ReadSpeed`、`Motor_ReadState`、`Motor_ReadInPosition`、`Motor_ReadStall`、`Motor_SetSpeed`、`Motor_Stop`、`Motor_StopAll`、`Motor_GetLastDeviceError` |
| GDB 观察 | `g_test_round`、`g_test_step`、`g_test_status[]`、`g_test_group_status[]`、`g_test_speed_run[]`、`g_test_speed_stop[]`、`g_test_group_speed[]`、`g_test_enabled_on/off[]` 等 |
| Host 测试 | `tests/host/test_motor.c` + fake HAL；Windows/MSVC 下运行 `tests/host/run_host_tests.ps1`，26 组通过 |

预期现象：上电 1 秒后，M1/M2/M3 依次短时旋转并停止；随后三台一起短时旋转并广播停止。若某台失败，测试继续下一台，不会卡在地址 1。

> 旧的“三轴 100 RPM 连续旋转测试”已被本测试替代，不再位于 `main.c`。之前的 `build/MotorTest/` 测试固件已废弃并删除。

**报告结束**