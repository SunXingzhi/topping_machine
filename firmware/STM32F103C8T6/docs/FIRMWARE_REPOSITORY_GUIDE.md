# 固件仓库整理与提交方案

## PD42S1 三轴 CAN 电机驱动固件

| 项目 | 内容 |
|---|---|
| 仓库根目录 | `F:\topping\topping_machine` |
| 固件目录 | `firmware/STM32F103C8T6` |
| 当前分支 | `main`（与 `origin/main` 同步） |
| 当前状态 | 已整理，随本次提交 |
| 目标 | 整理测试代码、保持 `main.c` 干净、补齐 `.gitignore`，然后按开源仓库规范提交 |

> **执行状态（2026-10-30）**：测试已移至 `tests/`，生产 `main.c` 已清理，固件 `.gitignore` 已新增，根 `.gitignore` 已恢复。Debug/Release/Test 构建和 host 测试均通过。本文第 1 节保留整理前快照，最终提交内容见第 7、8 节。

---

## 1. 当前仓库状态

### 1.1 已修改但未暂存

| 文件 | 说明 |
|---|---|
| `.gitignore` | 当前被追加了 43 行“具体到某个 build 文件”的固件忽略项 |
| `firmware/STM32F103C8T6/CMakeLists.txt` | 已加入 `motor.c` 和 HEX/BIN 生成 |
| `firmware/STM32F103C8T6/Core/Src/main.c` | 当前包含整套接口验证测试，约 515 行 |

### 1.2 未跟踪的新文件

| 路径 | 建议 |
|---|---|
| `Core/Inc/motor.h` | 提交 |
| `Core/Src/motor.c` | 提交 |
| `docs/` | 提交（设计说明、P5 报告、本方案） |
| `tests/`（待创建） | 提交 |
| `motor.obj` | 不提交，删除或忽略 |
| `test_motor.obj` | 不提交，删除或忽略 |
| `电机参考资料/`（约 103 MB） | 不提交，加入固件 `.gitignore` |
| `电机参考资料.7z`（约 65 MB） | 不提交，加入固件 `.gitignore` |

### 1.3 当前 Git 跟踪状态

- `build/` 和 `.idea/` 当前**没有**被 Git 跟踪，只是本地文件。
- 因为没有被跟踪，不需要执行 `git rm --cached`；只需配置通用 `.gitignore`，避免以后误加入。
- `.mxproject` 当前已被跟踪，文件很小、使用相对路径；建议保留，不纳入本次删除范围。
- `STM32F103C8T6.ioc`、`CMakeLists.txt`、`cmake/stm32cubemx/CMakeLists.txt`、`Drivers/` 保留跟踪。

---

## 2. 目标目录结构

```text
firmware/STM32F103C8T6/
├── .gitignore                       # 已新增：固件局部忽略规则
├── CMakeLists.txt                   # 已修改：驱动源文件 + 可选测试
├── CMakePresets.json                # 已修改：新增 Test preset
├── Core/
│   ├── Inc/
│   │   ├── motor.h                  # 新增：公共驱动接口
│   │   └── ...
│   └── Src/
│       ├── motor.c                  # 新增：CAN 传输 + 设备服务
│       ├── main.c                   # 已修改：只保留生产初始化和空主循环
│       └── ...
├── tests/
│   ├── README.md                    # 测试说明、构建命令、GDB 观察变量
│   ├── embedded/
│   │   ├── motor_validation_test.c  # 从 main.c 抽出的实机验证测试
│   │   └── motor_validation_test.h
│   └── host/                        # 已归档：fake HAL 协议测试
│       ├── fake_hal/
│       │   ├── main.h
│       │   └── can.h
│       ├── test_motor.c
│       └── run_host_tests.ps1
└── docs/
    ├── PD42S1_Motor_Driver_Requirements_Design_v1.0.md
    ├── P5_Build_and_Test_Report.md
    └── FIRMWARE_REPOSITORY_GUIDE.md
```

---

## 3. `main.c` 清理方案

### 3.1 生产版 `main.c`

只保留：

1. `HAL_Init()`
2. `SystemClock_Config()`
3. `MX_GPIO_Init()`
4. `MX_CAN_Init()`
5. `Motor_Init()`
6. 空主循环

不再包含：

- 测试宏；
- 测试全局变量；
- `Motor_TestOne()` / `Motor_TestPing()` / `Motor_TestGroupStop()`；
- GDB 测试数组；
- 自动运动逻辑。

### 3.2 测试入口

测试代码移到 `tests/embedded/motor_validation_test.c`，对外只暴露：

```c
void MotorTest_RunValidation(void);
```

`main.c` 只在测试构建时保留一个很短的入口：

```c
#ifdef MOTOR_TEST_ENABLE
  MotorTest_RunValidation();
#endif
```

这样：

- 默认生产固件**不含测试代码、不自动运动**；
- 需要测试时通过 CMake 打开 `MOTOR_BUILD_TESTS=ON`；
- `main.c` 保持干净，测试逻辑独立维护。

---

## 4. CMake 方案

### 4.1 驱动源文件

保留当前：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/motor.c
)
```

### 4.2 可选测试

新增：

```cmake
option(MOTOR_BUILD_TESTS "Build on-target motor validation test" OFF)

if(MOTOR_BUILD_TESTS)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/embedded/motor_validation_test.c
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/embedded
    )
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        MOTOR_TEST_ENABLE=1
    )
endif()
```

### 4.3 Test preset

已在 `CMakePresets.json` 增加：

- configure preset `Test`，继承 `Debug`；
- `MOTOR_BUILD_TESTS=ON`；
- `binaryDir` 使用 `build/Test`；
- build preset `Test`。

这样生产构建和测试构建互不覆盖。

---

## 5. `tests/` 目录内容

### 5.1 `tests/embedded/`

存放当前 `main.c` 里的实机验证测试，包括：

- 逐台测试 M1/M2/M3，单台失败不阻塞其它电机；
- 读取接口验证；
- 单轴停止、失能、使能状态；
- 广播停止；
- 原始结果码；
- GDB 观察变量。

### 5.2 `tests/host/`（已归档）

把之前的 fake HAL 协议测试也归档到仓库：

- `fake_hal/main.h`、`fake_hal/can.h`；
- `test_motor.c`；
- `run_host_tests.ps1`。

特点：

- 不需要真实 STM32/驱动器；
- 验证帧组包、校验和、应答解析、重试等协议逻辑；
- 目前依赖 Windows + MSVC，属于开发者测试，不参与固件默认构建。

如果不想引入 Windows/MSVC 脚本，也可以只提交 `tests/embedded/`，host 测试继续作为临时工具。

### 5.3 `tests/README.md`

至少说明：

- 生产构建：`cmake --preset Debug` / `Release`
- 测试构建：`cmake --preset Test`（或 `-DMOTOR_BUILD_TESTS=ON`）
- 测试现象和 GDB 变量
- 安全警告：测试构建上电会自动使能/运动
- host 测试的运行方法（若纳入）

---

## 6. `.gitignore` 方案

### 6.1 已新增固件局部 `.gitignore`

位置：`firmware/STM32F103C8T6/.gitignore`

建议内容：

```gitignore
# Build output
build/
cmake-build-*/
out/
*.elf
*.hex
*.bin
*.map
*.obj
*.o
*.a
*.lib
*.d
*.su
*.lst
*.axf
*.s19
*.exe
*.pdb
*.ilk
*.exp

# CMake / IDE
CMakeUserPresets.json
.idea/
.vscode/
.vs/
.clangd/
.cache/
compile_commands.json

# Test / debug output
Testing/
*.gcda
*.gcno
*.gcov
*.log

# OS
.DS_Store
Thumbs.db
desktop.ini

# Local vendor reference material
电机参考资料/
电机参考资料.7z

# Python host test cache
__pycache__/
*.pyc
```

### 6.2 根 `.gitignore` 处理

整理前，根 `.gitignore` 被追加了 43 行“具体文件级”固件忽略项，例如：

```text
firmware/STM32F103C8T6/build/Release/STM32F103C8T6.hex
firmware/STM32F103C8T6/.idea/editor.xml
...
```

这种写法维护成本高，新增一个 build 文件就会重新出现在 `git status`。

已执行：

1. 恢复根 `.gitignore` 到 HEAD 版本；
2. 使用 `firmware/STM32F103C8T6/.gitignore` 统一管理固件构建/IDE/本地参考资料；
3. 保留根 `.gitignore` 里已有的项目级规则。

### 6.3 不忽略的内容

以下文件应继续跟踪：

- `STM32F103C8T6.ioc`
- `.mxproject`
- `CMakeLists.txt`
- `CMakePresets.json`
- `cmake/stm32cubemx/CMakeLists.txt`
- `Drivers/`（含 LICENSE）
- `startup_stm32f103xb.s`
- `STM32F103xx_FLASH.ld`
- `Core/Inc/`、`Core/Src/` 源码
- `tests/` 源码
- `docs/`

---

## 7. 本次提交范围

### 7.1 应提交

| 路径 | 动作 |
|---|---|
| `firmware/STM32F103C8T6/.gitignore` | 新增 |
| `Core/Inc/motor.h` | 新增 |
| `Core/Src/motor.c` | 新增 |
| `Core/Src/main.c` | 修改：移除测试代码 |
| `CMakeLists.txt` | 修改：驱动 + 可选测试 |
| `CMakePresets.json` | 修改：Test preset |
| `tests/README.md` | 新增 |
| `tests/embedded/motor_validation_test.c/.h` | 新增 |
| `tests/host/...` | 新增 |
| `docs/PD42S1_Motor_Driver_Requirements_Design_v1.0.md` | 新增 |
| `docs/P5_Build_and_Test_Report.md` | 新增/更新 |
| `docs/FIRMWARE_REPOSITORY_GUIDE.md` | 新增 |

### 7.2 不提交

| 路径 | 原因 |
|---|---|
| `build/` | CMake/Ninja 构建产物 |
| `.idea/` | IDE 本地配置 |
| `motor.obj`、`test_motor.obj` | 本地编译对象 |
| `电机参考资料/` | 约 103 MB 厂商参考资料，且不宜作为固件源码提交 |
| `电机参考资料.7z` | 约 65 MB 压缩包 |
| `*.elf`、`*.hex`、`*.bin`、`*.map` | 发布产物，不进入源码仓库 |
| `CMakeUserPresets.json` | 本地用户配置 |

### 7.3 `.mxproject` 处理

当前 `.mxproject` 已被跟踪，内容为相对路径的 CubeMX 生成文件列表，体积很小。建议**保留**，不纳入删除范围。

如果要彻底按“生成物不入库”处理，也可以单独提交一次“untrack `.mxproject`”，但这会改变现有 CubeMX 工作流，建议本次不动。

---

## 8. Git 提交方案

### 8.1 分支

- 分支保持 `main`；
- 不创建新分支；
- 提交后 push 到 `origin/main`（用户已明确要求）。

### 8.2 提交前状态

当前没有暂存内容。提交前应确保：

```text
git status --short
```

只出现准备提交的源码、测试、文档和 `.gitignore`，不出现 build/IDE/obj/参考资料。

### 8.3 建议提交信息

推荐一次性提交当前固件功能：

```text
feat(firmware): 增加 PD42S1 三轴 CAN 电机驱动

- 新增 motor.h/motor.c，实现自定义 CAN 协议、三轴控制与状态读取
- 测试代码移至 tests/，生产 main.c 仅保留初始化
- CMake 增加可选测试构建与 Test preset
- 增加固件 .gitignore，排除构建产物、IDE 配置和本地参考资料
- 补充需求设计、测试报告和仓库说明文档
```

如果希望更细粒度，也可以拆成两次提交：

1. `chore(firmware): 配置固件构建产物忽略规则`
2. `feat(firmware): 增加 PD42S1 三轴 CAN 电机驱动与验证测试`

---

## 9. 提交前验证

按顺序执行：

1. 移动/抽取测试代码，清理 `main.c`。
2. 更新 `.gitignore`。
3. 生产构建：

   ```text
   cmake --preset Debug
   cmake --build --preset Debug --clean-first
   cmake --preset Release
   cmake --build --preset Release --clean-first
   ```

4. 测试构建（`Test` preset）：

   ```text
   cmake --preset Test
   cmake --build --preset Test --clean-first
   ```

5. 语法/静态检查：

   ```text
   arm-none-eabi-gcc -std=c11 -Wall -Wextra -Werror -pedantic -fsyntax-only ...
   ```

6. `git diff --check`，确认没有空白错误。
7. `git status --short`，确认没有 build/IDE/obj/参考资料。
8. `git diff --cached --stat`，确认暂存内容符合预期。
9. 提交后 `git status` 应为 clean（本地 build/.idea/参考资料被忽略）。

> 本次不包含实机联调；仅保证源码结构、构建和已有测试记录。

---

## 10. 已确认事项

- [x] 使用 `tests/` 作为测试目录，内含 `embedded/` 和 `host/`。
- [x] 把原 `main.c` 的实机验证测试移到 `tests/embedded/`。
- [x] 生产 `main.c` 只保留初始化，不再自动测试/运动。
- [x] 增加 `MOTOR_BUILD_TESTS` 选项和 `Test` preset，默认 OFF。
- [x] 把 `tests/host/` 的 fake HAL 测试一并纳入仓库。
- [x] 使用固件局部 `.gitignore`，恢复根 `.gitignore` 的具体文件列表。
- [x] 不提交 `电机参考资料/` 和 `电机参考资料.7z`。
- [x] 保留 `.mxproject` 跟踪。
- [x] 分支保持 `main`，提交后 push 到 `origin/main`。
- [x] 使用一次 `feat(firmware)` 提交。

---

## 11. 执行结果

1. 测试已抽取到 `tests/embedded/`，`main.c` 已清理。
2. `MOTOR_BUILD_TESTS` 选项和 `Test` preset 已加入。
3. 固件 `.gitignore` 已新增，根 `.gitignore` 已恢复。
4. `tests/host/` 已归档，host 测试通过。
5. 构建结果：

   | 构建 | RAM | FLASH | 结果 |
   |---|---:|---:|---|
   | Debug | 1632 B | 7972 B | 通过 |
   | Release | 1632 B | 4628 B | 通过 |
   | Test | 1720 B | 14480 B | 通过 |

6. Host 测试输出：`ALL MOTOR HOST TESTS PASSED`。
7. 提交与 push 在本次操作中执行；提交哈希见 `git log`。

---

**文档结束**