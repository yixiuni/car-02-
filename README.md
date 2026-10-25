# STM32 小车 H题课程复现

本工程使用 WHEELTEC C10B 主板上的 STM32F103RCT6，复现 2024 年 H 题的四项行驶任务。硬件采用两台 MG513X 电机、两只 65 mm 橡胶轮、幻尔八路灰度和 MPU6050。保留 STM32 是本次课程复现的明确选择；原赛题要求 TI MSPM0。

当前开发版本为 **T321**：已完成软件测试、烧录与回读核对，**尚未完成实车验证**。CROSS 直线/弯道/接线目标速度为 1000/650/370 mm/s，NORMAL 为 800/460/370 mm/s；支持 NORMAL/CROSS 和 1/4 圈选择。下载 [T321 BIN](firmware/T321/stm32-car.bin) / [HEX](firmware/T321/stm32-car.hex)，构建与校验信息见 [manifest](firmware/T321/manifest.json)。

此前 T319 两次记录为 29.975 / 29.730 秒，但随后多次失败；T320 在第二圈 B 点失败。因此目前不能宣称已稳定实现四圈 30 秒。详见 [T321 修改与验证](docs/T321沿弧位置约束与验证.md)、[T319 失败情况](docs/T319弯道位置校正与验证.md)、[T320 失败记录](docs/data/t320-cross-four-run-01.json)。

已发布基线 **DUL5** 支持普通/交叉两条路线、1/4 圈选择和原配 OLED 显示。两种路线目标速度均为 **370 mm/s**，过点不减速，最后一圈回 A 主动刹停。已实测：普通单圈 **11.739 秒**、交叉四圈 **51.695 秒**，各成功一轮、无故障。

基于 [yixiuni/stm32-car](https://github.com/yixiuni/stm32-car) 继续开发。可直接下载本车已验证的 [BIN](firmware/DUL5/stm32-car.bin) / [HEX](firmware/DUL5/stm32-car.hex)，镜像信息见 [manifest](firmware/DUL5/manifest.json)。本车标定参数已写入；另一台车需要重新标定。默认普通 Debug/Release 构建仍保留标定锁，DUL5 对应 `--dual-mode`，当前 T321 须使用 `--time-trial-arc-phase` 构建。

## 接线

| 功能 | 接口及引脚 | 配置 |
| --- | --- | --- |
| 电机 A | CN3，PA6 / PA7 | TIM3 CH1 / CH2，板载 AT8236 |
| 电机 B | CN2，PB0 / PB1 | TIM3 CH3 / CH4，板载 AT8236 |
| 编码器 A | CN3 的 PB6 / PB7 | TIM4，AB 双相 x4 计数 |
| 编码器 B | CN2 的 PC6 / PC7 | TIM8，AB 双相 x4 计数 |
| 八路灰度 | CN6：PC10→模块 RX，PC11←模块 TX | UART4，115200，8N1，5V 供电、共地 |
| MPU6050 | H6：PB14 SCL，PB15 SDA | 软件 I²C，地址 0x68，信号上拉到 3.3V |
| USER | PA0 | 高电平按下 |
| 使能开关 | PC13 | 低电平允许运动 |
| 指示灯 | PC2 / PC3 | 运行及过点提示 / 故障，低电平亮 |
| 蜂鸣器 | PA15 | 默认高电平有效，需确认实物极性 |
| OLED | PC14 时钟、PB5 数据、PB3 DC、PB4 RST | 原配 128×64 软件 SPI 屏 |
| SWD | PA13 / PA14 | 保留；关闭 JTAG 以释放 PB3/PB4/PA15 |

MCU 使用 8 MHz 外部晶振，72 MHz 系统时钟；PWM 为 20 kHz。芯片宏、启动文件和链接脚本对应 RCT6 的 256 KB Flash、48 KB RAM。[针序及原理图依据](docs/接口核对.md)。

## 使用当前版本

| 停车时操作 USER | 作用 |
| --- | --- |
| 长按约 1 秒后松开 | 切换 NORMAL（普通）/ CROSS（交叉） |
| 快速双击 | 当前路线的圈数在 1 ↔ 4 之间切换 |
| 短按一次 | 等待双击判定，静止校准陀螺仪后启动 |
| 运行中按一下 | 立即中止，松开不自动重启 |

上电默认普通 1 圈，交叉预设 4 圈；两条路线分别记忆本次通电期间的圈数，重新上电恢复默认。OLED 显示路线、`LAPS` 设定圈数、`DONE` 完成圈数、运行状态与用时。停车后可重新选择并再次启动。

每次运行前把**左右驱动轮轴线中点对准 A、车头朝 B**，前排探头在白底；交叉模式会自动转向 C。可拔电脑线运行。要读取上一轮数据时，保持电池供电、接回电脑，先读数据再复位或开始新一轮；新一轮会覆盖上一轮轨迹。

首次使用另一台车，按 [H题实现与标定](docs/H题实现与标定.md) 测量与校准。硬件参数集中在 `Core/Inc/car_config.h`，灰度阈值在 `Core/Inc/gray_calibration.h`。当前使用八路模拟灰度的软件标定，无需按传感器学习键。

## 实车验证

| 固件 | 模式 | 速度 | 用时 | 结果 |
| --- | --- | --- | --- | --- |
| DUL5 | 普通 1 圈 | 370 mm/s | 11.739 s | 5 次过点，无故障 |
| DUL5 | 交叉 4 圈 | 370 mm/s | 51.695 s | 17 次过点，无故障 |

交叉四圈逐控制更新统计：左右 PWM 峰值 667/666‰，上限 700‰，没有触顶；最大活动控制间隔 8 ms。这是各一次成功运行记录，不是长期可靠性或硬件最高速度证明。详见 [软件验证记录](docs/软件验证记录.md)、[普通单圈数据](docs/data/dul5-oval-success-2026-10-25.json)、[交叉四圈数据](docs/data/dul5-cross-four-success-2026-10-25.json)。

## 四项任务

场地仅有两条半径 400 mm、宽约 18 mm 的黑色半圆。AB 和 CD 长 1000 mm，AC 和 BD 长约 1280.6 mm；不得添加横向起终点线。所有任务起步航向均为 0°（朝 B）；任务 3/4 自动转入右下方的 A→C 方向，并计入起步弯行产生的位移。

| 任务 | 路径 | 完成条件 |
| --- | --- | --- |
| 1 | A→B | B 停车并提示，15 秒内 |
| 2 | A→B→右弧C→D→左弧A | 一圈后 A 停车，30 秒内 |
| 3 | A→C→右弧B→D→左弧A | 一圈后 A 停车，40 秒内 |
| 4 | 重复任务3 | 四圈后 A 停车；固件设 160 秒保护超时，这不是题目给定时限 |

开始 A、每个中间端点及最终停车点均产生一次声光事件。中间端点不等待蜂鸣完成，斜线交叉处不计点。最终点等轮速下降并检查投影覆盖后提示。

## 控制与故障

`car_control.c` 使用直线/入弧/循弧分段状态机，最终进入 `BRAKING → DONE`。当前高速双模式确认黑线后直接循弧，旧独立诊断保留 `CAPTURE`。直线允许全白，位置和航向闭环控制方向。弧线使用灰度偏差和曲率前馈，轮速 PI 控制两轮。所有驱动命令均为前进或零，不使用反转原地掉头。

进入弧线要求接近预期端点、估计车体覆盖该点并收到连续三帧新黑线。DUL5 在这一窗口内第一帧合法黑线就预先转向，三帧确认后才触发过点/切段；白色或全黑歧义帧取消预转向。离开弧线要求足够的行驶距离和转角、航向及端点位置匹配，以及连续白底边沿。合格的弧线末端用于校正位置，并按前置探头的圆弧几何关系缓慢修正航向；这一近似依赖平稳循迹，需要实车检查。

灰度通信故障、IMU 失效、编码器异常、控制周期超时、堵转、持续丢线或错过端点均停止输出并锁定故障。`0xFF` 全黑视为歧义图案，不作为终点或计圈标记。默认 `CAR_GRAY_USE_ANALOG=1`：初始化手动模式后查询 `0x02`，校验 21 字节模拟灰度帧，再按每路阈值和滞回判黑白。`gray_raw_status` 是软件合成的兼容状态字节，真实八路模拟值见 `hiwonder_gray_analog_values`。旧数字查询仅在关闭模拟模式时使用，它没有校验和。

主循环约每 5 ms 采样和控制一次，控制算法按实际间隔计算。灰度约每 10 ms 以上产生新帧，灰度微分和连续帧计数只在新帧更新，避免重复计算旧数据。传感器数据通过 SWD 查看 `app_diagnostics`。

## 构建与测试

需要 Python 3、本机 C 编译器（运行测试），以及 ARM GCC（构建固件）。

```sh
python3 tests/run_tests.py
# Clang/GCC 支持时启用内存及未定义行为检查
python3 tests/run_tests.py --sanitize

# 当前 T321 试验版，ARM GCC 已在 PATH
python3 tools/build_firmware.py --time-trial-arc-phase
python3 tests/run_tests.py --trial-profile time-trial-arc-phase --sanitize

# 历史 DUL5 基线
python3 tools/build_firmware.py --dual-mode

# 或明确指定工具链 bin 目录
python3 tools/build_firmware.py --time-trial-arc-phase --toolchain /path/to/arm-toolchain/bin

# 保留标定锁的常规版本（不是当前实车运行版本）
python3 tools/build_firmware.py --config Release
```

T321 输出到 `build-local/ArcPhaseDebug/`，DUL5 输出到 `build-local/DualModeDebug/`，包含 ELF、HEX、BIN；常规 Release 输出到 `build-local/Release/`。`firmware/T321/` 保存本次已烧录但待实跑的镜像；`firmware/DUL5/` 保留历史基线。构建缓存和本机工具链不上传。

已有 CMake/Ninja 时也可用全新构建目录，避免复用旧 Windows 构建缓存：

```sh
cmake -S . -B build-local/cmake-t321 -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug -DCAR_TIME_TRIAL_ARC_PHASE=ON
cmake --build build-local/cmake-t321
```

本次通过 Python 构建入口实际验证 Debug/Release；该入口读取 CMake 中的源码列表。CubeMX 重新生成工程不在本次验证范围内；当前 TIM3、TIM4、TIM8、UART4 和软件 I²C 由自有驱动初始化。

## 回收 DUL5 轨迹

历史 DUL5 镜像魔数 `0x44554C35`。使用 OpenOCD/ST-Link 连接已完成运行且保持电池供电的小车；以下命令不复位、不启动电机，先检查版本和零输出后保存到 `build-local/captures/`：

```sh
openocd -f interface/stlink.cfg -c "transport select swd" \
  -f target/stm32f1x.cfg \
  -c 'stm32f1x.cpu configure -event examine-end {}' \
  -c "adapter speed 950" -c "gdb port disabled" \
  -c "tcl port disabled" -c "telnet port disabled" \
  -c init -f tools/read_dul5.tcl -c shutdown
```

脚本使用固定文件名，再次读取前自行另存旧记录。重编译后若 RAM 符号布局变化，需要按 ELF 核对地址；该脚本对应本仓库附带的 DUL5 镜像。

## 代码职责

| 模块 | 职责 |
| --- | --- |
| `board_c10b` | PWM、按键、使能、灯、蜂鸣器、异常关断 |
| `encoder` | TIM4/TIM8 计数、回绕、毫米及轮速换算 |
| `mpu6050` / `imu_filter` | 寄存器配置和采集 / 零偏校准与航向积分 |
| `hiwonder_gray` / `gray_analog` / `gray_protocol` | UART 接收、模拟帧校验与阈值分类 / 旧数字协议及顺序转换 |
| `gray_learn` / `gray_calibration.h` | 专用静止采样固件 / 实测软件阈值 |
| `motion_control` | 参数校验、轮速 PI、加速限制、前进命令限幅 |
| `car_control` | 四项任务、导航、过点、计圈和故障状态 |
| `course_mode` / `task1_test` | 双模式按键、圈数选择、每轮启动与中止控制 |
| `oled` / `oled_ui` | 分批软件 SPI 刷新、模式与运行状态显示 |
| `app` | 硬件采样、调度、非阻塞提示及诊断 |
| `tests/sim_course.c` | 独立差速运动、轮速响应与赛道灰度模型 |
## 30 秒目标试验进展

| 版本 | 实跑结果 | 当前结论 |
| --- | --- | --- |
| T317 | 交叉四圈 35.667 s | 一次留存成功记录 |
| T319 | 两次 29.975 / 29.730 s，随后约五六次失败 | 不稳定，不能按两次成功验收 |
| T320 | 第二圈 B，11.872 s，fault 7 | 已保存失败现场 |
| T321 | 尚未实跑 | 增加有边界的沿弧位置约束，待验证 |

T321 软件回归通过；扩展模型 224/225，唯一失败是原有 NORMAL 场景，CROSS/C/D 扩展各 45/45。额外弧线里程比例误差模型 63/63；这些是模型证据，不是实车成功率。详见 [部署记录](docs/data/t321-validation-2026-10-25.json)。

## 回收 T321 轨迹

T321 魔数为 `0x54333231`。跑后保持电池供电、接回电脑，不复位；每次使用新的文件前缀。脚本检查固件版本与零电机输出，不主动启动小车。

```sh
openocd -f interface/stlink.cfg -c 'transport select swd' \
  -f target/stm32f1x.cfg \
  -c 'stm32f1x.cpu configure -event examine-end {}' \
  -c 'adapter speed 950' -c 'gdb port disabled' \
  -c 'tcl port disabled' -c 'telnet port disabled' \
  -c init -c 'set capture_prefix build-local/captures/t321-run-01' \
  -f tools/read_t321.tcl -c shutdown
```

该脚本对应发布的 T321 镜像布局。重新构建后，先按 ELF 核对 RAM 地址。研究脚本与历史报告保留开发过程，部分依赖指定历史提交或本地实验缓存；日常构建与测试使用上方正式入口。
