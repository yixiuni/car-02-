# STM32 小车 H题课程复现

本工程使用 WHEELTEC C10B 主板上的 STM32F103RCT6，复现 2024 年 H 题的四项行驶任务。硬件采用两台 MG513X 电机、两只 65 mm 橡胶轮、幻尔八路灰度和 MPU6050。保留 STM32 是本次课程复现的明确选择；原赛题要求 TI MSPM0。

已发布基线 **DUL5** 支持普通/交叉两条路线、1/4 圈选择和原配 OLED 显示。两种路线目标速度均为 **370 mm/s**，过点不减速，最后一圈回 A 主动刹停。已实测：普通单圈 **11.739 秒**、交叉四圈 **51.695 秒**，各成功一轮、无故障。

基于 [yixiuni/stm32-car](https://github.com/yixiuni/stm32-car) 继续开发。可直接下载本车已验证的 [BIN](firmware/DUL5/stm32-car.bin) / [HEX](firmware/DUL5/stm32-car.hex)，镜像信息见 [manifest](firmware/DUL5/manifest.json)。本车标定参数已写入；另一台车需要重新标定。默认普通 Debug/Release 构建仍保留标定锁，当前已验证版本须使用 `--dual-mode` 构建。

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

# 当前 DUL5，ARM GCC 已在 PATH
python3 tools/build_firmware.py --dual-mode

# 或明确指定工具链 bin 目录
python3 tools/build_firmware.py --dual-mode --toolchain /path/to/arm-toolchain/bin

# 保留标定锁的常规版本（不是当前实车运行版本）
python3 tools/build_firmware.py --config Release
```

DUL5 输出到 `build-local/DualModeDebug/`，包含 ELF、HEX、BIN；常规 Release 输出到 `build-local/Release/`。构建缓存与本机工具链不上传；仅在 `firmware/DUL5/` 保存本车已验证镜像。

已有 CMake/Ninja 时也可用全新构建目录，避免复用旧 Windows 构建缓存：

```sh
cmake -S . -B build-local/cmake-dual -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug -DCAR_DUAL_MODE_BUILD=ON
cmake --build build-local/cmake-dual
```

本次通过 Python 构建入口实际验证 Debug/Release；该入口读取 CMake 中的源码列表。CubeMX 重新生成工程不在本次验证范围内；当前 TIM3、TIM4、TIM8、UART4 和软件 I²C 由自有驱动初始化。

## 回收 DUL5 轨迹

当前镜像魔数 `0x44554C35`。使用 OpenOCD/ST-Link 连接已完成运行且保持电池供电的小车；以下命令不复位、不启动电机，先检查版本和零输出后保存到 `build-local/captures/`：

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

新增独立 T301 算法试验构建，包含分段速度规划、转向增益调整和轮速协同限幅。
当前主板已恢复 **T309原始成功镜像**（直线800 / 弧线430mm/s，实测交叉四圈37.161秒）。T310在首圈D→A弧线出口未被确认，8.402秒触发保护停车，已撤回；其模型通过结果不能当作实车稳定性证明。失败轨迹和恢复回读均已保存。后续使用T309基线，30秒目标尚未达到。
研究参数在部分模型中达到 30 秒以内，但未通过扰动测试。
详见 [30秒目标与算法优化](docs/30秒目标与算法优化.md)。
