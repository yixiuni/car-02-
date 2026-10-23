# STM32 小车 H题课程复现

本工程使用 WHEELTEC C10B 主板上的 STM32F103RCT6，复现 2024 年 H 题的四项行驶任务。硬件采用两台 MG513X 电机、两只 65 mm 橡胶轮、幻尔八路灰度和 MPU6050。保留 STM32 是本次课程复现的明确选择；原赛题要求 TI MSPM0。

固件已实现编码器轮速闭环、陀螺仪航向、无黑线直行、半圆循迹、端点转接、四圈计数和声光提示。**交付配置禁止未标定启动**：轮径已填 65 mm，编码器每轮计数、轮距、车体及探头位置必须按实车填写。固件的软件验证结果见 [软件验证记录](docs/软件验证记录.md)，实际接线和运动性能需要在车上验收。

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
| SWD | PA13 / PA14 | 保留；仅关闭 JTAG 以释放 PA15 |

MCU 使用 8 MHz 外部晶振，72 MHz 系统时钟；PWM 为 20 kHz。芯片宏、启动文件和链接脚本对应 RCT6 的 256 KB Flash、48 KB RAM。[针序及原理图依据](docs/接口核对.md)。

## 第一次运行

先按 [H题实现与标定](docs/H题实现与标定.md) 完成参数测量和方向检查。所有配置集中在 `Core/Inc/car_config.h`。

1. 首次烧录可保持 `CAR_CALIBRATION_CONFIRMED=0`，查看 `app_diagnostics`，手动转轮检查原始编码器计数，逐路遮挡检查灰度。
2. 填入两轮 x4 后每轮一圈计数、轮距、车体前后长度、半宽和探头前置距离。确认左右轮、编码器正负、灰度顺序、陀螺仪 Z 轴正负后，设置 `CAR_CALIBRATION_CONFIRMED=1`，重新编译。
3. 主板与灰度模块一起断电再上电。模块每次上电只接受一次模式设置；黑线学习为目标，白底学习为背景。上电后保持小车静止，等待约 2 秒陀螺仪零偏校准完成。
4. 停车时长按 USER 约 0.8 秒，切换任务 1→2→3→4→1；蜂鸣和闪灯次数表示任务编号。长按后的松手不会启动。
5. 把**左右轮轴中点放在 A**。任务 1/2 车头朝 B，任务 3/4 车头朝 C。开启使能，短按并松开 USER 启动；行驶中按 USER 或关闭使能即停车。

每次重新启动都当作从 A 开始的新任务。故障、停止或完成后不会自动恢复；重新摆车后再短按。程序正常停车使用双输入低的滑行模式，停车位置依赖低速接近、实车滑行距离和车体尺寸标定。

## 四项任务

场地仅有两条半径 400 mm、宽约 18 mm 的黑色半圆。AB 和 CD 长 1000 mm，AC 和 BD 长约 1280.6 mm；不得添加横向起终点线。任务 1/2 的起步航向为 0°，任务 3/4 为向右下约 38.66°。

| 任务 | 路径 | 完成条件 |
| --- | --- | --- |
| 1 | A→B | B 停车并提示，15 秒内 |
| 2 | A→B→右弧C→D→左弧A | 一圈后 A 停车，30 秒内 |
| 3 | A→C→右弧B→D→左弧A | 一圈后 A 停车，40 秒内 |
| 4 | 重复任务3 | 四圈后 A 停车；固件设 160 秒保护超时，这不是题目给定时限 |

开始 A、每个中间端点及最终停车点均产生一次声光事件。中间端点不等待蜂鸣完成，斜线交叉处不计点。最终点等轮速下降并检查投影覆盖后提示。

## 控制与故障

`car_control.c` 使用 `STRAIGHT → CAPTURE → ARC` 分段状态机；最终进入 `BRAKING → DONE`。直线允许全白，位置和航向闭环控制方向。弧线使用灰度偏差和曲率前馈，轮速 PI 控制两轮。所有驱动命令均为前进或零，不使用反转原地掉头。

进入弧线要求接近预期端点、车体覆盖该点并收到连续三帧黑线。离开弧线要求足够的行驶距离和转角、航向及端点位置匹配，以及连续白底边沿。合格的弧线末端用于校正位置，并按前置探头的圆弧几何关系缓慢修正航向；这一近似依赖平稳循迹，需要实车检查。

灰度通信故障、IMU 失效、编码器异常、控制周期超时、堵转、持续丢线或错过端点均停止输出并锁定故障。`0xFF` 全黑视为歧义图案，不作为终点或计圈标记。单字节灰度协议没有校验和，不能识别所有单字节噪声。

主循环约每 5 ms 采样和控制一次，控制算法按实际间隔计算。灰度约每 10 ms 以上产生新帧，灰度微分和连续帧计数只在新帧更新，避免重复计算旧数据。传感器数据通过 SWD 查看 `app_diagnostics`。

## 构建与测试

需要 Python 3、本机 C 编译器（运行测试），以及 ARM GCC（构建固件）。

```sh
python3 tests/run_tests.py
# Clang/GCC 支持时启用内存及未定义行为检查
python3 tests/run_tests.py --sanitize

# ARM GCC 已在 PATH
python3 tools/build_firmware.py --config Debug
python3 tools/build_firmware.py --config Release

# 或明确指定工具链 bin 目录
python3 tools/build_firmware.py --toolchain /path/to/arm-toolchain/bin --config Release
```

新固件输出到 `build-local/Debug/` 和 `build-local/Release/`，包括 ELF、HEX、BIN。仓库原有 `build/` 中的二进制属于旧横线计圈程序，**不要将它们作为本次固件烧录**。

已有 CMake/Ninja 时也可用全新构建目录，避免复用旧 Windows 构建缓存：

```sh
cmake -S . -B build-local/cmake-release -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-local/cmake-release
```

本次通过 Python 构建入口实际验证 Debug/Release；该入口读取 CMake 中的源码列表。CubeMX 重新生成工程不在本次验证范围内；当前 TIM3、TIM4、TIM8、UART4 和软件 I²C 由自有驱动初始化。

## 代码职责

| 模块 | 职责 |
| --- | --- |
| `board_c10b` | PWM、按键、使能、灯、蜂鸣器、异常关断 |
| `encoder` | TIM4/TIM8 计数、回绕、毫米及轮速换算 |
| `mpu6050` / `imu_filter` | 寄存器配置和采集 / 零偏校准与航向积分 |
| `hiwonder_gray` / `gray_protocol` | UART 中断接收 / 帧窗口、超时、极性与顺序 |
| `motion_control` | 参数校验、轮速 PI、加速限制、前进命令限幅 |
| `car_control` | 四项任务、导航、过点、计圈和故障状态 |
| `app` | 硬件采样、调度、非阻塞提示及诊断 |
| `tests/sim_course.c` | 独立差速运动、轮速响应与赛道灰度模型 |
