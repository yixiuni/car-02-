# stm32-car：C10B + 幻尔八路 UART 循迹

本工程已根据 `完整对话整理.md`、C10B 原理图和幻尔官方协议，从原来的 F103C8T6 / 五路 GPIO / 外接 TB6612 配置迁移为 **WHEELTEC C10B（STM32F103RCT6）+ 板载 AT8236 + 幻尔 LineFollower_8CH UART**。

保留原工程的按键启停、丢线找线和横向起终点线计圈功能。默认目标 1 圈；`CAR_TARGET_LAPS=0` 时持续循迹，由 USER 按键或使能开关停车。横线计圈要求轨道只有一处覆盖八路探头的全黑标记，交叉口或其他宽黑区域不能当作可靠终点。

## 接线和接口

详细引脚、接口针序、闲置资源及旧配置纠正见 [接口核对](docs/接口核对.md)。

| 功能 | C10B 引脚 / 接口 | 软件配置 |
| --- | --- | --- |
| 灰度模块 | CN6.3 PC10 → 模块 RX；CN6.4 PC11 ← 模块 TX | UART4，115200，8N1；中断接收 |
| 灰度电源 | CN6.1 5V；CN6.2 GND | 模块需要 5V，与主板共地 |
| MPU6050 | H6.3 PB14 SCL；H6.4 PB15 SDA | 开漏软件 I²C，7 位地址 0x68，仅读取 WHO_AM_I |
| 电机 A | U6 / CN3：PA6 IN1，PA7 IN2 | TIM3 CH1 / CH2 |
| 电机 B | U5 / CN2：PB0 IN1，PB1 IN2 | TIM3 CH3 / CH4 |
| USER 按键 | PA0 | 高电平有效，25 ms 消抖 |
| 使能开关 | PC13 | 低电平允许行驶，高电平停车 |
| 运行灯 / 故障灯 | PC2 / PC3 | 低电平点亮 |
| 下载 / 调试 | H10.2 PA14 SWCLK；H10.4 PA13 SWDIO | 保留 SWD；H10.3 GND；H10.1 3V3 为参考 |

主控使用 8 MHz 外部晶振，系统 72 MHz，APB1 36 MHz，TIM3 72 MHz。PWM 为 20 kHz，PSC=0、ARR=3599。工程宏为 `STM32F103xE`，启动文件使用高密度 `startup_stm32f103xe.s`；链接空间按 RCT6 的 256 KB Flash / 48 KB SRAM 设置。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `Core/Inc/board_c10b.h` | 集中定义经原理图核对的板级引脚 |
| `Core/Src/board_c10b.c` | 四路电机 PWM、USER/使能输入、LED 和异常关断 |
| `Core/Src/hiwonder_gray.c` | UART4 配置、上电模式命令、中断接收和非阻塞查询 |
| `Core/Src/gray_protocol.c` | 单字节状态接收窗口、超时、极性和左右顺序转换；不依赖 HAL |
| `Core/Src/car_control.c` | 八路差速循迹、按钮事件、计圈、通信故障和丢线停车；不依赖 HAL |
| `Core/Src/mpu6050.c` | PB14/PB15 软件 I²C 身份读取，ACK 和 SCL 超时检查 |
| `Core/Src/app.c` | 调用驱动和控制逻辑，更新调试诊断字段 |
| `Core/Inc/car_config.h` | 速度命令、方向、探头顺序、圈数、控制和通信时间参数 |
| `stm32-car.ioc` | RCT6、引脚和外设资源配置记录 |
| `tests/` | 控制模拟、UART 协议模拟、原理图引脚与工程配置审计 |

原来的 `chapter.c/.h` 已拆分为上述模块，旧 C8 启动文件和链接文件已替换。修改前的 94 个项目文件已归档至 `D:/STM32project/backups/stm32-car_before_c10b_2026-10-13/project_sources.zip`；同目录保存三个项目的源码哈希。`stm32-item` 和 `car-for2` 源码保持原样。

## 灰度协议与故障处理

程序初始化 UART4 后等待 1000 ms，发送 `0x00` 请求手动模式，再等待 50 ms。随后发送 `0x01` 请求八路状态；应答为 1 字节，bit0～bit7 对应 S1～S8。每次应答后保持至少 10 ms 查询间隔；查询超时为 20 ms。收到第一字节后等待 2 ms，期间若又有字节，拒绝该次应答。UART 溢出、噪声、帧错误、迟到或非请求期间的数据会使数据失效。数据超过 100 ms 未更新也失效。

**幻尔模块每次上电只接受一次模式初始化。修改前的诊断可能已设置了运行模式；使用本固件前应让主控与灰度模块一起完全断电再上电。只按 MCU Reset 不会清除传感器模式。** 模块在程序运行中单独重新上电后，也应重新给整套系统上电，让程序再次发送手动初始化命令。

UART 状态位与模块 GPIO 电平相反，默认按 **1 = 学习目标** 解释，请把黑线学习为目标、白底学习为背景。不要沿用旧五路 GPIO 的低电平取反逻辑。`CAR_GRAY_TARGET_BIT_IS_ONE` 可在实测证实需要时修改；S1 位于车辆右侧时把 `CAR_GRAY_REVERSE_ORDER` 改为 1。

单字节状态协议没有校验和，无法证明任意单字节噪声都不是状态，也不能自动证明模块处于正确模式。程序不会把 `55 AA` 模拟量长帧作为完整合法状态帧解析；当前循迹只使用手动状态查询，不发送 `0x02/0x03`，也不提供模拟量/阈值采集。

没有有效数据时禁止启动；行驶中数据失效立即将控制输出置零，不执行丢线找线。通信恢复后保持故障停车，必须松开后重新按 USER。使能开关关闭同样停车，重新开启后需要重新按 USER。按住 USER 上电或复位不会自动启动。

## 循迹和计圈

八路探头从车辆左到右对应 bit0～bit7，权重为 `[-200,-143,-86,-29,29,86,143,200]`。中间两路都检测到黑线时偏差为零。程序计算目标位置均值和相邻控制周期的偏差变化，输出左右轮 PWM 命令：黑线在右则左轮加速，黑线在左则右轮加速。

有有效串口应答但八路均未检测到黑线时，沿最后偏差方向低速找线；持续约 600 ms 后故障停车。全黑横线以低占空比通过。

定圈模式要求从 `0xFF` 横线启动；全黑持续 15 ms 才确认进入，普通黑线持续 100 ms 才确认离开。每圈至少间隔 1000 ms。达目标圈数后输出置零并保持 `CAR_DONE`。这只适用于有唯一横向标记的轨道，实际惯性可能让小车滑过标记，尚未实测停止位置。没有可靠标记时将 `CAR_TARGET_LAPS` 设为 0，采用连续运行及手动停止。

**AT8236 的两个输入均为零时是滑行/休眠。本固件停车采用两输入低电平，不宣称短路制动或精确停在起点。** 异常处理会直接关闭 TIM3 输出并把 PA6/PA7/PB0/PB1 强制拉低。

PWM 参数是千分比占空比命令，不是实测轮速。本工程尚未读取编码器，也不显示速度、距离或姿态。MPU6050 身份读取不会改变模块配置，其失败只记录诊断，不阻止本循迹算法启动。

## 首次实车检查

1. 主控和灰度模块一起断电重启，按官方步骤完成黑线/白底学习，识别时保持与学习时相同的探头高度。
2. 先保持电机禁止状态，使用 SWD 查看 `app_diagnostics.gray_online`，确认 `gray_samples` 持续增加。逐路遮挡并验证 `gray_raw_status` 和 `black_mask` 的 bit0～bit7 对应关系。
3. 架空车轮，确认电机 A / CN3 是否为左轮。默认 `CAR_LEFT_IS_MOTOR_A=1`，若 A 是右轮改为 0。分别确认正转，必要时修改 `CAR_MOTOR_A_FORWARD_IN1` / `CAR_MOTOR_B_FORWARD_IN1`。
4. 确认 USER 按下才读到高电平；使能开关低电平才允许行驶。测试禁止、手动停止和通信断开后电机输出归零。
5. 先降低 `CAR_BASE_SPEED`，再实测循迹增益、丢线恢复和起终点横线识别。此工程没有编码器速度闭环，需要根据车体调整参数。

`app_diagnostics` 还包含 `state`、`enabled`、`left_pwm/right_pwm`、`completed_laps`、`gray_timeouts/errors/unexpected_bytes`、`mpu_online` 和 `mpu_identity`。MPU6050 预期身份为 0x68，仅此结果不能证明采样或姿态解算正确。

## 构建与检查

```powershell
# 在已配置 STM32Cube GNU 工具链与 Ninja PATH 的终端执行
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release

# 本机控制与协议测试，可指定其他本机 GCC
.\tests\run_tests.ps1 -Compiler 'C:\MinGW\bin\gcc.exe'
```

固件生成于 `build/Debug/stm32-car.elf/.hex/.bin` 和 `build/Release/`。应选择本次重新生成的 RCT6 固件。受限环境可能触发 `Illegal System DLL Relocation`，可在本机正常终端执行同样命令。

CubeMX 的 `stm32-car.ioc` 记录 TIM3/UART4 引脚和参数，但本工程运行时使用板级寄存器驱动初始化外设。UART4 NVIC 的启用由 `HiwonderGray_Init` 管理，`.ioc` 不生成另一份 UART4 IRQ；现有 IRQ 在 `stm32f1xx_it.c` 的 USER CODE 区域调用自有接收驱动。若使用 CubeMX 重新生成代码，应保留 USER CODE 和顶层 CMake 的应用源文件列表，并重新执行接口审计、编译及实车通信检查。本轮未验证 CubeMX 重新生成后的结果。

具体软件检查与固件容量见 [软件验证记录](docs/软件验证记录.md)。

## 验证边界

本次完成的是源码迁移、配置核对、ARM 编译和主机模拟测试。没有烧录或擦除主控，没有重新操作 ST-Link，也没有验证真实 UART 返回、轮子方向、传感器学习或实车循迹。

用户提供的 2026-10-13 对话记录显示 MPU6050 基本通信正常，灰度 UART 查询仍无返回。这是该记录中的检测结果，并非本轮实测；固件加入 UART4 驱动不等于线缆、供电和模块通信已通过。

协议依据：[幻尔 STM32 UART 示例](https://docs.hiwonder.com/projects/8-ch-Line-Follower/en/latest/docs/4.STM32_Communication.html)、[幻尔通信协议](https://docs.hiwonder.com/projects/8-ch-Line-Follower/en/latest/docs/2.Quick_Start.html)、[AT8236 厂商数据手册](https://datasheet.lcsc.com/lcsc/2109242230_ZHONGKEWEI-AT8236_C2827823.pdf)。本地依据和逐项修正见接口核对文档。
