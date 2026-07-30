# 外设连接与调用

最后核对日期：2026-07-28。

## 阅读说明

本文依据实机连接确认，以及当前代码的 CubeMX 初始化、`device_define.h` 映射、`PeripheralRecEnable()`、任务循环和实际发送函数整理。实机连接由用户确认时，其优先级高于仅从初始化代码推断出的用途。

- **运行**：当前构建路径会初始化并实际调用。
- **条件运行**：受编译宏、运行模式或数据到达事件控制。
- **保留**：CubeMX 已初始化或存在驱动，但当前主任务链没有有效调用。
- **未连接**：代码可能仍初始化或调用接口，但实机没有连接对应外设，不能视为有效通信链路。
- 频率是源码中的标称调度频率；中断接收类外设按数据到达触发，不等同于固定任务周期。
- 若注释、宏名和实际句柄冲突，以当前函数调用和 CubeMX 参数为准；若代码与已确认实机接线冲突，以实机接线为准并标记遗留代码。

## 实机连接基线

- 裁判系统只连接在 `hero_down`；`hero_up` 没有直连裁判系统。
- `hero_up` 仅通过 FDCAN3 B2B 获取下板筛选回传的部分裁判数据，目前包括 0x222 中的 HP，以及 0x224 中的射击标志和弹速。
- `hero_up` 实机没有激光测距模块；USART10 的初始化和接收代码属于遗留配置。
- `hero_down` 没有连接算法上位机 CDC；USB CDC 相关任务代码不代表存在实际链路。
- 上下板均没有实际连接或调用舵机；下板 TIM2 CH1 的 50 Hz PWM 启动属于遗留/预留配置。

## 公共任务频率

上下板 `configTICK_RATE_HZ` 下的周期宏均以毫秒使用：

| 调用阶段 | 周期 | 标称频率 | 说明 |
|---|---:|---:|---|
| `IMUTask` | 2 ms | 500 Hz | 读取 BMI088、温控、姿态更新，并通知 Estimate |
| `EstimateTask` | 通知驱动 | 约 500 Hz | 等待 IMUTask 通知 |
| `ControlTask` | 通知驱动 | 约 500 Hz | 等待 EstimateTask 通知，随后发送 CAN 控制帧 |
| `DecisionTask` | 10 ms | 100 Hz | 更新控制目标 |
| `UpperPCCommTask` | 4 ms | 250 Hz | 上位机发送数据；上板同时发送 B2B target |
| `DebugTask` | 1 ms | 1 kHz | 具体调试帧可在函数内再次分频 |
| `StateMachineTask` | 10 ms | 100 Hz | 整车状态更新 |
| `MonitorTask` | 50 ms | 20 Hz | 看门狗、任务监控；上板同时刷新灯带 |
| `MusicTask` | 20 ms | 50 Hz | 查询音乐/错误提示队列 |
| 下板 `UIOperationTask` | 3 ms 基础周期 | 实际约 27.8 Hz | 每 12 次执行一次 UI 发送，约 36 ms |
| 遥控接收任务 | 事件驱动 | 随 UART 帧到达 | 500 ms 无数据超时 |

`CONTROL_TASK_PERIOD_SET` 虽定义为 3 ms，但当前 ControlTask 不按该宏延时，而是由 2 ms IMU 通知链驱动，因此标称控制频率约 500 Hz。

## 通用接口参数

### FDCAN

上下板三路 FDCAN 配置相同，均为 Classic CAN、Normal mode。根据 `HSE_VALUE=24 MHz`、PLL1 Q 输出 120 MHz 和当前位时序计算：

| 总线 | Prescaler | TimeSeg1 | TimeSeg2 | 标称位速率 |
|---|---:|---:|---:|---:|
| FDCAN1 | 24 | 2 tq | 2 tq | 1 Mbit/s |
| FDCAN2 | 3 | 29 tq | 10 tq | 1 Mbit/s |
| FDCAN3 | 24 | 2 tq | 2 tq | 1 Mbit/s |

当前使用 Classic CAN 帧，数据段参数不参与实际 Classic CAN 位速率。源码中个别带宽注释仍写 FDCAN2 为 781 kbit/s，与当前时钟和位时序不一致。

### UART

| 句柄 | 当前参数 | 逻辑映射 |
|---|---|---|
| UART5 | 100000，9-bit + even parity + 2 stop | DT7/VT13 遥控器；等效常见 DBUS 8E2 数据格式 |
| USART1 | 115200，8N1 | 下板裁判系统；上板未连接 |
| UART7 | 921600，8N1 | 上板 MiniPC 接收；上下板调试数据发送代码使用该口 |
| UART8 | 1000000，8N1 | `PITCH_UART` 映射，当前主链未使用 |
| UART9 | 1000000，8N1 | `DEBUG_UART` 映射，但实际 DebugTransmit 使用 UART7 |
| USART10 下板 | 921600，8N1 | 激光测距 |
| USART10 上板 | 9600，8N1 | 激光测距遗留配置，实机未连接 |
| USART2 | 921600，8N1 | 下板 `MASTER_485_UART` / 上板 `SHOOT_485_UART`，当前保留 |
| USART3 | 921600，8N1 | 下板 `SERVENT_485_UART` / 上板 `SERVANT_485_UART`，当前保留 |

UART 接收主要采用 DMA + Receive-to-idle，中断回调解析后立即重启 DMA。

### SPI、PWM 与 USB

- SPI2：120 MHz PLL 时钟 / 32，约 3.75 MHz，Mode 0、8-bit、全双工，用于 BMI088。
- SPI6：D3PCLK1 120 MHz / 16，约 7.5 MHz，8-bit、TX-only、CPOL=0/CPHA=2EDGE，用于板载 WS2812 单灯。
- TIM3 CH4：240 MHz timer clock / 24 / 10000 = 1 kHz PWM，用于 BMI088 加热片。
- TIM12 CH2：蜂鸣器 PWM；播放函数会动态调整 ARR/CCR，不能用初始化 ARR 表示固定音频频率。
- TIM2 CH1 下板：初始化为约 50 Hz PWM，启动后当前代码未更新比较值，实机未连接舵机。
- TIM2 CH1 上板：启动时被 `WS2812_PWM_Init()` 重配为 800 kHz PWM + DMA，用于外接 WS2812 灯带。
- USB：USB OTG HS 控制器使用 embedded FS PHY，CDC 没有 UART 波特率概念。上板用于算法上位机；下板虽然保留 CDC 代码，但实机未连接算法上位机。

## hero_down 外设

| 外设/设备 | MCU 接口 | 状态 | 调用位置与频率 |
|---|---|---|---|
| 板载 BMI088 IMU | SPI2，约 3.75 MHz | 运行 | `IMUTask` 每 2 ms 读取并执行 EKF，约 500 Hz |
| IMU 加热片 | TIM3 CH4，1 kHz PWM | 运行 | `IMUTask` 每 2 ms更新温控输出 |
| DT7/VT13 遥控器 | UART5，100000，8E2，DMA idle | 运行 | UART 帧到达触发回调与事件组；500 ms 超时保护 |
| 裁判系统 | USART1，115200 8N1，DMA idle | 运行 | 接收按帧到达；UI 通过同口 DMA 发送，实际约 27.8 Hz |
| 激光测距 | USART10，921600 8N1，DMA idle | 运行 | 数据到达后调用 `distance_datacheck()`，无固定轮询频率 |
| 算法上位机 CDC | USB CDC FS | 未连接 | 下板仍初始化 USB 并保留 CDC 收发代码，`UpperPCCommTask` 也会每 4 ms调用发送，但实机没有算法上位机链路 |
| 调试上位机 | UART7，921600 8N1，DMA TX | 条件运行 | `DebugTask` 1 kHz 调用；当前未选择有效下板调试帧宏时不实际发送 |
| FDCAN1 关节电机 | FDCAN1，1 Mbit/s | 运行 | 4 个 DM MIT 关节，ID 0x01-0x04；Control 链内每 2 slot，约 250 Hz |
| FDCAN1 拨弹电机 | FDCAN1，1 Mbit/s | 运行 | DM/J4310，命令 ID 对应 0x108、响应 0x018；每 5 slot，约 100 Hz |
| FDCAN2 底盘轮电机 | FDCAN2，1 Mbit/s | 运行但当前零输出宏开启 | DJI 0x201-0x204，组帧 0x200；约 250 Hz |
| FDCAN2 履带电机 | FDCAN2，1 Mbit/s | 运行但当前零输出宏开启 | DM MIT ID 0x05/0x06；约 125 Hz |
| 超级电容 | FDCAN2，1 Mbit/s | 运行 | 接收 ID 0x211；控制/功率帧 0x2FF 约 100 Hz |
| 双板通信 | FDCAN3，1 Mbit/s | 运行 | 0x220 机体姿态和 0x223 拨盘约 500 Hz；0x221/0x222/0x224 约 100 Hz |
| 蜂鸣器 | TIM12 CH2 PWM | 运行 | 启动提示；`MusicTask` 20 ms轮询队列，音乐宏当前关闭 |
| TIM2 CH1 遗留 PWM | TIM2 CH1，约 50 Hz | 未连接 | InitTask 仍启动 PWM，但上下板均无舵机，下板也未设置有效脉宽 |
| 板载 WS2812 | SPI6，约 7.5 MHz | 保留 | SPI6 已初始化且驱动存在，当前下板任务未调用颜色发送函数 |
| USART2/USART3 485 | 921600 8N1 | 保留 | 原双板 485 已由 FDCAN3 B2B 替代，接收 DMA 启动代码被注释 |
| UART8/UART9 | 1000000 8N1 | 保留 | 已初始化，当前主任务链无有效调用 |

## hero_up 外设

| 外设/设备 | MCU 接口 | 状态 | 调用位置与频率 |
|---|---|---|---|
| 板载 BMI088 IMU | SPI2，约 3.75 MHz | 运行 | `IMUTask` 每 2 ms 读取、EKF 解算并通知控制链，约 500 Hz |
| IMU 加热片 | TIM3 CH4，1 kHz PWM | 运行 | `IMUTask` 每 2 ms更新温控输出 |
| DT7/VT13 遥控器 | UART5，100000，8E2，DMA idle | 运行 | UART 帧到达触发；500 ms 超时保护 |
| 裁判系统直连 | USART1，115200 8N1 | 未连接 | 上板没有裁判系统物理连接；相关 USART1 DMA 初始化和解析代码属于遗留路径 |
| 裁判数据子集 | FDCAN3 B2B，1 Mbit/s | 运行 | 从下板获取部分裁判数据：0x222 携带 HP，0x224 携带射击标志和弹速；上板不能据此获得完整裁判协议数据 |
| MiniPC 串口 | UART7，921600 8N1，DMA idle | 运行 | 接收按帧到达，调用 `UpperCommRecHandler()` |
| 算法上位机 | USB CDC FS | 运行 | CDC 接收异步；`UpperPCCommTask` 每 4 ms发送，250 Hz |
| 调试数据 | UART7，921600 8N1，DMA TX | 运行 | `DebugTask` 1 kHz，函数内隔次发送，实际约 500 Hz；与 MiniPC 共用 UART7 |
| 激光测距 | USART10，9600 8N1 | 未连接 | 上板实机没有激光模块；DMA 接收及回调重启代码是遗留配置，且没有数据解析 |
| FDCAN1 pitch 电机 | FDCAN1，1 Mbit/s | 运行 | LK 电机 ID 0x141；Control 链约 500 Hz，状态读取/控制按 3 slot 分配 |
| FDCAN2 摩擦轮 | FDCAN2，1 Mbit/s | 运行 | 6 个 DJI 电机，接收 0x201-0x207，发送组帧 0x200/0x1FF；约 500 Hz |
| FDCAN3 yaw 电机 | FDCAN3，1 Mbit/s | 运行但当前 `ZERO_YAW` 开启 | DM MIT CMD 0x07、响应 0x017；控制帧约 500 Hz |
| 双板通信 | FDCAN3，1 Mbit/s | 运行 | 0x228 云台姿态约 500 Hz；0x229 target 当前由 4 ms UpperPCCommTask 发送，实际约 250 Hz |
| 板载 WS2812 单灯 | SPI6，约 7.5 MHz | 运行 | InitTask 启动阶段发送红/黄/绿状态，共三次 |
| 外接 WS2812 灯带 | TIM2 CH1，800 kHz PWM + DMA | 运行 | InitTask 初始化；MonitorTask 每 50 ms刷新，约 20 Hz |
| 蜂鸣器 | TIM12 CH2 PWM | 运行 | `MusicTask` 20 ms轮询队列；音乐宏当前关闭 |
| USART2/USART3 485 | 921600 8N1 | 保留 | UART 已初始化，但 `PeripheralRecEnable()` 未启动对应 DMA，B2B 使用 FDCAN3 |
| UART8/UART9 | 1000000 8N1 | 保留 | 当前 pitch 走 FDCAN1，调试实际走 UART7 |

## 已确认的不一致与注意事项

- `Board2Board.h` 文件头仍有 `hfdcan1` 旧描述，但 `B2B_CAN` 宏和实际调用均为 `hfdcan3`。
- 上板 `B2BSendGimbalTarget()` 注释写 100 Hz，但当前位于 4 ms 的 `UpperPCCommTask` 中且没有分频，实际约 250 Hz。
- `device_define.h` 只负责端口映射；USART2/3、USART10 的实际波特率以 `Core/Src/usart.c` 为准。
- `DEBUG_UART` 宏映射 UART9，但当前上下板 `DebugTransmit()` 都直接使用 UART7。
- 上板 UART7 同时承担 MiniPC DMA 接收与调试 DMA 发送；修改协议或提高负载时必须检查共用端口冲突。
- 上板 USART1、USART10 和下板 USB CDC 存在“软件仍初始化但实机未连接”的情况，排查通信问题时不能仅凭 HAL 初始化判断设备存在。
- 上下板均没有舵机；下板 TIM2 CH1 的 50 Hz PWM 不应被解释为当前舵机应用。
- 上板 IWDG 已初始化并由 50 ms MonitorTask 刷新；下板 `MX_IWDG1_Init()` 当前被注释，但 MonitorTask 仍调用刷新函数。
