# Hero FreeRTOS 任务架构改造方案

> 参照 SerialLeg2026-master 的 INS_task → task_estimate → task_control 通知链模式

## 目标架构

```
                     ┌──────────────────────┐
                     │  Timer Svc  prio 9   │
                     └──────────────────────┘

  BMI088 GPIO EXTI ──▶  vTaskNotifyGiveFromISR
                           │
  ┌────────────────────────▼──────────────────────────────┐
  │  IMUTask         prio 7  栈 512  中断+通知驱动        │
  │  ┌─ SPI DMA 读 IMU 原始数据                            │
  │  ├─ EKF 姿态解算 (IMUSolverUseEKFUserFunc)             │
  │  ├─ IMU 温控                                           │
  │  ├─ PoseUpdateFromIMU → GimbalPoseUpdate               │
  │  └─ xTaskNotifyGive(EstimateTaskHandle)  ◀── 通知     │
  └───────────────────────┬───────────────────────────────┘
                          │ 通知 (每1ms, 可配置降频)
  ┌───────────────────────▼───────────────────────────────┐
  │  EstimateTask    prio 6  栈 512  通知驱动              │
  │  ┌─ ChassisEstimateUpdate()   底盘轮速/里程计/云台偏角│
  │  ├─ JointEstimateUpdate()     关节角度/速度/接触检测   │
  │  ├─ GimbalEstimateUpdate()    云台观测 (当前空, 预留)  │
  │  ├─ ShootEstimateUpdate()     拨盘角度/堵转检测        │
  │  └─ xTaskNotifyGive(ControlTaskHandle)  ◀── 通知     │
  └───────────────────────┬───────────────────────────────┘
                          │ 通知 (每1ms)
  ┌───────────────────────▼───────────────────────────────┐
  │  ControlTask     prio 5  栈 512  通知驱动              │
  │  ┌─ ChassisControlUpdate()   底盘速度闭环+功率限制     │
  │  ├─ GimbalControlUpdate()    云台yaw LADRC闭环        │
  │  ├─ ShootControlUpdate()     拨盘/摩擦轮闭环           │
  │  ├─ JointControlUpdate()     关节力控/位控闭环        │
  │  └─ MotorControlCANSend()    统一 CAN 发送            │
  └───────────────────────────────────────────────────────┘

  并行任务 (与通知链无关, 独立运行):
  ┌────────────────┐  ┌─────────────────┐  ┌──────────────────┐
  │RemoteRecTask   │  │DecisionTask     │  │StateMachineTask  │
  │prio 7 事件驱动 │  │prio 5 定时10ms  │  │prio 6 定时10ms    │
  │遥控器+裁判接收 │  │chassis/joint/   │  │robotState状态机  │
  │                │  │gimbal/shoot输入 │  │                  │
  └────────────────┘  └─────────────────┘  └──────────────────┘
```

## 信号传递全链路

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   BMI088    │     │  UART5/6    │     │  CAN1/2/3   │
│  (SPI+GPIO) │     │ (遥控/裁判) │     │ (电机反馈)  │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │ EXTI中断          │ UART IDLE中断      │ CAN Rx中断
       ▼                   ▼                    ▼
  vTaskNotify       xEventGroupSetBits   更新 const 指针
  GiveFromISR       FromISR              (_chassisMotorRec...)
       │                   │                    │
       ▼                   ▼                    │
  ┌──────────┐     ┌──────────────┐             │
  │ IMUTask  │     │RemoteRecTask │             │
  │ (prio 7) │     │  (prio 7)    │             │
  └────┬─────┘     └──────┬───────┘             │
       │ xTaskNotifyGive  │ xTaskNotifyGive     │
       ▼                  ▼                     │
  ┌────────────┐   ┌──────────────┐             │
  │EstimateTask│   │DecisionTask  │◄────────────┘
  │ (prio 6)   │   │  (prio 5)    │  (读 const 指针)
  └─────┬──────┘   └──────┬───────┘
        │ xNotifyGive     │ 写入 ChassisControl.GimbalCoordinateInput
        ▼                 │        GimbalControl.GimbalTargetInput 等
  ┌────────────┐          │
  │ControlTask │◄─────────┘ (读 DecisionTask 写入的目标值)
  │ (prio 5)   │
  └─────┬──────┘
        │ MotorControlCANSend()
        ▼
  ┌──────────────┐
  │  CAN 发送    │ → 电机执行
  └──────────────┘
```

## 优先级重新分配

| 旧 | 新 | 任务 | 原因 |
|----|-----|------|------|
| — | **7** | IMUTask (↑从5) | 数据源头，必须最高，与 SerialLeg INS_task 一致 |
| 7 | **7** | RemoteRecTask | 遥控接收不能被抢占，保持 |
| 7 | **6** | StateMachineTask (↓) | 状态机不需要比 IMU 高 |
| 6 | — | ~~ControlTask(旧)~~ | 拆分为 Estimate+Control |
| — | **6** | **EstimateTask (新)** | 仅次于 IMU，立即消费姿态数据 |
| — | **5** | **ControlTask (新)** | 等待 estimate 完成后执行 |
| 5 | **5** | UpperPCCommTask | 保持 |
| 4 | **5** | DecisionTask (↑) | 提高到与 Control 同级并行 |
| 4 | **4** | DebugTask | 保持 |
| 3 | **3** | UIOperationTask | 保持 |
| 3 | **3** | SuperSeeTask | 保持 |
| 7 | **4** | MonitorTask (↓) | 监控不需要最高优先级，用 SerialLeg 同优先级 |
| 2 | **2** | MusicTask | 保持 |

## 修改文件清单 (6个文件)

| # | 文件 | 改动类型 | 改动内容 |
|---|------|---------|---------|
| 1 | `Tasks/Src/initial_task.c` | ✏️ 修改 | 新增 `estimateTaskHandle`、新增 `EstimateTask` 创建(prio6)、调整 IMUTask prio 5→7、调降 StateMachineTask 7→6、调降 MonitorTask 7→4、调整 DecisionTask 4→5 |
| 2 | `Tasks/Src/robot_control_task.c` | ✏️ 修改 | 新增 `EstimateTask()` 函数(搬入4个EstimateUpdate) + `ControlTask()` 函数改写为 `ulTaskNotifyTake` 通知驱动(搬出4个Estimate, 保留4个Control+CANSend)，ControlInit 保留在 ControlTask |
| 3 | `Tasks/Inc/robot_control_task.h` | ✏️ 修改 | 新增 `void EstimateTask(void*);` 声明 |
| 4 | `Tasks/Src/peripheral_receive_task.c` | ✏️ 修改 | IMUTask 末尾 `PoseUpdateFromIMU` 后加 `xTaskNotifyGive(estimateTaskHandle)` + 降频计数器宏 `ESTIMATE_TASK_NOTIFY_DECIMATION` |
| 5 | `Tasks/Inc/general_task_include.h` | ✏️ 修改 | 新增 `extern TaskHandle_t estimateTaskHandle;` + 降频宏定义 |
| 6 | `GeneralHeader/general_config_label.h` | ✏️ 修改 | 新增周期常量 `ESTIMATE_TASK_PERIOD_SET` `CONTROL_TASK_PERIOD_SET` 等 |

## 新增配置宏

```c
// 参照 SerialLeg Custom/Config/config.h
// 降频因子: 1=不降频, N=每N个IMU周期通知1次estimate
#define ESTIMATE_TASK_NOTIFY_DECIMATION  1U
#define CONTROL_TASK_NOTIFY_DECIMATION   1U

// 估计任务周期 = 1ms (与IMU同频, 降频因子可调)
#define ESTIMATE_TASK_PERIOD_SET  pdMS_TO_TICKS(1)
```

## 实施步骤

### Step 1: 更新全局声明和宏
- [ ] `general_task_include.h` 新增 `extern TaskHandle_t estimateTaskHandle;` + 降频宏
- [ ] `general_config_label.h` 新增周期常量
- [ ] `robot_control_task.h` 新增 `void EstimateTask(void*);`

### Step 2: 新增 EstimateTask + 改写 ControlTask
- [ ] `robot_control_task.c` 中新增 `EstimateTask()` 函数:
  ```c
  void EstimateTask(void* argument) {
      while(1) {
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          ChassisEstimateUpdate();
          JointEstimateUpdate();
          GimbalEstimateUpdate();
          ShootEstimateUpdate();
          xTaskNotifyGive(controlTaskHandle);
          // 周期监控
      }
  }
  ```
- [ ] `ControlTask()` 改写:
  ```c
  void ControlTask(void* argument) {
      ControlInit();  // 保持
      // ...电机初始化 (保持)...
      while(1) {
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          ChassisControlUpdate();
          GimbalControlUpdate();
          ShootControlUpdate();
          JointControlUpdate();
          MotorControlCANSend();
          // 周期监控
      }
  }
  ```

### Step 3: 更新 IMUTask 通知
- [ ] `peripheral_receive_task.c` 中 IMUTask 末尾加入:
  ```c
  extern TaskHandle_t estimateTaskHandle;
  static uint8_t estimate_notify_div = 0U;
  // ...在 PoseUpdateFromIMU 之后...
  if (++estimate_notify_div >= ESTIMATE_TASK_NOTIFY_DECIMATION) {
      estimate_notify_div = 0U;
      if (estimateTaskHandle != NULL) {
          xTaskNotifyGive(estimateTaskHandle);
      }
  }
  ```

### Step 4: 更新任务创建和优先级
- [ ] `initial_task.c`:
  ```diff
  +TaskHandle_t estimateTaskHandle;
   
  -xTaskCreate(MonitorTask,        "MonitorTask_",       128,  NULL,  7,  &monitorTaskHandle      );
  +xTaskCreate(MonitorTask,        "MonitorTask_",       128,  NULL,  4,  &monitorTaskHandle      );
   xTaskCreate(RemoteRecTask,      "RemoteRecTask_",     256,  NULL,  7,  &remoteRecTaskHandle    );
  -xTaskCreate(StateMachineTask,   "StateMachineTask_",  2048, NULL,  7,  &stateMachineTaskHandle );
  +xTaskCreate(StateMachineTask,   "StateMachineTask_",  2048, NULL,  6,  &stateMachineTaskHandle );
  -xTaskCreate(DecisionTask,       "DecisionTask_",      512,  NULL,  4,  &decisionTaskHandle     );
  +xTaskCreate(DecisionTask,       "DecisionTask_",      512,  NULL,  5,  &decisionTaskHandle     );
  +xTaskCreate(EstimateTask,       "EstimateTask_",      512,  NULL,  6,  &estimateTaskHandle     );
  -xTaskCreate(ControlTask,        "ControlTask_",       512,  NULL,  6,  &controlTaskHandle      );
  +xTaskCreate(ControlTask,        "ControlTask_",       512,  NULL,  5,  &controlTaskHandle      );
  -xTaskCreate(IMUTask,            "IMUTask_",           512,  NULL,  5,  &imuTaskHandle           );
  +xTaskCreate(IMUTask,            "IMUTask_",           512,  NULL,  7,  &imuTaskHandle           );
  ```

### Step 5: 验证
- [ ] Keil MDK 编译零错误零警告
- [ ] 通过 debug 确认通知链顺序: IMUTask → EstimateTask → ControlTask
- [ ] 实机测试: 云台跟随、底盘运动、发射功能正常
