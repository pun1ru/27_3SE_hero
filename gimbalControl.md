# GimbalControl — 云台控制系统文档

## 1. 系统总览

云台控制是 Hero 上板（hero_up）的核心功能，负责 pitch/yaw 两轴云台的姿态控制和射击瞄准。系统支持**普通模式**和**世界系模式（WorldGimbal）**两种控制框架。

## 2. 控制模式

### 2.1 普通模式（`worldGimbal.enable = 0`）

用户指令（遥控器/PC鼠标/WASD）直接映射为云台电机目标角度增量：

- Yaw：指令累积到 `GimbalTargetInput.yaw_angle_d`，经过 LTD 滤波 → LTDPID 闭环 → yaw 电机输出
- Pitch：指令累积到 `GimbalTargetInput.pitch_angle_d`，经过 LTD 滤波 → ADRC（ESO+LESF）闭环 → pitch 电机输出
- Small Pitch：独立目标角 `GimbalTargetInput.small_pitch_angle_d`

**适用场景：** 遥控器右拨中（`NORM_RC_SW_MID`），非吊射模式。

### 2.2 世界系模式（WorldGimbal, `worldGimbal.enable = 1`）

用户指令被解释为**世界坐标系中的增量旋转**，通过虚拟目标指向向量 `f_des_B` 在机体坐标系中表示期望瞄准方向，再用阻尼最小二乘 IK 反解真实电机角。

**适用场景：** 遥控器右拨下（`NORM_RC_SW_DOWN`，吊射模式 sniper_on），或在 PC 模式下狙击开镜时。

## 3. WorldGimbal — 世界系云台控制原理

### 3.1 坐标系定义

- **机体坐标系 B**：x=前，y=右，z=下（右手系）
- **水平对齐系 H**：z_H = 重力反方向（世界"上"），x_H = 底盘正向在水平面中的投影，y_H = z_H × x_H（水平面内的"右"）
- **重力方向 g_B**：大地竖直向下在 B 系中的表达，由底盘 roll/pitch 计算得：
  ```
  g_B = [-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
  ```
  仅依赖 roll/pitch（有重力加速度参考，不漂），**独立于 yaw 漂移**。

### 3.2 控制流程（三段式）

```
DecisionTask(10ms)
  └── GimbalInputUpdate()
        └── WorldGimbalInputUpdate(&worldGimbal, dyaw_deg, dpitch_deg)
            → 绕 g_B 旋转 f_des_B（世界系 yaw）
            → 绕 right_B 旋转 f_des_B（世界系 pitch，奇异点有保护）

IMUTask(2ms) → 通知
  └── EstimateTask
        └── WorldGimbalEstimateUpdate(&worldGimbal)
            → 读取底盘 IMU（485下板）：计算 g_B
            → 读取电机编码器：FK 计算 f_real_B（当前真实指向）
            → 计算 angle_error_deg（指向误差）
            → 反算 world_yaw_deg / world_pitch_deg（供 UI/485 帧）

EstimateTask → 通知
  └── ControlTask
        ├── WorldGimbalIKSolve(&worldGimbal)
        │     → 阻尼最小二乘 IK：从 f_des_B 反解 yaw/pitch 电机目标角
        │     → Jacobian: J = [a_B × f_real, b_B × f_real]
        │     → 正规方程: (JᵀJ + λI)·dq = Jᵀe
        └── WorldGimbalApplyToTargets(&worldGimbal)
              → 将 IK 结果写入 GimbalTargetInput
              → 后续走普通 ADRC/LTD 闭环链路
```

### 3.3 关键数学

#### FK（正运动学）

```
f_real = R_yaw(q_yaw) · R_pitch(q_pitch) · [1, 0, 0]ᵀ
b_B    = R_yaw(q_yaw) · [0, 1, 0]ᵀ

旋转轴：
  a_B (yaw轴)  = [0, 0, 1]  （B系 z 轴）
  b0_B (pitch轴初始方向) = [0, 1, 0]  （B系 y 轴，随 yaw 旋转）
```

旋转用罗德里格斯公式（Rodrigues' rotation formula）：
```
v_rot = v·cosθ + (k×v)·sinθ + k·(k·v)·(1-cosθ)
```

#### IK（阻尼最小二乘反解）

目标：找到 `q = [q_yaw, q_pitch]` 使得 `f_real(q) ≈ f_des_B`。

- 误差定义：`e = (f_real × f_des) × f_real`（指向误差的切向分量）
- Jacobian 列：`Jy = a_B × f_real`，`Jp = b_B × f_real`
- 正规方程（2×2）：`(JᵀJ + λI)·dq = Jᵀe`
- 步长裁剪：`|dq| ≤ WORLDGIMBAL_IK_MAX_STEP_RAD (0.05 rad ≈ 2.86°)`
- 收敛判定：`|f_real × f_des| < WORLDGIMBAL_IK_CONVERGE_RAD (0.0005 rad ≈ 0.03°)`
- 最大迭代：`WORLDGIMBAL_IK_MAX_ITERS (10)`

#### 奇异点保护

当 `f_des_B` 接近竖直方向（`g_B × f_des_B → 0`）时，pitch 右轴退化：
- 复用上一帧的有效 right_B 方向完成 pitch 旋转
- 若上一帧也无效，跳过此次 pitch（指向完全竖直且无历史）

### 3.4 世界系欧拉角

从 `f_des_B` 和 `g_B` 反算世界系方位角（供上位机显示）：

- **World Elevation**（世界系 pitch）：`sin(elev) = -dot(f_des_B, g_B)`
- **World Azimuth**（世界系 yaw）：`f_des_B` 在水平面投影 → 与底盘正向水平投影的夹角

## 4. 控制输入来源

| 模式 | 终端 | Yaw 来源 | Pitch 来源 | 备注 |
|------|------|---------|-----------|------|
| 普通模式 | CONTROL_FROM_REMOTE | 遥控器 CH2 | 遥控器 CH3 | 带分离模式（CHASSIS_SEPARATE）修正 |
| 世界系模式 | CONTROL_FROM_REMOTE (sniper_on) | WorldGimbalInputUpdate | WorldGimbalInputUpdate | 狙击开镜时 |
| PC 模式 | CONTROL_FROM_PC | 鼠标 + WASD | 鼠标 + WASD | sniper_on → WorldGimbal；sniper_off → 直接角度控制 |
| PC C键瞄准 | CONTROL_FROM_PC + aim_mode | 上位机 target_yaw_angle_d | 上位机 target_pitch_angle_d | 单次触发（立即清零 aim_mode） |
| Q键预设 | sniper_on | WorldGimbalSetWorldAngles | 世界系 pitch 40° | 快速预设瞄准角 |
| 停止 | CONTROL_STOP | 锁定当前角 | 锁定当前角 | 所有输出清零 |

## 5. 控制算法

### 5.1 Pitch — LTD + ADRC (ESO + LESF)

```
目标角度 → LTD(最速跟踪微分器) → 平滑目标角
                                     ↓
                  ESO(扩张状态观测器) ← 平滑目标角 + 电机实际角
                       ↓
                  LESF(线性误差状态反馈) → 电机电流/扭矩输出
```

- **LTD**：r=20, h=0.003, 限幅 (-30°, 60°)
- **ESO**：wo=20, b0=3.0, z3_limit=2000
- **LESF**：k_p=70, k_d=8, k_i=0, output_limit=1000

### 5.2 Pitch — 狙击/关节模式（编码器位置控制）

当 `robotState.sniper == SNIPER_ON` 或 `robotState.joint_mode == ROBOT_JOINT_MODE_CLIMB`：
- 切换为编码器直接位置控制（LK_MultiLoop_angleControl_limited）
- 使用编码器（非 IMU）避免漂移
- 精度 0.01°，速度限制 50 dps

### 5.3 Yaw — LTD + LTDPID

```
目标角度 → LTD(最速跟踪微分器) → 平滑目标角
                                     ↓
                            LTDPID 闭环 → yaw 电机输出（转发到 hero_down 485）
```

- **LTD**：r=30, h=0.003, 限幅 (-180°, 180°)
- **LTDPID**：P=10, I=200, D=30000, output_limit=30000

### 5.4 Yaw 反馈来源

- **普通模式（sniper_off）**：IMU yaw 角（通过 EKF 解算）
- **吊射模式（sniper_on）**：电机编码器机械角（防 IMU 漂移）
- 切换时有 `shit_delay_count` 保护（防目标突变）

## 6. 观测量

| 信号 | 来源 | 更新频率 | 备注 |
|------|------|---------|------|
| pitch_angle_d | IMU (BMI088 → EKF) 或 编码器 | 500Hz (2ms) | sniper_on → 编码器 |
| yaw_angle_d | IMU 或 编码器 | 500Hz | sniper_on → 编码器 |
| roll_angle_d | IMU (BMI088 → EKF) | 500Hz | 仅观测 |
| pitch_angular_velocity_dps | IMU 陀螺仪 | 500Hz | 用于 ADRC ESO |
| yaw_angular_velocity_dps | IMU 陀螺仪 | 500Hz | 有高频噪声，需滤波 |
| g_B[3] | 底盘 IMU (485 下板) | ~500Hz | 世界系重力方向 |
| f_real_B[3] | 电机编码器 → FK | 通知驱动 | 世界系当前指向 |
| world_yaw/pitch_deg | 反算 | 通知驱动 | 世界系欧拉角 |

## 7. 特殊模式

### 7.1 狙击模式（Sniper Mode, `SNIPER_ON`）

- 触发：遥控器右拨下，或 PC 模式按 X 键
- 行为：pitch 切换为编码器位置控制（防 IMU 漂移），可切换世界系模式
- mouse_fix：按 E 键锁止鼠标输入，仅保留 WASD 微调

### 7.2 关节爬升模式（Joint Climb, `ROBOT_JOINT_MODE_CLIMB`）

- 触发：遥控器特定拨杆组合
- 行为：pitch 切换为编码器位置控制，关节模式切换时重置 ADRC 状态

### 7.3 CONTROL_STOP 保护

- 触发：双拨上、血量归零、遥控信号丢失
- 行为：所有目标锁定当前角度，ADRC/LTD 状态清零，电机输出清零

## 8. 联系方式

- 参考代码：`hero_up/PrivateApplications/MainControl/gimbalControl.c`
- WorldGimbal 实现：`hero_up/PrivateApplications/MainControl/worldGimbal.c`
- 射击控制：`hero_up/PrivateApplications/MainControl/shootControl.c`
- 任务调度：`hero_up/Tasks/Src/task_decision.c`, `task_estimate.c`, `task_control.c`
