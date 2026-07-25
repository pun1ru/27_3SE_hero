# 拨盘观测与控制指南

本文描述 `hero_down` 当前拨盘实现，重点说明角度坐标、多圈观测、预置对齐、拨弹、堵转恢复，以及拨盘电机与主控 MCU 非同步掉电/复位时的风险。

相关代码：

- 反馈解析：`hero_down/Tasks/Src/peripheral_receive_task.c`
- 观测与目标生成：`hero_down/PrivateApplications/MainControl/stirControl.c`
- 数据结构：`hero_down/PrivateApplications/MainControl/stirControl.h`
- 目标下发：`hero_down/Tasks/Src/peripheral_transmit_task.c`
- 电机协议封装：`hero_down/PrivateDrivers/DMJ4310/DMJ4310.c`
- 角度范围常量：`hero_down/GeneralHeader/general_config_label.h`

## 1. 先说结论

1. 拨盘反馈的 16 bit 位置字段按 `[-17 * 360 deg, +17 * 360 deg]` 解码，即 `[-6120 deg, +6120 deg]`。`17 * 360 deg` 是半量程，整个反馈区间跨度是 `34 * 360 deg = 12240 deg`。
2. 电机反馈不是软件可无限使用的连续角度。代码通过检测反馈从正端跳到负端或从负端跳到正端，维护 `quan_shu_r`，再构造 `stir_all_angle_d`。
3. 当前跨界补偿存在明显疑点：反馈区间跨越一次的跨度是 `12240 deg`，代码却只补偿 `6120 deg`。如果电机反馈确实在 `+6120 deg` 和 `-6120 deg` 之间回绕，那么当前 `stir_all_angle_d` 在跨界后仍会跳变 `6120 deg`，并不连续。
4. `quan_shu_r` 不是普通意义上的“机械转了一圈”。它记录的是反馈协议量程的跨界次数；按当前解码，一个完整反馈周期相当于 34 个机械圈。
5. 当前掉电检测只清理了一部分观测量，没有同时清理 `quan_shu_r`、连续角度和目标角度。因此“电机单独掉电、MCU 不复位”与“MCU 单独复位、电机不复位”都可能造成坐标基准不一致。
6. 最可靠的现行操作方式是让拨盘电机和主控同步上电/复位，并在允许拨弹前等待反馈稳定、重新执行预置对齐。若必须支持任一侧独立复位，需要重构观测器和复位握手。

## 2. 信号链路

```text
拨盘电机 CAN 反馈 0x018
        |
        v
peripheral_receive_task.c
  p_int -> pos_d                 原始有限量程角度
        |
        v
GetStirRealAngle()
  pos_d + 跨界计数 -> stir_all_angle_d
        |
        +---------------------------+
        |                           |
        v                           v
StirTargetAngleSet()          拨弹/堵转状态机
  对齐到 60 deg 分度点          修改 stir_all_target_pos_d
        |                           |
        +-------------+-------------+
                      v
             deg -> rad
                      |
                      v
MotorControlCANSend()，100 Hz
  ctrl_motor2(position_rad, velocity)
                      |
                      v
              电机内部位置控制
```

MCU 侧没有对拨盘再做一层位置 PID。主控生成位置目标和速度参数，使用 `ctrl_motor2()` 将两个 `float` 原样打包，位置闭环由电机内部完成。

## 3. 角度坐标和关键变量

| 变量 | 当前含义 | 单位/范围 |
| --- | --- | --- |
| `p_int` | CAN 反馈中的 16 bit 位置字段 | `0..65535` |
| `DM_MOTO_MAX_ENCODE_D` | 反馈解码半量程 | `6 * 60 * 17 = 6120 deg` |
| `pos_d` | 由 `p_int` 解码的有限量程位置 | `[-6120, +6120] deg` |
| `stir_angle_last/cur` | 跨界判断使用的前后两次 `pos_d` | deg |
| `quan_shu_r` | 软件记录的反馈量程跨界次数 | 无量纲，不是机械圈数 |
| `stir_all_angle_d` | 软件尝试构造的连续反馈角度 | deg |
| `stir_all_target_pos_d` | 连续坐标系中的拨盘目标 | deg |
| `stir_all_target_pos_rad` | 下发给电机的位置目标 | rad |
| `stir_real_angle` | 另一套增量累计角度，当前不参与主要控制 | deg |
| `stir_reset_flag` | 软件认为拨盘反馈中断后的重置标志 | 0/非 0 |

反馈解码公式为：

```text
R = 17 * 360 = 6120 deg
pos_d = p_int / 65535 * (2R) - R
```

因此：

```text
反馈半量程 R       = 6120 deg = 17 圈
反馈完整跨度 2R    = 12240 deg = 34 圈
理论分辨率         = 12240 / 65535 = 0.1868 deg/count
```

这里的 `pos_d` 名称带 `_d`，后续也确实按“度”使用。下发前才乘 `PI / 180` 转为弧度。

## 4. 当前多圈角度算法

`GetStirRealAngle()` 先读取当前 `pos_d`，然后在接近量程端点时判断是否发生回绕：

```text
last > +6020 且 current < -6020  -> quan_shu_r++
last < -6020 且 current > +6020  -> quan_shu_r--
```

当前连续角度计算为：

```text
stir_all_angle_d = pos_d + quan_shu_r * 6120 deg
```

### 4.1 这里为什么可疑

若 16 bit 反馈按常见方式从 `+6120 deg` 回绕到 `-6120 deg`，一次跳变的跨度是 `12240 deg`。连续化通常应补偿完整跨度：

```text
continuous = pos_d + wrap_count * 12240 deg
```

当前代码只补偿 `6120 deg`。正向跨界示例：

```text
跨界前：pos_d ~= +6120, quan_shu_r = 0 -> all ~= +6120
跨界后：pos_d ~= -6120, quan_shu_r = 1 -> all ~= 0
```

结果仍然突变约 `-6120 deg`。所以在没有电机协议实测波形之前，不能把当前 `stir_all_angle_d` 当作已经正确的无限多圈角度。

需要用调试器同时记录 `p_int` 和 `pos_d`，让电机跨过量程边界一次，确认实际跳变到底是：

- `+6120 -> -6120`：补偿周期应是 `12240 deg`；
- `+6120 -> 0` 或其他形式：说明当前 `uint_to_float()` 映射或对电机协议的理解还需要修正。

### 4.2 `stir_real_angle` 不是可靠替代品

代码还有一套增量累计：

```text
delta = stir_angle_cur - stir_angle_last
stir_real_angle += delta
```

但原有的跨界修正已经被注释，而且注释中的 `720/1440 deg` 还是旧量程。因此它在当前 `+/-6120 deg` 边界同样会产生大跳变。当前主要到位判断实际使用的是 `stir_all_angle_d`，不是 `stir_real_angle`。

## 5. 预置位对齐

拨盘一圈有 6 个弹位，相邻弹位为 `60 deg`。初始化参数：

```text
STIR_PRESET_ANGLE = 35 deg
temp_select_angle_d = 50 deg
```

`StirTargetAngleSet()` 的核心过程是：

```text
temp  = pos_d - 35
error = fmod(temp, 60)
根据 [-10, 50] deg 的非对称窗口调整 error
target = pos_d - error + quan_shu_r * 6120
```

所以目标点族本质上是：

```text
35 deg + k * 60 deg
```

阈值不是普通的最近点 `+/-30 deg`，而是偏向拨弹前进方向，允许最多约 `50 deg` 的负向移动或约 `10 deg` 的正向移动，避免退保护时为了找“最近点”反向顶住弹丸。

预置对齐主要在以下时机发生：

- 控制终端从 `CONTROL_STOP` 进入工作态，且已经收到拨盘反馈；
- 拨盘掉线后恢复，并满足恢复条件；
- 堵转恢复完成，或回位再次堵转后就地重对齐。

## 6. 拨弹目标生成

拨弹的正常方向是减小目标角度，堵转反转方向是增大目标角度。

### 6.1 一次拨弹

```text
stir_all_target_pos_d -= 60 deg
```

一次走完一个弹位，随后等待 `stir_mode == STIR_LOCK` 才允许下一次触发。

### 6.2 二次拨弹（默认）

```text
第一段：target -= 20 deg
到位误差 < 5 deg
等待 300 ms
第二段：target -= 40 deg
等待 stir_mode == STIR_LOCK
```

在 `sniper_on` 且不处于堵转/堵转恢复时：

- `ch1 > 0.1` 选择一次拨弹；
- `ch1 < -0.1` 选择二次拨弹；
- 位于死区内保持当前选择；
- 离开 sniper 后恢复默认二次拨弹。

### 6.3 目标下发

每次目标变化后：

```text
stir_all_target_pos_rad = stir_all_target_pos_d * PI / 180
```

`MotorControlCANSend()` 以 100 Hz 下发：

```text
ctrl_motor2(stir_all_target_pos_rad, stir_target_vol)
```

`stir_target_vol` 通常为 `300`。其准确物理单位需要以拨盘电机混合控制协议为准；代码只表明它作为第二个 `float` 直接发送。

## 7. 堵转观测和恢复

### 7.1 堵转判据

在 `ShootEstimateUpdate()` 中：

```text
abs(vel_radps) < 1.0 且 abs(toq) > 8.0 -> stall_count += 10
否则 stall_count > 0                    -> stall_count -= 1
stall_count >= 500                       -> stir_block_flag = 1
stall_count == 0                         -> stir_block_flag = 0
```

EstimateTask 由 2 ms IMU 通知链驱动时，连续满足约 50 个样本，即约 100 ms，会触发堵转。退出采用慢衰减而不是立即清零。

### 7.2 恢复状态机

```text
堵转上升沿
  -> 中止当前拨弹
  -> 保存堵转前目标
  -> 目标改为 当前连续反馈 + 20 deg
  -> 进入反转阶段

反转到位（误差 < 4 deg）
  -> 恢复堵转前目标
  -> 进入回位阶段

回位到位（误差 < 4 deg）
  -> StirTargetAngleSet() 重新对齐 60 deg 弹位
  -> 恢复结束
```

异常分支：

- 反转途中再次堵转：放弃继续反转，直接回原目标；
- 回位途中再次堵转：放弃原目标，在当前位置重新对齐弹位；
- 恢复期间强制保证电机为启动状态，避免普通堵转锁电逻辑阻止反转。

该状态机的所有到位判断都依赖 `stir_all_angle_d`。因此一旦多圈坐标在量程跨界或复位后不连续，堵转恢复也会误判。

## 8. 掉电与复位逻辑

### 8.1 当前如何判断拨盘掉线

`frame_counter` 每解析一帧 `0x018` 加一。`ShootEstimateUpdate()` 每 100 次执行检查一次计数器。如果检查点之间没有变化，就设置 `stir_reset_flag`。

若 EstimateTask 约为 500 Hz，检测窗口约为 200 ms。代码注释写“连续三个周期不变”，但当前赋值顺序使 `last_last_count` 始终等于 `last_count`，实际主要比较的是前后两个检查点，并没有保留三个独立历史样本。

设置 `stir_reset_flag` 时只清零：

- `stir_real_angle`
- `stir_real_angle_d`
- `stir_angle_last`
- `stir_angle_cur`
- `stir_real_angle_rad`

没有清零或重建：

- `quan_shu_r`
- `stir_all_angle_d`
- `stir_all_target_pos_d`
- `stall_count` 和堵转恢复阶段

而且随后仍会无条件调用一次 `GetStirRealAngle()`，使用最后一帧旧反馈重新写入刚清零的部分字段；在 `stir_reset_flag == 0` 时还会再调用一次。这里存在重复更新和复位不原子的现象。

### 8.2 掉线后的恢复条件

当前恢复条件是：

```text
stir_reset_flag != 0
且裁判系统 power_management_shooter_output 有效
且 frame_counter 相对上个检查点发生变化
```

满足后会：

1. 清除 `stir_reset_flag`；
2. 调用 `GetStirRealAngle()`；
3. 调用 `StirTargetAngleSet()` 重新生成目标。

`power_management_shooter_output` 只是裁判系统的发射机构供电许可，不等价于“拨盘电机坐标已经复位并稳定”。因此它不能单独作为角度观测器重建完成的依据。

## 9. 非同步复位为什么会异常

| 场景 | 软件状态 | 电机状态 | 主要风险 |
| --- | --- | --- | --- |
| 电机和 MCU 同时复位 | 软件计数、目标清零 | 电机坐标重新初始化 | 风险最低，但仍要等首帧并执行预置对齐 |
| 电机单独掉电，MCU 不复位 | `quan_shu_r` 和旧目标仍保留 | 电机内部位置原点/累计位置可能重置 | 新反馈与旧软件跨界计数不属于同一坐标纪元，目标可能突然偏移很多圈 |
| MCU 单独复位，电机不断电 | `quan_shu_r` 丢失为 0，目标先为 0 | 电机内部坐标和位置仍保留 | 软件失去此前的跨界历史；重新使能前若未及时按当前反馈对齐，可能先发送错误目标 |
| 只有 CAN 暂时中断 | 软件误认为电机下电并置 reset flag | 电机实际坐标未变 | 恢复路径会重对齐目标，可能改变原本有效的运动阶段 |

根本原因是当前系统没有“坐标纪元”或复位握手：主控无法区分电机是断电重启、通信短断，还是只丢了若干帧；电机也不知道主控是否丢失了软件多圈计数。

此外，`GetStirRealAngle()` 同时由 DecisionTask 的 `ShootInputUpdate()` 和 EstimateTask 的 `ShootEstimateUpdate()` 调用，EstimateTask 内部还可能连续调用两次。多任务共同修改 `stir_angle_last`、`quan_shu_r` 等观测状态，破坏了观测器应有的单写者约束，也增加了量程边界处重复计圈或漏计圈的风险。

## 10. 现阶段使用约束

在未重构观测器前，建议遵守以下流程：

1. 尽量让拨盘电机与主控 MCU 同步上电、同步复位，不单独重启其中一侧。
2. 上电后先保持 `CONTROL_STOP`，确认 `frame_counter` 持续增长且 `state` 正常。
3. 进入工作态后确认已经执行 `StirTargetAngleSet()`，并检查：

   ```text
   abs(stir_all_target_pos_d - stir_all_angle_d) < 5 deg
   ```

4. 确认 `stir_reset_flag == 0`、`stir_stall_recovery_state == 0`、`stir_block_flag == 0` 后再允许拨弹。
5. 若电机曾单独掉电、CAN 长时间断开或 MCU 单独复位，应重新进入保护态并让系统重新对齐；不要直接沿用掉线前目标继续发射。
6. 调试跨界时同时观察 `p_int`、`pos_d`、`stir_angle_last`、`quan_shu_r`、`stir_all_angle_d` 和 `stir_all_target_pos_d`，不能只看最终目标。

## 11. 推荐的后续重构

### 11.1 建立单一连续角度观测器

只允许 EstimateTask 更新观测状态。使用相邻样本差值解包：

```text
period = 12240 deg             // 需先通过实测确认
delta = raw_now - raw_last

if delta >  period / 2: delta -= period
if delta < -period / 2: delta += period

continuous += delta
raw_last = raw_now
```

DecisionTask 只读取 `continuous`，不得再次调用观测更新函数。

### 11.2 将反馈有效性作为显式状态

建议至少区分：

```text
STIR_OBSERVER_UNINITIALIZED
STIR_OBSERVER_VALID
STIR_OBSERVER_FEEDBACK_LOST
STIR_OBSERVER_REINITIALIZING
```

只有 `VALID` 状态允许拨弹和堵转恢复。首帧或恢复首帧应原子地完成：

```text
raw_last = raw_now
continuous = raw_now 或新的局部零点
target = AlignToPocket(continuous)
清除拨弹阶段和堵转阶段
```

### 11.3 明确复位策略

如果不需要跨重启保留绝对总圈数，最稳妥的是每次反馈重新上线都建立新的局部坐标系，并把目标对齐到当前反馈；不要保留旧 `quan_shu_r`。

如果必须跨电机/MCU 独立复位保留绝对总圈数，则需要至少一种额外信息：

- 电机提供不会随掉电丢失的绝对多圈位置；
- 主控在非易失存储中保存位置，并有可靠的电机复位计数；
- 双方通过启动序号/复位计数握手，明确是否处于同一坐标纪元。

仅依赖当前 16 bit 位置字段和 RAM 中的 `quan_shu_r`，无法无歧义恢复独立复位前的无限多圈位置。

### 11.4 清理现有重复和遗留状态

建议后续统一处理：

- 删除 `ShootEstimateUpdate()` 中重复的 `GetStirRealAngle()` 调用；
- 删除 DecisionTask 对观测器的写入；
- 明确 `stir_real_angle` 是否仍需要，不需要则移除；
- 修正掉线检测的历史采样逻辑；
- 复位时原子清理连续角度、目标、拨弹阶段和堵转阶段；
- 以反馈新鲜度或时间戳判断有效性，不把裁判系统供电许可当成电机坐标有效证明；
- 将 `DM_MOTO_MAX_ENCODE_D` 重命名为能表达“半量程”的名称，另设完整周期常量，避免再次把 `6120` 与 `12240` 混淆。

## 12. 调试观察表

| 观察量 | 正常表现 | 异常提示 |
| --- | --- | --- |
| `frame_counter` | 持续增长 | 不变表示反馈丢失；突变/归零可能是 MCU 重启 |
| `p_int` | 随位置连续变化，边界处回绕 | 固定、随机跳变或与 `pos_d` 不符 |
| `pos_d` | 位于 `[-6120, +6120]` | 超范围说明解码或内存异常 |
| `quan_shu_r` | 只在量程端点跨界时变化一次 | 普通运动中变化、一次跨界变化多次 |
| `stir_all_angle_d` | 正常运动中连续 | 跨界跳变说明补偿周期错误或漏/重复计数 |
| `stir_all_target_pos_d` | 预置时落在 `35 + 60k`，拨弹时按阶段减少 | 掉线恢复后突然偏移多圈 |
| `target - feedback` | 静止到位时绝对值小于约 `5 deg` | 长期很大时禁止继续拨弹 |
| `stir_reset_flag` | 反馈稳定时为 0 | 恢复反馈后仍不清除，检查裁判供电许可和计数逻辑 |
| `stir_block_flag` | 正常运动为 0 | 低速高转矩持续约 100 ms 后置 1 |
| `stir_stall_recovery_state` | 正常为 0 | 长时间停在 1/2，检查连续角度与到位误差 |

