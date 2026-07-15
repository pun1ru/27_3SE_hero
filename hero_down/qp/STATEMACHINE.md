# DecisionAO 状态机

```mermaid
stateDiagram-v2
    [*] --> Protected

    state Protected {
        entry: CTRL_STOP, CHS_FOLLOW, SNIPER_OFF, FRIC_OFF, STIR_LOCK<br>JOINT_NORMAL, STAND_NORMAL, MOUSE_FIX_OFF, CAN_DISABLE

        SwitchR_up_SIG --> Protected: 自迁移
        RC_LOST_SIG --> Protected: 自迁移
        Switch_RC_SIG --> NormalMode: ctrl=CTRL_REMOTE
        Switch_PC_SIG --> NormalMode: ctrl=CTRL_PC

        state NormalMode {
            entry: CHS_FOLLOW, SNIPER_OFF, FRIC_OFF<br>STIR_ANGLE_CONTROL, JOINT_NORMAL<br>STAND_NORMAL, CAN_ENABLE

            Switch_protected_SIG --> Protected
            Switch_PC_SIG --> NormalMode: ctrl=CTRL_PC(不迁移)
            Switch_RC_SIG --> NormalMode: ctrl=CTRL_REMOTE(不迁移)
            SWITCH_SNIPER_SIG --> SniperMode
            SWITCH_STAND_SIG --> Stand_High
            SWITCH_REVOLVE_SIG --> Revolve
            SWITCH_STAIR_SIG --> PreStair
            SWITCH_BACK_SIG --> FollowBck

            state SniperMode {
                entry: SNIPER_ON / exit: SNIPER_OFF
                SWITCH_SNIPER_SIG --> NormalMode
                SWITCH_WORLD_SIG --> WorldGimbal

                state WorldGimbal {
                    entry: WORLD_ENABLE_ON / exit: WORLD_ENABLE_OFF
                    SWITCH_WORLD_SIG --> SniperMode
                }
            }

            state PreStair {
                entry: JOINT_PRESTAIR
                SWITCH_STAIR_SIG --> NormalMode
                STAIR_OK_SIG --> StairUp
            }

            state StairUp {
                entry: JOINT_STAIRUP / exit: JOINT_NORMAL
                SWITCH_STAIR_SIG --> NormalMode
            }

            state Stand_High {
                entry: STAND_HIGH / exit: STAND_NORMAL
                SWITCH_STAND_SIG --> NormalMode
            }

            state Revolve {
                entry: CHS_REVOLVE / exit: CHS_FOLLOW
                SWITCH_REVOLVE_SIG --> NormalMode
            }

            state FollowBck {
                entry: CHS_FOLLOW_BACK / exit: CHS_FOLLOW
                SWITCH_BACK_SIG --> NormalMode
            }
        }
    }
```

## 继承链

| 状态 | 超状态 |
|------|--------|
| Protected | QHsm_top |
| NormalMode | Protected |
| SniperMode | NormalMode |
| WorldGimbal | SniperMode |
| PreStair | NormalMode |
| StairUp | NormalMode |
| Stand_High | NormalMode |
| Revolve | NormalMode |
| FollowBck | NormalMode |

## 事件冒泡（RC_LOST 为例）

```
RC_LOST_SIG 从任意 NormalMode 子状态:
  → SniperMode 不处理 → Q_SUPER(NormalMode)
  → NormalMode 不处理 → Q_SUPER(Protected)
  → Protected 处理 → Q_TRAN(Protected) → 所有子状态 exit 执行 → 回到安全态
```

## TODO
按键状态处理,有发送的事件的键盘键位按照原来一样的发送事件
然后没有专门的事件的发送的就直接写入DecisionAO的状态机,比如fric模式,cam_target,mouse_fix,stir模式,新加的can_enable和disable让我手动写入,默认写disable告诉我在哪里写,然后RC_OK和RC_LOST的根据有没有收到遥控器信号来处理
我在想这和原来的状态处理有什么区别吗,有简化很多地方吗