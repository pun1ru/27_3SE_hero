/**
 * @file    system_idf.c
 * @brief   系统辨识 — 阶跃 / 扫频测试激励实现
 */

#include "system_idf.h"

/*--- 初始化 -----------------------------------------------------------------*/

/**
 * @brief   初始化系统辨识结构体
 */
void SysIDInit(volatile SysIDTest *sysid){
    sysid->step_torque_nm = 0.0f;
    sysid->step_duration_ms = 0;
    sysid->ch0_threshold = 0.1f;
    sysid->ch1_threshold = 0.1f;
    sysid->need_sniper = 1;
    sysid->elapsed_ms = 0;
    sysid->active = 0;
    sysid->done = 0;
    sysid->out_torque_nm = 0.0f;
    sysid->out_velocity_radps = 0.0f;
}

/*--- 阶跃输入 ----------------------------------------------------------------*/

/**
 * @brief   启动阶跃力矩测试
 */
void SysIDStepStart(volatile SysIDTest *sysid, float torque_nm, uint32_t duration_ms){
    sysid->step_torque_nm = torque_nm;
    sysid->step_duration_ms = duration_ms;
    sysid->elapsed_ms = 0;
    sysid->active = 1;
    sysid->done = 0;
    sysid->out_torque_nm = torque_nm;
}

/**
 * @brief   阶跃状态更新
 */
void SysIDStepUpdate(volatile SysIDTest *sysid, uint32_t dt_ms){
    if(!sysid->active){
        return;
    }

    sysid->elapsed_ms += dt_ms;

    if(sysid->elapsed_ms >= sysid->step_duration_ms){
        /* 阶跃到期：力矩归零，标记完成 */
        sysid->out_torque_nm = 0.0f;
        sysid->active = 0;
        sysid->done = 1;
    }
}

/**
 * @brief   查询阶跃是否进行中
 */
uint8_t SysIDStepActive(const volatile SysIDTest *sysid){
    return sysid->active;
}

/**
 * @brief   查询阶跃是否已完成
 */
uint8_t SysIDStepDone(const volatile SysIDTest *sysid){
    return sysid->done;
}
