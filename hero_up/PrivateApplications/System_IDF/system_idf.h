#ifndef _SYSTEM_IDF_H_
#define _SYSTEM_IDF_H_

#include "general_task_include.h"

/* ── 系统辨识测试总开关（注释即关闭）── */
/* ── 系统辨识测试总开关（注释即关闭）── */
//#define TEST_YAW

/**
 * @brief 系统辨识测试结构体
 * @note  用于向控制对象注入阶跃/扫频等测试激励，记录响应数据完成辨识
 */
typedef struct
{
    /* ── 阶跃配置 ── */
    float     step_torque_nm;       /**< 阶跃力矩幅值 (Nm) */
    uint32_t  step_duration_ms;     /**< 阶跃持续时间 (ms) */

    /* ── 触发条件 ── */
    float     ch0_threshold;        /**< ch0 触发阈值 */
    float     ch1_threshold;        /**< ch1 备用触发阈值 */
    uint8_t   need_sniper;          /**< 是否需要狙击模式: 1=是, 0=否 */

    /* ── 运行时状态 ── */
    uint32_t  elapsed_ms;           /**< 阶跃已运行时间 (ms) */
    uint8_t   active;               /**< 阶跃进行中 */
    uint8_t   done;                 /**< 本次阶跃已完成（保持done=1直到外部复位） */

    /* ── 当前输出（供调试帧读取） ── */
    float     out_torque_nm;        /**< 当前输出力矩 (Nm) */
    float     out_velocity_radps;   /**< 当前输出速度 (rad/s) */
} SysIDTest;

/**
 * @brief   初始化系统辨识结构体
 * @param   sysid  SysIDTest 指针
 * @retval  void
 */
void SysIDInit(volatile SysIDTest* sysid);

/**
 * @brief   启动阶跃力矩测试
 * @param   sysid       SysIDTest 指针
 * @param   torque_nm   阶跃力矩幅值 (Nm)
 * @param   duration_ms 阶跃持续时间 (ms)
 * @retval  void
 * @note    调用后每个控制周期调用 SysIDStepUpdate() 计时，
 *          duration_ms 后自动结束阶跃
 */
void SysIDStepStart(volatile SysIDTest* sysid, float torque_nm, uint32_t duration_ms);

/**
 * @brief   阶跃状态更新（每个控制周期调用一次）
 * @param   sysid  SysIDTest 指针
 * @param   dt_ms  本周期时间增量 (ms)
 * @retval  void
 * @note    阶跃激活期间 out_torque_nm = step_torque_nm，
 *          到期后清零并置 active=0, done=1
 */
void SysIDStepUpdate(volatile SysIDTest* sysid, uint32_t dt_ms);

/**
 * @brief   查询阶跃是否进行中
 * @param   sysid  SysIDTest 指针
 * @retval  1=进行中, 0=未激活
 */
uint8_t SysIDStepActive(const volatile SysIDTest* sysid);

/**
 * @brief   查询阶跃是否已完成
 * @param   sysid  SysIDTest 指针
 * @retval  1=已完成, 0=未完成
 */
uint8_t SysIDStepDone(const volatile SysIDTest* sysid);

#endif
