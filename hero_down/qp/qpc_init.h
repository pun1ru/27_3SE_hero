#ifndef QPC_INIT_H_
#define QPC_INIT_H_

/**
 * @file    qpc_init.h
 * @brief   QP/C 胶水层 — 把 QP 框架接入 hero_down 工程
 * @note    所有使用 QP 的文件只需 #include "qpc_init.h" 即可
 *
 * QP 三层结构速记：
 *   QEP (Event Processor)  → 状态机引擎    — qep_hsm.c / qep_msm.c
 *   QF  (Framework)        → AO 框架       — qf_*.c
 *   Port (FreeRTOS移植)    → 跟RTOS对接    — qf_port.c
 */

/*--- QP 框架入口 ---*/
#include "qpc.h"

/*--- 前向声明：QP 框架回调函数（必须在应用中实现） ---*/

/**
 * @brief  框架启动回调 — QF_run() 启动调度器之前调用
 * @note   在这里做硬件初始化 + 创建非AO的FreeRTOS任务
 *         类比：main() 函数里 vTaskStartScheduler() 之前的代码
 */
void QF_onStartup(void);

/**
 * @brief  框架清理回调 — QF_stop() 时调用
 * @note   正常运行时不会走到这里
 */
void QF_onCleanup(void);

/**
 * @brief  QP 错误处理 — 断言失败 / 严重错误时调用
 * @param  module  出错的模块名
 * @param  id      错误ID（通常是行号）
 * @note   在这里打日志、闪灯、或者死循环等看门狗复位
 */
void Q_onError(char const *module, int_t id);

/**
 * @brief  QP/C 框架初始化 — 在 InitTask 末尾调用
 * @note   内部创建事件池、订阅系统、构造并启动所有 AO
 */
void QpInit(void);

/*--- 全局 QP 任务句柄（AO 对应的 FreeRTOS Task） ---*/

#endif // QPC_INIT_H_
