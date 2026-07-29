#ifndef QPC_INIT_H_
#define QPC_INIT_H_

#include "qpc.h"

/**
 * @brief   QF_run 启动回调
 * @param   void
 * @retval  void
 */
void QF_onStartup(void);

/**
 * @brief   QF_stop 清理回调
 * @param   void
 * @retval  void
 */
void QF_onCleanup(void);

/**
 * @brief   处理 QP 断言失败
 * @param   module 断言所在模块
 * @param   id 断言编号
 * @retval  void
 */
void Q_onError(char const *module, int_t id);

/**
 * @brief   初始化 QP/C 并启动主动对象
 * @param   void
 * @retval  void
 */
void QpInit(void);

#endif
