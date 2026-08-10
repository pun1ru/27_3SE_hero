#ifndef QPC_INIT_H_
#define QPC_INIT_H_

#include "qpc.h"

/**
 * @brief   QF_run startup callback
 * @param   void
 * @retval  void
 */
void QF_onStartup(void);

/**
 * @brief   QF_stop cleanup callback
 * @param   void
 * @retval  void
 */
void QF_onCleanup(void);

/**
 * @brief   Handle a QP assertion failure
 * @param   module Module containing the assertion
 * @param   id Assertion identifier
 * @retval  void
 */
void Q_onError(char const* module, int_t id);

/**
 * @brief   Initialize the QP/C framework
 * @param   void
 * @retval  void
 * @note    Generated active-object pools, subscriptions and startup belong
 *          here after the QM model is generated.
 */
void QpInit(void);

#endif
