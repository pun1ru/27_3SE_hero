#ifndef SHOOT_SPEED_BEST_CONTROL_H_
#define SHOOT_SPEED_BEST_CONTROL_H_

#include "kalman_filter.h"

/**
 * @brief   初始化弹速卡尔曼滤波器
 * @retval  void
 */
void BulletKF_Init(void);

/**
 * @brief   更新弹速滤波结果
 * @param   measured_speed_mps 裁判系统测得的弹速，单位 m/s
 * @retval  滤波后的弹速，单位 m/s
 */
float BulletKF_Update(float measured_speed_mps);

#endif
