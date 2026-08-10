#include "kalman_filter.h"
/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif
#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

void BulletKF_Init(void);
float BulletKF_Update(float measured_speed);