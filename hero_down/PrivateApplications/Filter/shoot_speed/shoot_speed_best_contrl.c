#include "shoot_speed_best_contrl.h"

#include <string.h>

#define BULLET_SPEED_PROCESS_NOISE 0.006f
#define BULLET_SPEED_MEASUREMENT_NOISE 0.01f
#define BULLET_SPEED_INITIAL_MPS 15.75f
#define BULLET_SPEED_MIN_VARIANCE 0.001f
#define BULLET_SPEED_MIN_VALID_MPS 15.0f
#define BULLET_SPEED_MAX_VALID_MPS 16.0f

static KalmanFilter_t g_bullet_speed_kf;

void BulletKF_Init(void){
    const float state_transition[1] = {1.0f};
    const float process_noise[1] = {BULLET_SPEED_PROCESS_NOISE};
    const float measurement_matrix[1] = {1.0f};
    const float measurement_noise[1] = {BULLET_SPEED_MEASUREMENT_NOISE};
    const float initial_covariance[1] = {10.0f};
    const float minimum_variance[1] = {BULLET_SPEED_MIN_VARIANCE};

    Kalman_Filter_Init(&g_bullet_speed_kf, 1, 0, 1);
    g_bullet_speed_kf.UseAutoAdjustment = 0;

    memcpy(g_bullet_speed_kf.F_data, state_transition, sizeof(state_transition));
    memcpy(g_bullet_speed_kf.Q_data, process_noise, sizeof(process_noise));
    memcpy(g_bullet_speed_kf.H_data, measurement_matrix, sizeof(measurement_matrix));
    memcpy(g_bullet_speed_kf.R_data, measurement_noise, sizeof(measurement_noise));
    memcpy(g_bullet_speed_kf.P_data, initial_covariance, sizeof(initial_covariance));
    memcpy(g_bullet_speed_kf.StateMinVariance, minimum_variance, sizeof(minimum_variance));

    g_bullet_speed_kf.xhat_data[0] = BULLET_SPEED_INITIAL_MPS;
}

float BulletKF_Update(float measured_speed_mps){
    if((measured_speed_mps < BULLET_SPEED_MIN_VALID_MPS)
       || (measured_speed_mps > BULLET_SPEED_MAX_VALID_MPS)){
        return g_bullet_speed_kf.FilteredValue[0];
    }

    g_bullet_speed_kf.MeasuredVector[0] = measured_speed_mps;
    Kalman_Filter_Update(&g_bullet_speed_kf);
    return g_bullet_speed_kf.FilteredValue[0];
}
