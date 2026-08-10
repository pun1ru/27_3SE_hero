#ifndef WORLD_GIMBAL_H_
#define WORLD_GIMBAL_H_

#include <stdint.h>

#define WORLD_GIMBAL_DISABLED 0U
#define WORLD_GIMBAL_ENABLED 1U

/**
 * @brief 世界系云台目标、估计和 IK 输出
 */
typedef struct {
    struct {
        float f_des_B[3];
        float last_right_B[3];
        uint8_t last_right_valid;
        uint8_t init_done;
    } WorldGimbalTargetInput;

    struct {
        float g_B[3];
        float chassis_roll_deg;
        float chassis_pitch_deg;
        float chassis_yaw_deg;
        float f_real_B[3];
        float b_B[3];
        float angle_error_deg;
        float world_yaw_deg;
        float world_pitch_deg;
    } WorldGimbalEstimate;

    struct {
        float q_yaw_cmd_deg;
        float q_pitch_cmd_deg;
        float q_yaw_cmd_rad;
        float q_pitch_cmd_rad;
        uint8_t converged;
        uint8_t iters_used;
    } WorldGimbalControl;

    uint8_t enable;
} WorldGimbal;

extern const WorldGimbal *const _worldGimbal;

/**
 * @brief   初始化世界系云台状态
 * @param   void
 * @retval  void
 */
void WorldGimbalInitialize(void);

/**
 * @brief   设置世界系云台使能状态
 * @param   is_enabled 使用 WORLD_GIMBAL_ENABLED 或 WORLD_GIMBAL_DISABLED
 * @retval  void
 */
void WorldGimbalSetEnabled(uint8_t is_enabled);

/**
 * @brief   将世界系目标对齐到当前云台指向
 * @param   void
 * @retval  void
 */
void WorldGimbalAlignToCurrent(void);

/**
 * @brief   按世界系角度增量更新虚拟目标
 * @param   yaw_delta_d yaw 增量，单位 deg
 * @param   pitch_delta_d pitch 增量，单位 deg
 * @retval  void
 */
void WorldGimbalInputUpdate(float yaw_delta_d, float pitch_delta_d);

/**
 * @brief   更新世界系云台观测
 * @param   void
 * @retval  void
 */
void WorldGimbalEstimateUpdate(void);

/**
 * @brief   使用阻尼最小二乘法求解两轴目标角
 * @param   void
 * @retval  void
 */
void WorldGimbalIKSolve(void);

/**
 * @brief   将 IK 输出写入云台控制目标
 * @param   void
 * @retval  void
 */
void WorldGimbalApplyToTargets(void);

/**
 * @brief   使用世界系绝对角覆盖虚拟目标
 * @param   world_yaw_d 世界系 yaw，单位 deg
 * @param   world_pitch_d 世界系 pitch，单位 deg
 * @retval  void
 */
void WorldGimbalSetWorldAngles(float world_yaw_d, float world_pitch_d);

#endif
