#ifndef _WORLD_GIMBAL_H_
#define _WORLD_GIMBAL_H_

#include <stdint.h>

/* ====================================================================================================================
 * 世界系云台控制结构体 WorldGimbal
 * 原理：维护虚拟目标指向 f_des_B（在机体坐标系B中），用户yaw指令绕大地竖直方向 g_B 旋转 f_des_B，
 *       pitch指令绕虚拟水平右轴旋转 f_des_B，再用阻尼最小二乘IK反解真实yaw/pitch电机角。
 * 坐标系：B系 x=前 y=右 z=下（右手系）
 * 旋转顺序：Z-Y-X (Yaw→Pitch→Roll)，与 rotation_Martix.h 一致
 * ==================================================================================================================== */
#define WORLDGIMBAL_IK_MAX_ITERS      10
#define WORLDGIMBAL_IK_LAMBDA         0.01f
#define WORLDGIMBAL_IK_MAX_STEP_RAD   0.05f
#define WORLDGIMBAL_IK_CONVERGE_RAD   0.0005f

typedef struct
{
    struct
    {
        float f_des_B[3];
        float last_right_B[3];
        uint8_t last_right_valid;
        uint8_t init_done;
    } WorldGimbalTargetInput;

    struct
    {
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

    struct
    {
        float q_yaw_cmd_deg;
        float q_pitch_cmd_deg;
        float q_yaw_cmd_rad;
        float q_pitch_cmd_rad;
        uint8_t converged;
        uint8_t iters_used;
    } WorldGimbalControl;

    uint8_t enable;
} WorldGimbal;

/* API */
void WorldGimbalInit(WorldGimbal* wg);
void WorldGimbalAlignToCurrent(WorldGimbal* wg);
void WorldGimbalInputUpdate(WorldGimbal* wg, float dyaw_deg, float dpitch_deg);
void WorldGimbalEstimateUpdate(WorldGimbal* wg);
void WorldGimbalIKSolve(WorldGimbal* wg);
void WorldGimbalApplyToTargets(WorldGimbal* wg);
void WorldGimbalSetWorldAngles(WorldGimbal* wg, float world_yaw_deg, float world_pitch_deg);

extern WorldGimbal worldGimbal;
extern const WorldGimbal* _worldGimbal;

#endif
