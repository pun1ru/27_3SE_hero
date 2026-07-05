#include "MIT.h"
#include "general_task_include.h"

/* ==================================================================
 * 批量操作封装
 * ================================================================== */

void DM_MITControl_JointsSendTorq(FDCAN_HandleTypeDef* hcan, const float torq[4])
{
    static const uint16_t joint_ids[4] = {0x01, 0x02, 0x03, 0x04};
    for (uint8_t i = 0; i < 4; i++)
        DM_MITControl_Send(hcan, joint_ids[i], 0.0f, 0.0f, 0.0f, 0.0f, torq[i]);
}

void Motors_Start(FDCAN_HandleTypeDef* hcan, const uint16_t* ids, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
        start_motor(hcan, ids[i]);
}

void Motors_Lock(FDCAN_HandleTypeDef* hcan, const uint16_t* ids, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
        lock_motor(hcan, ids[i]);
}

void Motors_ClearError(FDCAN_HandleTypeDef* hcan, const uint16_t* ids, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
        clear_error(hcan, ids[i]);
}
