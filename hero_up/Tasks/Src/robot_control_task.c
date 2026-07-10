/**
 * @file robot_control_task.c
 * @brief 【已废弃】所有代码已搬迁至三段式任务架构
 *
 * 代码搬迁映射：
 *   GimbalInputUpdate     → PrivateApplications/MainControl/gimbalControl.c
 *   GimbalPoseUpdate      → PrivateApplications/MainControl/gimbalControl.c
 *   GimbalInit            → PrivateApplications/MainControl/gimbalControl.c
 *   GimbalControlUpdate   → PrivateApplications/MainControl/gimbalControl.c
 *   GimbalEstimateUpdate  → PrivateApplications/MainControl/gimbalControl.c
 *   ShootInputUpdate      → PrivateApplications/MainControl/shootControl.c
 *   ShootControlUpdate    → PrivateApplications/MainControl/shootControl.c
 *   ShootEstimateUpdate   → PrivateApplications/MainControl/shootControl.c
 *   CRC16_Modbus          → PrivateApplications/MainControl/shootControl.c
 *   WorldGimbal (全部API)  → PrivateApplications/MainControl/worldGimbal.c
 *   ALLHighFreqCal        → 拆分至 GimbalControlUpdate + EstimateTask + ControlTask
 *   DecisionTask          → Tasks/Src/task_decision.c
 *   ControlTask           → Tasks/Src/task_control.c
 *
 * 新任务架构：
 *   IMUTask(p5,2ms) ──通知──▶ EstimateTask(p5,阻塞) ──通知──▶ ControlTask(p5,阻塞)
 *   DecisionTask(p5,10ms) 独立运行
 */

/* 此文件保留在工程中以确保 Keil 工程无缺失文件引用 */
#include "general_task_include.h"
