#include "shoot_speed_best_contrl.h"
/* bullet_speed_kf.h */

#define BULLET_SPEED_Q 0.006f // 过程噪声
#define BULLET_SPEED_R 0.01f  // 测量噪声
#define INITIAL_SPEED 15.75f  // 初始速度估计
#define MIN_VARIANCE 0.001f   // 最小协方差

KalmanFilter_t BulletSpeed_KF;

/* 初始化函数 */
void BulletKF_Init(void){
    // 初始化滤波器维度 (1状态量，0控制量，1观测量)
    Kalman_Filter_Init(&BulletSpeed_KF, 1, 0, 1);
    /*状态量维度（1）
        我们只需要估计一个物理量：弹速本身。若需要建模加速度，可扩展为2维状态（速度+加速度）。

        控制量维度（0）
        弹速变化不受外部控制量（如推力）影响，因此不需要控制矩阵 B。什么叫做控制矩阵

        观测量维度（1）
        每个时刻仅通过传感器获得一个弹速测量值。
        */

    // 禁用自动调整功能
    BulletSpeed_KF.UseAutoAdjustment = 0;

    // 配置系统矩阵
    float F_init[1] = {1.0f}; // 状态转移矩阵
    float Q_init[1] = {BULLET_SPEED_Q};
    /*作用
    表示模型预测的不确定性。数值越大，滤波器对变化的响应越快。

    选择依据
    弹速波动范围小（15.5~16.0），模型应保持稳定：

    若弹速变化剧烈，需增大Q（例如0.1）
    若变化平缓，减小Q（例如0.001）*/
    float H_init[1] = {1.0f}; // 观测矩阵
                              /*物理意义
                              将状态量映射到观测量。此处直接测量弹速，因此：*/
    float R_init[1] = {BULLET_SPEED_R};
    /*表示传感器测量误差。数值越大，滤波器越信任预测值。*/
    float P_init[1] = {10.0f}; // 初始协方差
    /*作用
        表示初始状态估计的不确定性。较大的值使滤波器快速收敛。

        数学意义
        ，初始误差方差设为10，允许较大初始偏差。*/
    memcpy(BulletSpeed_KF.F_data, F_init, sizeof(F_init));
    memcpy(BulletSpeed_KF.Q_data, Q_init, sizeof(Q_init));
    memcpy(BulletSpeed_KF.H_data, H_init, sizeof(H_init));
    memcpy(BulletSpeed_KF.R_data, R_init, sizeof(R_init));
    memcpy(BulletSpeed_KF.P_data, P_init, sizeof(P_init));

    // 设置初始状态
    BulletSpeed_KF.xhat_data[0] = INITIAL_SPEED;

    // 配置最小协方差
    float min_var[1] = {MIN_VARIANCE}; // 避免协方差矩阵 P 过度趋近于零，导致滤波器失去对状态变化的敏感性。
    memcpy(BulletSpeed_KF.StateMinVariance, min_var, sizeof(min_var));
}
#define MIN_VALID_SPEED 15.0
#define MAX_VALID_SPEED 16.0
/* 更新函数 */
float BulletKF_Update(float measured_speed){
    // 1. 数据有效性检查
    if(measured_speed < MIN_VALID_SPEED || measured_speed > MAX_VALID_SPEED){
        // 运行到这一行的话，你的发射机构完蛋了！
        return BulletSpeed_KF.FilteredValue[0];
    }

    // 更新测量值
    BulletSpeed_KF.MeasuredVector[0] = measured_speed;

    // 执行卡尔曼滤波
    Kalman_Filter_Update(&BulletSpeed_KF);

    // 返回最优估计值
    return BulletSpeed_KF.FilteredValue[0];
}

/******************** 使用示例 ********************/
// int main()
//{
//     BulletKF_Init();
//
//     // 模拟弹速测量数据 (15.5-16.0范围波动)
//     float test_data[] = {15.6, 15.8, 15.7, 15.9, 15.8, 16.0};
//
//     for(int i=0; i<sizeof(test_data)/sizeof(float); i++){
//         float est = BulletKF_Update(test_data[i]);
//         printf("Measured: %5.2f  Estimated: %5.2f\n", test_data[i], est);
//     }
//
//     return 0;
// }
