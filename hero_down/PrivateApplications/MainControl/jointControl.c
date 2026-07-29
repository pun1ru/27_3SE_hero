#include "jointControl.h"
#include "general_task_include.h"
/* 全局实例 */
JointControl jointControl = {0};
const JointControl* _jointControl = &jointControl;

extern const DMJ4310MotorRec* _jointMotorEec;
extern float distance;
extern Pose gimbalPose;
float k_toq_sign_map[JOINT_CTRL_MOTOR_NUM] = {1.0f, -1.0f, -1.0f, 1.0f};
static const QEvt stair_detected_event = {.sig = STAIR_OK_SIG};
static uint8_t stair_detected_event_posted;

matrix_data_t g_joint_rot_leg_to_body_data[9] = {0};
matrix_data_t g_joint_rot_body_to_imu_data[9] = {0};
matrix_data_t g_joint_rot_imu_to_world_data[9] = {0};

matrix_t g_joint_rot_leg_to_body;
matrix_t g_joint_rot_body_to_imu;
matrix_t g_joint_rot_imu_to_world;

matrix_data_t g_joint_leg_jacobian_dpos_dtheta_data[JOINT_CTRL_LEG_NUM][3] = {0};
matrix_t g_joint_leg_jacobian_dpos_dtheta[JOINT_CTRL_LEG_NUM];

/* 各腿在机身坐标系下的名义安装原点，用于把四连杆解算结果映射到机身位置。 */
static const float g_joint_leg_origin_body_default_m[JOINT_CTRL_LEG_NUM][JOINT_CTRL_AXIS_NUM] = {
	{0.24345f, 0.17475f, -0.16122f},
	{0.24345f, -0.17475f, -0.16122f},
	{-0.10970f, -0.16475f, -0.25150f},
	{-0.10970f, 0.16475f, -0.25150f},
};
/* 接触判定用的力矩阈值，低于该值时认为关节处于空载/非接触状态。 */
static const float g_joint_contact_torque_threshold_nm = 0.002f;
/* 轮端等效半径，用于把四连杆末端中心位置换算到轮面接触高度。 */
static const float g_joint_wheel_radius_m = 0.080f;
//现在后腿参数需要部份改变,后腿安装点坐标x,y,z绝对值变成109.7,164.75,251.5,
// 然后后腿电机端坐标值x是安装点z-132.54,坐标值z是安装点z+23.68,
typedef struct
{
	/* 四连杆几何参数：连杆长度、关节偏置以及分支选择符号。 */
	float mx_m;
	float mz_m;
	float l_oa_m;
	float l_ab_m;
	float l_mb_m;
	float l_ow_m;
	float branch_sign;
} JointFourBarParam;

/* 前腿四连杆几何标定参数。 */
static const JointFourBarParam g_joint_param_front = {
	.mx_m = -0.04234f,
	.mz_m = 0.03067f,
	.l_oa_m = 0.05000f,
	.l_ab_m = 0.05000f,
	.l_mb_m = 0.06000f,
	.l_ow_m = 0.11650f,
	.branch_sign = 1.0f,
};

/*
 * 后腿四连杆几何标定参数：
 * - 安装原点：|x|=109.7 mm, |y|=164.75 mm, |z|=251.5 mm
 * - 电机端偏置：x 偏置 -132.54 mm, z 偏置 +23.68 mm
 * - 杆长：OA=75 mm, AB=60 mm, MB=105.24 mm, OW=139 mm
 */
static const JointFourBarParam g_joint_param_rear = {
	.mx_m = -0.13254f,
	.mz_m = 0.02368f,
	.l_oa_m = 0.07500f,
	.l_ab_m = 0.10524f,
	.l_mb_m = 0.06000f,
	.l_ow_m = 0.15100f,
	.branch_sign = 1.0f,
};

static const float g_joint_body_com_m[JOINT_CTRL_AXIS_NUM] = {0.0f, 0.0f, -0.150f};
static const float g_joint_track_contact_body_m[JOINT_CTRL_AXIS_NUM] = {0.19211f, 0.0f, -0.1995f};

float g_joint_motor_torque_cmd_nm[JOINT_CTRL_MOTOR_NUM] = {0};

float g_joint_motor_torque_fdb_nm[JOINT_CTRL_MOTOR_NUM] = {0};

float g_joint_leg_end_effector_angle_rad[JOINT_CTRL_LEG_NUM] = {0};

float g_joint_control_dt_s = 0.001f;
uint8_t g_joint_control_enable = 0;
int g_joint_state = JOINT_STOP;
JointForceControlContext g_joint_force_ctrl = {0};
JointBodyState g_joint_body_state_body_dbg = {0};
static uint8_t g_joint_mode = JOINT_NORMAL;
static float g_joint_climb_pitch_hold_d = 20.0f;
static uint8_t g_joint_stand_mode = JOINT_STAND_MODE_NORMAL;
static uint8_t g_joint_jump_mode = JOINT_JUMP_MODE_OFF;
static float g_joint_climb_pitch_i_nm = 0.0f;
float g_joint_climb_rear_target_angle_rad = 0.0f;
LTD g_joint_climb_rear_target_ltd = {0};

static float g_joint_motor_angle_rad[JOINT_CTRL_MOTOR_NUM] = {0};

static const float g_joint_motor_torque_min_nm[JOINT_CTRL_MOTOR_NUM] = {
	-10.0f, -10.0f, -0.0f, -0.0f,
};
static const float g_joint_motor_torque_max_nm[JOINT_CTRL_MOTOR_NUM] = {
	0.0f, 0.0f, 7.0f, 7.0f,
};

static const float g_joint_motor_angle_min_rad[JOINT_CTRL_MOTOR_NUM] = {
	-2.70f, -2.70f, -0.25f, -0.25f,//-0.70
};
static const float g_joint_motor_angle_max_rad[JOINT_CTRL_MOTOR_NUM] = {
	-1.90f, -1.90f, 2.00f, 2.10f,
};

void JointGetMotorAngleLimitsRad(float angle_min_rad_out[JOINT_CTRL_MOTOR_NUM],
					 float angle_max_rad_out[JOINT_CTRL_MOTOR_NUM])
{
	for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
	{
		angle_min_rad_out[i] = g_joint_motor_angle_min_rad[i];
		angle_max_rad_out[i] = g_joint_motor_angle_max_rad[i];
	}
}

static const float g_joint_torque_to_angle_sign[JOINT_CTRL_MOTOR_NUM] = {
	1.0f, 1.0f, 1.0f, 1.0f,
};

static float JointClamp(float x, float low, float high)
{
	if (x < low)
		return low;
	if (x > high)
		return high;
	return x;
}

static float JointLpf(float prev, float cur, float alpha)
{
	return alpha * prev + (1.0f - alpha) * cur;
}

static float JointWrapToPi(float x)
{
	return atan2f(sinf(x), cosf(x));
}

static void JointMat3Mul(const float a[3][3], const float b[3][3], float out[3][3])
{
	for (uint8_t r = 0; r < 3; r++)
	{
		for (uint8_t c = 0; c < 3; c++)
		{
			out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
		}
	}
}

static void JointCreateRotationMatrixRpyDeg(float yaw_d,
					    float pitch_d,
					    float roll_d,
					    float r[3][3])
{
	const float deg_to_rad = 0.017453292519943295f;
	const float yaw = yaw_d * deg_to_rad;
	const float pitch = pitch_d * deg_to_rad;
	const float roll = roll_d * deg_to_rad;
	const float cy = cosf(yaw);
	const float sy = sinf(yaw);
	const float cp = cosf(pitch);
	const float sp = sinf(pitch);
	const float cr = cosf(roll);
	const float sr = sinf(roll);
	r[0][0] = cy * cp;
	r[0][1] = cy * sp * sr - sy * cr;
	r[0][2] = cy * sp * cr + sy * sr;
	r[1][0] = sy * cp;
	r[1][1] = sy * sp * sr + cy * cr;
	r[1][2] = sy * sp * cr - cy * sr;
	r[2][0] = -sp;
	r[2][1] = cp * sr;
	r[2][2] = cp * cr;
}

static void JointExtractEulerRpyDeg(const float r[3][3],
				    float* yaw_d,
				    float* pitch_d,
				    float* roll_d)
{
	const float rad_to_deg = 57.29577951308232f;
	float pitch = asinf(JointClamp(-r[2][0], -1.0f, 1.0f));
	float cp = cosf(pitch);
	float yaw;
	float roll;

	if (fabsf(cp) > 1e-6f)
	{
		yaw = atan2f(r[1][0], r[0][0]);
		roll = atan2f(r[2][1], r[2][2]);
	}
	else
	{
		yaw = atan2f(-r[0][1], r[1][1]);
		roll = 0.0f;
	}

	*yaw_d = yaw * rad_to_deg;
	*pitch_d = pitch * rad_to_deg;
	*roll_d = roll * rad_to_deg;
}

static void JointRotateImuStateToBody(const JointBodyState* imu_state,
					  JointBodyState* body_state_out)
{
	const float imu_ref_yaw_d = 0.0f;
	const float imu_ref_pitch_d = -15.25f;
	const float imu_ref_roll_d = 0.0f;
	/* IMU 安装方向的名义旋转矩阵，后面会结合首帧参考姿态再做一次补偿标定。 */
	static const float r_bi[3][3] = {
		{-0.965925813f, 0.0f, -0.258819045f},
		{0.0f, -1.0f, 0.0f},
		{-0.258819045f, 0.0f, 0.965925813f},
	};
	static uint8_t calib_inited = 0u;
	static float r_ib_cal[3][3] = {0};
	static float r_bi_cal[3][3] = {0};
	float r_wi[3][3];
	float r_wi_ref[3][3];
	float r_ib_nom[3][3];
	float r_wb_ref[3][3];
	float r_bias[3][3];
	float temp[3][3];
	float r_wb[3][3];
	float rate_i[3];
	float rate_b[3] = {0.0f, 0.0f, 0.0f};

	if (imu_state == NULL || body_state_out == NULL)
	{
		return;
	}

	JointCreateRotationMatrixRpyDeg(imu_state->yaw_d,
					imu_state->pitch_d,
					imu_state->roll_d,
					r_wi);

	for (uint8_t r = 0; r < 3; r++)
	{
		for (uint8_t c = 0; c < 3; c++)
		{
			r_ib_nom[r][c] = r_bi[c][r];
		}
	}

	if (!calib_inited)
	{
		/* 首帧用参考姿态把 IMU 坐标系和机身坐标系对齐，得到固定安装偏差补偿。 */
		JointCreateRotationMatrixRpyDeg(imu_ref_yaw_d,
						imu_ref_pitch_d,
						imu_ref_roll_d,
						r_wi_ref);
		JointMat3Mul(r_wi_ref, r_ib_nom, r_wb_ref);

		for (uint8_t r = 0; r < 3; r++)
		{
			for (uint8_t c = 0; c < 3; c++)
			{
				r_bias[r][c] = r_wb_ref[c][r];
			}
		}

		JointMat3Mul(r_ib_nom, r_bias, temp);
		for (uint8_t r = 0; r < 3; r++)
		{
			for (uint8_t c = 0; c < 3; c++)
			{
				r_ib_cal[r][c] = temp[r][c];
				r_bi_cal[r][c] = temp[c][r];
			}
		}

		calib_inited = 1u;
	}

	JointMat3Mul(r_wi, r_ib_cal, r_wb);
	JointExtractEulerRpyDeg(r_wb,
				&body_state_out->yaw_d,
				&body_state_out->pitch_d,
				&body_state_out->roll_d);
	/* 机身俯仰正方向与 IMU 输出符号相反，这里统一到控制侧约定。 */
	body_state_out->pitch_d = -body_state_out->pitch_d;

	rate_i[0] = imu_state->roll_rate_dps;
	rate_i[1] = imu_state->pitch_rate_dps;
	rate_i[2] = imu_state->yaw_rate_dps;

	for (uint8_t r = 0; r < 3; r++)
	{
		for (uint8_t c = 0; c < 3; c++)
		{
			rate_b[r] += r_bi_cal[r][c] * rate_i[c];
		}
	}

	body_state_out->roll_rate_dps = rate_b[0];
	body_state_out->pitch_rate_dps = -rate_b[1];
	body_state_out->yaw_rate_dps = rate_b[2];

	/* 将世界系运动加速度（MotionAccel_n，已去重力）旋转到机体系 */
	{
		float accel_w[3];
		float accel_b[3] = {0.0f, 0.0f, 0.0f};

		accel_w[0] = imu_state->accel_x;
		accel_w[1] = imu_state->accel_y;
		accel_w[2] = imu_state->accel_z;

		/* r_wb: 机体系→世界系, 其转置 r_wb^T: 世界系→机体系 */
		for (uint8_t r = 0; r < 3; r++)
		{
			for (uint8_t c = 0; c < 3; c++)
			{
				accel_b[r] += r_wb[c][r] * accel_w[c];
			}
		}

		body_state_out->accel_x = accel_b[0];
		body_state_out->accel_y = accel_b[1];
		body_state_out->accel_z = accel_b[2];
	}
}

void JointForceControlConvertBodyState(const JointBodyState* body_state_in,
				       JointBodyState* body_state_out)
{
	JointRotateImuStateToBody(body_state_in, body_state_out);
}

/* ===== 电机角度 ↔ 反馈位置 变换共享参数（保证正反变换严格互逆） ===== */
static float  g_joint_fb_sign[JOINT_CTRL_MOTOR_NUM] = {
	-1.0f, 1.0f, 1.0f, -1.0f,
};
static float  g_joint_fb_zero_offset_rad[JOINT_CTRL_MOTOR_NUM] = {
	0.13439703f, -0.0965452194f, -0.125964165f, 0.0433616638f,
};
static uint8_t g_joint_fb_offset_inited = 0u;

void JointBuildMotorAnglesRadFromFeedback(const float feedback_pos_rad[JOINT_CTRL_MOTOR_NUM],
					  float motor_angles_rad_out[JOINT_CTRL_MOTOR_NUM])
{
	/* 电机反馈转关节角的方向约定：先做方向翻转，再减去标定零位。 */

	static const float k_front_motor_ref_rad = -2.63370184126f; /* -150.9 deg */
	static const float k_rear_motor_ref_rad = 1.57079632679f;   /* +90.0 deg */

	static const uint8_t k_enable_front_auto_offset = 1u;
	static const uint8_t k_enable_rear_auto_offset = 1u;
	static const float k_front_encoder_ref_rad[2] = {
		0.0645391941f, -1.05366182f, 
	};
	static const float k_rear_encoder_ref_rad[2] = {
		0.147525072f, 1.90201545f,   
	};

	if (!g_joint_fb_offset_inited)
	{
		if (k_enable_front_auto_offset)
		{
			/* 前腿用标定参考角和编码器参考值反推零位偏置。 */
			g_joint_fb_zero_offset_rad[LEG_LF] =
				k_front_encoder_ref_rad[0] - g_joint_fb_sign[LEG_LF] * k_front_motor_ref_rad;
			g_joint_fb_zero_offset_rad[LEG_RF] =
				k_front_encoder_ref_rad[1] - g_joint_fb_sign[LEG_RF] * k_front_motor_ref_rad;
		}

		if (k_enable_rear_auto_offset)
		{
			/* 后腿同理：以电机端 +90° 时的编码器读数作为新的零位参考。 */
			g_joint_fb_zero_offset_rad[LEG_RB] =
				k_rear_encoder_ref_rad[0] - g_joint_fb_sign[LEG_RB] * k_rear_motor_ref_rad;
			g_joint_fb_zero_offset_rad[LEG_LB] =
				k_rear_encoder_ref_rad[1] - g_joint_fb_sign[LEG_LB] * k_rear_motor_ref_rad;
		}
		g_joint_fb_offset_inited = 1u;
	}

	for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
	{
		motor_angles_rad_out[i] = JointWrapToPi(g_joint_fb_sign[i] * (feedback_pos_rad[i] - g_joint_fb_zero_offset_rad[i]));
	}
}

static void JointGetMotorMapParam(float joint_motor_sign_out[JOINT_CTRL_MOTOR_NUM],
				  float joint_motor_zero_offset_rad_out[JOINT_CTRL_MOTOR_NUM])
{
	/* 直接读取 JointBuildMotorAnglesRadFromFeedback 计算出的共享参数，保证正反变换严格互逆。 */
	for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
	{
		joint_motor_sign_out[i] = g_joint_fb_sign[i];
		joint_motor_zero_offset_rad_out[i] = g_joint_fb_zero_offset_rad[i];
	}
}

void JointBuildFeedbackPosRadFromMotorAngles(const float motor_angles_rad_in[JOINT_CTRL_MOTOR_NUM],
					     float feedback_pos_rad_out[JOINT_CTRL_MOTOR_NUM])
{
	float k_joint_motor_sign[JOINT_CTRL_MOTOR_NUM] = {0};
	float k_joint_motor_zero_offset_rad[JOINT_CTRL_MOTOR_NUM] = {0};
	JointGetMotorMapParam(k_joint_motor_sign, k_joint_motor_zero_offset_rad);

	for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
	{
		feedback_pos_rad_out[i] = k_joint_motor_sign[i] * motor_angles_rad_in[i] + k_joint_motor_zero_offset_rad[i];
	}
}

float JointGetClimbRearTargetAngleRad(void)
{
	return g_joint_climb_rear_target_angle_rad;
}

static uint8_t JointSolveWheelCenterJacobian(const JointFourBarParam* param,
					      float theta_rad,
					      float* cx_m,
					      float* cz_m,
					      float* dcx_dtheta,
					      float* dcz_dtheta)
{
	const float eps = 1e-7f;
	float bx = param->mx_m + param->l_mb_m * cosf(theta_rad);
	float bz = param->mz_m + param->l_mb_m * sinf(theta_rad);
	float d = sqrtf(bx * bx + bz * bz);
	float oa2 = param->l_oa_m * param->l_oa_m;
	float ab2 = param->l_ab_m * param->l_ab_m;

	if (d < eps)
		return 0;

	{
		float a = (oa2 - ab2 + d * d) / (2.0f * d);
		float h2 = oa2 - a * a;
		float ux = bx / d;
		float uz = bz / d;
		float nx = -uz;
		float nz = ux;
		float h;
		float ax;
		float az;
		float dbx = -param->l_mb_m * sinf(theta_rad);
		float dbz = param->l_mb_m * cosf(theta_rad);
		float rhs2;
		float det;
		float dax_dtheta;
		float daz_dtheta;
		float scale = param->l_ow_m / param->l_oa_m;

		if (h2 < 0.0f)
		{
			if (h2 > -1e-8f)
				h2 = 0.0f;
			else
				return 0;
		}

		h = param->branch_sign * sqrtf(h2);
		ax = a * ux + h * nx;
		az = a * uz + h * nz;

		rhs2 = (ax - bx) * dbx + (az - bz) * dbz;
		det = az * bx - ax * bz;

		if (fabsf(det) < eps)
			return 0;

		dax_dtheta = (-az * rhs2) / det;
		daz_dtheta = (ax * rhs2) / det;

		if (cx_m != NULL)
			*cx_m = scale * ax;
		if (cz_m != NULL)
			*cz_m = scale * az;
		if (dcx_dtheta != NULL)
			*dcx_dtheta = scale * dax_dtheta;
		if (dcz_dtheta != NULL)
			*dcz_dtheta = scale * daz_dtheta;
	}

	return 1;
}
void JointUpdateLegPoseFromMotorAngle(void)
{
	for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
	{
		const JointFourBarParam* param = (i == LEG_LF || i == LEG_RF) ? &g_joint_param_front : &g_joint_param_rear;
		float cx_m = 0.0f;
		float cz_m = 0.0f;
		uint8_t ok = JointSolveWheelCenterJacobian(param,
						     g_joint_motor_angle_rad[i],
						     &cx_m,
						     &cz_m,
						     NULL,
						     NULL);

		if (ok)
		{
			g_joint_force_ctrl.leg_pos_body_m[i][0] = g_joint_leg_origin_body_default_m[i][0] + cx_m;
			g_joint_force_ctrl.leg_pos_body_m[i][1] = g_joint_leg_origin_body_default_m[i][1];
			g_joint_force_ctrl.leg_pos_body_m[i][2] = g_joint_leg_origin_body_default_m[i][2] + cz_m - g_joint_wheel_radius_m;
			g_joint_leg_end_effector_angle_rad[i] = atan2f(cz_m, cx_m);
		}
		else
		{
			g_joint_force_ctrl.leg_pos_body_m[i][0] = g_joint_leg_origin_body_default_m[i][0];
			g_joint_force_ctrl.leg_pos_body_m[i][1] = g_joint_leg_origin_body_default_m[i][1];
			g_joint_force_ctrl.leg_pos_body_m[i][2] = g_joint_leg_origin_body_default_m[i][2] - g_joint_wheel_radius_m;
			g_joint_leg_end_effector_angle_rad[i] = 0.0f;
		}
	}
}

void JointUpdateLegJacobiansFromMotorAngle(void)
{
	for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
	{
		const JointFourBarParam* param = (i == LEG_LF || i == LEG_RF) ? &g_joint_param_front : &g_joint_param_rear;
		float dpx = 0.0f;
		float dpz = 0.0f;
		uint8_t ok = JointSolveWheelCenterJacobian(param,
						     g_joint_motor_angle_rad[i],
						     NULL,
						     NULL,
						     &dpx,
						     &dpz);
		if (ok)
		{
			g_joint_leg_jacobian_dpos_dtheta_data[i][0] = dpx;
			g_joint_leg_jacobian_dpos_dtheta_data[i][1] = 0.0f;
			g_joint_leg_jacobian_dpos_dtheta_data[i][2] = dpz;
		}
		else
		{
			g_joint_leg_jacobian_dpos_dtheta_data[i][0] = 0.0f;
			g_joint_leg_jacobian_dpos_dtheta_data[i][1] = 0.0f;
			g_joint_leg_jacobian_dpos_dtheta_data[i][2] = 0.0f;
		}
	}
}

void JointEstimateBodyHeightVelocity(const JointBodyState* body_state,
				     const float joint_vel_radps[JOINT_CTRL_MOTOR_NUM],
				     float* body_height_m_out,
				     float* body_height_vel_mps_out)
{
	const float deg_to_rad = 0.017453292519943295f;
	float roll_rad;
	float pitch_rad;
	float sin_roll;
	float cos_roll;
	float sin_pitch;
	float cos_pitch;
	float r31;
	float r32;
	float r33;
	float wx;
	float wy;
	float wz;
	float h_sum = 0.0f;
	float hdot_sum = 0.0f;
	uint8_t contact_cnt = 0u;

	if (body_state == NULL || joint_vel_radps == NULL || body_height_m_out == NULL || body_height_vel_mps_out == NULL)
		return;

	roll_rad = body_state->roll_d * deg_to_rad;
	pitch_rad = body_state->pitch_d * deg_to_rad;
	sin_roll = sinf(roll_rad);
	cos_roll = cosf(roll_rad);
	sin_pitch = sinf(pitch_rad);
	cos_pitch = cosf(pitch_rad);

	r31 = -sin_pitch;
	r32 = cos_pitch * sin_roll;
	r33 = cos_pitch * cos_roll;

	wx = body_state->roll_rate_dps * deg_to_rad;
	wy = body_state->pitch_rate_dps * deg_to_rad;
	wz = body_state->yaw_rate_dps * deg_to_rad;

	for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
	{
		if (!g_joint_force_ctrl.contact_flag[i])
			continue;

		{
			const float px = g_joint_force_ctrl.leg_pos_body_m[i][0];
			const float py = g_joint_force_ctrl.leg_pos_body_m[i][1];
			const float pz = g_joint_force_ctrl.leg_pos_body_m[i][2];
			const float jx = g_joint_leg_jacobian_dpos_dtheta_data[i][0];
			const float jy = g_joint_leg_jacobian_dpos_dtheta_data[i][1];
			const float jz = g_joint_leg_jacobian_dpos_dtheta_data[i][2];
			const float theta_dot = joint_vel_radps[i];
			const float pdx = jx * theta_dot;
			const float pdy = jy * theta_dot;
			const float pdz = jz * theta_dot;
			const float cx = wy * pz - wz * py;
			const float cy = wz * px - wx * pz;
			const float cz = wx * py - wy * px;
			const float vx = cx + pdx;
			const float vy = cy + pdy;
			const float vz = cz + pdz;
			const float hi = -(r31 * px + r32 * py + r33 * pz);
			const float hdot_i = -(r31 * vx + r32 * vy + r33 * vz);

			h_sum += hi;
			hdot_sum += hdot_i;
			contact_cnt++;
		}
	}

	if (contact_cnt > 0u)
	{
		*body_height_m_out = h_sum / (float)contact_cnt;
		*body_height_vel_mps_out = hdot_sum / (float)contact_cnt;
	}
	else
	{
		*body_height_vel_mps_out = 0.0f;
	}
}
static void JointBuildWrenchDemand(const JointBodyState* body_state,
				   const JointBodyTarget* body_target,
				   float body_height_m,
				   float body_height_vel_mps,
				   JointBodyWrenchDemand* wrench)
{
	float e_att[JOINT_CTRL_AXIS_NUM] = {0};
	float e_rate[JOINT_CTRL_AXIS_NUM] = {0};

	e_att[0] = body_target->roll_d - body_state->roll_d;
	e_att[1] = body_target->pitch_d - body_state->pitch_d;
	e_att[2] = body_target->yaw_d - body_state->yaw_d;

	e_rate[0] = body_target->roll_rate_dps - body_state->roll_rate_dps;
	e_rate[1] = body_target->pitch_rate_dps - body_state->pitch_rate_dps;
	e_rate[2] = body_target->yaw_rate_dps - body_state->yaw_rate_dps;

	for (uint8_t i = 0; i < JOINT_CTRL_AXIS_NUM; i++)
	{
		wrench->torque_b[i] = g_joint_force_ctrl.kp_att[i] * e_att[i] +
					     g_joint_force_ctrl.kd_att[i] * e_rate[i];
	}

	wrench->force_b[0] = 0.0f;
	wrench->force_b[1] = 0.0f;

	{
		float e_h = body_target->heave_m - body_height_m;
		float e_h_dot = body_target->heave_vel_mps - body_height_vel_mps;
		float gravity_ff = 0.55*g_joint_force_ctrl.body_mass_kg * g_joint_force_ctrl.gravity_mps2;
		wrench->force_b[2] = gravity_ff + g_joint_force_ctrl.kp_heave * e_h + g_joint_force_ctrl.kd_heave * e_h_dot;
	}
}

/* Normal mode force distribution: map body wrench to leg contact forces. */
static void JointDistributeLegForce(const JointBodyWrenchDemand* wrench,
				    float leg_force_out[JOINT_CTRL_LEG_NUM][JOINT_CTRL_AXIS_NUM])
{
	uint8_t contact_cnt = 0u;
	float rx[JOINT_CTRL_LEG_NUM] = {0};
	float ry[JOINT_CTRL_LEG_NUM] = {0};
	float rz[JOINT_CTRL_LEG_NUM] = {0};
	float den_x2 = 0.0f;
	float den_y2 = 0.0f;
	float den_z2 = 0.0f;
	const float eps = 1e-6f;

	for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
	{
		if (g_joint_force_ctrl.contact_flag[i])
		{
			rx[i] = g_joint_force_ctrl.leg_pos_body_m[i][0] - g_joint_body_com_m[0];
			ry[i] = g_joint_force_ctrl.leg_pos_body_m[i][1] - g_joint_body_com_m[1];
			rz[i] = g_joint_force_ctrl.leg_pos_body_m[i][2] - g_joint_body_com_m[2];
			contact_cnt++;
			den_x2 += rx[i] * rx[i];
			den_y2 += ry[i] * ry[i];
			den_z2 += rz[i] * rz[i];
		}
	}

	if (contact_cnt == 0)
	{
		memset(leg_force_out, 0, sizeof(float) * JOINT_CTRL_LEG_NUM * JOINT_CTRL_AXIS_NUM);
		return;
	}

	for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
	{
		if (!g_joint_force_ctrl.contact_flag[i])
		{
			leg_force_out[i][0] = 0.0f;
			leg_force_out[i][1] = 0.0f;
			leg_force_out[i][2] = 0.0f;
			continue;
		}

		{
			const float base_fx = wrench->force_b[0] / (float)contact_cnt;
			const float base_fy = wrench->force_b[1] / (float)contact_cnt;
			const float base_fz = wrench->force_b[2] / (float)contact_cnt;
			float d_fx = 0.0f;
			float d_fy = 0.0f;
			float d_fz = 0.0f;

			if (den_y2 + den_z2 > eps)
			{
				const float kx = wrench->torque_b[0] / (den_y2 + den_z2);
				d_fz += ry[i] * kx;
				d_fy += -rz[i] * kx;
			}

			if (den_z2 + den_x2 > eps)
			{
				const float ky = wrench->torque_b[1] / (den_z2 + den_x2);
				d_fx += rz[i] * ky;
				d_fz += -rx[i] * ky;
			}

			if (den_x2 + den_y2 > eps)
			{
				const float kz = wrench->torque_b[2] / (den_x2 + den_y2);
				d_fy += rx[i] * kz;
				d_fx += -ry[i] * kz;
			}

			leg_force_out[i][0] = base_fx + d_fx;
			leg_force_out[i][1] = base_fy + d_fy;
			leg_force_out[i][2] = base_fz + d_fz;
		}
	}

	{
		float sum_fx = 0.0f;
		float sum_fy = 0.0f;
		float sum_fz = 0.0f;
		float mean_fx;
		float mean_fy;
		float mean_fz;

		for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
		{
			if (!g_joint_force_ctrl.contact_flag[i])
				continue;
			sum_fx += leg_force_out[i][0];
			sum_fy += leg_force_out[i][1];
			sum_fz += leg_force_out[i][2];
		}

		mean_fx = (sum_fx - wrench->force_b[0]) / (float)contact_cnt;
		mean_fy = (sum_fy - wrench->force_b[1]) / (float)contact_cnt;
		mean_fz = (sum_fz - wrench->force_b[2]) / (float)contact_cnt;

		for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
		{
			if (!g_joint_force_ctrl.contact_flag[i])
				continue;

			leg_force_out[i][0] -= mean_fx;
			leg_force_out[i][1] -= mean_fy;
			leg_force_out[i][2] -= mean_fz;

			leg_force_out[i][0] = JointClamp(leg_force_out[i][0],
				-g_joint_force_ctrl.force_limit_n[i][0],
				 g_joint_force_ctrl.force_limit_n[i][0]);
			leg_force_out[i][1] = JointClamp(leg_force_out[i][1],
				-g_joint_force_ctrl.force_limit_n[i][1],
				 g_joint_force_ctrl.force_limit_n[i][1]);
			leg_force_out[i][2] = JointClamp(leg_force_out[i][2],
				-g_joint_force_ctrl.force_limit_n[i][2],
				 g_joint_force_ctrl.force_limit_n[i][2]);
		}
	}
}

static float JointLegForceToMotorTorque(uint8_t leg_id, const float leg_force_l[JOINT_CTRL_AXIS_NUM])
{
	const matrix_data_t* j = g_joint_leg_jacobian_dpos_dtheta[leg_id].pData;
	return j[0] * leg_force_l[0] + j[1] * leg_force_l[1] + j[2] * leg_force_l[2];
}

void JointControlMatrixInit(void)
{
	matrix_init(&g_joint_rot_leg_to_body, 3, 3, g_joint_rot_leg_to_body_data);
	matrix_init(&g_joint_rot_body_to_imu, 3, 3, g_joint_rot_body_to_imu_data);
	matrix_init(&g_joint_rot_imu_to_world, 3, 3, g_joint_rot_imu_to_world_data);

	for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
	{
		matrix_init(&g_joint_leg_jacobian_dpos_dtheta[i], 3, 1, g_joint_leg_jacobian_dpos_dtheta_data[i]);
	}
}

void JointControlPlaceholdersReset(void)
{
	(void)LEG_LF;
	(void)LEG_RF;
	(void)LEG_RB;
	(void)LEG_LB;

	JointControlMatrixInit();
	memset(g_joint_rot_leg_to_body_data, 0, sizeof(g_joint_rot_leg_to_body_data));
	memset(g_joint_rot_body_to_imu_data, 0, sizeof(g_joint_rot_body_to_imu_data));
	memset(g_joint_rot_imu_to_world_data, 0, sizeof(g_joint_rot_imu_to_world_data));
	memset(g_joint_leg_jacobian_dpos_dtheta_data, 0, sizeof(g_joint_leg_jacobian_dpos_dtheta_data));

	memset(g_joint_motor_torque_cmd_nm, 0, sizeof(g_joint_motor_torque_cmd_nm));
	memset(g_joint_motor_torque_fdb_nm, 0, sizeof(g_joint_motor_torque_fdb_nm));
	memset(g_joint_motor_angle_rad, 0, sizeof(g_joint_motor_angle_rad));
	memset(&g_joint_force_ctrl, 0, sizeof(g_joint_force_ctrl));

	g_joint_control_enable = 0;
	g_joint_state = JOINT_STOP;
}

void JointForceControlTuningParamInit(void)
{
	g_joint_force_ctrl.kp_att[0] = 11.0f;
	g_joint_force_ctrl.kp_att[1] = -15.0f;
	g_joint_force_ctrl.kp_att[2] = 0.0f;
	g_joint_force_ctrl.kd_att[0] = 0.8f;
	g_joint_force_ctrl.kd_att[1] = -1.0f;
	g_joint_force_ctrl.kd_att[2] = 0.0f;

	g_joint_force_ctrl.kp_heave = 6500.0f;
	g_joint_force_ctrl.kd_heave = 200.0f;

	for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
	{
		g_joint_force_ctrl.contact_flag[i] = 1;
		g_joint_force_ctrl.force_limit_n[i][0] = 120.0f;
		g_joint_force_ctrl.force_limit_n[i][1] = 120.0f;
		g_joint_force_ctrl.force_limit_n[i][2] = 350.0f;
	}
}

void JointForceControlInit(float body_mass_kg, float dt_s)
{
	JointControlPlaceholdersReset();

	g_joint_control_dt_s = dt_s;
	g_joint_control_enable = 1;
	g_joint_state = JOINT_NORMAL;

	g_joint_force_ctrl.body_mass_kg = body_mass_kg;
	g_joint_force_ctrl.gravity_mps2 = 9.81f;
	g_joint_force_ctrl.dt_s = dt_s;

	LTDInitialize(&g_joint_climb_rear_target_ltd, 19.0f, dt_s, -0.40f, 0.9f);//kkg
 g_joint_climb_rear_target_ltd.x1=0;
	g_joint_climb_rear_target_ltd.x2=0;

	JointForceControlTuningParamInit();
	JointStairUpDetectReset();

	g_joint_rot_leg_to_body_data[0] = 1.0f;
	g_joint_rot_leg_to_body_data[4] = 1.0f;
	g_joint_rot_leg_to_body_data[8] = 1.0f;

	g_joint_rot_body_to_imu_data[0] = 1.0f;
	g_joint_rot_body_to_imu_data[4] = 1.0f;
	g_joint_rot_body_to_imu_data[8] = 1.0f;

	g_joint_rot_imu_to_world_data[0] = 1.0f;
	g_joint_rot_imu_to_world_data[4] = 1.0f;
	g_joint_rot_imu_to_world_data[8] = 1.0f;
}

void JointForceControlSetMotorAngleRad(const float motor_angle_rad[JOINT_CTRL_MOTOR_NUM])
{
	memcpy(g_joint_motor_angle_rad,
	       motor_angle_rad,
	       sizeof(g_joint_motor_angle_rad));
}

void JointForceControlEstimateUpdate(void)
{
	JointUpdateLegPoseFromMotorAngle();
	JointUpdateLegJacobiansFromMotorAngle();
}

void JointForceControlSetJointMode(uint8_t joint_mode, float climb_pitch_hold_d)
{
	g_joint_mode = joint_mode;
	g_joint_climb_pitch_hold_d = climb_pitch_hold_d;
}

void JointForceControlSetStandMode(uint8_t stand_mode)
{
	g_joint_stand_mode = stand_mode;
}

void JointForceControlSetJumpMode(uint8_t jump_mode)
{
	g_joint_jump_mode = jump_mode;
}

void JointForceControlSetContact(const uint16_t frame_counter[JOINT_CTRL_LEG_NUM],
				 const float torque_fdb_nm[JOINT_CTRL_LEG_NUM],
				 uint8_t ctrl_active)
{
	static uint16_t last_frame_counter[JOINT_CTRL_LEG_NUM] = {0};
	static uint8_t stale_cycle_cnt[JOINT_CTRL_LEG_NUM] = {0};
	static uint8_t frame_counter_inited = 0;
	static uint8_t last_ctrl_active = 0u;
	static uint16_t force_contact_cycle = 0u;
	const uint8_t offline_stale_cycle_threshold = 5u;
	const uint16_t force_contact_bootstrap_cycles = 500u;

	if (!last_ctrl_active && ctrl_active)
	{
		force_contact_cycle = force_contact_bootstrap_cycles;
	}
	last_ctrl_active = ctrl_active ? 1u : 0u;

	for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
	{
		uint8_t frame_updated = frame_counter_inited ? (frame_counter[i] != last_frame_counter[i]) : 1u;
		uint8_t motor_online = 1u;
		float abs_torque = fabsf(torque_fdb_nm[i]);

		if (frame_updated)
		{
			stale_cycle_cnt[i] = 0u;
		}
		else if (stale_cycle_cnt[i] < 255u)
		{
			stale_cycle_cnt[i]++;
		}

		if (stale_cycle_cnt[i] >= offline_stale_cycle_threshold)
		{
			motor_online = 0u;
		}

		g_joint_motor_torque_fdb_nm[i] = torque_fdb_nm[i];
		g_joint_force_ctrl.contact_flag[i] = (motor_online && abs_torque >= g_joint_contact_torque_threshold_nm) ? 1u : 0u;
		if (force_contact_cycle > 0u)
		{
			g_joint_force_ctrl.contact_flag[i] = 1u;
		}
		last_frame_counter[i] = frame_counter[i];
	}

	if (force_contact_cycle > 0u)
	{
		force_contact_cycle--;
	}

	frame_counter_inited = 1;
}

void JointForceControlStep(const JointBodyState* body_state,
			   const JointBodyTarget* body_target,
			   float body_height_m,
			   float body_height_vel_mps)
{
	float leg_force_cmd[JOINT_CTRL_LEG_NUM][JOINT_CTRL_AXIS_NUM] = {0};
	const float force_lpf_alpha = 0.85f;

	if (!g_joint_control_enable || g_joint_state == JOINT_STOP)
	{
		memset(&g_joint_body_state_body_dbg, 0, sizeof(g_joint_body_state_body_dbg));
		memset(g_joint_motor_torque_cmd_nm, 0, sizeof(g_joint_motor_torque_cmd_nm));
		return;
	}

	g_joint_body_state_body_dbg = *body_state;

	JointBuildWrenchDemand(body_state,
			       body_target,
			       body_height_m,
			       body_height_vel_mps,
			       &g_joint_force_ctrl.wrench_demand);
	JointDistributeLegForce(&g_joint_force_ctrl.wrench_demand, leg_force_cmd);

	for (uint8_t i = 0; i < JOINT_CTRL_LEG_NUM; i++)
	{
		float leg_force_l[JOINT_CTRL_AXIS_NUM] = {0};
		const float angle = g_joint_motor_angle_rad[i];
		float torque_max_nm = g_joint_motor_torque_max_nm[i];
		float torque_min_nm = g_joint_motor_torque_min_nm[i];
		float front_pos_torque_ref_nm = 0.0f;

		if ((i == LEG_LF || i == LEG_RF) && g_joint_stand_mode == JOINT_STAND_MODE_PRE_STAIR)
		{
			torque_min_nm = -0.0f;
		}

		if ((i == LEG_RB || i == LEG_LB) && g_joint_stand_mode == JOINT_STAND_MODE_PRE_STAIR)
		{
			torque_max_nm = 5.0f;
		}

		for (uint8_t a = 0; a < JOINT_CTRL_AXIS_NUM; a++)
		{
			leg_force_l[a] = JointLpf(g_joint_force_ctrl.leg_force_lpf_n[i][a],
					      leg_force_cmd[i][a],
					      force_lpf_alpha);
			g_joint_force_ctrl.leg_force_lpf_n[i][a] = leg_force_l[a];
			g_joint_force_ctrl.leg_force_cmd_b_n[i][a] = leg_force_cmd[i][a];
		}

		g_joint_motor_torque_cmd_nm[i] = JointLegForceToMotorTorque(i, leg_force_l);

		if ((i == LEG_LF || i == LEG_RF))
		{
			if (angle < -1.780796327f)
			{
				torque_max_nm = 0.0f;
				front_pos_torque_ref_nm = 0.0f;
			}
			else
			{
				torque_max_nm = 3.0f;
				{
					const float angle_threshold_rad = -1.780796327f;
					const float angle_span = g_joint_motor_angle_max_rad[i] - angle_threshold_rad;
					if (angle_span > 1e-6f)
					{
						const float k = 3.0f / angle_span;
						front_pos_torque_ref_nm = k * (angle - angle_threshold_rad);
						front_pos_torque_ref_nm = JointClamp(front_pos_torque_ref_nm, 0.0f, 0.0f);
					}
				}
			}
		}

		g_joint_motor_torque_cmd_nm[i] = JointClamp(g_joint_motor_torque_cmd_nm[i],
					     torque_min_nm,
						     torque_max_nm);//-0.8-4

		if ((i == LEG_LF || i == LEG_RF) && angle >= -1.780796327f && angle < g_joint_motor_angle_max_rad[i])
		{
			g_joint_motor_torque_cmd_nm[i] = JointClamp(front_pos_torque_ref_nm,
						     torque_min_nm,
							     torque_max_nm);
		}

		if ((i == LEG_LF || i == LEG_RF) && angle >= -2.70f && angle <= -2.67f)
		{
			g_joint_motor_torque_cmd_nm[i] = JointClamp(-4.0f,
						     torque_min_nm,
							     torque_max_nm);
		}

			const float cmd = g_joint_motor_torque_cmd_nm[i];
			const float dir = g_joint_torque_to_angle_sign[i];
			const float drive = cmd * dir;
			if ((angle >= g_joint_motor_angle_max_rad[i] ) ||
			    (angle <= g_joint_motor_angle_min_rad[i]))
			{
				g_joint_motor_torque_cmd_nm[i] = 0.0f;
			}
		

		g_joint_force_ctrl.motor_torque_ref_nm[i] = g_joint_motor_torque_cmd_nm[i];
		jointControl.JointMotorControl.mit_Tff[i] = k_toq_sign_map[i] * g_joint_motor_torque_cmd_nm[i];
	}
}

/* ===== 上台阶检测状态机 ===== */
#define STAIR_UP_DIST_HIGH_THRESHOLD   266.0f
#define STAIR_UP_DIST_LOW_THRESHOLD    110.0f
#define STAIR_UP_HIGH_CYCLE_COUNT      10u

typedef enum
{
	STAIR_DETECT_IDLE = 0,
	STAIR_DETECT_APPROACHING,
	STAIR_DETECT_TRIGGERED,
} StairDetectState;

static StairDetectState g_stair_detect_state = STAIR_DETECT_IDLE;
static uint8_t g_stair_high_cycle_cnt = 0u;
static uint8_t g_stair_detected_flag = 0u;

/*
 * 检测逻辑：
 *   IDLE:        距离 > 266 → 进入 APPROACHING，开始计数
 *   APPROACHING: 距离 > 266 → 计数器累加（积累"靠近台阶"的证据）
 *                100 < 距离 ≤ 266 → 保持计数不动，继续等（下降过程中不丢进度）
 *                距离 < 100 且 计数 ≥ 阈值 → 触发！进入 TRIGGERED
 *   TRIGGERED:   永久保持，由外部 Reset 清除
 */
void JointStairUpDetect(void)
{
	const float dist = distance;

	switch (g_stair_detect_state)
	{
	case STAIR_DETECT_IDLE:
		if (dist > STAIR_UP_DIST_HIGH_THRESHOLD)
		{
			g_stair_high_cycle_cnt = 1u;
			g_stair_detect_state = STAIR_DETECT_APPROACHING;
		}
		break;

	case STAIR_DETECT_APPROACHING:
		if (dist > STAIR_UP_DIST_HIGH_THRESHOLD)
		{
			/* 仍在台阶面前方，继续积累证据 */
			if (g_stair_high_cycle_cnt < 255u)
				g_stair_high_cycle_cnt++;
		}
		else if (dist < STAIR_UP_DIST_LOW_THRESHOLD)
		{
			/* 距离骤降：前腿已爬上台阶 */
			if (g_stair_high_cycle_cnt >= STAIR_UP_HIGH_CYCLE_COUNT)
			{
				g_stair_detected_flag = 1u;
				g_stair_detect_state = STAIR_DETECT_TRIGGERED;
				g_joint_stand_mode = JOINT_STAND_MODE_STAIR_UP;
			}
			/* cnt 不足则忽略，保持 APPROACHING 继续等待 */
		}
		/* else: 100 ≤ dist ≤ 266，下降过渡期，保持计数不动 */
		break;

	case STAIR_DETECT_TRIGGERED:
		/* 已触发，保持标志；由外部调用 Reset 清除 */
		break;

	default:
		g_stair_detect_state = STAIR_DETECT_IDLE;
		break;
	}
}

void JointStairUpDetectReset(void)
{
	g_stair_detect_state = STAIR_DETECT_IDLE;
	g_stair_high_cycle_cnt = 0u;
	g_stair_detected_flag = 0u;
}

uint8_t JointStairUpIsDetected(void)
{
	return g_stair_detected_flag;
}



/*============================================================================
 * 从 robot_control_task.c 搬迁 — 关节全局变量 + 三个函数
 *============================================================================*/

JointBodyTarget g_joint_body_target_cmd = {0};
static JointBodyState g_joint_body_state_obs = {0};
float g_joint_body_pitch_ctrl_d = 0.0f;

float climb_joint_pos[4]={0.143,-0.1200,-1.13,1.00};
float normal_joint_pos[4]={0.05-0.9416,-0.003114+0.761,0.00819 +0.9416,0.009343-0.761};

/*============================================================================
 * JointInputUpdate — 关节输入决策 (DecisionTask 调用)
 *============================================================================*/
void JointInputUpdate(void)
{
	/* 在此处直接更新关节目标，不再通过数组映射 */
	g_joint_body_target_cmd.roll_d = 0.0f;
	g_joint_body_target_cmd.pitch_d = 0.0f;
	g_joint_body_target_cmd.yaw_d = gimbalPose.yaw_d;
	g_joint_body_target_cmd.roll_rate_dps = 0.0f;
	g_joint_body_target_cmd.pitch_rate_dps = 0.0f;
	g_joint_body_target_cmd.yaw_rate_dps = 0.0f;
	if (pDecisionAO->joint_mode == JOINT_PRESTAIR)
		g_joint_body_target_cmd.heave_m = 0.3900f;
	else
		g_joint_body_target_cmd.heave_m = 0.3400f;
	g_joint_body_target_cmd.heave_vel_mps = 0.0f;
}

/*============================================================================
 * JointEstimateUpdate — 关节观测估计 (EstimateTask 调用)
 *============================================================================*/
void JointEstimateUpdate(void)
{
	JointBodyState body_state_for_height = {0};
	float body_height_m_obs = jointControl.JointEstimate.body_height_m;
	float body_height_vel_mps_obs = jointControl.JointEstimate.body_height_vel_mps;
	float joint_vel_radps_raw[4];

	for(uint8_t i = 0; i < 4; i++)//更新数据
	{
		jointControl.JointEstimate.frame_counter[i] = _jointMotorEec[i].frame_counter;
		jointControl.JointEstimate.id[i] = (uint32_t)_jointMotorEec[i].id;
		jointControl.JointEstimate.state[i] = _jointMotorEec[i].state;
		jointControl.JointEstimate.pos_d[i] = _jointMotorEec[i].pos_d*180/3.141592f;//注意原来是弧度
		jointControl.JointEstimate.vel_radps[i] = _jointMotorEec[i].vel_radps*180/3.141592f;
		joint_vel_radps_raw[i] = _jointMotorEec[i].vel_radps;
		jointControl.JointEstimate.toq[i] = _jointMotorEec[i].toq;//反馈扭矩
		jointControl.JointEstimate.Kp[i] = _jointMotorEec[i].Kp;
		jointControl.JointEstimate.Kd[i] = _jointMotorEec[i].Kd;
		jointControl.JointEstimate.Tmos[i] = _jointMotorEec[i].Tmos;
		jointControl.JointEstimate.Tcoil[i] = _jointMotorEec[i].Tcoil;
	}
	float motor_angles_rad[4];
	float feedback_pos_rad[4];
	for(uint8_t i = 0; i < 4; i++)
	{
		feedback_pos_rad[i] = _jointMotorEec[i].pos_d;
	}
	JointBuildMotorAnglesRadFromFeedback(feedback_pos_rad, motor_angles_rad);
	for(uint8_t i = 0; i < 4; i++)
	{
		jointControl.JointEstimate.motor_angles_rad[i] = motor_angles_rad[i];
	}
	//更新输入角
	JointForceControlSetMotorAngleRad(motor_angles_rad);
	//更新接触点在机体系中的位置
	JointUpdateLegPoseFromMotorAngle();
	//更新雅可比矩阵
	JointUpdateLegJacobiansFromMotorAngle();
	//更新接触检测
	JointForceControlSetContact(jointControl.JointEstimate.frame_counter,
					jointControl.JointEstimate.toq,
					(pDecisionAO->ctrl_terminal != CONTROL_STOP) ? 1u : 0u);

	/* 观测态由 gimbalPose 提供，角速度先置 0（后续可替换为滤波角速度） */
	g_joint_body_state_obs.pitch_d = gimbalPose.pitch_d;
	g_joint_body_state_obs.yaw_d = gimbalPose.yaw_d;
	g_joint_body_state_obs.roll_d = gimbalPose.roll_d;
	g_joint_body_state_obs.roll_rate_dps = gimbalPose.roll_radps*57.29578f;
	g_joint_body_state_obs.pitch_rate_dps = gimbalPose.pitch_radps*57.29578f;
	g_joint_body_state_obs.yaw_rate_dps =gimbalPose.yaw_radps*57.29578f;
	g_joint_body_state_obs.accel_x = gimbalPose.accel_x;
	g_joint_body_state_obs.accel_y = gimbalPose.accel_y;
	g_joint_body_state_obs.accel_z = gimbalPose.accel_z;

	JointForceControlConvertBodyState(&g_joint_body_state_obs, &body_state_for_height);
	//更新机体高度与竖直速度观测（世界系 z）
	JointEstimateBodyHeightVelocity(&body_state_for_height,
				       joint_vel_radps_raw,
				       &body_height_m_obs,
				       &body_height_vel_mps_obs);
	jointControl.JointEstimate.body_height_m = body_height_m_obs;
	jointControl.JointEstimate.body_height_vel_mps = body_height_vel_mps_obs;

	/* 上台阶检测：仅在 PRESTAIR 模式下使能，NORMAL / PRE_DOWN_STAIR 模式清除状态 */
	if (pDecisionAO->joint_mode == JOINT_PRESTAIR)
	{
		JointStairUpDetect();
		if (JointStairUpIsDetected() && stair_detected_event_posted == 0U)
		{
			QACTIVE_POST(AO_DecisionAO, &stair_detected_event, NULL);
			stair_detected_event_posted = 1U;
		}
	}
	else if (pDecisionAO->joint_mode == JOINT_NORMAL
	         || _robotState->stand_mode == ROBOT_STAND_MODE_PRE_DOWN_STAIR) /* TODO: migrate PRE_DOWN_STAIR to HSM */
	{
		JointStairUpDetectReset();
		stair_detected_event_posted = 0U;
	}
}//代办:检查速度映射

/*============================================================================
 * JointControlUpdate — 关节闭环控制 (ControlTask 调用)
 *============================================================================*/
void JointControlUpdate(void)
{
	JointBodyState body_state = g_joint_body_state_obs;
	JointBodyState body_state_ctrl = {0};
	JointBodyTarget body_target = g_joint_body_target_cmd;
	static float preclimb_motor_target_angle_rad[JOINT_CTRL_MOTOR_NUM] = {
		-2.05f, -2.05f, 1.05f, 1.05f,
	};//-2.25f, -2.25f, 1.05f, 1.05f
	static uint8_t last_joint_mode = JOINT_NORMAL;  /* 匹配新HSM宏 */
	static uint8_t prev_stand_stair = 0;  /* STAIR_UP 斜坡持续标记，函数级 static */
	extern float g_joint_motor_torque_cmd_nm[JOINT_CTRL_MOTOR_NUM];
	float mit_pos_cmd_rad[JOINT_CTRL_MOTOR_NUM] = {0};
	const float body_height_m = jointControl.JointEstimate.body_height_m;
	const float body_height_vel_mps = jointControl.JointEstimate.body_height_vel_mps;

	/*
	 * 若高度相关目标暂未填写，默认跟随当前观测，避免一上电就产生大幅 heave 力。
	 * 一旦你在 JointInputUpdate 中写入非零目标，这里会被覆盖。
	 */
	if (g_joint_body_target_cmd.heave_m == 0.0f)
		body_target.heave_m = body_height_m;
	if (g_joint_body_target_cmd.heave_vel_mps == 0.0f)
		body_target.heave_vel_mps = body_height_vel_mps;
	JointForceControlConvertBodyState(&body_state, &body_state_ctrl);
	g_joint_body_pitch_ctrl_d = body_state_ctrl.pitch_d;

	if (pDecisionAO->joint_mode == JOINT_STAIRUP)
	{
		JointForceControlSetJointMode(JOINT_NORMAL, 0.0f);
	}
	else
	{
		JointForceControlSetJointMode(JOINT_NORMAL, 0.0f);
	}
	JointForceControlSetStandMode(pDecisionAO->stand_mode);
	JointForceControlSetJumpMode(_robotState->jump_mode);  /* jump_mode 不在DecisionAO中 */
	last_joint_mode = pDecisionAO->joint_mode;

	JointForceControlStep(&body_state_ctrl,
				      &body_target,
				      body_height_m,
				      body_height_vel_mps);

	for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
	{
		mit_pos_cmd_rad[i] = jointControl.JointEstimate.motor_angles_rad[i];
	}
	JointBuildFeedbackPosRadFromMotorAngles(mit_pos_cmd_rad, mit_pos_cmd_rad);
	for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
	{
		jointControl.JointMotorControl.mit_p[i] =0.0f;
		jointControl.JointMotorControl.mit_v[i] = 0.0f;
		jointControl.JointMotorControl.mit_Kp[i] = 0.0f;
		jointControl.JointMotorControl.mit_Kd[i] = 0.0f;
	}

	enum
	{
		MIT_MODE_CLIMB = 0,
		MIT_MODE_STAIR_UP,
		MIT_MODE_PRECLIMB,
		MIT_MODE_REVOLVE,
		MIT_MODE_FORCE_LIMIT,
		MIT_MODE_COUNT
	};//- + - +
	static const float mit_kp_table[MIT_MODE_COUNT][JOINT_CTRL_MOTOR_NUM] = {
		[MIT_MODE_CLIMB] = { [LEG_LF] = 30.0f, [LEG_RF] = 30.0f, [LEG_RB] = 20.0f, [LEG_LB] = 20.0f },
		[MIT_MODE_STAIR_UP] = { [LEG_LF] = 40.0f, [LEG_RF] = 40.0f, [LEG_RB] = 40.0f, [LEG_LB] = 40.0f },
		[MIT_MODE_PRECLIMB] = { [LEG_LF] = 60.0f, [LEG_RF] = 60.0f, [LEG_RB] = 60.0f, [LEG_LB] = 60.0f },
		[MIT_MODE_REVOLVE]  = { [LEG_LF] = 30.0f, [LEG_RF] = 30.0f, [LEG_RB] = 30.0f, [LEG_LB] = 30.0f }, // TODO: 陀螺模式位控Kp
		[MIT_MODE_FORCE_LIMIT] = { [LEG_LF] = 30.0f, [LEG_RF] = 30.0f, [LEG_RB] = 30.0f, [LEG_LB] = 30.0f },
	};
	static const float mit_kd_table[MIT_MODE_COUNT][JOINT_CTRL_MOTOR_NUM] = {
		[MIT_MODE_CLIMB] = { [LEG_LF] = 1.0f, [LEG_RF] = 1.0f, [LEG_RB] = 1.0f, [LEG_LB] = 1.0f },
		[MIT_MODE_STAIR_UP] = { [LEG_LF] = 3.0f, [LEG_RF] = 3.0f, [LEG_RB] = 3.0f, [LEG_LB] = 3.0f },
		[MIT_MODE_PRECLIMB] = { [LEG_LF] = 3.0f, [LEG_RF] = 3.0f, [LEG_RB] = 3.0f, [LEG_LB] = 3.0f },
		[MIT_MODE_REVOLVE]  = { [LEG_LF] = 3.0f, [LEG_RF] = 3.0f, [LEG_RB] = 3.0f, [LEG_LB] = 3.0f }, // TODO: 陀螺模式位控Kd
		[MIT_MODE_FORCE_LIMIT] = { [LEG_LF] = 5.0f, [LEG_RF] = 5.0f, [LEG_RB] = 5.0f, [LEG_LB] = 5.0f },
	};
	static const float mit_tff_table[MIT_MODE_COUNT][JOINT_CTRL_MOTOR_NUM] = {
		[MIT_MODE_CLIMB] = { [LEG_LF] = -3.0f, [LEG_RF] = 3.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f },
		[MIT_MODE_STAIR_UP] = { [LEG_LF] = 0.0f, [LEG_RF] = 0.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f },
		[MIT_MODE_PRECLIMB] = { [LEG_LF] = 0.0f, [LEG_RF] = 0.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f },
		[MIT_MODE_REVOLVE]  = { [LEG_LF] = 0.0f, [LEG_RF] = 0.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f }, // TODO: 陀螺模式前馈Tff
		[MIT_MODE_FORCE_LIMIT] = { [LEG_LF] = 0.0f, [LEG_RF] = 0.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f },
	};
	/* ===== 统一MIT位控角度计算 ===== */
	uint8_t  mit_angle_active = 0u;
	uint8_t  mit_mode_idx      = MIT_MODE_FORCE_LIMIT;
	float    motor_target_angle_rad[JOINT_CTRL_MOTOR_NUM] = {0};

	if (pDecisionAO->joint_mode == JOINT_PRESTAIR)  /* 原 ROBOT_STAND_MODE_PRE_STAIR → 新HSM PreStair */
	{
		/* PRE_STAIR: 前腿MIT位控收腿到-2.50rad，后腿继续力控 */
		float front_angles_rad[JOINT_CTRL_MOTOR_NUM];
		float front_pos_rad[JOINT_CTRL_MOTOR_NUM];
		front_angles_rad[LEG_LF] = -2.60f;
		front_angles_rad[LEG_RF] = -2.60f;
		front_angles_rad[LEG_RB] = jointControl.JointEstimate.motor_angles_rad[LEG_RB];
		front_angles_rad[LEG_LB] = jointControl.JointEstimate.motor_angles_rad[LEG_LB];
		JointBuildFeedbackPosRadFromMotorAngles(front_angles_rad, front_pos_rad);
		/* 仅前腿走MIT位控，后腿保持力控(Tff由JointForceControlStep写入) */
		for (uint8_t leg = LEG_LF; leg <= LEG_RF; leg++)
		{
			jointControl.JointMotorControl.mit_p[leg]  = front_pos_rad[leg];
			jointControl.JointMotorControl.mit_v[leg]  = 0.0f;
			jointControl.JointMotorControl.mit_Kp[leg] = 15.0f;
			jointControl.JointMotorControl.mit_Kd[leg] = 0.0f;
		}
		/* 不设 mit_angle_active，避免统一应用覆盖后腿力控Tff */
	}
	else if (pDecisionAO->joint_mode == JOINT_STAIRUP)  /* 原 ROBOT_STAND_MODE_STAIR_UP → 新HSM StairUp */
	{
		mit_mode_idx = MIT_MODE_STAIR_UP;
		/* 目标角度缓慢变化，每周期收敛0.1rad */
		static float    ramp_angle[JOINT_CTRL_MOTOR_NUM] = {0};
		static uint8_t  stair_up_ramp_inited = 0;
		const float target_angle[JOINT_CTRL_MOTOR_NUM] = {
			[LEG_LF] = -2.60f, [LEG_RF] = -2.60f,
			[LEG_RB] =  1.70f, [LEG_LB] =  1.70f,
		};
		/* 首次进入或从其他模式切回时，重新从当前角度开始斜坡 */
		if (!stair_up_ramp_inited || !prev_stand_stair)
		{
			for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
				ramp_angle[i] = jointControl.JointEstimate.motor_angles_rad[i];
			stair_up_ramp_inited = 1u;
		}
		for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
		{
			float err = target_angle[i] - ramp_angle[i];
			if (err >  0.1f)       ramp_angle[i] += 0.003f;
			else if (err < -0.1f)  ramp_angle[i] -= 0.003f;
			else                   ramp_angle[i] = target_angle[i];
			motor_target_angle_rad[i] = ramp_angle[i];
		}
		mit_angle_active = 1u;
		prev_stand_stair = 1u;
	}
	else if (pDecisionAO->joint_mode == JOINT_PRESTAIR)  /* 原 ROBOT_JOINT_MODE_PRECLIMB → 新HSM PreStair */
	{
		mit_mode_idx = MIT_MODE_PRECLIMB;
		const float pitch_min_d = 0.0f;
		const float pitch_max_d = 20.0f;
		const float front_angle_pitch0_rad  = -2.25f;
		const float front_angle_pitch20_rad = -2.66f;
		float pitch_d = body_state_ctrl.pitch_d;
		const float front_slope = (front_angle_pitch20_rad - front_angle_pitch0_rad) / (pitch_max_d - pitch_min_d);

		if (pitch_d < pitch_min_d)
			pitch_d = pitch_min_d;
		else if (pitch_d > pitch_max_d)
			pitch_d = pitch_max_d;

		const float front_target_angle_rad = front_angle_pitch0_rad + front_slope * (pitch_d - pitch_min_d);
		motor_target_angle_rad[LEG_LF] = front_target_angle_rad;
		motor_target_angle_rad[LEG_RF] = front_target_angle_rad;
		motor_target_angle_rad[LEG_RB] = preclimb_motor_target_angle_rad[LEG_RB];
		motor_target_angle_rad[LEG_LB] = preclimb_motor_target_angle_rad[LEG_LB];
		mit_angle_active = 1u;
	}

	/* 退出 STAIR_UP 时清除持续标记，使下次重新斜坡 */
	if (pDecisionAO->joint_mode != JOINT_STAIRUP)  /* 原 ROBOT_STAND_MODE_STAIR_UP → 新HSM */
		prev_stand_stair = 0u;

	uint16_t rev_exit_hold_cnt = 0;
	/* 陀螺退出保持：退出陀螺后强制位控200周期，平滑过渡 */
	{
		static float    rev_exit_hold_angle_rad[JOINT_CTRL_MOTOR_NUM] = {0};
		static uint8_t  last_revolve = 0;

		if (pDecisionAO->chassis_mode != CHASSIS_REVOLVE && last_revolve)
		{
			rev_exit_hold_angle_rad[LEG_LF] = -1.702f;
			rev_exit_hold_angle_rad[LEG_RF] = -1.702f;
			rev_exit_hold_angle_rad[LEG_RB] =  1.319f;
			rev_exit_hold_angle_rad[LEG_LB] =  1.319f;
			rev_exit_hold_cnt = 600;
		}
		last_revolve = (pDecisionAO->chassis_mode == CHASSIS_REVOLVE) ? 1u : 0u;

		if (rev_exit_hold_cnt > 0)
		{
			rev_exit_hold_cnt--;
			for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
				motor_target_angle_rad[i] = rev_exit_hold_angle_rad[i];
			mit_mode_idx    = MIT_MODE_REVOLVE;
			mit_angle_active = 1u;
		}
	}

	/* ===== 统一应用MIT位控参数 ===== */
	if (mit_angle_active)
	{
		JointBuildFeedbackPosRadFromMotorAngles(motor_target_angle_rad, mit_pos_cmd_rad);
		for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
		{
			jointControl.JointMotorControl.mit_p[i]   = mit_pos_cmd_rad[i];
			jointControl.JointMotorControl.mit_v[i]   = 0.0f;
			jointControl.JointMotorControl.mit_Kp[i]  = mit_kp_table[mit_mode_idx][i];
			jointControl.JointMotorControl.mit_Kd[i]  = mit_kd_table[mit_mode_idx][i];
			jointControl.JointMotorControl.mit_Tff[i] = mit_tff_table[mit_mode_idx][i];
		}
	}

	/* ===== 力控模式下强制位置限幅 ===== */
	{
		float motor_angle_min_rad[JOINT_CTRL_MOTOR_NUM] = {0};
		float motor_angle_max_rad[JOINT_CTRL_MOTOR_NUM] = {0};
		uint8_t limit_hit[JOINT_CTRL_MOTOR_NUM] = {0};
		uint8_t limit_active = 0u;

		JointGetMotorAngleLimitsRad(motor_angle_min_rad, motor_angle_max_rad);
		if(pDecisionAO->joint_mode == JOINT_NORMAL)  /* 原 ROBOT_STAND_MODE_NORMAL → 新HSM */
		{
			motor_angle_max_rad[LEG_LF]-=0.20f;
			motor_angle_max_rad[LEG_RF]-=0.20f;
		}
		const float limit_inset_rad = 0.05f;
		for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
		{
			const float angle = jointControl.JointEstimate.motor_angles_rad[i];
			if (angle < motor_angle_min_rad[i])
			{
				motor_target_angle_rad[i] = motor_angle_min_rad[i] + limit_inset_rad;
				limit_hit[i] = 1u;
				limit_active = 1u;
			}
			else if (angle > motor_angle_max_rad[i])
			{
				motor_target_angle_rad[i] = motor_angle_max_rad[i] - limit_inset_rad;
				limit_hit[i] = 1u;
				limit_active = 1u;
			}
			else
			{
				motor_target_angle_rad[i] = angle;
			}
		}

		if (limit_active)
		{
			JointBuildFeedbackPosRadFromMotorAngles(motor_target_angle_rad, mit_pos_cmd_rad);
			for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
			{
				if (!limit_hit[i])
					continue;
				jointControl.JointMotorControl.mit_p[i]  = mit_pos_cmd_rad[i];
				jointControl.JointMotorControl.mit_v[i]  = 0.0f;
				jointControl.JointMotorControl.mit_Kp[i] = mit_kp_table[MIT_MODE_FORCE_LIMIT][i];
				jointControl.JointMotorControl.mit_Kd[i] = mit_kd_table[MIT_MODE_FORCE_LIMIT][i];
			}
		}
	}

	/* Y方向急停时前腿位控保持，防止惯性压弯前腿角度突变 */
	/* PRE_DOWN_STAIR 模式下跳过急停位控 */
	if (_robotState->stand_mode != ROBOT_STAND_MODE_PRE_DOWN_STAIR /* TODO: add HSM PreDownStair */ && pDecisionAO->joint_mode != JOINT_PRESTAIR && pDecisionAO->joint_mode != JOINT_STAIRUP && rev_exit_hold_cnt == 0)
	{
		static float last_speed_y_mps = 0.0f;
		static uint16_t decel_hold_cnt = 0;
		static float decel_hold_angle_rad[2] = {0.0f, 0.0f}; /* LF, RF */
		const float decel_enter_speed_mps = 0.30f;  /* 高于此速度视为"运动中" */
		const float decel_exit_speed_mps = 0.20f;   /* 低于此速度视为"已急停" */
		const uint16_t decel_hold_cycles = 200;     /* 位控保持周期数 */

		/* 检测急停边沿：目标速度从高位骤降至低位 */
		if (last_speed_y_mps > decel_enter_speed_mps &&
		    _chassisControl->ChassisRealNeedInput.speed_y_mps < decel_exit_speed_mps)
		{
			decel_hold_cnt = decel_hold_cycles;
			/* 记录急停瞬间的前腿关节角作为位控目标 */
			decel_hold_angle_rad[0] = jointControl.JointEstimate.motor_angles_rad[LEG_LF];
			decel_hold_angle_rad[1] = jointControl.JointEstimate.motor_angles_rad[LEG_RF];
		}

		if (decel_hold_cnt > 0)
		{
			decel_hold_cnt--;
			/* 前腿切位控模式，锁定急停瞬间角度 */
			float hold_angles_rad[JOINT_CTRL_MOTOR_NUM];
			hold_angles_rad[LEG_LF] = decel_hold_angle_rad[0];
			hold_angles_rad[LEG_RF] = decel_hold_angle_rad[1];
			hold_angles_rad[LEG_RB] = jointControl.JointEstimate.motor_angles_rad[LEG_RB];
			hold_angles_rad[LEG_LB] = jointControl.JointEstimate.motor_angles_rad[LEG_LB];

			float hold_pos_rad[JOINT_CTRL_MOTOR_NUM];
			JointBuildFeedbackPosRadFromMotorAngles(hold_angles_rad, hold_pos_rad);

			for (uint8_t leg = LEG_LF; leg <= LEG_RF; leg++)
			{
				jointControl.JointMotorControl.mit_p[leg]   = hold_pos_rad[leg];
				jointControl.JointMotorControl.mit_v[leg]   = 0.0f;
				jointControl.JointMotorControl.mit_Kp[leg]  = mit_kp_table[MIT_MODE_FORCE_LIMIT][leg];
				jointControl.JointMotorControl.mit_Kd[leg]  = mit_kd_table[MIT_MODE_FORCE_LIMIT][leg];
			}
		}

		last_speed_y_mps = _chassisControl->ChassisRealNeedInput.speed_y_mps;
	}

	/* X方向移动时前腿位控保持，防止横向惯性压弯前腿角度突变 */
	/* PRE_DOWN_STAIR 模式下跳过 */
	if (_robotState->stand_mode != ROBOT_STAND_MODE_PRE_DOWN_STAIR /* TODO: add HSM PreDownStair */ && pDecisionAO->chassis_mode != CHASSIS_REVOLVE && pDecisionAO->joint_mode != JOINT_PRESTAIR && pDecisionAO->joint_mode != JOINT_STAIRUP && rev_exit_hold_cnt == 0)
	{
		static float last_speed_x_mps = 0.0f;
		static float x_hold_angle_rad[2] = {0.0f, 0.0f}; /* LF, RF */
		static uint8_t x_hold_active = 0;
		const float x_speed_deadband = 0.05f;  /* 低于此值视为无X输入 */

		/* 检测X方向速度输入开始边沿：从静止到运动，记录输入前的关节角 */
		if (fabs(last_speed_x_mps) < x_speed_deadband &&
		    fabs(_chassisControl->ChassisRealNeedInput.speed_x_mps) >= x_speed_deadband)
		{
			x_hold_angle_rad[0] = jointControl.JointEstimate.motor_angles_rad[LEG_LF];
			x_hold_angle_rad[1] = jointControl.JointEstimate.motor_angles_rad[LEG_RF];
			x_hold_active = 1;
		}

		/* X方向速度输入结束，释放保持 */
		if (fabs(_chassisControl->ChassisRealNeedInput.speed_x_mps) < x_speed_deadband)
		{
			x_hold_active = 0;
		}

		if (x_hold_active)
		{
			/* 前腿切位控模式，锁定速度输入前的角度 */
			float hold_angles_rad[JOINT_CTRL_MOTOR_NUM];
			hold_angles_rad[LEG_LF] = x_hold_angle_rad[0];
			hold_angles_rad[LEG_RF] = x_hold_angle_rad[1];
			hold_angles_rad[LEG_RB] = jointControl.JointEstimate.motor_angles_rad[LEG_RB];
			hold_angles_rad[LEG_LB] = jointControl.JointEstimate.motor_angles_rad[LEG_LB];

			float hold_pos_rad[JOINT_CTRL_MOTOR_NUM];
			JointBuildFeedbackPosRadFromMotorAngles(hold_angles_rad, hold_pos_rad);

			for (uint8_t leg = LEG_LF; leg <= LEG_RF; leg++)
			{
				jointControl.JointMotorControl.mit_p[leg]   = hold_pos_rad[leg];
				jointControl.JointMotorControl.mit_v[leg]   = 0.0f;
				jointControl.JointMotorControl.mit_Kp[leg]  = mit_kp_table[MIT_MODE_FORCE_LIMIT][leg];
				jointControl.JointMotorControl.mit_Kd[leg]  = mit_kd_table[MIT_MODE_FORCE_LIMIT][leg];
			}
		}

		last_speed_x_mps = _chassisControl->ChassisRealNeedInput.speed_x_mps;
	}
}
