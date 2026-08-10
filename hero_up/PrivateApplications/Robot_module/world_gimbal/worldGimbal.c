/**
 * @file worldGimbal.c
 * @brief 世界系虚拟目标和阻尼最小二乘 IK
 * @note 机体坐标系 B：x 向前、y 向右、z 向下
 */

#include "worldGimbal_internal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "algorism.h"
#include "general_config_label.h"
#include "gimbalControl.h"
#include "task_receive.h"

static WorldGimbal g_world_gimbal = {0};
const WorldGimbal *const _worldGimbal = &g_world_gimbal;

extern volatile float g_b2b_body_pitch_d;
extern volatile float g_b2b_body_roll_d;
extern volatile float g_b2b_body_yaw_d;

static const float yaw_axis_body[3] = {0.0f, 0.0f, 1.0f};
static const float pitch_axis_zero_body[3] = {0.0f, 1.0f, 0.0f};
static const float forward_zero_body[3] = {1.0f, 0.0f, 0.0f};

static float vector_dot(const float *left, const float *right);
static void vector_cross(const float *left, const float *right, float *result);
static float vector_norm(const float *vector);
static void vector_normalize(float *vector);
static void vector_copy(float *destination, const float *source);
static void vector_rotate(const float *axis, float angle_rad,
                          const float *vector, float *result);
static void forward_kinematics(float yaw_rad, float pitch_rad,
                               float *forward_out, float *pitch_axis_out);
static float current_yaw_d(void);
static float pitch_encoder_to_d(uint16_t encoder);
static void compute_gravity_body(float chassis_roll_d, float chassis_pitch_d,
                                 float *gravity_out);
static void world_angles_to_target_vector(float yaw_d, float pitch_d,
                                          const float *gravity_body,
                                          float *target_out);

static float vector_dot(const float *left, const float *right){
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

static void vector_cross(const float *left, const float *right, float *result){
    result[0] = left[1] * right[2] - left[2] * right[1];
    result[1] = left[2] * right[0] - left[0] * right[2];
    result[2] = left[0] * right[1] - left[1] * right[0];
}

static float vector_norm(const float *vector){
    return sqrtf(vector[0] * vector[0] +
                 vector[1] * vector[1] +
                 vector[2] * vector[2]);
}

static void vector_normalize(float *vector){
    float norm = vector_norm(vector);

    if(norm > WORLD_GIMBAL_NORMALIZE_EPSILON){
        float inverse_norm = 1.0f / norm;
        vector[0] *= inverse_norm;
        vector[1] *= inverse_norm;
        vector[2] *= inverse_norm;
    }
}

static void vector_copy(float *destination, const float *source){
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
}

static void vector_rotate(const float *axis, float angle_rad,
                          const float *vector, float *result){
    float normalized_axis[3];
    float cross_product[3];
    float cosine = cosf(angle_rad);
    float sine = sinf(angle_rad);
    float one_minus_cosine = 1.0f - cosine;
    float axis_dot_vector;

    vector_copy(normalized_axis, axis);
    vector_normalize(normalized_axis);
    axis_dot_vector = vector_dot(normalized_axis, vector);
    vector_cross(normalized_axis, vector, cross_product);

    result[0] = vector[0] * cosine + cross_product[0] * sine +
                normalized_axis[0] * axis_dot_vector * one_minus_cosine;
    result[1] = vector[1] * cosine + cross_product[1] * sine +
                normalized_axis[1] * axis_dot_vector * one_minus_cosine;
    result[2] = vector[2] * cosine + cross_product[2] * sine +
                normalized_axis[2] * axis_dot_vector * one_minus_cosine;
}

static void forward_kinematics(float yaw_rad, float pitch_rad,
                               float *forward_out, float *pitch_axis_out){
    float after_pitch[3];

    vector_rotate(pitch_axis_zero_body, pitch_rad, forward_zero_body, after_pitch);
    vector_rotate(yaw_axis_body, yaw_rad, after_pitch, forward_out);
    vector_normalize(forward_out);

    if(pitch_axis_out != NULL){
        vector_rotate(yaw_axis_body, yaw_rad, pitch_axis_zero_body, pitch_axis_out);
        vector_normalize(pitch_axis_out);
    }
}

static float current_yaw_d(void){
    float yaw_d = (_DMyawMotorRec->pos_d - yaw_dm_forward_offset_rad) *
                  WORLD_GIMBAL_RAD_TO_DEG;
    return AngleLimit(yaw_d, -WORLD_GIMBAL_HALF_TURN_D, WORLD_GIMBAL_HALF_TURN_D);
}

static float pitch_encoder_to_d(uint16_t encoder){
    float pitch_d = ((float)encoder - PITCH_OFFSET_MACHENICAL_ANGLE) *
                    WORLD_GIMBAL_FULL_ROTATION_D / LK_FULL_CIRCLE_MECHENICAL_ANGLE;
    return AngleLimit(pitch_d, -WORLD_GIMBAL_HALF_TURN_D, WORLD_GIMBAL_HALF_TURN_D);
}

static void compute_gravity_body(float chassis_roll_d, float chassis_pitch_d,
                                 float *gravity_out){
    float roll_rad = chassis_roll_d * WORLD_GIMBAL_DEG_TO_RAD;
    float pitch_rad = chassis_pitch_d * WORLD_GIMBAL_DEG_TO_RAD;
    float sin_pitch = sinf(pitch_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_roll = sinf(roll_rad);
    float cos_roll = cosf(roll_rad);

    gravity_out[0] = -sin_pitch;
    gravity_out[1] = cos_pitch * sin_roll;
    gravity_out[2] = cos_pitch * cos_roll;
    vector_normalize(gravity_out);
}

static void world_angles_to_target_vector(float yaw_d, float pitch_d,
                                          const float *gravity_body,
                                          float *target_out){
    float yaw_rad = yaw_d * WORLD_GIMBAL_DEG_TO_RAD;
    float pitch_rad = pitch_d * WORLD_GIMBAL_DEG_TO_RAD;
    float up_body[3] = {
        -gravity_body[0],
        -gravity_body[1],
        -gravity_body[2]};
    float horizontal_forward[3] = {
        1.0f - gravity_body[0] * gravity_body[0],
        -gravity_body[0] * gravity_body[1],
        -gravity_body[0] * gravity_body[2]};
    float horizontal_right[3];
    float horizontal_norm = vector_norm(horizontal_forward);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);
    float target_horizontal[3] = {
        cos_pitch * cos_yaw,
        cos_pitch * sin_yaw,
        sin_pitch};

    if(horizontal_norm > WORLD_GIMBAL_HORIZONTAL_EPSILON){
        float inverse_norm = 1.0f / horizontal_norm;
        horizontal_forward[0] *= inverse_norm;
        horizontal_forward[1] *= inverse_norm;
        horizontal_forward[2] *= inverse_norm;
    } else {
        horizontal_forward[0] = 1.0f;
        horizontal_forward[1] = 0.0f;
        horizontal_forward[2] = 0.0f;
    }

    vector_cross(up_body, horizontal_forward, horizontal_right);
    target_out[0] = target_horizontal[0] * horizontal_forward[0] +
                    target_horizontal[1] * horizontal_right[0] +
                    target_horizontal[2] * up_body[0];
    target_out[1] = target_horizontal[0] * horizontal_forward[1] +
                    target_horizontal[1] * horizontal_right[1] +
                    target_horizontal[2] * up_body[1];
    target_out[2] = target_horizontal[0] * horizontal_forward[2] +
                    target_horizontal[1] * horizontal_right[2] +
                    target_horizontal[2] * up_body[2];
    vector_normalize(target_out);
}

void WorldGimbalInitialize(void){
    memset(&g_world_gimbal, 0, sizeof(g_world_gimbal));
    world_target_vector[0] = 1.0f;
    world_gravity_vector[2] = 1.0f;
    world_gimbal_enabled = WORLD_GIMBAL_DISABLED;
}

void WorldGimbalSetEnabled(uint8_t is_enabled){
    world_gimbal_enabled = is_enabled ? WORLD_GIMBAL_ENABLED : WORLD_GIMBAL_DISABLED;
}

void WorldGimbalAlignToCurrent(void){
    float yaw_rad = current_yaw_d() * WORLD_GIMBAL_DEG_TO_RAD;
    float pitch_rad = pitch_encoder_to_d(_pitchMotorRec->mechanical_angle) *
                      WORLD_GIMBAL_DEG_TO_RAD;

    forward_kinematics(yaw_rad, pitch_rad, world_target_vector, NULL);
    world_target_initialized = 1U;
    world_last_right_valid = 0U;
}

void WorldGimbalInputUpdate(float yaw_delta_d, float pitch_delta_d){
    if(world_gimbal_enabled == WORLD_GIMBAL_DISABLED){
        return;
    }
    if(!world_target_initialized){
        WorldGimbalAlignToCurrent();
    }

    if(fabsf(yaw_delta_d) > WORLD_GIMBAL_INPUT_EPSILON_D){
        vector_rotate(world_gravity_vector,
                      yaw_delta_d * WORLD_GIMBAL_DEG_TO_RAD,
                      world_target_vector,
                      world_target_vector);
        vector_normalize(world_target_vector);
    }

    if(fabsf(pitch_delta_d) > WORLD_GIMBAL_INPUT_EPSILON_D){
        float right_body[3];
        float right_norm;

        vector_cross(world_gravity_vector, world_target_vector, right_body);
        right_norm = vector_norm(right_body);
        if(right_norm > WORLD_GIMBAL_HORIZONTAL_EPSILON){
            vector_normalize(right_body);
            vector_rotate(right_body,
                          pitch_delta_d * WORLD_GIMBAL_DEG_TO_RAD,
                          world_target_vector,
                          world_target_vector);
            vector_normalize(world_target_vector);
            vector_copy(world_last_right_vector, right_body);
            world_last_right_valid = 1U;
        } else if(world_last_right_valid){
            vector_rotate(world_last_right_vector,
                          pitch_delta_d * WORLD_GIMBAL_DEG_TO_RAD,
                          world_target_vector,
                          world_target_vector);
            vector_normalize(world_target_vector);
        }
    }
}

void WorldGimbalEstimateUpdate(void){
    float yaw_rad;
    float pitch_rad;

    world_chassis_roll_d = g_b2b_body_roll_d;
    world_chassis_pitch_d = g_b2b_body_pitch_d;
    world_chassis_yaw_d = g_b2b_body_yaw_d;
    compute_gravity_body(world_chassis_roll_d,
                         world_chassis_pitch_d,
                         world_gravity_vector);

    yaw_rad = current_yaw_d() * WORLD_GIMBAL_DEG_TO_RAD;
    pitch_rad = pitch_encoder_to_d(_pitchMotorRec->mechanical_angle) *
                WORLD_GIMBAL_DEG_TO_RAD;
    forward_kinematics(yaw_rad,
                       pitch_rad,
                       world_real_vector,
                       world_pitch_axis_vector);

    if(world_target_initialized){
        float target_dot_real = vector_dot(world_target_vector, world_real_vector);
        const float *target = world_target_vector;
        const float *gravity = world_gravity_vector;
        float elevation_sine;
        float target_dot_gravity;
        float horizontal_target[3];
        float horizontal_forward[3];
        float horizontal_target_norm;
        float horizontal_forward_norm;

        target_dot_real = DoubleEdgeLimiter(target_dot_real,
                                            WORLD_GIMBAL_UNIT_MIN,
                                            WORLD_GIMBAL_UNIT_MAX);
        world_angle_error_d = acosf(target_dot_real) * WORLD_GIMBAL_RAD_TO_DEG;

        elevation_sine = DoubleEdgeLimiter(-vector_dot(target, gravity),
                                           WORLD_GIMBAL_UNIT_MIN,
                                           WORLD_GIMBAL_UNIT_MAX);
        world_pitch_d = asinf(elevation_sine) * WORLD_GIMBAL_RAD_TO_DEG;

        target_dot_gravity = vector_dot(target, gravity);
        horizontal_target[0] = target[0] - target_dot_gravity * gravity[0];
        horizontal_target[1] = target[1] - target_dot_gravity * gravity[1];
        horizontal_target[2] = target[2] - target_dot_gravity * gravity[2];
        horizontal_target_norm = vector_norm(horizontal_target);

        horizontal_forward[0] = 1.0f - gravity[0] * gravity[0];
        horizontal_forward[1] = -gravity[0] * gravity[1];
        horizontal_forward[2] = -gravity[0] * gravity[2];
        horizontal_forward_norm = vector_norm(horizontal_forward);

        if(horizontal_target_norm > WORLD_GIMBAL_HORIZONTAL_EPSILON &&
           horizontal_forward_norm > WORLD_GIMBAL_HORIZONTAL_EPSILON){
            float cross_forward_target[3];
            float forward_dot_target;

            vector_normalize(horizontal_target);
            vector_normalize(horizontal_forward);
            vector_cross(horizontal_forward, horizontal_target, cross_forward_target);
            forward_dot_target = DoubleEdgeLimiter(vector_dot(horizontal_forward,
                                                              horizontal_target),
                                                   WORLD_GIMBAL_UNIT_MIN,
                                                   WORLD_GIMBAL_UNIT_MAX);
            world_yaw_d = atan2f(vector_dot(gravity, cross_forward_target),
                                 forward_dot_target) *
                          WORLD_GIMBAL_RAD_TO_DEG;
        } else {
            world_yaw_d = 0.0f;
        }
    } else {
        world_pitch_d = 0.0f;
        world_yaw_d = 0.0f;
    }
}

void WorldGimbalIKSolve(void){
    float yaw_rad;
    float pitch_rad;
    uint8_t iteration;

    if(world_gimbal_enabled == WORLD_GIMBAL_DISABLED || !world_target_initialized){
        return;
    }

    yaw_rad = current_yaw_d() * WORLD_GIMBAL_DEG_TO_RAD;
    pitch_rad = pitch_encoder_to_d(_pitchMotorRec->mechanical_angle) *
                WORLD_GIMBAL_DEG_TO_RAD;

    for(iteration = 0U; iteration < WORLD_GIMBAL_IK_MAX_ITERS; iteration++){
        float real_vector[3];
        float pitch_axis[3];
        float rotation_error[3];
        float direction_error[3];
        float yaw_jacobian[3];
        float pitch_jacobian[3];
        float rotation_error_norm;
        float matrix_00;
        float matrix_01;
        float matrix_11;
        float gradient_0;
        float gradient_1;
        float determinant;
        float yaw_step;
        float pitch_step;

        forward_kinematics(yaw_rad, pitch_rad, real_vector, pitch_axis);
        vector_cross(real_vector, world_target_vector, rotation_error);
        rotation_error_norm = vector_norm(rotation_error);
        if(rotation_error_norm < WORLD_GIMBAL_IK_CONVERGE_RAD){
            break;
        }

        vector_cross(rotation_error, real_vector, direction_error);
        vector_cross(yaw_axis_body, real_vector, yaw_jacobian);
        vector_cross(pitch_axis, real_vector, pitch_jacobian);

        matrix_00 = vector_dot(yaw_jacobian, yaw_jacobian) + WORLD_GIMBAL_IK_LAMBDA;
        matrix_01 = vector_dot(yaw_jacobian, pitch_jacobian);
        matrix_11 = vector_dot(pitch_jacobian, pitch_jacobian) + WORLD_GIMBAL_IK_LAMBDA;
        gradient_0 = vector_dot(yaw_jacobian, direction_error);
        gradient_1 = vector_dot(pitch_jacobian, direction_error);
        determinant = matrix_00 * matrix_11 - matrix_01 * matrix_01;
        if(fabsf(determinant) < WORLD_GIMBAL_SINGULAR_EPSILON){
            break;
        }

        yaw_step = (matrix_11 * gradient_0 - matrix_01 * gradient_1) / determinant;
        pitch_step = (matrix_00 * gradient_1 - matrix_01 * gradient_0) / determinant;
        yaw_step = DoubleEdgeLimiter(yaw_step,
                                     -WORLD_GIMBAL_IK_MAX_STEP_RAD,
                                     WORLD_GIMBAL_IK_MAX_STEP_RAD);
        pitch_step = DoubleEdgeLimiter(pitch_step,
                                       -WORLD_GIMBAL_IK_MAX_STEP_RAD,
                                       WORLD_GIMBAL_IK_MAX_STEP_RAD);
        yaw_rad += yaw_step;
        pitch_rad += pitch_step;
    }

    world_yaw_command_rad = yaw_rad;
    world_pitch_command_rad = pitch_rad;
    world_yaw_command_d = yaw_rad * WORLD_GIMBAL_RAD_TO_DEG;
    world_pitch_command_d = pitch_rad * WORLD_GIMBAL_RAD_TO_DEG;
    world_ik_converged = (iteration < WORLD_GIMBAL_IK_MAX_ITERS) ? 1U : 0U;
    world_ik_iterations = iteration;
}

void WorldGimbalApplyToTargets(void){
    if(world_gimbal_enabled == WORLD_GIMBAL_DISABLED || !world_target_initialized){
        return;
    }

    GimbalSetTarget(world_yaw_command_d, world_pitch_command_d);
}

void WorldGimbalSetWorldAngles(float world_yaw_target_d, float world_pitch_target_d){
    if(world_gimbal_enabled == WORLD_GIMBAL_DISABLED){
        return;
    }

    world_angles_to_target_vector(world_yaw_target_d,
                                  world_pitch_target_d,
                                  world_gravity_vector,
                                  world_target_vector);
    world_target_initialized = 1U;
    world_last_right_valid = 0U;
}
