#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 提供姿态和航向参考系统（AHRS）功能的模块 / A module providing Attitude and Heading Reference System (AHRS) functionality
constructor_args:
  - beta: 0.05
  - gyro_topic_name: "imu_gyro"
  - accl_topic_name: "imu_accl"
  - quaternion_topic_name: "ahrs_quaternion"
  - euler_topic_name: "ahrs_euler"
  - task_stack_depth: 2048
template_args: []
required_hardware: ramfs
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "libxr.hpp"
#include "transform.hpp"

class MadgwickAHRS : public LibXR::Application {
 public:
  MadgwickAHRS(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
               float beta, const char* gyro_topic_name,
               const char* accl_topic_name, const char* quaternion_topic_name,
               const char* euler_topic_name, uint32_t task_stack_depth)
      : beta_(beta),
        gyro_topic_name_(gyro_topic_name),
        accl_topic_name_(accl_topic_name),
        quaternion_topic_(LibXR::Topic::CreateTopic<decltype(quaternion_)>(quaternion_topic_name)),
        euler_topic_(LibXR::Topic::CreateTopic<decltype(euler_)>(euler_topic_name)),
        cmd_file_(LibXR::RamFS::CreateFile("ahrs", CommandFunc, this)) {
    UNUSED(hw);
    app.Register(*this);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    thread_.Create(this, ThreadFunc, "ahrs", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  void OnMonitor() override {
    if (std::isinf(quaternion_.x()) || std::isinf(quaternion_.y()) ||
        std::isinf(quaternion_.z()) || std::isinf(quaternion_.w()) ||
        std::isnan(quaternion_.x()) || std::isnan(quaternion_.y()) ||
        std::isnan(quaternion_.z()) || std::isnan(quaternion_.w())) {
      XR_LOG_WARN("AHRS: NaN data detected\r\n");
    };
  }

  static void ThreadFunc(MadgwickAHRS* ahrs) {
    LibXR::Topic::SyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        ahrs->gyro_topic_name_, ahrs->gyro_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> accl_suber(
        ahrs->accl_topic_name_);

    accl_suber.StartWaiting();

    while (true) {
      gyro_suber.Wait();
      const auto imu_timestamp = gyro_suber.GetTimestamp();

      if (accl_suber.Available()) {
        ahrs->accl_ = accl_suber.GetData();
        accl_suber.StartWaiting();
      }

      ahrs->Update(imu_timestamp);

      ahrs->quaternion_topic_.Publish(ahrs->quaternion_, imu_timestamp);
      ahrs->euler_topic_.Publish(ahrs->euler_, imu_timestamp);
    }
  }

  void Update(LibXR::MicrosecondTimestamp imu_timestamp) {
    // NOLINTBEGIN
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;
    float q_2q0, q_2q1, q_2q2, q_2q3, q_4q0, q_4q1, q_4q2, q_8q1, q_8q2, q0q0,
        q1q1, q2q2, q3q3;
    // NOLINTEND
    if (this->time_initialized_) {
      this->dt_ = (imu_timestamp - this->last_time_).ToSecondf();
    } else {
      this->dt_ = 0.0f;
      this->time_initialized_ = true;
    }
    this->last_time_ = imu_timestamp;

    float ax = this->accl_.x();
    float ay = this->accl_.y();
    float az = this->accl_.z();

    float gx = this->gyro_.x();
    float gy = this->gyro_.y();
    float gz = this->gyro_.z();

    /* Rate of change of quaternion from gyroscope */
    q_dot1 = 0.5f * (-this->quaternion_.x() * gx - this->quaternion_.y() * gy -
                     this->quaternion_.z() * gz);
    q_dot2 = 0.5f * (this->quaternion_.w() * gx + this->quaternion_.y() * gz -
                     this->quaternion_.z() * gy);
    q_dot3 = 0.5f * (this->quaternion_.w() * gy - this->quaternion_.x() * gz +
                     this->quaternion_.z() * gx);
    q_dot4 = 0.5f * (this->quaternion_.w() * gz + this->quaternion_.x() * gy -
                     this->quaternion_.y() * gx);

    /* Compute feedback only if accelerometer measurement valid (avoids NaN in
     * accelerometer normalisation) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      /* Normalise accelerometer measurement */
      recip_norm = InvSqrtf(ax * ax + ay * ay + az * az);
      ax *= recip_norm;
      ay *= recip_norm;
      az *= recip_norm;

      /* Auxiliary variables to avoid repeated arithmetic */
      q_2q0 = 2.0f * this->quaternion_.w();
      q_2q1 = 2.0f * this->quaternion_.x();
      q_2q2 = 2.0f * this->quaternion_.y();
      q_2q3 = 2.0f * this->quaternion_.z();
      q_4q0 = 4.0f * this->quaternion_.w();
      q_4q1 = 4.0f * this->quaternion_.x();
      q_4q2 = 4.0f * this->quaternion_.y();
      q_8q1 = 8.0f * this->quaternion_.x();
      q_8q2 = 8.0f * this->quaternion_.y();
      q0q0 = this->quaternion_.w() * this->quaternion_.w();
      q1q1 = this->quaternion_.x() * this->quaternion_.x();
      q2q2 = this->quaternion_.y() * this->quaternion_.y();
      q3q3 = this->quaternion_.z() * this->quaternion_.z();

      /* Gradient decent algorithm corrective step */
      s0 = q_4q0 * q2q2 + q_2q2 * ax + q_4q0 * q1q1 - q_2q1 * ay;
      s1 = q_4q1 * q3q3 - q_2q3 * ax + 4.0f * q0q0 * this->quaternion_.x() -
           q_2q0 * ay - q_4q1 + q_8q1 * q1q1 + q_8q1 * q2q2 + q_4q1 * az;
      s2 = 4.0f * q0q0 * this->quaternion_.y() + q_2q0 * ax + q_4q2 * q3q3 -
           q_2q3 * ay - q_4q2 + q_8q2 * q1q1 + q_8q2 * q2q2 + q_4q2 * az;
      s3 = 4.0f * q1q1 * this->quaternion_.z() - q_2q1 * ax +
           4.0f * q2q2 * this->quaternion_.z() - q_2q2 * ay;

      /* normalise step magnitude */
      recip_norm = InvSqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

      s0 *= recip_norm;
      s1 *= recip_norm;
      s2 *= recip_norm;
      s3 *= recip_norm;

      /* Apply feedback step */
      q_dot1 -= beta_ * s0;
      q_dot2 -= beta_ * s1;
      q_dot3 -= beta_ * s2;
      q_dot4 -= beta_ * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    this->quaternion_.w() += q_dot1 * this->dt_;
    this->quaternion_.x() += q_dot2 * this->dt_;
    this->quaternion_.y() += q_dot3 * this->dt_;
    this->quaternion_.z() += q_dot4 * this->dt_;

    /* Normalise quaternion */
    recip_norm = InvSqrtf(this->quaternion_.w() * this->quaternion_.w() +
                          this->quaternion_.x() * this->quaternion_.x() +
                          this->quaternion_.y() * this->quaternion_.y() +
                          this->quaternion_.z() * this->quaternion_.z());
    this->quaternion_.w() *= recip_norm;
    this->quaternion_.x() *= recip_norm;
    this->quaternion_.y() *= recip_norm;
    this->quaternion_.z() *= recip_norm;

    this->euler_ = this->quaternion_.ToEulerAngle();
  }

  float InvSqrtf(float x) {
#if 0
  /* Fast inverse square-root */
  /* See: http://en.wikipedia.org/wiki/Fast_inverse_square_root */
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
#else
    return 1.0f / sqrtf(x);
#endif
  }

 private:
  static int CommandFunc(MadgwickAHRS* ahrs, int argc, char** argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<"  show [time_ms] [interval_ms]            - Show Euler angles and  "
          "quaternion periodically.\r\n">();
      LibXR::STDIO::Printf<"  print_quat  [time_ms] [interval_ms]     - Show quaternion in "
          "VOFA+ format periodically.\r\n">();
      LibXR::STDIO::Printf<"  test                                    - Test gyroscope "
          "calibration.\r\n">();
    } else if (argc == 2) {
      if (strcmp(argv[1], "test") == 0) {
        LibXR::STDIO::Printf<"Please keep the device steady, start measurement\r\n">();
        LibXR::Thread::Sleep(3000);
        LibXR::STDIO::Printf<"Please wait\r\n">();
        float start_yaw = ahrs->euler_.Yaw();
        LibXR::Thread::Sleep(10000);
        float yaw = ahrs->euler_.Yaw();

        LibXR::STDIO::Printf<"Zero offset:%f°/min\r\n">(
                             (yaw - start_yaw) / M_PI * 180.0f * 6.0f);
      }
    } else if (argc == 4) {
      std::string cmd(argv[1]);
      int time = std::stoi(argv[2]);
      int interval = std::stoi(argv[3]);

      interval = std::clamp(interval, 2, 1000);

      if (cmd == "show") {
        while (time > 0) {
          LibXR::STDIO::Printf<"Euler: pitch=%+7.5f, roll=%+7.5f, yaw=%+7.5f Quat: w="
              "%+6.4f, x=%+6.4f, y=%+6.4f, z=%+6.4f, dt=%+7.5f\r\n">(
              ahrs->euler_.Pitch(), ahrs->euler_.Roll(), ahrs->euler_.Yaw(),
              ahrs->quaternion_.w(), ahrs->quaternion_.x(),
              ahrs->quaternion_.y(), ahrs->quaternion_.z(), ahrs->dt_);
          LibXR::Thread::Sleep(interval);
          time -= interval;
        }
      } else if (cmd == "print_quat") {
        while (time > 0) {
          LibXR::STDIO::Printf<"%f,%f,%f,%f\n">(ahrs->quaternion_.w(),
                               ahrs->quaternion_.x(), ahrs->quaternion_.y(),
                               ahrs->quaternion_.z());
          LibXR::Thread::Sleep(interval);
          time -= interval;
        }
      } else {
        LibXR::STDIO::Printf<"Error: Unknown command: %s\r\n">(argv[1]);
      }
    } else {
      LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
    }

    return 0;
  }

  float beta_;
  const char* gyro_topic_name_;
  const char* accl_topic_name_;

  LibXR::Topic quaternion_topic_;
  LibXR::Topic euler_topic_;

  LibXR::Quaternion<float> quaternion_;
  LibXR::EulerAngle<float> euler_;
  Eigen::Matrix<float, 3, 1> accl_, gyro_;

  LibXR::MicrosecondTimestamp last_time_ = 0;
  bool time_initialized_ = false;
  float dt_ = 0;

  LibXR::Thread thread_;

  LibXR::RamFS::File cmd_file_;
};
