#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP

#include <Eigen/Core>

namespace ins_localizer
{
struct GNSSData
{
  double stamp;
  Eigen::Vector3d position;
  double gps_bias;
  double status;
};

struct IMUData
{
  double stamp;
  Eigen::Vector3f acc;
  Eigen::Vector3f gyro;
};

struct VelocityData
{
  double stamp;
  float velocity;
};

struct GNSSDopplerData
{
  double stamp;
  Eigen::Vector3f gnss_vel;
  Eigen::Vector4f quaternion;
  double gnss_doppler_bias;
  double est_status;
  double status;
};

struct StampedPose
{
  double stamp;
  Eigen::Matrix4d pose;
  Eigen::Vector3d vel;
  Eigen::VectorXd imu_bias;

};
}

#endif
