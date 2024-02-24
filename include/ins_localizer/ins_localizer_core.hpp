#ifndef INS_LOCALIZER_CORE_HPP
#define INS_LOCALIZER_CORE_HPP

#include <yaml-cpp/yaml.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/navigation/DopplerYawFactor.h>

#include "ins_localizer/sensor_data.hpp"
#include "ins_localizer/viewer.hpp"

using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::B;

namespace ins_localizer
{
enum class METHOD
{
  ISAM2 = 0,
  LEVEN = 1,
};

struct IMUParam
{
  double acc_noise;
  double gyro_noise;
  double bias_noise;

  int min_cnt_for_integration;
};

struct INSParam
{
  bool is_online;
  bool visualization;
  METHOD method;
  IMUParam imu;

  double robust_huber_param;

  void loadYAML(const std::string& yaml)
  {
    try
    {
      YAML::Node conf = YAML::LoadFile(yaml)["ins"];

      this->is_online = conf["is_online"].as<bool>();
      this->visualization = conf["visualization"].as<bool>();
      this->method = static_cast<METHOD>(conf["method"].as<int>());
      this->imu.acc_noise = conf["imu"]["acc_noise"].as<double>();
      this->imu.gyro_noise = conf["imu"]["gyro_noise"].as<double>();
      this->imu.bias_noise = conf["imu"]["bias_noise"].as<double>();
      this->imu.min_cnt_for_integration = conf["imu"]["min_cnt_for_integration"].as<int>();
      this->robust_huber_param = conf["robust_huber_param"].as<double>();
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "Error: Cannot load parameters from YAML" << std::endl;
      std::cerr << "File: " << yaml << std::endl;
      std::cerr << "Message: " << e.what() << std::endl;
      exit(1);
    }
  }
};

class INSLocalizer
{
public:
  INSLocalizer(const INSParam& param);
  ~INSLocalizer();
  void imuCallback(const IMUData& imu_data);
  void velocityCallback(const VelocityData& velocity_data);
  void gnss_dopplerCallback(const GNSSDopplerData& gnss_doppler_data);
  bool gnssPositionCallback(const GNSSData& gnss_data);
  void getWholeTrajectory(std::vector<StampedPose>& matrix_vector);

private:
  // Param
  INSParam param_;

  // Graph
  std::unique_ptr<gtsam::ISAM2> isam2_optimizer_;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph_;
  gtsam::Values graph_values_;

  std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrator_;
  bool is_initialized_;

  // Noise
  gtsam::noiseModel::Diagonal::shared_ptr noise_model_;
  gtsam::noiseModel::Isotropic::shared_ptr vel_model_;
  gtsam::noiseModel::Isotropic::shared_ptr bias_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr stop_model_;

  // Key
  int key_;
  std::unordered_map<int, double> key_stamp_map_;
  double pitch_;
  double yaw_;
  double vel_e_;
  double vel_n_;
  double vel_u_;
  

  // Estimation state
  gtsam::NavState prev_state_;
  gtsam::NavState prop_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  double last_gnss_stamp_;
  double last_imu_stamp_;
  gtsam::Values current_result_;

  // Preintegrated IMU count
  int preintegrated_imu_cnt_ = 0;

  // Velocity
  bool is_velocity_loaded_;
  bool is_stopped_;
  double velocity_;
  double last_velocity_;

  // gnss_doppler_yaw
  double doppler_yaw_;
  double doppler_est_status_;
  double doppler_status_;
  double gnss_doppler_bias_;
  double gnss_vel_e_;
  double gnss_vel_n_;
  double gnss_vel_u_;

  // Viewer
  OdometryViewer viewer_;

  // Functions
  void initialize(const double stamp, const Eigen::Vector3d& position);
  void optimizeGraph();
};
}

#endif
