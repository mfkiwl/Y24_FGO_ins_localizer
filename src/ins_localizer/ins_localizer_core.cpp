#include "ins_localizer/ins_localizer_core.hpp"
#include "math.h"
#include <iostream>
#include <fstream>
#include <iomanip>

namespace ins_localizer
{
INSLocalizer::INSLocalizer(const INSParam& param)
{
  param_ = param;

  // Initialize ISAM2 optimizer
  if (param_.method == METHOD::ISAM2)
  {
    gtsam::ISAM2Params isam2_param;
    isam2_param.relinearizeThreshold = 0.1;
    isam2_param.relinearizeSkip = 1;
    isam2_optimizer_.reset(new gtsam::ISAM2(isam2_param));
  }

  // Visualizer
  if (param_.visualization == true)
  {
    viewer_.openViewer();
  }

  key_ = 0;
  is_initialized_ = false;
}

INSLocalizer::~INSLocalizer()
{
  if (param_.visualization == true)
  {
    viewer_.closeViewer();
  }
}

void INSLocalizer::initialize(const double stamp, const Eigen::Vector3d& position)
{
  // Add prior
  gtsam::Rot3 prior_rot = gtsam::Rot3::Quaternion(1.0, 0, 0, 0);
  gtsam::Point3 prior_point(position(0), position(1), position(2));
  gtsam::Pose3 prior_pose(prior_rot, prior_point);
  gtsam::Vector3 prior_vel(0, 0, 0);
  gtsam::imuBias::ConstantBias prior_imu_bias;

  graph_values_.insert(X(key_), prior_pose);
  graph_values_.insert(V(key_), prior_vel);
  graph_values_.insert(B(key_), prior_imu_bias);

  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 6.28, 6.28, 6.28, 0.5, 0.5, 0.5).finished());
  auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  bias_noise_ = gtsam::noiseModel::Isotropic::Precision(6, param_.imu.bias_noise);
  stop_model_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());

  graph_.reset(new gtsam::NonlinearFactorGraph());
  graph_->addPrior(X(key_), prior_pose, pose_noise);
  graph_->addPrior(V(key_), prior_vel, vel_noise);
  graph_->addPrior(B(key_), prior_imu_bias, bias_noise_);

  key_stamp_map_[key_] = stamp;

  // Setup IMU preintegrator
  auto imu_param = gtsam::PreintegrationParams::MakeSharedU();
  imu_param->setAccelerometerCovariance(gtsam::I_3x3 * std::pow(param_.imu.acc_noise, 2));
  imu_param->setGyroscopeCovariance(gtsam::I_3x3 * std::pow(param_.imu.gyro_noise, 2));
  
  imu_preintegrator_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(imu_param, prior_imu_bias);

  // Setup velocity state
  is_velocity_loaded_ = false;
  is_stopped_ = true;

  // Setup state
  prev_state_ = gtsam::NavState(prior_pose, prior_vel);
  prop_state_ = prev_state_;
  prev_bias_ = prior_imu_bias;

  last_gnss_stamp_ = stamp;
  last_imu_stamp_ = stamp;

  is_initialized_ = true;
}

void INSLocalizer::imuCallback(const IMUData& imu_data)
{
  // Do nothing until graph is initialized
  if (!is_initialized_)
    return;
  
  // Do nothing with past data
  if (imu_data.stamp <= last_gnss_stamp_)
    return;
  
  double dt = imu_data.stamp - last_imu_stamp_;
  gtsam::Vector3 measured_acc(imu_data.acc(0), imu_data.acc(1), imu_data.acc(2));
  gtsam::Vector3 measured_gyro(imu_data.gyro(0), imu_data.gyro(1), imu_data.gyro(2));

  imu_preintegrator_->integrateMeasurement(measured_acc, measured_gyro, dt);
  last_imu_stamp_ = imu_data.stamp;

  preintegrated_imu_cnt_++;
}

void INSLocalizer::velocityCallback(const VelocityData& velocity_data)
{
  // Do nothing until graph is initialized
  if (!is_initialized_)
    return;
  
  if (velocity_data.stamp <= last_gnss_stamp_)
    return;

  is_velocity_loaded_ = true;
  is_stopped_ = is_stopped_ && (velocity_data.velocity == 0);
  velocity_ = velocity_data.velocity;
}

void INSLocalizer::gnss_dopplerCallback(const GNSSDopplerData& gnss_doppler_data)
{
  // Do nothing until graph is initialized
  if (!is_initialized_)
    return;
  
  if (gnss_doppler_data.stamp <= last_gnss_stamp_)
    return;
    
  // doppler_yaw_ = -1*atan2(gnss_doppler_data.gnss_vel(0),gnss_doppler_data.gnss_vel(1)); //GNSS Doppler Heading calculation
  
  doppler_yaw_ = -1*gnss_doppler_data.gnss_vel(2) + M_PI/2; //Eagleye Heading
  doppler_yaw_ = std::fmod(doppler_yaw_ + 3 * M_PI / 2, 2 * M_PI) - M_PI;

  // while (doppler_yaw_ > M_PI) doppler_yaw_ -= 2 * M_PI;
  // while (doppler_yaw_ < -M_PI) doppler_yaw_ += 2 * M_PI;


  // gnss_vel_e_ = gnss_doppler_data.gnss_vel(0);
  // gnss_vel_n_ = gnss_doppler_data.gnss_vel(1);
  // gnss_vel_u_ = gnss_doppler_data.gnss_vel(2);

  doppler_est_status_ = gnss_doppler_data.est_status;
  doppler_status_ = gnss_doppler_data.status;
  gnss_doppler_bias_ = gnss_doppler_data.gnss_doppler_bias;
  
}

// Return true if you can get the estimated pose for this timestamp
bool INSLocalizer::gnssPositionCallback(const GNSSData& gnss_data)
{
  // Initialize graph if not initialized yet
  if (!is_initialized_)
  {
    initialize(gnss_data.stamp, gnss_data.position);
    return param_.is_online;
  }

  // Do nothing if enough IMU data are not preintegrated
  if (preintegrated_imu_cnt_ < param_.imu.min_cnt_for_integration)
    return false;

  // Update key and reset imu count
  key_++;
  preintegrated_imu_cnt_ = 0;
  last_gnss_stamp_ = gnss_data.stamp;
  key_stamp_map_[key_] = gnss_data.stamp;

  // Add IMU Factor
  auto tmp_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imu_preintegrator_);
  gtsam::ImuFactor imu_factor(X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), tmp_imu);
  // graph_->add(imu_factor);

  if (key_ < 10){
    graph_->add(imu_factor);
  }
  
  gtsam::imuBias::ConstantBias zero_bias;
  graph_->add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key_ - 1), B(key_), zero_bias, bias_noise_));
  
  

  if (is_velocity_loaded_ && is_stopped_)
  {
    gtsam::Rot3 relative_rot = gtsam::Rot3::Quaternion(1, 0, 0, 0);
    gtsam::Point3 relative_point(0, 0, 0);
    gtsam::Pose3 relative_pose(relative_rot, relative_point);
    graph_->add(gtsam::BetweenFactor<gtsam::Pose3>(X(key_ - 1), X(key_), relative_pose, stop_model_));
    vel_e_ = 0.0;
    vel_n_ = 0.0;
    vel_u_ = 0.0;
    gtsam::Vector3 vel_enu(vel_e_, vel_n_, vel_u_);
    auto vel_enu_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.5, 0.5, 0.5).finished());
    graph_->addPrior(V(key_), vel_enu, vel_enu_noise);
    std::cout << "\033[33m";
    
  }
  else
  {
    graph_->add(imu_factor);
    // Add GNSS_Doppler
    gtsam::Rot3 doppler_rot = gtsam::Rot3::Rz(doppler_yaw_);
    gtsam::Point3 doppler_point(0, 0, 0);
    gtsam::Pose3 doppler_pose(doppler_rot, doppler_point);
    auto yaw_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1000000.0, 1000000.0, gnss_doppler_bias_, 1000000.0, 1000000.0, 1000000.0).finished());
    // auto yaw_noise = gtsam::noiseModel::Isotropic::Sigma(1, gnss_doppler_bias_); //for coustom Factor

    // Add GNSS Doppler Heading
    if (doppler_status_ == 1)
    // if (gnss_data.status == 4 && velocity_ > 3 )
    {
      // gtsam::DopplerYawFactor doppler_yaw_Factor(X(key_), doppler_yaw_, yaw_noise); //for coustom Factor
      // graph_->add(doppler_yaw_Factor); //for coustom Factor
      graph_->addPrior(X(key_), doppler_pose, yaw_noise);

    }
    // Add GPS Factor
    auto correction_noise = gtsam::noiseModel::Robust::Create(
                                gtsam::noiseModel::mEstimator::Huber::Create(param_.robust_huber_param),
                                gtsam::noiseModel::Diagonal::Variances(gtsam::Vector3(gnss_data.gps_bias, gnss_data.gps_bias, gnss_data.gps_bias)));
    gtsam::Point3 gps_point(gnss_data.position(0), gnss_data.position(1), gnss_data.position(2));
    gtsam::GPSFactor gps_factor(X(key_), gps_point, correction_noise);
    
    if (gnss_data.status == 4.0 )
    {
      graph_->add(gps_factor);
      gtsam::Pose3 pose = current_result_.at<gtsam::Pose3>(X(key_-1));
    //Pose3からRot3（回転）を取得
      gtsam::Rot3 rot = pose.rotation();

    //Rot3からyaw角を計算
      pitch_ = rot.pitch();
      yaw_ = rot.yaw()-M_PI/2;
      yaw_ = std::fmod(yaw_ + 3 * M_PI / 2, 2 * M_PI) - M_PI;
      vel_e_ = last_velocity_ * cos(yaw_);
      vel_n_ = last_velocity_ * sin(yaw_);
      vel_u_ = last_velocity_ * sin(-1*pitch_-2.3*M_PI/180);
      gtsam::Vector3 vel_enu(vel_e_, vel_n_, vel_u_);
      auto vel_enu_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 100.0, 100.0, 100.0).finished());
      if (doppler_est_status_ == 1){
        graph_->addPrior(V(key_-1), vel_enu, vel_enu_noise);
      }
      // graph_->addPrior(V(key_-1), vel_enu, vel_enu_noise);
    }
    else
    {
      gtsam::Pose3 pose = current_result_.at<gtsam::Pose3>(X(key_-1));
    //Pose3からRot3（回転）を取得
      gtsam::Rot3 rot = pose.rotation();

    //Rot3からyaw角を計算
      pitch_ = rot.pitch();
      yaw_ = rot.yaw()-M_PI/2;
      yaw_ = std::fmod(yaw_ + 3 * M_PI / 2, 2 * M_PI) - M_PI;
      vel_e_ = last_velocity_ * cos(yaw_);
      vel_n_ = last_velocity_ * sin(yaw_);
      vel_u_ = last_velocity_ * sin(-1*pitch_-2.3*M_PI/180);
      // gtsam::Vector3 vel_enu(vel_e_, vel_n_, vel_u_);
      gtsam::Vector3 vel_enu(vel_e_, vel_n_, vel_u_);
    // auto vel_enu_noise = gtsam::noiseModel::Isotropic::Sigma(3, 10.0);
      auto vel_enu_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 35.0, 35.0, 35.0).finished());
      if (doppler_est_status_ == 1){
        graph_->addPrior(V(key_-1), vel_enu, vel_enu_noise);
      }
      // graph_->addPrior(V(key_-1), vel_enu, vel_enu_noise);
    }

    std::cout << "\033[32m";
  }

  // データ確認用
  // std::string file_name_y_predict_b = "/home/oem/ins_localizer/FGO_csv/test_vel/vel_test.csv";
  // std::ofstream ofs_y_predict_b;
  // ofs_y_predict_b.open(file_name_y_predict_b,std::ios::app);
  // if(!ofs_y_predict_b){
  //   std::cout << "ファイルエラー" << std::endl;
  // }else{
    
  // }  
  // ofs_y_predict_b << key_stamp_map_[key_]<< "," << vel_e_ << "," << vel_n_ << "," << vel_u_  << "," << yaw_ << "," << pitch_ << "," << velocity_ << "," << doppler_yaw_ << "," << std::endl;
  //   // std::cout << "現在のファイルの位置:y_pred" << ofs_y_predict_b.tellp() << std::endl;
  // ofs_y_predict_b.close();
  
  last_velocity_ = velocity_;
  gtsam::NavState prop_state = imu_preintegrator_->predict(prev_state_, prev_bias_);
  graph_values_.insert(X(key_), prop_state.pose());
  graph_values_.insert(V(key_), prop_state.v());
  graph_values_.insert(B(key_), prev_bias_);

  // Reset stop state
  is_stopped_ = true;
  is_velocity_loaded_ = false;

  if (param_.is_online)
  {
    // Update graph with optimization
    optimizeGraph();

    // Update state
    prev_state_ = gtsam::NavState(current_result_.at<gtsam::Pose3>(X(key_)), current_result_.at<gtsam::Vector3>(V(key_)));
    prev_bias_ = current_result_.at<gtsam::imuBias::ConstantBias>(B(key_));

    // Reset IMU preintegration
    imu_preintegrator_->resetIntegrationAndSetBias(prev_bias_);

    if (param_.visualization == true)
    {
      Eigen::Matrix4f optimized_pose = current_result_.at<gtsam::Pose3>(X(key_)).matrix().cast<float>();
      Eigen::Matrix4f gps_pose = Eigen::Matrix4f::Identity();
      gps_pose(0, 3) = gnss_data.position(0);
      gps_pose(1, 3) = gnss_data.position(1);
      gps_pose(2, 3) = gnss_data.position(2);

      viewer_.addGreenTraj(optimized_pose);
      if (gnss_data.gps_bias < 1.0)
        viewer_.addBlueTraj(gps_pose);
      else
        viewer_.addRedTraj(gps_pose);
    }

    return true;
  }

  return false;
}

void INSLocalizer::optimizeGraph()
{
  switch (param_.method)
  {
    case METHOD::ISAM2:
    {
      isam2_optimizer_->update(*graph_, graph_values_);
      isam2_optimizer_->update();
      current_result_ = isam2_optimizer_->calculateEstimate();

      graph_->resize(0);
      graph_values_.clear();

      prev_state_ = gtsam::NavState(current_result_.at<gtsam::Pose3>(X(key_)), current_result_.at<gtsam::Vector3>(V(key_)));
      break;
    }
    case METHOD::LEVEN:
    {
      gtsam::LevenbergMarquardtOptimizer leven_optimizer(*graph_, graph_values_);
      current_result_ = leven_optimizer.optimize();
      break;
    }
  }
}

void INSLocalizer::getWholeTrajectory(std::vector<StampedPose>& matrix_vector)
{
  matrix_vector.resize(key_ + 1);

  // Optimize graph if offline mode
  if (param_.is_online == false)
  {
    optimizeGraph();
  }

  for (int i = 1; i <= key_; i++)
  {
    StampedPose estimation;
    estimation.stamp = key_stamp_map_[i];
    estimation.pose = current_result_.at<gtsam::Pose3>(X(i)).matrix();
    estimation.vel = current_result_.at<gtsam::Vector3>(V(i)).matrix();

    auto imu_bias = current_result_.at<gtsam::imuBias::ConstantBias>(B(i));
    // Create a matrix or vector as per your requirement
    Eigen::VectorXd bias_vector(6); // Example if you need a 6x1 vector
    bias_vector << imu_bias.accelerometer(), imu_bias.gyroscope(); // Assuming these methods exist
    estimation.imu_bias = bias_vector;
    //std::cout << key_ << " / "<<  std::flush;
    //estimation.imu_bias = current_result_.at<gtsam::imuBias::ConstantBias>(B(i)).matrix();

    matrix_vector.at(i) = estimation;
  }
}
}
