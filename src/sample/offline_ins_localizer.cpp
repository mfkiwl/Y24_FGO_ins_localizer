#include "ins_localizer/ins_localizer_core.hpp"
#include "math.h"

#include <iostream>
#include <fstream>
#include <sstream>

// Input
// argv[1]: GNSS csv
// argv[2]: IMU CSV
// argv[3]: YAML
// argv[4]: OUTPUT CSV

struct GNSSIndex
{
  bool has_header;
  int stamp;
  int x;
  int y;
  int z;
  int status;
};

struct IMUIndex
{
  bool has_header;
  int stamp;
  int acc_x;
  int acc_y;
  int acc_z;
  int gyro_x;
  int gyro_y;
  int gyro_z;
};

struct VelocityIndex
{
  bool has_header;
  int stamp;
  int velocity;
};

struct GNSSdopplerIndex
{
  bool has_header;
  int stamp;
  int velognss_vel_x;
  int velognss_vel_y;
  int velognss_vel_z;
  int est_status;
  int status;
  int velognss_quaternion_w;
  int velognss_quaternion_x;
  int velognss_quaternion_y;
  int velognss_quaternion_z;
};

struct Index
{
  GNSSIndex gnss;
  IMUIndex imu;
  VelocityIndex velocity;
  GNSSdopplerIndex gnss_doppler;

  void loadYAML(const std::string& yaml)
  {
    try
    {
      YAML::Node conf = YAML::LoadFile(yaml)["csv_format"];
      this->gnss.has_header = conf["gnss"]["has_header"].as<bool>();
      this->gnss.stamp = conf["gnss"]["stamp_index"].as<int>();
      this->gnss.x = conf["gnss"]["x_index"].as<int>();
      this->gnss.y = conf["gnss"]["y_index"].as<int>();
      this->gnss.z = conf["gnss"]["z_index"].as<int>();
      this->gnss.status = conf["gnss"]["status_index"].as<int>();

      this->imu.has_header = conf["imu"]["has_header"].as<bool>();
      this->imu.stamp = conf["imu"]["stamp_index"].as<int>();
      this->imu.acc_x = conf["imu"]["acc_x_index"].as<int>();
      this->imu.acc_y = conf["imu"]["acc_y_index"].as<int>();
      this->imu.acc_z = conf["imu"]["acc_z_index"].as<int>();
      this->imu.gyro_x = conf["imu"]["gyro_x_index"].as<int>();
      this->imu.gyro_y = conf["imu"]["gyro_y_index"].as<int>();
      this->imu.gyro_z = conf["imu"]["gyro_z_index"].as<int>();

      this->velocity.has_header = conf["velocity"]["has_header"].as<bool>();
      this->velocity.stamp = conf["velocity"]["stamp_index"].as<int>();
      this->velocity.velocity = conf["velocity"]["velocity_index"].as<int>();

      this->gnss_doppler.has_header = conf["gnss_doppler"]["has_header"].as<bool>();
      this->gnss_doppler.stamp = conf["gnss_doppler"]["stamp_index"].as<int>();
      this->gnss_doppler.velognss_vel_x = conf["gnss_doppler"]["gnss_vel_x_index"].as<int>();
      this->gnss_doppler.velognss_vel_y = conf["gnss_doppler"]["gnss_vel_y_index"].as<int>();
      this->gnss_doppler.velognss_vel_z = conf["gnss_doppler"]["gnss_vel_z_index"].as<int>();
      this->gnss_doppler.est_status = conf["gnss_doppler"]["status_est_index"].as<int>();
      this->gnss_doppler.status = conf["gnss_doppler"]["status_index"].as<int>();
      // this->gnss_doppler.velognss_quaternion_w = conf["gnss_doppler"]["quaternion_w_index"].as<int>();
      // this->gnss_doppler.velognss_quaternion_x = conf["gnss_doppler"]["quaternion_x_index"].as<int>();
      // this->gnss_doppler.velognss_quaternion_y = conf["gnss_doppler"]["quaternion_y_index"].as<int>();
      // this->gnss_doppler.velognss_quaternion_z = conf["gnss_doppler"]["quaternion_z_index"].as<int>();
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "Error: Cannot load index parameters from YAML" << std::endl;
      std::cerr << "File: " << yaml << std::endl;
      std::cerr << "Message: " << e.what() << std::endl;
      exit(1);
    }
  }
};

std::vector<std::string> split(const std::string input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter))
    result.push_back(field);

  return result;
}

int main(int argc, char** argv)
{
  std::string gnss_csv(argv[1]);
  std::string imu_csv(argv[2]);
  std::string velocity_csv(argv[3]);
  std::string gnss_doppler_csv(argv[4]);
  std::string yaml(argv[5]);
  std::string output_csv(argv[6]);

  Index index;
  index.loadYAML(yaml);

  // Prepare sensor data vec
  std::vector<ins_localizer::GNSSData> gnss_data_vec;
  std::ifstream gnss_ifs(gnss_csv);
  std::string line;
  if (index.gnss.has_header)
    std::getline(gnss_ifs, line);
  
  while (std::getline(gnss_ifs, line))
  {
    std::vector<std::string> str_vec = split(line, ',');
    ins_localizer::GNSSData gnss_data;
    gnss_data.stamp = std::stod(str_vec[index.gnss.stamp]);
    gnss_data.position(0) = std::stod(str_vec[index.gnss.x]);
    gnss_data.position(1) = std::stod(str_vec[index.gnss.y]);
    gnss_data.position(2) = std::stod(str_vec[index.gnss.z]);
    gnss_data.status = std::stod(str_vec[index.gnss.status]);
    int status = std::stod(str_vec[index.gnss.status]);
    switch (status)
    {
      case 4:
        gnss_data.gps_bias = 0.01;
        break;
      case 2:
      case 5:
        gnss_data.gps_bias = 10.0;
        break;
      default:
        gnss_data.gps_bias = 100.0;
    }

    gnss_data_vec.push_back(gnss_data);
  }
  gnss_ifs.close();

  std::vector<ins_localizer::IMUData> imu_data_vec;
  std::ifstream imu_ifs(imu_csv);
  if (index.imu.has_header)
    std::getline(imu_ifs, line);
  
  while (std::getline(imu_ifs, line))
  {
    std::vector<std::string> str_vec = split(line, ',');
    ins_localizer::IMUData imu_data;
    imu_data.stamp = std::stod(str_vec[index.imu.stamp]);
    imu_data.acc(0) = std::stof(str_vec[index.imu.acc_x]);
    imu_data.acc(1) = -1*std::stof(str_vec[index.imu.acc_y]);
    imu_data.acc(2) = std::stof(str_vec[index.imu.acc_z]);
    imu_data.gyro(0) = std::stof(str_vec[index.imu.gyro_x]);
    imu_data.gyro(1) = -1*std::stof(str_vec[index.imu.gyro_y]);
    imu_data.gyro(2) = std::stof(str_vec[index.imu.gyro_z]);

    imu_data_vec.push_back(imu_data);
  }
  imu_ifs.close();

  std::vector<ins_localizer::VelocityData> velocity_data_vec;
  std::ifstream velocity_ifs(velocity_csv);
  if (index.velocity.has_header)
    std::getline(velocity_ifs, line);
  
  while (std::getline(velocity_ifs, line))
  {
    std::vector<std::string> str_vec = split(line, ',');
    ins_localizer::VelocityData velo_data;
    velo_data.stamp = std::stod(str_vec[index.velocity.stamp]);
    velo_data.velocity = std::stof(str_vec[index.velocity.velocity]);

    velocity_data_vec.push_back(velo_data);
  }
  velocity_ifs.close();

  std::vector<ins_localizer::GNSSDopplerData> gnss_doppler_data_vec;
  std::ifstream gnss_doppler_ifs(gnss_doppler_csv);
  if (index.gnss_doppler.has_header)
    std::getline(gnss_doppler_ifs, line);

  while (std::getline(gnss_doppler_ifs, line))
  {
    std::vector<std::string> str_vec = split(line, ',');
    ins_localizer::GNSSDopplerData gnss_doppler_data;
    gnss_doppler_data.stamp = std::stod(str_vec[index.gnss_doppler.stamp]);
    gnss_doppler_data.gnss_vel(0) = std::stof(str_vec[index.gnss_doppler.velognss_vel_x]);
    gnss_doppler_data.gnss_vel(1) = std::stof(str_vec[index.gnss_doppler.velognss_vel_y]);
    gnss_doppler_data.gnss_vel(2) = std::stof(str_vec[index.gnss_doppler.velognss_vel_z]);
    gnss_doppler_data.est_status = std::stof(str_vec[index.gnss_doppler.est_status]);
    gnss_doppler_data.status = std::stof(str_vec[index.gnss_doppler.status]);
    // gnss_doppler_data.quaternion(0) = std::stof(str_vec[index.gnss_doppler.velognss_quaternion_w]);
    // gnss_doppler_data.quaternion(1) = std::stof(str_vec[index.gnss_doppler.velognss_quaternion_x]);
    // gnss_doppler_data.quaternion(2) = std::stof(str_vec[index.gnss_doppler.velognss_quaternion_y]);
    // gnss_doppler_data.quaternion(3) = std::stof(str_vec[index.gnss_doppler.velognss_quaternion_z]);
    //gnss_doppler_data.gnss_doppler_bias = 10.0*M_PI/180;
    // gnss_doppler_data.gnss_doppler_bias = 1.0*M_PI/180;
    gnss_doppler_data.gnss_doppler_bias = 0.1;
    // gnss_doppler_data.gnss_doppler_bias = 1.0;

    gnss_doppler_data_vec.push_back(gnss_doppler_data);
  }
  gnss_doppler_ifs.close();


  // Prepare optimization param
  ins_localizer::INSParam ins_param;
  ins_param.loadYAML(yaml);
  
  ins_localizer::INSLocalizer localizer(ins_param);

  // Estimate
  int imu_index = 0;
  int velocity_index = 0;
  int gnss_doppler_index = 0;
  int cnt = 0;
  for (const auto& current_gnss_data : gnss_data_vec)
  {
    while (imu_index < imu_data_vec.size() && imu_data_vec[imu_index].stamp < current_gnss_data.stamp)
    {
      localizer.imuCallback(imu_data_vec[imu_index]);
      imu_index++;
    }

    while (velocity_index < velocity_data_vec.size() && velocity_data_vec[velocity_index].stamp < current_gnss_data.stamp)
    {
      localizer.velocityCallback(velocity_data_vec[velocity_index]);
      velocity_index++;
    }

    while (gnss_doppler_index < gnss_doppler_data_vec.size() && gnss_doppler_data_vec[gnss_doppler_index].stamp < current_gnss_data.stamp)
    {
      localizer.gnss_dopplerCallback(gnss_doppler_data_vec[gnss_doppler_index]);
      gnss_doppler_index++;
    }

    bool estimate_ready = localizer.gnssPositionCallback(current_gnss_data);
    cnt++;
    std::cout << cnt << " / " << gnss_data_vec.size() << "\r" << std::flush;
  }

  // Get result
  std::vector<ins_localizer::StampedPose> estimated_result;
  localizer.getWholeTrajectory(estimated_result);

  std::ofstream ofs(output_csv);
  ofs << "stamp,x,y,z,qw,qx,qy,qz,vel_e,vel_n,vel_u" << "\n";
  for (const auto& stamped_pose : estimated_result)
  {
    Eigen::Matrix3d rotation_matrix = stamped_pose.pose.topLeftCorner(3, 3);
    Eigen::Quaterniond quaternion(rotation_matrix);

    ofs << std::setprecision(16) << stamped_pose.stamp << ","
        << std::fixed << std::setprecision(5)
        << stamped_pose.pose(0, 3) << ","
        << stamped_pose.pose(1, 3) << ","
        << stamped_pose.pose(2, 3) << ","
        << quaternion.w() << ","
        << quaternion.x() << ","
        << quaternion.y() << ","
        << quaternion.z() << ","
        << stamped_pose.vel(0) << ","
        << stamped_pose.vel(1) << ","
        << stamped_pose.vel(2) << "\n";
        // << stamped_pose.imu_bias(0,0) << ","
        // << stamped_pose.imu_bias(1,0) << ","
        // << stamped_pose.imu_bias(2,0) << ","
        // << stamped_pose.imu_bias(3,0) << ","
        // << stamped_pose.imu_bias(4,0) << ","
        // << stamped_pose.imu_bias(5,0) << "\n";
        
  }
  ofs.close();

  return 0;
}
