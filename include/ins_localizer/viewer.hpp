#ifndef INS_LOCALIZER_VIEWER_HPP
#define INS_LOCALIZER_VIEWER_HPP

#include <thread>

#include <glk/primitives/primitives.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/pointcloud_buffer_pcl.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <implot.h>

namespace ins_localizer
{
class OdometryViewer
{
public:
  OdometryViewer();
  void openViewer();
  void closeViewer();
  void addRedTraj(const Eigen::Matrix4f& transform);
  void addGreenTraj(const Eigen::Matrix4f& transform);
  void addBlueTraj(const Eigen::Matrix4f& transform);
  bool isClosed();

private:
  std::vector<Eigen::Matrix4f> traj_que_;
  std::vector<Eigen::Matrix4f> red_traj_que_;
  std::vector<Eigen::Matrix4f> blue_traj_que_;
  std::vector<Eigen::Matrix4f> green_traj_que_;
  std::mutex mutex_;
  Eigen::Vector3f view_point_;

  bool is_closed_;
  bool track_ = true;

  int traj_cnt_ = 0;
  int div_ = 10;

  std::thread viewer_thread_;

  Eigen::Vector3f red_traj_anchor_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f green_traj_anchor_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f blue_traj_anchor_ = Eigen::Vector3f::Zero();

  std::string high_fps_traj_name_ = "high_fps_traj";

  const double traj_thresh_ = 0.5;

  void viewerLoop();
};
}

#endif
