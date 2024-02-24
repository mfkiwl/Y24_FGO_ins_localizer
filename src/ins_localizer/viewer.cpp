#include "ins_localizer/viewer.hpp"

#include <boost/bind.hpp>

namespace ins_localizer
{
Eigen::Vector3f toVector(const Eigen::Matrix4f& matrix)
{
  return matrix.block(0, 3, 3, 1);
}

OdometryViewer::OdometryViewer()
{
}

void OdometryViewer::openViewer()
{
  viewer_thread_ = std::thread(boost::bind(&OdometryViewer::viewerLoop, this));
  is_closed_ = false;
}

void OdometryViewer::closeViewer()
{
  is_closed_ = true;
  viewer_thread_.join();
}

void OdometryViewer::addRedTraj(const Eigen::Matrix4f& transform)
{
  Eigen::Vector3f diff = red_traj_anchor_ - toVector(transform);
  double distance = diff.norm();

  if (distance > traj_thresh_)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    red_traj_que_.push_back(transform);
    red_traj_anchor_ = toVector(transform);
  }
}

void OdometryViewer::addGreenTraj(const Eigen::Matrix4f& transform)
{
  Eigen::Vector3f diff = green_traj_anchor_ - toVector(transform);
  double distance = diff.norm();

  std::lock_guard<std::mutex> lock(mutex_);
  traj_que_.push_back(transform);
  if (distance > traj_thresh_)
  {
    green_traj_que_.push_back(transform);
    green_traj_anchor_ = toVector(transform);
  }

  view_point_ = toVector(transform);
}
void OdometryViewer::addBlueTraj(const Eigen::Matrix4f& transform)
{
  Eigen::Vector3f diff = blue_traj_anchor_ - toVector(transform);
  double distance = diff.norm();

  if (distance > traj_thresh_)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    blue_traj_que_.push_back(transform);
    blue_traj_anchor_ = toVector(transform);
  }
}

void OdometryViewer::viewerLoop()
{
  auto viewer = guik::LightViewer::instance();

  viewer->register_ui_callback("Console", [&]() {
    if (ImGui::Checkbox("Track", &track_))
    {
      if (track_)
      {
        viewer->reset_center();
        viewer->lookat(view_point_);
      }
    }
  });

  while (viewer->spin_once() && !is_closed_)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto transform : traj_que_)
    {
      viewer->update_drawable(high_fps_traj_name_, glk::Primitives::coordinate_system(), guik::VertexColor(transform));
    }
    traj_que_.clear();
    for (const auto transform : red_traj_que_)
    {
      viewer->update_drawable("traj" + std::to_string(traj_cnt_), glk::Primitives::coordinate_system(), guik::FlatRed(transform));
      traj_cnt_++;
    }
    red_traj_que_.clear();
    for (const auto transform : green_traj_que_)
    {
      viewer->update_drawable("traj" + std::to_string(traj_cnt_), glk::Primitives::coordinate_system(), guik::FlatGreen(transform));
      traj_cnt_++;
    }
    green_traj_que_.clear();
    for (const auto transform : blue_traj_que_)
    {
      viewer->update_drawable("traj" + std::to_string(traj_cnt_), glk::Primitives::coordinate_system(), guik::FlatBlue(transform));
      traj_cnt_++;
    }
    blue_traj_que_.clear();

    if (track_)
      viewer->lookat(view_point_);
  }

  viewer->destroy();
}
}
