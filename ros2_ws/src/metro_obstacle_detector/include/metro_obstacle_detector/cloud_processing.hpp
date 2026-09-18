#pragma once
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "metro_obstacle_detector/track_path.hpp"

namespace metro_obstacle_detector
{
using ProcessingCloud = pcl::PointCloud<pcl::PointXYZI>;
struct MotionEstimate
{
  bool valid{false};
  float rmse_m{0.0F};
  Eigen::Matrix4f previous_to_current{Eigen::Matrix4f::Identity()};
};
MotionEstimate estimate_motion(const ProcessingCloud::ConstPtr & previous,
  const ProcessingCloud::ConstPtr & current, double dt);
std::vector<std::vector<int>> adaptive_clusters(const ProcessingCloud::ConstPtr & cloud,
  double base_radius, double growth, double maximum_radius, int minimum, int maximum);
std::vector<SeparatorBoundary> find_separators(
  const ProcessingCloud::ConstPtr & cloud, double floor_z);
}  // namespace metro_obstacle_detector
