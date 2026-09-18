#include "metro_obstacle_detector/cloud_processing.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <algorithm>
#include <cmath>

namespace metro_obstacle_detector
{
namespace
{
ProcessingCloud::Ptr registration_cloud(const ProcessingCloud::ConstPtr & cloud)
{
  auto cropped = std::make_shared<ProcessingCloud>();
  for (const auto & p : *cloud) {
    // Infrastructure outside the train corridor; avoid fitting to the target.
    if (p.x > 2 && p.x < 45 && std::abs(p.y) > 1.3 && std::abs(p.y) < 8 &&
      p.z > -2.5 && p.z < 3.5) {cropped->push_back(p);}
  }
  auto result = std::make_shared<ProcessingCloud>();
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(0.45F, 0.45F, 0.45F);
  voxel.setInputCloud(cropped);
  voxel.filter(*result);
  return result;
}
}
MotionEstimate estimate_motion(const ProcessingCloud::ConstPtr & previous,
  const ProcessingCloud::ConstPtr & current, double dt)
{
  MotionEstimate result;
  if (!previous || dt <= 0 || dt > 0.5) {return result;}
  auto source = registration_cloud(previous);
  auto target = registration_cloud(current);
  if (source->size() < 100 || target->size() < 100) {return result;}
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaximumIterations(15);
  icp.setMaxCorrespondenceDistance(1.0);
  icp.setTransformationEpsilon(1e-5);
  ProcessingCloud aligned;
  icp.align(aligned);
  if (!icp.hasConverged()) {return result;}
  const auto transform = icp.getFinalTransformation();
  const double fitness = icp.getFitnessScore(1.0);
  const double angle = std::acos(std::clamp(
    (static_cast<double>(transform.block<3, 3>(0, 0).trace()) - 1.0) / 2.0, -1.0, 1.0));
  if (!transform.allFinite() || !std::isfinite(fitness) || fitness > 0.04 ||
    transform.block<3, 1>(0, 3).norm() > std::min(2.0, 25.0 * dt) ||
    std::abs(transform(2, 3)) > 0.15 || angle > 0.06) {return result;}
  // Require coverage as well as low residual; a tiny matched patch is not enough.
  pcl::search::KdTree<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  std::size_t matched = 0;
  for (const auto & p : aligned) {
    if (tree.nearestKSearch(p, 1, indices, distances) && distances[0] < 0.16F) {++matched;}
  }
  if (matched < 0.7 * aligned.size()) {return result;}
  result.valid = true;
  result.rmse_m = std::sqrt(fitness);
  result.previous_to_current = transform;
  return result;
}

std::vector<std::vector<int>> adaptive_clusters(const ProcessingCloud::ConstPtr & cloud,
  double base_radius, double growth, double maximum_radius, int minimum, int maximum)
{
  std::vector<std::vector<int>> result;
  if (cloud->empty()) {return result;}
  pcl::search::KdTree<pcl::PointXYZI> tree;
  tree.setInputCloud(cloud);
  std::vector<bool> visited(cloud->size(), false);
  auto radius = [&](int i) {
    return std::clamp(base_radius + growth * std::max(0.0, double((*cloud)[i].x) - 25.0),
      base_radius, maximum_radius);
  };
  std::vector<int> neighbours;
  std::vector<float> distances;
  for (std::size_t seed = 0; seed < cloud->size(); ++seed) {
    if (visited[seed]) {continue;}
    std::vector<int> queue{static_cast<int>(seed)};
    visited[seed] = true;
    for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
      const int i = queue[cursor];
      const double r = radius(i);
      tree.radiusSearch((*cloud)[i], r, neighbours, distances);
      for (std::size_t j = 0; j < neighbours.size(); ++j) {
        const int next = neighbours[j];
        // Symmetric adjacency avoids range-band seams and ordering dependence.
        const double limit = std::min(r, radius(next));
        if (!visited[next] && distances[j] <= limit * limit) {
          visited[next] = true;
          queue.push_back(next);
        }
      }
    }
    if (queue.size() >= static_cast<std::size_t>(minimum) &&
      queue.size() <= static_cast<std::size_t>(maximum)) {result.push_back(std::move(queue));}
  }
  return result;
}
std::vector<SeparatorBoundary> find_separators(
  const ProcessingCloud::ConstPtr & cloud, double floor_z)
{
  auto elevated = std::make_shared<ProcessingCloud>();
  auto projected = std::make_shared<ProcessingCloud>();
  for (const auto & p : *cloud) {
    if (p.x >= 3 && p.x <= 70 && std::abs(p.y) < 6 &&
      p.z > floor_z + .4 && p.z < floor_z + 2.8)
    {
      elevated->push_back(p);
      auto flat = p; flat.z = 0; projected->push_back(flat);
    }
  }
  std::vector<Eigen::Vector2f> posts;
  for (const auto & indices : adaptive_clusters(projected, .25, 0, .25, 15, 12000)) {
    Eigen::Vector3f lo = Eigen::Vector3f::Constant(1e6F), hi = -lo;
    for (int i : indices) {
      const auto p = (*elevated)[i].getVector3fMap();
      lo = lo.cwiseMin(p); hi = hi.cwiseMax(p);
    }
    const auto size = hi - lo;
    if (size.x() < .8F && size.y() < .8F && size.z() > 1.4F) {
      posts.push_back((.5F * (hi + lo)).head<2>());
    }
  }
  std::vector<SeparatorBoundary> result;
  // A single upright shape could be a person: require a supported chain.
  for (std::size_t i = 0; i < posts.size(); ++i) {
    for (std::size_t j = i + 1; j < posts.size(); ++j) {
      const auto delta = posts[j] - posts[i];
      if (std::abs(delta.x()) < 6) {continue;}
      const double slope = delta.y() / delta.x();
      if (std::abs(slope) > .25) {continue;}
      const double intercept = posts[i].y() - slope * posts[i].x();
      std::vector<float> positions;
      for (const auto & p : posts) {
        if (std::abs(p.y() - intercept - slope * p.x()) < .2) {positions.push_back(p.x());}
      }
      std::sort(positions.begin(), positions.end());
      positions.erase(std::unique(positions.begin(), positions.end(), [](float a, float b) {
        return std::abs(a-b) < 2;
      }), positions.end());
      if (positions.size() < 3) {continue;}
      const bool duplicate = std::any_of(result.begin(), result.end(), [&](const auto & line) {
        return std::abs(line.intercept_m - intercept) < .5 && std::abs(line.slope - slope) < .03;
      });
      if (!duplicate) {result.push_back({positions.front(), positions.back(), intercept, slope});}
      if (result.size() == 4) {return result;}
    }
  }
  return result;
}
}  // namespace metro_obstacle_detector
