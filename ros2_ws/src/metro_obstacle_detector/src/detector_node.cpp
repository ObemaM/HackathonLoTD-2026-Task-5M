#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "metro_obstacle_detector/axis_mapping.hpp"
#include "metro_obstacle_detector/track_path.hpp"
#include "metro_obstacle_detector/object_tracker.hpp"
#include "metro_obstacle_detector/cloud_processing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace metro_obstacle_detector
{

namespace
{

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

std::string join_topic(const std::string & prefix, const std::string & suffix)
{
  if (prefix.empty()) {
    return "/" + suffix;
  }
  if (prefix.back() == '/') {
    return prefix + suffix;
  }
  return prefix + "/" + suffix;
}

struct ClusterInfo
{
  Eigen::Vector3f minimum;
  Eigen::Vector3f maximum;
  float distance_m;
  std::size_t point_count;
  ObjectState state{};
};

}  // namespace

class DetectorNode : public rclcpp::Node
{
public:
  DetectorNode()
  : Node("obstacle_detector")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar_points");
    output_prefix_ = declare_parameter<std::string>("output_prefix", "/obstacle_detector");
    detector_frame_id_ = declare_parameter<std::string>("detector_frame_id", "train_lidar");

    const auto forward_axis = declare_parameter<std::string>("forward_axis", "-y");
    const auto lateral_axis = declare_parameter<std::string>("lateral_axis", "x");
    const auto vertical_axis = declare_parameter<std::string>("vertical_axis", "z");
    const auto offsets = declare_parameter<std::vector<double>>(
      "detector_offset_xyz", {0.0, 0.0, 0.0});
    if (offsets.size() != 3U) {
      throw std::invalid_argument("detector_offset_xyz must contain exactly three values");
    }
    mapping_ = std::make_unique<AxisMapping>(
      forward_axis, lateral_axis, vertical_axis,
      Eigen::Vector3d(offsets[0], offsets[1], offsets[2]));

    min_valid_range_m_ = declare_parameter<double>("min_valid_range_m", 0.3);
    max_valid_range_m_ = declare_parameter<double>("max_valid_range_m", 210.0);
    voxel_leaf_size_m_ = declare_parameter<double>("voxel_leaf_size_m", 0.08);
    far_voxel_leaf_size_m_ = declare_parameter<double>("far_voxel_leaf_size_m", 0.04);

    corridor_min_forward_m_ = declare_parameter<double>("corridor.min_forward_m", 2.0);
    corridor_max_forward_m_ = declare_parameter<double>("corridor.max_forward_m", 200.0);
    corridor_center_lateral_m_ = declare_parameter<double>("corridor.center_lateral_m", 0.0);
    corridor_width_m_ = declare_parameter<double>("corridor.width_m", 2.1);
    corridor_bottom_z_m_ = declare_parameter<double>("corridor.bottom_z_m", -1.65);
    corridor_height_m_ = declare_parameter<double>("corridor.height_m", 3.0);
    floor_clearance_m_ = declare_parameter<double>("corridor.floor_clearance_m", 0.04);

    track_enabled_ = declare_parameter<bool>("track.enabled", true);
    RailEstimatorConfig track_config;
    track_config.min_forward_m = corridor_min_forward_m_;
    track_config.max_forward_m = declare_parameter<double>("track.max_forward_m", 90.0);
    track_config.initial_center_lateral_m = declare_parameter<double>(
      "track.initial_center_lateral_m", corridor_center_lateral_m_);
    track_config.search_half_width_m = declare_parameter<double>("track.search_half_width_m", 4.0);
    track_config.rail_z_min_m = declare_parameter<double>("track.rail_z_min_m", -1.75);
    track_config.rail_z_max_m = declare_parameter<double>("track.rail_z_max_m", -1.15);
    track_config.gauge_m = declare_parameter<double>("track.gauge_m", 1.52);
    track_config.max_gauge_error_m = declare_parameter<double>(
      "track.max_gauge_error_m", 0.18);
    track_config.rail_support_half_width_m = declare_parameter<double>(
      "track.rail_support_half_width_m", 0.14);
    track_config.forward_bin_size_m = declare_parameter<double>("track.forward_bin_size_m", 2.0);
    track_config.lateral_step_m = declare_parameter<double>("track.lateral_step_m", 0.05);
    track_config.min_points_per_rail = declare_parameter<int>("track.min_points_per_rail", 2);
    track_config.min_support_bins = declare_parameter<int>("track.min_support_bins", 6);
    track_config.max_center_step_m = declare_parameter<double>("track.max_center_step_m", 0.55);
    track_config.max_slope_change = declare_parameter<double>("track.max_slope_change", 0.10);
    track_config.seed_penalty = declare_parameter<double>("track.seed_penalty", 20.0);
    track_config.seed_search_bins = declare_parameter<int>("track.seed_search_bins", 5);
    track_config.seed_max_offset_m = declare_parameter<double>("track.seed_max_offset_m", 0.85);
    track_config.continuity_penalty = declare_parameter<double>("track.continuity_penalty", 6.0);
    track_config.prior_penalty = declare_parameter<double>("track.prior_penalty", 1.5);
    track_config.prior_max_deviation_m = declare_parameter<double>(
      "track.prior_max_deviation_m", 0.65);
    track_config.prior_deviation_growth_per_m = declare_parameter<double>(
      "track.prior_deviation_growth_per_m", 0.015);
    track_config.max_gap_bins = declare_parameter<int>("track.max_gap_bins", 1);
    track_config.candidate_min_separation_m = declare_parameter<double>(
      "track.candidate_min_separation_m", 0.25);
    track_config.branch_min_separation_m = declare_parameter<double>(
      "track.branch_min_separation_m", 0.45);
    track_config.branch_max_separation_m = declare_parameter<double>(
      "track.branch_max_separation_m", 1.80);
    track_config.branch_score_ratio = declare_parameter<double>(
      "track.branch_score_ratio", 0.75);
    track_config.branch_min_growth_m_per_bin = declare_parameter<double>(
      "track.branch_min_growth_m_per_bin", 0.05);
    track_config.branch_confirmation_bins = declare_parameter<int>(
      "track.branch_confirmation_bins", 4);
    track_config.max_extrapolation_m = declare_parameter<double>("track.max_extrapolation_m", 2.0);
    track_gauge_m_ = track_config.gauge_m;
    path_smoothing_alpha_ = declare_parameter<double>("track.smoothing_alpha", 0.35);
    track_fallback_frames_ = declare_parameter<int>("track.fallback_frames", 5);
    rail_path_estimator_ = std::make_unique<RailPathEstimator>(track_config);

    rail_mask_enabled_ = declare_parameter<bool>("rail_mask.enabled", true);
    rail_mask_lateral_tolerance_m_ = declare_parameter<double>(
      "rail_mask.lateral_tolerance_m", 0.12);
    rail_mask_min_z_m_ = declare_parameter<double>("rail_mask.min_z_m", -1.62);
    rail_mask_max_z_m_ = declare_parameter<double>("rail_mask.max_z_m", -1.05);
    longitudinal_mask_enabled_ = declare_parameter<bool>("longitudinal_mask.enabled", true);
    longitudinal_mask_config_.minimum_z_m = declare_parameter<double>(
      "longitudinal_mask.min_z_m", corridor_bottom_z_m_);
    longitudinal_mask_config_.maximum_z_m = declare_parameter<double>(
      "longitudinal_mask.max_z_m", corridor_bottom_z_m_ + 0.65);
    longitudinal_mask_config_.corridor_half_width_m = corridor_width_m_ / 2.0;
    longitudinal_mask_config_.forward_bin_size_m = declare_parameter<double>(
      "longitudinal_mask.forward_bin_size_m", 2.0);
    longitudinal_mask_config_.lateral_cell_size_m = declare_parameter<double>(
      "longitudinal_mask.lateral_cell_size_m", 0.05);
    longitudinal_mask_config_.lane_tolerance_m = declare_parameter<double>(
      "longitudinal_mask.lane_tolerance_m", 0.14);
    longitudinal_mask_config_.max_lateral_step_m = declare_parameter<double>(
      "longitudinal_mask.max_lateral_step_m", 0.24);
    longitudinal_mask_config_.min_points_per_bin = declare_parameter<int>(
      "longitudinal_mask.min_points_per_bin", 2);
    longitudinal_mask_config_.min_support_bins = declare_parameter<int>(
      "longitudinal_mask.min_support_bins", 8);
    longitudinal_mask_config_.max_gap_bins = declare_parameter<int>(
      "longitudinal_mask.max_gap_bins", 1);
    longitudinal_mask_config_.max_lanes = declare_parameter<int>(
      "longitudinal_mask.max_lanes", 6);
    fixed_longitudinal_offsets_ = declare_parameter<std::vector<double>>(
      "longitudinal_mask.fixed_offsets_m", {-1.0, 1.0});
    for (const double offset : fixed_longitudinal_offsets_) {
      if (std::abs(offset) > longitudinal_mask_config_.corridor_half_width_m) {
        throw std::invalid_argument("longitudinal_mask.fixed_offsets_m must be inside corridor");
      }
    }
    infrastructure_min_length_m_ = declare_parameter<double>(
      "infrastructure_filter.min_length_m", 3.0);
    infrastructure_max_height_above_floor_m_ = declare_parameter<double>(
      "infrastructure_filter.max_height_above_floor_m", 0.65);
    infrastructure_rail_overlap_margin_m_ = declare_parameter<double>(
      "infrastructure_filter.rail_overlap_margin_m", 0.18);

    cluster_tolerance_m_ = declare_parameter<double>("clustering.tolerance_m", 0.30);
    min_cluster_points_ = declare_parameter<int>("clustering.min_points", 3);
    max_cluster_points_ = declare_parameter<int>("clustering.max_points", 50000);
    min_cluster_height_m_ = declare_parameter<double>("clustering.min_height_m", 0.08);
    max_cluster_length_m_ = declare_parameter<double>("clustering.max_length_m", 8.0);
    max_cluster_width_m_ = declare_parameter<double>("clustering.max_width_m", 2.5);
    max_cluster_height_m_ = declare_parameter<double>("clustering.max_height_m", 3.2);
    max_visualized_clusters_ = declare_parameter<int>("max_visualized_clusters", 30);
    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", true);
    confirmation_required_frames_ = declare_parameter<int>("confirmation.required_frames", 3);
    confirmation_clear_frames_ = declare_parameter<int>("confirmation.clear_frames", 2);
    adaptive_growth_ = declare_parameter<double>("clustering.radius_growth", 0.006);
    adaptive_max_radius_ = declare_parameter<double>("clustering.max_tolerance_m", 0.60);
    motion_enabled_ = declare_parameter<bool>("motion.enabled", true);
    separators_enabled_ = declare_parameter<bool>("separators.enabled", true);
    accumulation_frames_ = declare_parameter<int>("motion.accumulation_frames", 1);
    if (adaptive_growth_ < 0 || adaptive_max_radius_ < cluster_tolerance_m_ ||
      accumulation_frames_ < 1 || accumulation_frames_ > 5)
    {throw std::invalid_argument("Invalid adaptive clustering or accumulation parameters");}

    if (corridor_max_forward_m_ <= corridor_min_forward_m_) {
      throw std::invalid_argument("corridor.max_forward_m must exceed corridor.min_forward_m");
    }
    if (corridor_width_m_ <= 0.0 || corridor_height_m_ <= 0.0) {
      throw std::invalid_argument("Corridor width and height must be positive");
    }
    if (voxel_leaf_size_m_ <= 0.0 || far_voxel_leaf_size_m_ <= 0.0 || cluster_tolerance_m_ <= 0.0) {
      throw std::invalid_argument("Voxel leaf size and cluster tolerance must be positive");
    }
    if (path_smoothing_alpha_ <= 0.0 || path_smoothing_alpha_ > 1.0 ||
      track_fallback_frames_ < 0 || confirmation_required_frames_ < 1 ||
      confirmation_clear_frames_ < 1)
    {
      throw std::invalid_argument("Invalid track smoothing or confirmation parameters");
    }

    detected_publisher_ = create_publisher<std_msgs::msg::Bool>(
      join_topic(output_prefix_, "detected"), 10);
    distance_publisher_ = create_publisher<std_msgs::msg::Float32>(
      join_topic(output_prefix_, "distance_m"), 10);
    processing_time_publisher_ = create_publisher<std_msgs::msg::Float32>(
      join_topic(output_prefix_, "processing_time_ms"), 10);
    track_valid_publisher_ = create_publisher<std_msgs::msg::Bool>(
      join_topic(output_prefix_, "track_valid"), 10);
    route_ambiguous_publisher_ = create_publisher<std_msgs::msg::Bool>(
      join_topic(output_prefix_, "route_ambiguous"), 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      join_topic(output_prefix_, "markers"), 10);

    if (publish_debug_clouds_) {
      filtered_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        join_topic(output_prefix_, "debug/filtered"), rclcpp::SensorDataQoS());
      candidate_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        join_topic(output_prefix_, "debug/candidates"), rclcpp::SensorDataQoS());
    }

    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DetectorNode::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Detector ready: input=%s, output=%s, corridor=[%.1f, %.1f] m x %.2f m x %.2f m",
      input_topic_.c_str(), output_prefix_.c_str(), corridor_min_forward_m_,
      corridor_max_forward_m_, corridor_width_m_, corridor_height_m_);
    RCLCPP_INFO(
      get_logger(), "Rail-relative corridor is %s", track_enabled_ ? "enabled" : "disabled");
  }

private:
  void publish_static_transform(const std::string & sensor_frame)
  {
    if (sensor_frame.empty() || sensor_frame == last_sensor_frame_) {
      return;
    }

    const Eigen::Matrix3d sensor_from_detector = mapping_->sensor_from_detector_rotation();
    const Eigen::Quaterniond quaternion(sensor_from_detector);
    const Eigen::Vector3d translation =
      -sensor_from_detector * mapping_->detector_offset();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = sensor_frame;
    transform.child_frame_id = detector_frame_id_;
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    static_tf_broadcaster_->sendTransform(transform);
    last_sensor_frame_ = sensor_frame;
  }

  void publish_cloud(
    const Cloud & cloud,
    const std_msgs::msg::Header & source_header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher) const
  {
    if (!publisher) {
      return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = source_header;
    message.header.frame_id = detector_frame_id_;
    publisher->publish(message);
  }

  std::vector<ClusterInfo> cluster_candidates(
    const Cloud::Ptr & candidates,
    const TrackPath & path, std::size_t current_points) const
  {
    std::vector<ClusterInfo> result;
    if (candidates->size() < static_cast<std::size_t>(min_cluster_points_)) {
      return result;
    }

    const auto cluster_indices = adaptive_clusters(candidates, cluster_tolerance_m_,
      adaptive_growth_, adaptive_max_radius_, min_cluster_points_, max_cluster_points_);

    for (const auto & indices : cluster_indices) {
      Eigen::Vector3f minimum = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
      Eigen::Vector3f maximum = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

      std::size_t observed = 0;
      for (const int point_index : indices) {
        if (static_cast<std::size_t>(point_index) < current_points) {++observed;}
        const auto & point = candidates->points.at(static_cast<std::size_t>(point_index));
        const Eigen::Vector3f value(point.x, point.y, point.z);
        minimum = minimum.cwiseMin(value);
        maximum = maximum.cwiseMax(value);
      }

      // Historical points must never keep an absent object alive by themselves.
      if (observed < 2U) {continue;}

      const Eigen::Vector3f size = maximum - minimum;
      if (size.z() < min_cluster_height_m_ ||
        size.x() > max_cluster_length_m_ ||
        size.y() > max_cluster_width_m_ ||
        size.z() > max_cluster_height_m_)
      {
        continue;
      }

      if (path.valid && size.x() >= infrastructure_min_length_m_ &&
        maximum.z() <= corridor_bottom_z_m_ + infrastructure_max_height_above_floor_m_)
      {
        const double middle_forward = 0.5 * (minimum.x() + maximum.x());
        const double path_center = path.center_at(middle_forward);
        const double left_rail = path_center + track_gauge_m_ / 2.0;
        const double right_rail = path_center - track_gauge_m_ / 2.0;
        const auto overlaps = [&](double lateral) {
            return lateral >= minimum.y() - infrastructure_rail_overlap_margin_m_ &&
                   lateral <= maximum.y() + infrastructure_rail_overlap_margin_m_;
          };
        if (overlaps(left_rail) || overlaps(right_rail)) {
          continue;
        }
      }

      result.push_back(
        ClusterInfo{minimum, maximum, minimum.x(), observed});
    }

    return result;
  }

  visualization_msgs::msg::Marker make_cube_marker(
    int id,
    const std::string & name_space,
    const std_msgs::msg::Header & header,
    const Eigen::Vector3f & minimum,
    const Eigen::Vector3f & maximum,
    float red,
    float green,
    float blue,
    float alpha) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.header.frame_id = detector_frame_id_;
    marker.ns = name_space;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    const Eigen::Vector3f center = 0.5F * (minimum + maximum);
    const Eigen::Vector3f size = (maximum - minimum).cwiseMax(Eigen::Vector3f::Constant(0.05F));
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = size.x();
    marker.scale.y = size.y();
    marker.scale.z = size.z();
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    return marker;
  }

  visualization_msgs::msg::Marker make_path_marker(
    int id,
    const std::string & name_space,
    const std_msgs::msg::Header & header,
    const TrackPath & path,
    double lateral_offset,
    double height,
    float red,
    float green,
    float blue,
    float alpha,
    float line_width) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.header.frame_id = detector_frame_id_;
    marker.ns = name_space;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = line_width;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    for (double forward = path.min_forward_m; forward <= path.max_forward_m; forward += 1.0) {
      geometry_msgs::msg::Point point;
      point.x = forward;
      point.y = path.center_at(forward) + lateral_offset;
      point.z = height;
      marker.points.push_back(point);
    }
    return marker;
  }

  visualization_msgs::msg::Marker make_longitudinal_lane_marker(
    int id,
    const std_msgs::msg::Header & header,
    const TrackPath & path,
    const LongitudinalLane & lane) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.header.frame_id = detector_frame_id_;
    marker.ns = "masked_longitudinal_structure";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = 0.8F;
    marker.color.g = 0.1F;
    marker.color.b = 1.0F;
    marker.color.a = 0.9F;
    for (double forward = lane.min_forward_m; forward <= lane.max_forward_m; forward += 1.0) {
      geometry_msgs::msg::Point point;
      point.x = forward;
      point.y = path.center_at(forward) + lane.offset_at(forward);
      point.z = corridor_bottom_z_m_ + 0.12;
      marker.points.push_back(point);
    }
    return marker;
  }

  void publish_markers(
    const std_msgs::msg::Header & source_header,
    const std::vector<ClusterInfo> & clusters,
    int nearest_index,
    const TrackPath & path,
    const std::vector<LongitudinalLane> & longitudinal_lanes) const
  {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker clear;
    clear.header = source_header;
    clear.header.frame_id = detector_frame_id_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    int separator_id = 40;
    for (const auto & boundary : separators_) {
      TrackPath line;
      line.valid = true;
      line.min_forward_m = boundary.min_forward_m;
      line.max_forward_m = boundary.max_forward_m;
      line.coefficients = {boundary.intercept_m, boundary.slope, 0};
      array.markers.push_back(make_path_marker(separator_id++, "separator_hint", source_header,
        line, 0, corridor_bottom_z_m_ + 1.0, .8F, .8F, .8F, .8F, .05F));
    }
    visualization_msgs::msg::Marker status;
    status.header = source_header;
    status.header.frame_id = detector_frame_id_;
    status.ns = "detector_status";
    status.id = 60;
    status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    status.action = visualization_msgs::msg::Marker::ADD;
    status.pose.orientation.w = 1;
    status.pose.position.x = 5;
    status.pose.position.z = 1;
    status.scale.z = .3;
    status.color.r = status.color.g = status.color.b = status.color.a = 1;
    status.text = std::string("motion=") + (motion_valid_ ? "valid" : "unavailable") +
      " path=" + (path.valid ? "estimated" : "manual/unverified") +
      " accumulation=" + std::to_string(accumulation_frames_);
    array.markers.push_back(status);

    if (path.valid) {
      const double half_width = corridor_width_m_ / 2.0;
      array.markers.push_back(make_path_marker(
          1, "track_center", source_header, path, 0.0, corridor_bottom_z_m_ + 0.08,
          0.1F, 0.8F, 1.0F, 0.95F, 0.08F));
      array.markers.push_back(make_path_marker(
          2, "estimated_rails", source_header, path, -track_gauge_m_ / 2.0,
          corridor_bottom_z_m_ + 0.10, 1.0F, 0.85F, 0.1F, 0.95F, 0.06F));
      array.markers.push_back(make_path_marker(
          3, "estimated_rails", source_header, path, track_gauge_m_ / 2.0,
          corridor_bottom_z_m_ + 0.10, 1.0F, 0.85F, 0.1F, 0.95F, 0.06F));
      array.markers.push_back(make_path_marker(
          4, "dynamic_corridor", source_header, path, -half_width,
          corridor_bottom_z_m_, 0.1F, 1.0F, 0.2F, 0.95F, 0.08F));
      array.markers.push_back(make_path_marker(
          5, "dynamic_corridor", source_header, path, half_width,
          corridor_bottom_z_m_, 0.1F, 1.0F, 0.2F, 0.95F, 0.08F));
      int lane_id = 20;
      for (const auto & lane : longitudinal_lanes) {
        array.markers.push_back(make_longitudinal_lane_marker(
            lane_id++, source_header, path, lane));
      }
      if (path.branch_ambiguous) {
        visualization_msgs::msg::Marker ambiguity;
        ambiguity.header = source_header;
        ambiguity.header.frame_id = detector_frame_id_;
        ambiguity.ns = "route_ambiguity";
        ambiguity.id = 10;
        ambiguity.type = visualization_msgs::msg::Marker::SPHERE;
        ambiguity.action = visualization_msgs::msg::Marker::ADD;
        ambiguity.pose.position.x = path.max_forward_m;
        ambiguity.pose.position.y = path.center_at(path.max_forward_m);
        ambiguity.pose.position.z = corridor_bottom_z_m_ + 0.25;
        ambiguity.pose.orientation.w = 1.0;
        ambiguity.scale.x = 0.60;
        ambiguity.scale.y = 0.60;
        ambiguity.scale.z = 0.60;
        ambiguity.color.r = 1.0F;
        ambiguity.color.g = 0.45F;
        ambiguity.color.b = 0.05F;
        ambiguity.color.a = 0.95F;
        array.markers.push_back(ambiguity);
      }
    } else {
      const Eigen::Vector3f corridor_min(
        static_cast<float>(corridor_min_forward_m_),
        static_cast<float>(corridor_center_lateral_m_ - corridor_width_m_ / 2.0),
        static_cast<float>(corridor_bottom_z_m_));
      const Eigen::Vector3f corridor_max(
        static_cast<float>(corridor_max_forward_m_),
        static_cast<float>(corridor_center_lateral_m_ + corridor_width_m_ / 2.0),
        static_cast<float>(corridor_bottom_z_m_ + corridor_height_m_));
      array.markers.push_back(
        make_cube_marker(1, "corridor_fallback", source_header, corridor_min, corridor_max,
          0.1F, 0.8F, 0.2F, 0.08F));
    }

    int marker_id = 100;
    int shown = 0;
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      if (shown >= max_visualized_clusters_) {
        break;
      }
      const bool nearest = static_cast<int>(index) == nearest_index;
      const auto & state = clusters[index].state;
      const float green = state.confirmed ? 0.05F : (state.hits > 1 ? 0.45F : 0.85F);
      array.markers.push_back(
        make_cube_marker(
          marker_id++, "candidate_boxes", source_header,
          clusters[index].minimum, clusters[index].maximum,
          1.0F, green, 0.05F, state.confirmed ? 0.65F : 0.35F));

      visualization_msgs::msg::Marker label;
      label.header = source_header;
      label.header.frame_id = detector_frame_id_;
      label.ns = "candidate_labels";
      label.id = marker_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = clusters[index].minimum.x();
      label.pose.position.y = 0.5F * (clusters[index].minimum.y() + clusters[index].maximum.y());
      label.pose.position.z = clusters[index].maximum.z() + 0.35F;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.35;
      label.color.r = 1.0F;
      label.color.g = green;
      label.color.b = 0.1F;
      label.color.a = 1.0F;
      std::ostringstream text;
      text.precision(1);
      text << "#" << state.id << " " << (state.confirmed ? "confirmed" : "candidate")
           << " " << state.hits << "/" << confirmation_required_frames_
           << (nearest ? " nearest " : " ") << std::fixed << clusters[index].distance_m
           << " m / " << clusters[index].point_count << " current pts";
      label.text = text.str();
      array.markers.push_back(label);
      ++shown;
    }

    marker_publisher_->publish(array);
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    const auto started = std::chrono::steady_clock::now();
    publish_static_transform(message->header.frame_id);

    Cloud sensor_cloud;
    try {
      pcl::fromROSMsg(*message, sensor_cloud);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot decode PointCloud2: %s", error.what());
      return;
    }

    auto valid_cloud = std::make_shared<Cloud>();
    valid_cloud->reserve(sensor_cloud.size());
    const double minimum_squared = min_valid_range_m_ * min_valid_range_m_;
    const double maximum_squared = max_valid_range_m_ * max_valid_range_m_;

    for (const auto & point : sensor_cloud.points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const double squared_range =
        static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y +
        static_cast<double>(point.z) * point.z;
      if (squared_range < minimum_squared || squared_range > maximum_squared) {
        continue;
      }

      const Eigen::Vector3d detector = mapping_->to_detector(
        Eigen::Vector3d(point.x, point.y, point.z));
      Point transformed;
      transformed.x = static_cast<float>(detector.x());
      transformed.y = static_cast<float>(detector.y());
      transformed.z = static_cast<float>(detector.z());
      transformed.intensity = point.intensity;
      valid_cloud->push_back(transformed);
    }

    // Keep the original 8 cm cloud for rail estimation and motion. A separate
    // denser far cloud improves small-object detection without changing the path.
    auto filtered_cloud = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setInputCloud(valid_cloud);
    const auto leaf = static_cast<float>(voxel_leaf_size_m_);
    voxel_filter.setLeafSize(leaf, leaf, leaf);
    voxel_filter.filter(*filtered_cloud);

    auto detection_cloud = std::make_shared<Cloud>();
    auto near_cloud = std::make_shared<Cloud>();
    auto far_cloud = std::make_shared<Cloud>();
    for (const auto & p : *valid_cloud) {
      (p.x < 30.0F ? near_cloud : far_cloud)->push_back(p);
    }
    voxel_filter.setInputCloud(near_cloud);
    voxel_filter.setLeafSize(leaf, leaf, leaf);
    voxel_filter.filter(*detection_cloud);
    Cloud filtered_far;
    voxel_filter.setInputCloud(far_cloud);
    const auto far_leaf = static_cast<float>(far_voxel_leaf_size_m_);
    voxel_filter.setLeafSize(far_leaf, far_leaf, far_leaf);
    voxel_filter.filter(filtered_far);
    *detection_cloud += filtered_far;

    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    const double dt = last_cloud_stamp_ < 0 ? 0.0 : stamp - last_cloud_stamp_;
    MotionEstimate motion;
    if (motion_enabled_ && dt > 0 && dt <= 0.5) {
      motion = estimate_motion(previous_cloud_, filtered_cloud, dt);
    }
    if (dt <= 0 || dt > 0.5) {
      tracker_.reset();
      current_track_path_ = TrackPath{};
      history_.clear();
      track_missing_frames_ = 0;
    } else if (motion_enabled_ && !motion.valid) {
      // Never accumulate unregistered points. Keep the prior path and object
      // tracks: the current frame still has conservative association gates.
      history_.clear();
    }
    if (motion.valid) {
      for (auto & sample : current_track_path_.centerline) {
        const Eigen::Vector4f p = motion.previous_to_current * Eigen::Vector4f(
          sample.forward_m, sample.lateral_m, corridor_bottom_z_m_, 1.0F);
        sample.forward_m = p.x();
        sample.lateral_m = p.y();
      }
      if (!current_track_path_.centerline.empty()) {
        current_track_path_.min_forward_m = current_track_path_.centerline.front().forward_m;
        current_track_path_.max_forward_m = current_track_path_.centerline.back().forward_m;
      }
      for (auto & history : history_) {
        pcl::transformPointCloud(*history, *history, motion.previous_to_current);
      }
    }
    previous_cloud_ = filtered_cloud;
    last_cloud_stamp_ = stamp;
    motion_valid_ = motion.valid;
    separators_ = separators_enabled_ ? find_separators(filtered_cloud, corridor_bottom_z_m_) :
      std::vector<SeparatorBoundary>{};

    if (track_enabled_) {
      std::vector<Eigen::Vector3f> track_points;
      track_points.reserve(filtered_cloud->size());
      for (const auto & point : filtered_cloud->points) {
        track_points.emplace_back(point.x, point.y, point.z);
      }
      const TrackPath estimated = rail_path_estimator_->estimate(
        track_points, current_track_path_.valid ? &current_track_path_ : nullptr,
        separators_);
      if (estimated.valid) {
        if (current_track_path_.valid) {
          TrackPath smoothed = estimated;
          for (auto & sample : smoothed.centerline) {
            sample.lateral_m = path_smoothing_alpha_ * sample.lateral_m +
              (1.0 - path_smoothing_alpha_) *
              current_track_path_.center_at(sample.forward_m);
          }
          current_track_path_ = std::move(smoothed);
        } else {
          current_track_path_ = estimated;
        }
        track_missing_frames_ = 0;
      } else if (++track_missing_frames_ > track_fallback_frames_) {
        current_track_path_ = TrackPath{};
      }
    } else {
      current_track_path_ = TrackPath{};
    }

    std_msgs::msg::Bool track_valid_message;
    track_valid_message.data = current_track_path_.valid;
    track_valid_publisher_->publish(track_valid_message);

    std_msgs::msg::Bool route_ambiguous_message;
    route_ambiguous_message.data = current_track_path_.valid &&
      current_track_path_.branch_ambiguous;
    route_ambiguous_publisher_->publish(route_ambiguous_message);

    current_longitudinal_lanes_.clear();
    if (longitudinal_mask_enabled_ && current_track_path_.valid) {
      std::vector<Eigen::Vector3f> low_structure_points;
      low_structure_points.reserve(filtered_cloud->size());
      for (const auto & point : filtered_cloud->points) {
        low_structure_points.emplace_back(point.x, point.y, point.z);
      }
      for (const double offset : fixed_longitudinal_offsets_) {
        current_longitudinal_lanes_.push_back(
          make_fixed_longitudinal_lane(current_track_path_, offset));
      }
      const auto discovered_lanes = find_longitudinal_lanes(
        low_structure_points, current_track_path_, longitudinal_mask_config_);
      for (const auto & lane : discovered_lanes) {
        const bool distinct = std::all_of(
          current_longitudinal_lanes_.begin(), current_longitudinal_lanes_.end(),
          [&](const LongitudinalLane & selected) {
            const double start = std::max(selected.min_forward_m, lane.min_forward_m);
            const double end = std::min(selected.max_forward_m, lane.max_forward_m);
            if (end <= start) {
              return true;
            }
            const double middle = 0.5 * (start + end);
            const bool equivalent =
              std::abs(selected.offset_at(start) - lane.offset_at(start)) <
              longitudinal_mask_config_.lane_tolerance_m &&
              std::abs(selected.offset_at(middle) - lane.offset_at(middle)) <
              longitudinal_mask_config_.lane_tolerance_m &&
              std::abs(selected.offset_at(end) - lane.offset_at(end)) <
              longitudinal_mask_config_.lane_tolerance_m;
            return !equivalent;
          });
        if (distinct) {current_longitudinal_lanes_.push_back(lane);}
      }
    }

    auto candidate_cloud = std::make_shared<Cloud>();
    candidate_cloud->reserve(detection_cloud->size() / 10U);
    const double half_width = corridor_width_m_ / 2.0;
    const double candidate_bottom = corridor_bottom_z_m_ + floor_clearance_m_;
    const double corridor_top = corridor_bottom_z_m_ + corridor_height_m_;

    for (const auto & point : detection_cloud->points) {
      if (point.x < corridor_min_forward_m_ || point.x > corridor_max_forward_m_) {
        continue;
      }
      if (current_track_path_.valid && point.x > current_track_path_.max_forward_m) {
        continue;
      }
      const double corridor_center = current_track_path_.valid ?
        current_track_path_.center_at(point.x) : corridor_center_lateral_m_;
      if (std::abs(point.y - corridor_center) > half_width) {
        continue;
      }
      if (point.z < candidate_bottom || point.z > corridor_top) {
        continue;
      }
      if (rail_mask_enabled_ && is_near_rail(
          Eigen::Vector3f(point.x, point.y, point.z), current_track_path_, track_gauge_m_,
          rail_mask_lateral_tolerance_m_, rail_mask_min_z_m_, rail_mask_max_z_m_))
      {
        continue;
      }
      if (longitudinal_mask_enabled_ && is_near_longitudinal_lane(
          Eigen::Vector3f(point.x, point.y, point.z), current_track_path_,
          current_longitudinal_lanes_, longitudinal_mask_config_.lane_tolerance_m,
          longitudinal_mask_config_.minimum_z_m, longitudinal_mask_config_.maximum_z_m))
      {
        continue;
      }
      candidate_cloud->push_back(point);
    }

    const auto current_points = candidate_cloud->size();
    auto current_far = std::make_shared<Cloud>();
    for (const auto & p : *candidate_cloud) {
      if (p.x >= 30.0F) {current_far->push_back(p);}
    }
    if (motion.valid && accumulation_frames_ > 1) {
      for (const auto & history : history_) {
        for (const auto & p : *history) {
          const double center = current_track_path_.valid ?
            current_track_path_.center_at(p.x) : corridor_center_lateral_m_;
          if (p.x >= 30 && p.x <= corridor_max_forward_m_ &&
            (!current_track_path_.valid || p.x <= current_track_path_.max_forward_m) &&
            std::abs(p.y - center) <= half_width && p.z >= candidate_bottom && p.z <= corridor_top)
          {candidate_cloud->push_back(p);}
        }
      }
    }
    history_.push_back(current_far);
    while (history_.size() >= static_cast<std::size_t>(accumulation_frames_)) {history_.pop_front();}
    auto clusters = cluster_candidates(candidate_cloud, current_track_path_, current_points);
    std::vector<ObjectObservation> observations;
    for (const auto & cluster : clusters) {
      observations.push_back({0.5F * (cluster.minimum + cluster.maximum), cluster.maximum - cluster.minimum});
    }
    const auto states = tracker_.update(observations, confirmation_required_frames_,
      confirmation_clear_frames_, motion.previous_to_current);
    for (std::size_t i = 0; i < clusters.size(); ++i) {clusters[i].state = states[i];}
    std::stable_sort(clusters.begin(), clusters.end(), [](const auto & a, const auto & b) {
      if (a.state.confirmed != b.state.confirmed) {return a.state.confirmed > b.state.confirmed;}
      return a.distance_m < b.distance_m;
    });
    int nearest_index = -1;
    float nearest_distance = std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      if (clusters[index].state.confirmed && clusters[index].distance_m < nearest_distance) {
        nearest_distance = clusters[index].distance_m;
        nearest_index = static_cast<int>(index);
      }
    }

    const bool raw_detected = !clusters.empty();

    std_msgs::msg::Bool detected_message;
    detected_message.data = nearest_index >= 0;
    detected_publisher_->publish(detected_message);

    std_msgs::msg::Float32 distance_message;
    distance_message.data = detected_message.data ? nearest_distance : -1.0F;
    distance_publisher_->publish(distance_message);

    if (publish_debug_clouds_) {
      publish_cloud(*filtered_cloud, message->header, filtered_cloud_publisher_);
      publish_cloud(*candidate_cloud, message->header, candidate_cloud_publisher_);
    }
    publish_markers(
      message->header, clusters, nearest_index, current_track_path_,
      current_longitudinal_lanes_);

    const auto finished = std::chrono::steady_clock::now();
    const float processing_ms = std::chrono::duration<float, std::milli>(finished - started).count();
    std_msgs::msg::Float32 processing_message;
    processing_message.data = processing_ms;
    processing_time_publisher_->publish(processing_message);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "input=%zu valid=%zu voxel=%zu track=%s/%zu route=%s max=%.1f m lanes=%zu candidates=%zu clusters=%zu raw=%s confirmed=%s distance=%.1f m time=%.1f ms",
      sensor_cloud.size(), valid_cloud->size(), filtered_cloud->size(),
      current_track_path_.valid ? "valid" : "fallback", current_track_path_.support_bins,
      current_track_path_.branch_ambiguous ? "ambiguous" : "single",
      current_track_path_.max_forward_m,
      current_longitudinal_lanes_.size(), candidate_cloud->size(), clusters.size(),
      raw_detected ? "true" : "false",
      detected_message.data ? "true" : "false", distance_message.data, processing_ms);
    if (nearest_index >= 0) {
      const auto & nearest = clusters[static_cast<std::size_t>(nearest_index)];
      const Eigen::Vector3f size = nearest.maximum - nearest.minimum;
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "nearest_bbox min=(%.2f, %.2f, %.2f) size=(%.2f, %.2f, %.2f), path_y10=%.2f path_y30=%.2f",
        nearest.minimum.x(), nearest.minimum.y(), nearest.minimum.z(),
        size.x(), size.y(), size.z(),
        current_track_path_.valid ? current_track_path_.center_at(10.0) : corridor_center_lateral_m_,
        current_track_path_.valid ? current_track_path_.center_at(30.0) : corridor_center_lateral_m_);
    }
  }

  std::string input_topic_;
  std::string output_prefix_;
  std::string detector_frame_id_;
  std::string last_sensor_frame_;

  std::unique_ptr<AxisMapping> mapping_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  double min_valid_range_m_;
  double max_valid_range_m_;
  double voxel_leaf_size_m_;
  double far_voxel_leaf_size_m_;
  double corridor_min_forward_m_;
  double corridor_max_forward_m_;
  double corridor_center_lateral_m_;
  double corridor_width_m_;
  double corridor_bottom_z_m_;
  double corridor_height_m_;
  double floor_clearance_m_;
  bool track_enabled_;
  double track_gauge_m_;
  double path_smoothing_alpha_;
  int track_fallback_frames_;
  int track_missing_frames_{0};
  bool rail_mask_enabled_;
  double rail_mask_lateral_tolerance_m_;
  double rail_mask_min_z_m_;
  double rail_mask_max_z_m_;
  bool longitudinal_mask_enabled_;
  LongitudinalMaskConfig longitudinal_mask_config_;
  std::vector<double> fixed_longitudinal_offsets_;
  double infrastructure_min_length_m_;
  double infrastructure_max_height_above_floor_m_;
  double infrastructure_rail_overlap_margin_m_;
  double cluster_tolerance_m_;
  int min_cluster_points_;
  int max_cluster_points_;
  double min_cluster_height_m_;
  double max_cluster_length_m_;
  double max_cluster_width_m_;
  double max_cluster_height_m_;
  int max_visualized_clusters_;
  bool publish_debug_clouds_;
  int confirmation_required_frames_;
  int confirmation_clear_frames_;
  ObjectTracker tracker_;
  double adaptive_growth_;
  double adaptive_max_radius_;
  bool motion_enabled_;
  bool motion_valid_{false};
  bool separators_enabled_;
  std::vector<SeparatorBoundary> separators_;
  int accumulation_frames_;
  double last_cloud_stamp_{-1.0};
  Cloud::ConstPtr previous_cloud_;
  std::deque<Cloud::Ptr> history_;

  std::unique_ptr<RailPathEstimator> rail_path_estimator_;
  TrackPath current_track_path_;
  std::vector<LongitudinalLane> current_longitudinal_lanes_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processing_time_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr track_valid_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr route_ambiguous_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr candidate_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

}  // namespace metro_obstacle_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<metro_obstacle_detector::DetectorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_detector"), "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
