#include <pcl/common/point_tests.h>
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
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "metro_obstacle_detector/axis_mapping.hpp"
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

    corridor_min_forward_m_ = declare_parameter<double>("corridor.min_forward_m", 2.0);
    corridor_max_forward_m_ = declare_parameter<double>("corridor.max_forward_m", 200.0);
    corridor_center_lateral_m_ = declare_parameter<double>("corridor.center_lateral_m", 0.0);
    corridor_width_m_ = declare_parameter<double>("corridor.width_m", 2.1);
    corridor_bottom_z_m_ = declare_parameter<double>("corridor.bottom_z_m", -1.65);
    corridor_height_m_ = declare_parameter<double>("corridor.height_m", 3.0);
    floor_clearance_m_ = declare_parameter<double>("corridor.floor_clearance_m", 0.04);

    cluster_tolerance_m_ = declare_parameter<double>("clustering.tolerance_m", 0.30);
    min_cluster_points_ = declare_parameter<int>("clustering.min_points", 3);
    max_cluster_points_ = declare_parameter<int>("clustering.max_points", 50000);
    min_cluster_height_m_ = declare_parameter<double>("clustering.min_height_m", 0.08);
    max_cluster_length_m_ = declare_parameter<double>("clustering.max_length_m", 8.0);
    max_cluster_width_m_ = declare_parameter<double>("clustering.max_width_m", 2.5);
    max_cluster_height_m_ = declare_parameter<double>("clustering.max_height_m", 3.2);
    max_visualized_clusters_ = declare_parameter<int>("max_visualized_clusters", 30);
    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", true);

    if (corridor_max_forward_m_ <= corridor_min_forward_m_) {
      throw std::invalid_argument("corridor.max_forward_m must exceed corridor.min_forward_m");
    }
    if (corridor_width_m_ <= 0.0 || corridor_height_m_ <= 0.0) {
      throw std::invalid_argument("Corridor width and height must be positive");
    }
    if (voxel_leaf_size_m_ <= 0.0 || cluster_tolerance_m_ <= 0.0) {
      throw std::invalid_argument("Voxel leaf size and cluster tolerance must be positive");
    }

    detected_publisher_ = create_publisher<std_msgs::msg::Bool>(
      join_topic(output_prefix_, "detected"), 10);
    distance_publisher_ = create_publisher<std_msgs::msg::Float32>(
      join_topic(output_prefix_, "distance_m"), 10);
    processing_time_publisher_ = create_publisher<std_msgs::msg::Float32>(
      join_topic(output_prefix_, "processing_time_ms"), 10);
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
    RCLCPP_WARN(
      get_logger(),
      "This is a straight, parameterized baseline. Automatic rail/curve estimation is not implemented yet.");
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

  std::vector<ClusterInfo> cluster_candidates(const Cloud::Ptr & candidates) const
  {
    std::vector<ClusterInfo> result;
    if (candidates->size() < static_cast<std::size_t>(min_cluster_points_)) {
      return result;
    }

    auto tree = std::make_shared<pcl::search::KdTree<Point>>();
    tree->setInputCloud(candidates);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> extractor;
    extractor.setClusterTolerance(cluster_tolerance_m_);
    extractor.setMinClusterSize(min_cluster_points_);
    extractor.setMaxClusterSize(max_cluster_points_);
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(candidates);
    extractor.extract(cluster_indices);

    for (const auto & indices : cluster_indices) {
      Eigen::Vector3f minimum = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
      Eigen::Vector3f maximum = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

      for (const int point_index : indices.indices) {
        const auto & point = candidates->points.at(static_cast<std::size_t>(point_index));
        const Eigen::Vector3f value(point.x, point.y, point.z);
        minimum = minimum.cwiseMin(value);
        maximum = maximum.cwiseMax(value);
      }

      const Eigen::Vector3f size = maximum - minimum;
      if (size.z() < min_cluster_height_m_ ||
        size.x() > max_cluster_length_m_ ||
        size.y() > max_cluster_width_m_ ||
        size.z() > max_cluster_height_m_)
      {
        continue;
      }

      result.push_back(
        ClusterInfo{minimum, maximum, minimum.x(), indices.indices.size()});
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

  void publish_markers(
    const std_msgs::msg::Header & source_header,
    const std::vector<ClusterInfo> & clusters,
    int nearest_index) const
  {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker clear;
    clear.header = source_header;
    clear.header.frame_id = detector_frame_id_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    const Eigen::Vector3f corridor_min(
      static_cast<float>(corridor_min_forward_m_),
      static_cast<float>(corridor_center_lateral_m_ - corridor_width_m_ / 2.0),
      static_cast<float>(corridor_bottom_z_m_));
    const Eigen::Vector3f corridor_max(
      static_cast<float>(corridor_max_forward_m_),
      static_cast<float>(corridor_center_lateral_m_ + corridor_width_m_ / 2.0),
      static_cast<float>(corridor_bottom_z_m_ + corridor_height_m_));
    array.markers.push_back(
      make_cube_marker(1, "corridor", source_header, corridor_min, corridor_max, 0.1F, 0.8F, 0.2F, 0.08F));

    int marker_id = 100;
    int shown = 0;
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      if (shown >= max_visualized_clusters_) {
        break;
      }
      const bool nearest = static_cast<int>(index) == nearest_index;
      array.markers.push_back(
        make_cube_marker(
          marker_id++, "candidate_boxes", source_header,
          clusters[index].minimum, clusters[index].maximum,
          nearest ? 1.0F : 1.0F, nearest ? 0.05F : 0.55F, 0.05F, nearest ? 0.75F : 0.45F));

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
      label.color.g = nearest ? 0.1F : 0.7F;
      label.color.b = 0.1F;
      label.color.a = 1.0F;
      std::ostringstream text;
      text.precision(1);
      text << std::fixed << clusters[index].distance_m << " m / " << clusters[index].point_count << " pts";
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

    auto filtered_cloud = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setInputCloud(valid_cloud);
    const auto leaf = static_cast<float>(voxel_leaf_size_m_);
    voxel_filter.setLeafSize(leaf, leaf, leaf);
    voxel_filter.filter(*filtered_cloud);

    auto candidate_cloud = std::make_shared<Cloud>();
    candidate_cloud->reserve(filtered_cloud->size() / 10U);
    const double half_width = corridor_width_m_ / 2.0;
    const double candidate_bottom = corridor_bottom_z_m_ + floor_clearance_m_;
    const double corridor_top = corridor_bottom_z_m_ + corridor_height_m_;

    for (const auto & point : filtered_cloud->points) {
      if (point.x < corridor_min_forward_m_ || point.x > corridor_max_forward_m_) {
        continue;
      }
      if (std::abs(point.y - corridor_center_lateral_m_) > half_width) {
        continue;
      }
      if (point.z < candidate_bottom || point.z > corridor_top) {
        continue;
      }
      candidate_cloud->push_back(point);
    }

    const auto clusters = cluster_candidates(candidate_cloud);
    int nearest_index = -1;
    float nearest_distance = std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      if (clusters[index].distance_m < nearest_distance) {
        nearest_distance = clusters[index].distance_m;
        nearest_index = static_cast<int>(index);
      }
    }

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
    publish_markers(message->header, clusters, nearest_index);

    const auto finished = std::chrono::steady_clock::now();
    const float processing_ms = std::chrono::duration<float, std::milli>(finished - started).count();
    std_msgs::msg::Float32 processing_message;
    processing_message.data = processing_ms;
    processing_time_publisher_->publish(processing_message);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "input=%zu valid=%zu voxel=%zu candidates=%zu clusters=%zu detected=%s distance=%.1f m time=%.1f ms",
      sensor_cloud.size(), valid_cloud->size(), filtered_cloud->size(), candidate_cloud->size(),
      clusters.size(), detected_message.data ? "true" : "false", distance_message.data, processing_ms);
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
  double corridor_min_forward_m_;
  double corridor_max_forward_m_;
  double corridor_center_lateral_m_;
  double corridor_width_m_;
  double corridor_bottom_z_m_;
  double corridor_height_m_;
  double floor_clearance_m_;
  double cluster_tolerance_m_;
  int min_cluster_points_;
  int max_cluster_points_;
  double min_cluster_height_m_;
  double max_cluster_length_m_;
  double max_cluster_width_m_;
  double max_cluster_height_m_;
  int max_visualized_clusters_;
  bool publish_debug_clouds_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processing_time_publisher_;
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

