#pragma once

#include <Eigen/Core>

#include <array>
#include <string>

namespace metro_obstacle_detector
{

struct SignedAxis
{
  int index;
  double sign;
};

class AxisMapping
{
public:
  AxisMapping(
    const std::string & forward_axis,
    const std::string & lateral_axis,
    const std::string & vertical_axis,
    const Eigen::Vector3d & detector_offset);

  Eigen::Vector3d to_detector(const Eigen::Vector3d & sensor_point) const;
  Eigen::Vector3d to_sensor(const Eigen::Vector3d & detector_point) const;

  const Eigen::Matrix3d & detector_from_sensor_rotation() const;
  Eigen::Matrix3d sensor_from_detector_rotation() const;
  const Eigen::Vector3d & detector_offset() const;

  static SignedAxis parse_axis(const std::string & value);

private:
  Eigen::Matrix3d detector_from_sensor_;
  Eigen::Vector3d detector_offset_;
};

}  // namespace metro_obstacle_detector

