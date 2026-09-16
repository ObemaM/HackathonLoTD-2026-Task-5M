#include "metro_obstacle_detector/axis_mapping.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace metro_obstacle_detector
{

SignedAxis AxisMapping::parse_axis(const std::string & raw_value)
{
  std::string value;
  value.reserve(raw_value.size());
  std::transform(
    raw_value.begin(), raw_value.end(), std::back_inserter(value),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});

  double sign = 1.0;
  if (!value.empty() && value.front() == '-') {
    sign = -1.0;
    value.erase(value.begin());
  } else if (!value.empty() && value.front() == '+') {
    value.erase(value.begin());
  }

  if (value == "x") {
    return {0, sign};
  }
  if (value == "y") {
    return {1, sign};
  }
  if (value == "z") {
    return {2, sign};
  }

  throw std::invalid_argument(
          "Axis must be one of x, y, z, -x, -y, -z; got: " + raw_value);
}

AxisMapping::AxisMapping(
  const std::string & forward_axis,
  const std::string & lateral_axis,
  const std::string & vertical_axis,
  const Eigen::Vector3d & detector_offset)
: detector_from_sensor_(Eigen::Matrix3d::Zero()), detector_offset_(detector_offset)
{
  const std::array<SignedAxis, 3> axes = {
    parse_axis(forward_axis),
    parse_axis(lateral_axis),
    parse_axis(vertical_axis)};

  std::array<bool, 3> used = {false, false, false};
  for (int row = 0; row < 3; ++row) {
    if (used.at(axes.at(row).index)) {
      throw std::invalid_argument("Forward, lateral and vertical axes must be different");
    }
    used.at(axes.at(row).index) = true;
    detector_from_sensor_(row, axes.at(row).index) = axes.at(row).sign;
  }

  const double determinant = detector_from_sensor_.determinant();
  if (std::abs(determinant - 1.0) > 1e-9) {
    throw std::invalid_argument(
            "Axis mapping must form a right-handed coordinate system (determinant +1)");
  }
}

Eigen::Vector3d AxisMapping::to_detector(const Eigen::Vector3d & sensor_point) const
{
  return detector_from_sensor_ * sensor_point + detector_offset_;
}

Eigen::Vector3d AxisMapping::to_sensor(const Eigen::Vector3d & detector_point) const
{
  return detector_from_sensor_.transpose() * (detector_point - detector_offset_);
}

const Eigen::Matrix3d & AxisMapping::detector_from_sensor_rotation() const
{
  return detector_from_sensor_;
}

Eigen::Matrix3d AxisMapping::sensor_from_detector_rotation() const
{
  return detector_from_sensor_.transpose();
}

const Eigen::Vector3d & AxisMapping::detector_offset() const
{
  return detector_offset_;
}

}  // namespace metro_obstacle_detector
