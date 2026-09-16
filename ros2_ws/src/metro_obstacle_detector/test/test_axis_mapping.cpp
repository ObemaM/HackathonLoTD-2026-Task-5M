#include <gtest/gtest.h>

#include <Eigen/Core>

#include "metro_obstacle_detector/axis_mapping.hpp"

using metro_obstacle_detector::AxisMapping;

TEST(AxisMapping, ConvertsDatasetAxesToDetectorFrame)
{
  const AxisMapping mapping("-y", "x", "z", Eigen::Vector3d::Zero());
  const Eigen::Vector3d sensor_point(2.0, -10.0, -1.5);
  const Eigen::Vector3d detector_point = mapping.to_detector(sensor_point);

  EXPECT_NEAR(detector_point.x(), 10.0, 1e-9);
  EXPECT_NEAR(detector_point.y(), 2.0, 1e-9);
  EXPECT_NEAR(detector_point.z(), -1.5, 1e-9);
  EXPECT_TRUE(mapping.to_sensor(detector_point).isApprox(sensor_point));
}

TEST(AxisMapping, AppliesDetectorOffset)
{
  const AxisMapping mapping("-y", "x", "z", Eigen::Vector3d(1.0, 2.0, 3.0));
  const Eigen::Vector3d result = mapping.to_detector(Eigen::Vector3d::Zero());
  EXPECT_TRUE(result.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(AxisMapping, RejectsLeftHandedOrDuplicateAxes)
{
  EXPECT_THROW(
    AxisMapping("x", "y", "-z", Eigen::Vector3d::Zero()),
    std::invalid_argument);
  EXPECT_THROW(
    AxisMapping("x", "x", "z", Eigen::Vector3d::Zero()),
    std::invalid_argument);
}

