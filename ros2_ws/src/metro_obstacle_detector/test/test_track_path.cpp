#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <functional>
#include <vector>

#include "metro_obstacle_detector/track_path.hpp"

using metro_obstacle_detector::RailEstimatorConfig;
using metro_obstacle_detector::RailPathEstimator;
using metro_obstacle_detector::TrackPath;
using metro_obstacle_detector::LongitudinalMaskConfig;
using metro_obstacle_detector::find_longitudinal_lanes;
using metro_obstacle_detector::is_near_longitudinal_lane;
using metro_obstacle_detector::is_near_rail;

namespace
{

void add_rails_range(
  std::vector<Eigen::Vector3f> & points,
  const RailEstimatorConfig & config,
  const std::function<double(double)> & center,
  double minimum_forward,
  double maximum_forward,
  int repetitions = 4)
{
  for (double x = minimum_forward; x < maximum_forward; x += 0.35) {
    for (int repetition = 0; repetition < repetitions; ++repetition) {
      const double jitter = (repetition - repetitions / 2) * 0.008;
      points.emplace_back(
        static_cast<float>(x),
        static_cast<float>(center(x) - config.gauge_m / 2.0 + jitter),
        -1.30F);
      points.emplace_back(
        static_cast<float>(x),
        static_cast<float>(center(x) + config.gauge_m / 2.0 + jitter),
        -1.30F);
    }
  }
}

void add_rails(
  std::vector<Eigen::Vector3f> & points,
  const RailEstimatorConfig & config,
  const std::function<double(double)> & center,
  int repetitions = 4)
{
  add_rails_range(
    points, config, center, config.min_forward_m + 0.1,
    config.max_forward_m, repetitions);
}

RailEstimatorConfig test_config()
{
  RailEstimatorConfig config;
  config.max_forward_m = 50.0;
  config.search_half_width_m = 4.0;
  config.min_points_per_rail = 3;
  config.min_support_bins = 6;
  config.seed_penalty = 20.0;
  return config;
}

}  // namespace

TEST(TrackPath, SelectsOwnStraightTrackInsteadOfNeighbour)
{
  const auto config = test_config();
  std::vector<Eigen::Vector3f> points;
  add_rails(points, config, [](double) {return 0.0;}, 4);
  add_rails(points, config, [](double) {return -2.8;}, 7);

  const TrackPath path = RailPathEstimator(config).estimate(points);

  ASSERT_TRUE(path.valid);
  EXPECT_GE(path.support_bins, 20U);
  EXPECT_NEAR(path.center_at(5.0), 0.0, 0.11);
  EXPECT_NEAR(path.center_at(40.0), 0.0, 0.15);
}

TEST(TrackPath, FitsACurvedTrack)
{
  const auto config = test_config();
  std::vector<Eigen::Vector3f> points;
  const auto curve = [](double x) {return -0.0008 * x * x + 0.01 * x;};
  add_rails(points, config, curve, 5);

  const TrackPath path = RailPathEstimator(config).estimate(points);

  ASSERT_TRUE(path.valid);
  EXPECT_NEAR(path.center_at(10.0), curve(10.0), 0.12);
  EXPECT_NEAR(path.center_at(30.0), curve(30.0), 0.15);
  EXPECT_NEAR(path.center_at(45.0), curve(45.0), 0.20);
}

TEST(TrackPath, StopsInsteadOfJumpingToDenserNeighbour)
{
  auto config = test_config();
  config.max_extrapolation_m = 2.0;
  std::vector<Eigen::Vector3f> points;
  add_rails_range(
    points, config, [](double) {return 0.0;},
    config.min_forward_m + 0.1, 24.0, 4);
  add_rails(points, config, [](double) {return -2.8;}, 8);

  const TrackPath path = RailPathEstimator(config).estimate(points);

  ASSERT_TRUE(path.valid);
  EXPECT_FALSE(path.branch_ambiguous);
  EXPECT_LT(path.max_forward_m, 30.0);
  EXPECT_NEAR(path.center_at(path.max_forward_m), 0.0, 0.15);
}

TEST(TrackPath, TruncatesAtAmbiguousTurnout)
{
  auto config = test_config();
  config.branch_score_ratio = 0.70;
  std::vector<Eigen::Vector3f> points;
  add_rails_range(
    points, config, [](double) {return 0.0;},
    config.min_forward_m + 0.1, 22.0, 5);
  add_rails_range(
    points, config, [](double x) {return 0.08 * (x - 20.0);},
    20.0, config.max_forward_m, 5);
  add_rails_range(
    points, config, [](double x) {return -0.08 * (x - 20.0);},
    20.0, config.max_forward_m, 5);

  const TrackPath path = RailPathEstimator(config).estimate(points);

  ASSERT_TRUE(path.valid);
  EXPECT_TRUE(path.branch_ambiguous);
  EXPECT_LT(path.max_forward_m, 32.0);
  EXPECT_NEAR(path.center_at(15.0), 0.0, 0.15);
}

TEST(TrackPath, RejectsInsufficientRailSupport)
{
  const auto config = test_config();
  const std::vector<Eigen::Vector3f> sparse = {
    {3.0F, -0.76F, -1.3F}, {3.0F, 0.76F, -1.3F}};

  EXPECT_FALSE(RailPathEstimator(config).estimate(sparse).valid);
}

TEST(TrackPath, CurrentCurveCanDepartFromPriorInFarField)
{
  const auto config = test_config();
  std::vector<Eigen::Vector3f> straight, curved;
  add_rails(straight, config, [](double) {return 0.0;}, 5);
  const auto curve = [](double x) {return x > 15 ? -.0015 * (x-15) * (x-15) : 0.0;};
  add_rails(curved, config, curve, 5);
  const auto prior = RailPathEstimator(config).estimate(straight);
  const auto path = RailPathEstimator(config).estimate(curved, &prior);
  ASSERT_TRUE(path.valid);
  EXPECT_GT(path.max_forward_m, 40);
  EXPECT_NEAR(path.center_at(40), curve(40), .25);
}

TEST(TrackPath, RailMaskOnlyRemovesNarrowExpectedRailBand)
{
  TrackPath path;
  path.valid = true;
  path.min_forward_m = 2.0;
  path.max_forward_m = 50.0;

  EXPECT_TRUE(is_near_rail(
      Eigen::Vector3f(10.0F, 0.76F, -1.30F), path, 1.52, 0.12, -1.45, -1.18));
  EXPECT_FALSE(is_near_rail(
      Eigen::Vector3f(10.0F, 0.76F, -0.80F), path, 1.52, 0.12, -1.45, -1.18));
  EXPECT_FALSE(is_near_rail(
      Eigen::Vector3f(10.0F, 0.30F, -1.30F), path, 1.52, 0.12, -1.45, -1.18));
}

TEST(TrackPath, FindsPersistentLowLanesButKeepsCompactObject)
{
  TrackPath path;
  path.valid = true;
  path.min_forward_m = 2.0;
  path.max_forward_m = 50.0;
  std::vector<Eigen::Vector3f> points;
  for (double x = 2.1; x < 48.0; x += 0.25) {
    points.emplace_back(static_cast<float>(x), -0.82F, -1.30F);
    points.emplace_back(static_cast<float>(x), -0.79F, -1.31F);
    points.emplace_back(static_cast<float>(x), 0.79F, -1.29F);
    points.emplace_back(static_cast<float>(x), 0.82F, -1.30F);
  }
  for (double x = 10.0; x < 11.0; x += 0.1) {
    points.emplace_back(static_cast<float>(x), 0.0F, -1.20F);
  }

  LongitudinalMaskConfig config;
  config.minimum_z_m = -1.65;
  config.maximum_z_m = -1.0;
  config.min_points_per_bin = 2;
  config.min_support_bins = 8;
  const auto lanes = find_longitudinal_lanes(points, path, config);

  ASSERT_GE(lanes.size(), 2U);
  EXPECT_TRUE(is_near_longitudinal_lane(
      Eigen::Vector3f(20.0F, 0.80F, -1.30F), path, lanes,
      config.lane_tolerance_m, config.minimum_z_m, config.maximum_z_m));
  EXPECT_FALSE(is_near_longitudinal_lane(
      Eigen::Vector3f(10.5F, 0.0F, -1.20F), path, lanes,
      config.lane_tolerance_m, config.minimum_z_m, config.maximum_z_m));
}

TEST(TrackPath, FollowsCurvedLongitudinalStructureOnlyWhereSupported)
{
  TrackPath path;
  path.valid = true;
  path.min_forward_m = 2.0;
  path.max_forward_m = 50.0;
  std::vector<Eigen::Vector3f> points;
  const auto offset = [](double x) {
      const double delta = x - 16.0;
      return 0.30 + 0.0008 * delta * delta;
    };
  for (double x = 16.1; x < 42.0; x += 0.25) {
    points.emplace_back(static_cast<float>(x), static_cast<float>(offset(x) - 0.015), -1.30F);
    points.emplace_back(static_cast<float>(x), static_cast<float>(offset(x) + 0.015), -1.29F);
  }
  // A compact low object must not become a longitudinal lane.
  for (double x = 7.0; x < 8.0; x += 0.08) {
    points.emplace_back(static_cast<float>(x), -0.25F, -1.20F);
  }

  LongitudinalMaskConfig config;
  config.minimum_z_m = -1.65;
  config.maximum_z_m = -1.0;
  config.min_points_per_bin = 2;
  config.min_support_bins = 8;
  config.max_lateral_step_m = 0.24;
  const auto lanes = find_longitudinal_lanes(points, path, config);

  ASSERT_FALSE(lanes.empty());
  EXPECT_TRUE(is_near_longitudinal_lane(
      Eigen::Vector3f(20.0F, static_cast<float>(offset(20.0)), -1.30F), path, lanes,
      config.lane_tolerance_m, config.minimum_z_m, config.maximum_z_m));
  EXPECT_TRUE(is_near_longitudinal_lane(
      Eigen::Vector3f(39.0F, static_cast<float>(offset(39.0)), -1.30F), path, lanes,
      config.lane_tolerance_m, config.minimum_z_m, config.maximum_z_m));
  EXPECT_FALSE(is_near_longitudinal_lane(
      Eigen::Vector3f(8.0F, -0.25F, -1.20F), path, lanes,
      config.lane_tolerance_m, config.minimum_z_m, config.maximum_z_m));
  EXPECT_FALSE(is_near_longitudinal_lane(
      Eigen::Vector3f(39.0F, static_cast<float>(offset(39.0) - 0.35), -1.30F), path, lanes,
      config.lane_tolerance_m, config.minimum_z_m, config.maximum_z_m));
}
