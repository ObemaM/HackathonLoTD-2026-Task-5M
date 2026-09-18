#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <vector>

namespace metro_obstacle_detector
{

struct RailEstimatorConfig
{
  double min_forward_m{2.0};
  double max_forward_m{90.0};
  double initial_center_lateral_m{0.0};
  double search_half_width_m{4.0};
  double rail_z_min_m{-1.75};
  double rail_z_max_m{-1.15};
  double gauge_m{1.52};
  double max_gauge_error_m{0.18};
  double rail_support_half_width_m{0.14};
  double forward_bin_size_m{2.0};
  double lateral_step_m{0.05};
  int min_points_per_rail{2};
  int min_support_bins{6};
  double max_center_step_m{0.55};
  double max_slope_change{0.10};
  double seed_penalty{2.0};
  int seed_search_bins{5};
  double seed_max_offset_m{0.85};
  double continuity_penalty{6.0};
  double prior_penalty{1.5};
  double prior_max_deviation_m{0.65};
  double prior_deviation_growth_per_m{0.015};
  int max_gap_bins{1};
  double candidate_min_separation_m{0.25};
  double branch_min_separation_m{0.45};
  double branch_max_separation_m{1.80};
  double branch_score_ratio{0.75};
  double branch_min_growth_m_per_bin{0.05};
  int branch_confirmation_bins{4};
  double max_extrapolation_m{2.0};
};

struct TrackCenterSample
{
  double forward_m{0.0};
  double lateral_m{0.0};
};

struct SeparatorBoundary
{
  double min_forward_m;
  double max_forward_m;
  double intercept_m;
  double slope;
};

struct TrackPath
{
  bool valid{false};
  bool branch_ambiguous{false};
  std::array<double, 3> coefficients{0.0, 0.0, 0.0};
  std::vector<TrackCenterSample> centerline;
  double min_forward_m{0.0};
  double max_forward_m{0.0};
  double ambiguity_forward_m{0.0};
  std::size_t support_bins{0U};
  double score{0.0};

  double center_at(double forward_m) const;
};

class RailPathEstimator
{
public:
  explicit RailPathEstimator(RailEstimatorConfig config);

  TrackPath estimate(
    const std::vector<Eigen::Vector3f> & points,
    const TrackPath * prior = nullptr,
    const std::vector<SeparatorBoundary> & separators = {}) const;

private:
  RailEstimatorConfig config_;
};

bool is_near_rail(
  const Eigen::Vector3f & point,
  const TrackPath & path,
  double gauge_m,
  double lateral_tolerance_m,
  double minimum_z_m,
  double maximum_z_m);

struct LongitudinalMaskConfig
{
  double minimum_z_m{-1.65};
  double maximum_z_m{-1.00};
  double corridor_half_width_m{1.05};
  double forward_bin_size_m{2.0};
  double lateral_cell_size_m{0.05};
  double lane_tolerance_m{0.14};
  double max_lateral_step_m{0.24};
  int min_points_per_bin{2};
  int min_support_bins{8};
  int max_gap_bins{1};
  int max_lanes{6};
};

struct LongitudinalLane
{
  std::vector<TrackCenterSample> offsets;
  double min_forward_m{0.0};
  double max_forward_m{0.0};
  std::size_t support_bins{0U};
  bool fixed{false};

  double offset_at(double forward_m) const;
  bool covers(double forward_m) const;
};

LongitudinalLane make_fixed_longitudinal_lane(
  const TrackPath & path,
  double offset_m);

std::vector<LongitudinalLane> find_longitudinal_lanes(
  const std::vector<Eigen::Vector3f> & points,
  const TrackPath & path,
  const LongitudinalMaskConfig & config);

bool is_near_longitudinal_lane(
  const Eigen::Vector3f & point,
  const TrackPath & path,
  const std::vector<LongitudinalLane> & lanes,
  double lateral_tolerance_m,
  double minimum_z_m,
  double maximum_z_m);

}  // namespace metro_obstacle_detector
