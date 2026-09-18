#include "metro_obstacle_detector/track_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace metro_obstacle_detector
{

namespace
{

struct RailCandidate
{
  double center_m;
  double emission;
  int support;
};

int rounded_index(double value)
{
  return static_cast<int>(std::floor(value + 0.5));
}

}  // namespace

double TrackPath::center_at(double forward_m) const
{
  if (!centerline.empty()) {
    const double clamped = std::clamp(
      forward_m, centerline.front().forward_m, centerline.back().forward_m);
    const auto upper = std::upper_bound(
      centerline.begin(), centerline.end(), clamped,
      [](double value, const TrackCenterSample & sample) {
        return value < sample.forward_m;
      });
    if (upper == centerline.begin()) {
      return upper->lateral_m;
    }
    if (upper == centerline.end()) {
      return centerline.back().lateral_m;
    }
    const auto & right = *upper;
    const auto & left = *(upper - 1);
    const double span = right.forward_m - left.forward_m;
    if (span <= std::numeric_limits<double>::epsilon()) {
      return left.lateral_m;
    }
    const double ratio = (clamped - left.forward_m) / span;
    return left.lateral_m + ratio * (right.lateral_m - left.lateral_m);
  }
  const double clamped = std::clamp(forward_m, min_forward_m, max_forward_m);
  return coefficients[0] + coefficients[1] * clamped +
         coefficients[2] * clamped * clamped;
}

RailPathEstimator::RailPathEstimator(RailEstimatorConfig config)
: config_(std::move(config))
{
  if (config_.max_forward_m <= config_.min_forward_m ||
    config_.search_half_width_m <= 0.0 || config_.gauge_m <= 0.0 ||
    config_.max_gauge_error_m <= 0.0 ||
    config_.rail_z_max_m <= config_.rail_z_min_m ||
    config_.forward_bin_size_m <= 0.0 || config_.lateral_step_m <= 0.0 ||
    config_.rail_support_half_width_m <= 0.0 || config_.min_points_per_rail < 1 ||
    config_.min_support_bins < 3 || config_.max_center_step_m <= 0.0 ||
    config_.max_slope_change <= 0.0 || config_.seed_search_bins < 1 ||
    config_.seed_max_offset_m <= 0.0 || config_.prior_max_deviation_m <= 0.0 ||
    config_.prior_deviation_growth_per_m < 0.0 ||
    config_.max_gap_bins < 0 || config_.candidate_min_separation_m <= 0.0 ||
    config_.branch_min_separation_m <= config_.candidate_min_separation_m ||
    config_.branch_max_separation_m <= config_.branch_min_separation_m ||
    config_.branch_score_ratio <= 0.0 || config_.branch_score_ratio > 1.0 ||
    config_.branch_min_growth_m_per_bin < 0.0 ||
    config_.branch_confirmation_bins < 1 || config_.max_extrapolation_m < 0.0)
  {
    throw std::invalid_argument("Invalid rail estimator configuration");
  }
}

TrackPath RailPathEstimator::estimate(
  const std::vector<Eigen::Vector3f> & points,
  const TrackPath * prior,
  const std::vector<SeparatorBoundary> & separators) const
{
  TrackPath result;
  result.min_forward_m = config_.min_forward_m;

  const int forward_bins = std::max(
    1, static_cast<int>(std::ceil(
      (config_.max_forward_m - config_.min_forward_m) /
      config_.forward_bin_size_m)));
  const int lateral_cells = std::max(
    3, static_cast<int>(std::ceil(
      2.0 * config_.search_half_width_m / config_.lateral_step_m)) + 1);
  const double lateral_min =
    config_.initial_center_lateral_m - config_.search_half_width_m;

  std::vector<int> histogram(
    static_cast<std::size_t>(forward_bins * lateral_cells), 0);
  for (const auto & point : points) {
    if (point.x() < config_.min_forward_m || point.x() >= config_.max_forward_m ||
      point.z() < config_.rail_z_min_m || point.z() > config_.rail_z_max_m ||
      point.y() < lateral_min ||
      point.y() > config_.initial_center_lateral_m + config_.search_half_width_m)
    {
      continue;
    }
    const int forward_index = static_cast<int>(
      (point.x() - config_.min_forward_m) / config_.forward_bin_size_m);
    const int lateral_index = rounded_index(
      (point.y() - lateral_min) / config_.lateral_step_m);
    if (forward_index >= 0 && forward_index < forward_bins &&
      lateral_index >= 0 && lateral_index < lateral_cells)
    {
      ++histogram[static_cast<std::size_t>(
          forward_index * lateral_cells + lateral_index)];
    }
  }

  const int support_radius = std::max(
    0, static_cast<int>(std::ceil(
      config_.rail_support_half_width_m / config_.lateral_step_m)));
  struct SupportStats
  {
    int count;
    double peak_m;
  };
  auto support_stats = [&](int forward_index, double lateral) {
      const int center_index = rounded_index(
        (lateral - lateral_min) / config_.lateral_step_m);
      int count = 0;
      int peak_count = -1;
      double peak_lateral = lateral;
      for (int offset = -support_radius; offset <= support_radius; ++offset) {
        const int index = center_index + offset;
        if (index >= 0 && index < lateral_cells) {
          const int cell_count = histogram[static_cast<std::size_t>(
              forward_index * lateral_cells + index)];
          count += cell_count;
          const double cell_lateral = lateral_min + index * config_.lateral_step_m;
          if (cell_count > peak_count ||
            (cell_count == peak_count &&
            std::abs(cell_lateral - lateral) < std::abs(peak_lateral - lateral)))
          {
            peak_count = cell_count;
            peak_lateral = cell_lateral;
          }
        }
      }
      return SupportStats{count, peak_lateral};
    };

  std::vector<double> emission(
    static_cast<std::size_t>(forward_bins * lateral_cells), 0.0);
  std::vector<int> minimum_support(
    static_cast<std::size_t>(forward_bins * lateral_cells), 0);
  std::vector<double> refined_center(
    static_cast<std::size_t>(forward_bins * lateral_cells), 0.0);
  for (int bin = 0; bin < forward_bins; ++bin) {
    for (int state = 0; state < lateral_cells; ++state) {
      const double center = lateral_min + state * config_.lateral_step_m;
      const SupportStats left = support_stats(bin, center + config_.gauge_m / 2.0);
      const SupportStats right = support_stats(bin, center - config_.gauge_m / 2.0);
      const int balanced = std::min(left.count, right.count);
      const double measured_gauge = left.peak_m - right.peak_m;
      const double raw = static_cast<double>(balanced) +
        0.20 * static_cast<double>(left.count + right.count);
      const std::size_t index = static_cast<std::size_t>(bin * lateral_cells + state);
      refined_center[index] = 0.5 * (left.peak_m + right.peak_m);
      if (balanced >= config_.min_points_per_rail &&
        std::abs(measured_gauge - config_.gauge_m) > config_.max_gauge_error_m)
      {
        continue;
      }
      const double reliability = balanced >= config_.min_points_per_rail ? 1.0 : 0.08;
      emission[index] = reliability * std::log1p(raw);
      minimum_support[index] = balanced;
    }
  }

  std::vector<std::vector<RailCandidate>> candidates(
    static_cast<std::size_t>(forward_bins));
  for (int bin = 0; bin < forward_bins; ++bin) {
    std::vector<RailCandidate> ranked;
    for (int state = 0; state < lateral_cells; ++state) {
      const std::size_t index = static_cast<std::size_t>(bin * lateral_cells + state);
      if (minimum_support[index] >= config_.min_points_per_rail) {
        ranked.push_back(RailCandidate{
          refined_center[index],
          emission[index], minimum_support[index]});
      }
    }
    std::sort(
      ranked.begin(), ranked.end(),
      [](const RailCandidate & left, const RailCandidate & right) {
        if (left.emission != right.emission) {
          return left.emission > right.emission;
        }
        return left.support > right.support;
      });
    for (const auto & candidate : ranked) {
      const bool separated = std::all_of(
        candidates[static_cast<std::size_t>(bin)].begin(),
        candidates[static_cast<std::size_t>(bin)].end(),
        [&](const RailCandidate & selected) {
          return std::abs(selected.center_m - candidate.center_m) >=
                 config_.candidate_min_separation_m;
        });
      if (separated) {
        candidates[static_cast<std::size_t>(bin)].push_back(candidate);
      }
    }
  }

  int seed_bin = -1;
  RailCandidate seed{};
  const int last_seed_bin = std::min(forward_bins, config_.seed_search_bins);
  for (int bin = 0; bin < last_seed_bin && seed_bin < 0; ++bin) {
    const double forward = config_.min_forward_m +
      (static_cast<double>(bin) + 0.5) * config_.forward_bin_size_m;
    const double expected = prior != nullptr && prior->valid ?
      prior->center_at(forward) : config_.initial_center_lateral_m;
    double best_value = -std::numeric_limits<double>::infinity();
    for (const auto & candidate : candidates[static_cast<std::size_t>(bin)]) {
      const double offset = candidate.center_m - expected;
      if (std::abs(offset) > config_.seed_max_offset_m) {
        continue;
      }
      const double value = candidate.emission - config_.seed_penalty * offset * offset;
      if (value > best_value) {
        best_value = value;
        seed = candidate;
        seed_bin = bin;
      }
    }
  }
  if (seed_bin < 0) {
    return result;
  }

  const double seed_forward = config_.min_forward_m +
    (static_cast<double>(seed_bin) + 0.5) * config_.forward_bin_size_m;
  std::vector<TrackCenterSample> samples{{seed_forward, seed.center_m}};
  double total_score = seed.emission;
  int last_supported_bin = seed_bin;
  double previous_slope = 0.0;
  bool have_slope = false;
  int gap_bins = 0;
  int ambiguous_bins = 0;
  double previous_ambiguity_separation = 0.0;
  std::size_t ambiguity_start_size = 0U;
  double ambiguity_forward = 0.0;

  for (int bin = seed_bin + 1; bin < forward_bins; ++bin) {
    const double forward = config_.min_forward_m +
      (static_cast<double>(bin) + 0.5) * config_.forward_bin_size_m;
    const auto & last_sample = samples.back();
    const double delta_forward = forward - last_sample.forward_m;
    const double predicted = last_sample.lateral_m + previous_slope * delta_forward;
    const double allowed_step = config_.max_center_step_m *
      static_cast<double>(bin - last_supported_bin);

    struct ScoredCandidate
    {
      RailCandidate candidate;
      double value;
      double slope;
    };
    std::vector<ScoredCandidate> valid;
    for (const auto & candidate : candidates[static_cast<std::size_t>(bin)]) {
      const double prediction_error = candidate.center_m - predicted;
      if (std::abs(prediction_error) > allowed_step) {
        continue;
      }
      const double slope = (candidate.center_m - last_sample.lateral_m) / delta_forward;
      if (have_slope && std::abs(slope - previous_slope) > config_.max_slope_change) {
        continue;
      }
      double prior_cost = 0.0;
      if (prior != nullptr && prior->valid) {
        const double prior_error = candidate.center_m - prior->center_at(forward);
        // The same curve moves toward the sensor between frames, so its far
        // section can legitimately differ more than the near anchor. Growth is
        // capped by continuity and remains well below adjacent-track spacing.
        const double allowed_prior_deviation = config_.prior_max_deviation_m +
          config_.prior_deviation_growth_per_m * std::max(0.0, forward - 15.0);
        if (std::abs(prior_error) > allowed_prior_deviation) {
          continue;
        }
        prior_cost = config_.prior_penalty * prior_error * prior_error;
      }
      valid.push_back(ScoredCandidate{
        candidate,
        candidate.emission - config_.continuity_penalty *
        prediction_error * prediction_error - prior_cost,
        slope});
      // A chain of narrow upright structures is only a soft route hint.
      // It never suppresses obstacle returns, and is not extended beyond support.
      for (const auto & boundary : separators) {
        if (forward < boundary.min_forward_m || forward > boundary.max_forward_m) {continue;}
        const double seed_side = seed.center_m -
          (boundary.intercept_m + boundary.slope * seed_forward);
        const double candidate_side = candidate.center_m -
          (boundary.intercept_m + boundary.slope * forward);
        if (std::abs(seed_side) > 1.0 && seed_side * candidate_side < 0) {
          valid.back().value -= 4.0;
        }
      }
    }

    if (valid.empty()) {
      bool competing_branches = false;
      double competing_separation = 0.0;
      const auto & bin_candidates = candidates[static_cast<std::size_t>(bin)];
      for (std::size_t left_index = 0; left_index < bin_candidates.size(); ++left_index) {
        if (std::abs(bin_candidates[left_index].center_m - predicted) >
          config_.branch_max_separation_m)
        {
          continue;
        }
        for (std::size_t right_index = left_index + 1;
          right_index < bin_candidates.size(); ++right_index)
        {
          if (std::abs(bin_candidates[right_index].center_m - predicted) >
            config_.branch_max_separation_m)
          {
            continue;
          }
          const double separation = std::abs(
            bin_candidates[left_index].center_m - bin_candidates[right_index].center_m);
          const double stronger = std::max(
            bin_candidates[left_index].emission, bin_candidates[right_index].emission);
          const double weaker = std::min(
            bin_candidates[left_index].emission, bin_candidates[right_index].emission);
          if (separation >= config_.branch_min_separation_m &&
            separation <= config_.branch_max_separation_m &&
            weaker >= config_.branch_score_ratio * stronger)
          {
            competing_branches = true;
            competing_separation = separation;
            break;
          }
        }
        if (competing_branches) {
          break;
        }
      }
      if (competing_branches) {
        const bool growing = ambiguous_bins > 0 &&
          competing_separation >= previous_ambiguity_separation +
          config_.branch_min_growth_m_per_bin;
        if (!growing) {
          ambiguous_bins = 1;
          ambiguity_start_size = samples.size();
          ambiguity_forward = forward;
        } else {
          ++ambiguous_bins;
        }
        previous_ambiguity_separation = competing_separation;
        if (ambiguous_bins >= config_.branch_confirmation_bins) {
          samples.resize(ambiguity_start_size);
          result.branch_ambiguous = true;
          result.ambiguity_forward_m = ambiguity_forward;
          break;
        }
        continue;
      }
      ambiguous_bins = 0;
      previous_ambiguity_separation = 0.0;
      if (++gap_bins > config_.max_gap_bins) {
        break;
      }
      continue;
    }
    gap_bins = 0;
    std::sort(
      valid.begin(), valid.end(),
      [](const ScoredCandidate & left, const ScoredCandidate & right) {
        return left.value > right.value;
      });
    const auto & best = valid.front();

    bool ambiguous = false;
    double ambiguous_separation = 0.0;
    for (const auto & alternative : candidates[static_cast<std::size_t>(bin)]) {
      const double separation = std::abs(
        alternative.center_m - best.candidate.center_m);
      const double prediction_error = std::abs(alternative.center_m - predicted);
      if (separation >= config_.branch_min_separation_m &&
        separation <= config_.branch_max_separation_m &&
        prediction_error <= config_.branch_max_separation_m &&
        alternative.emission >=
        config_.branch_score_ratio * best.candidate.emission)
      {
        ambiguous = true;
        ambiguous_separation = separation;
        break;
      }
    }
    if (ambiguous) {
      const bool growing = ambiguous_bins > 0 &&
        ambiguous_separation >= previous_ambiguity_separation +
        config_.branch_min_growth_m_per_bin;
      if (!growing) {
        ambiguous_bins = 1;
        ambiguity_start_size = samples.size();
        ambiguity_forward = forward;
      } else {
        ++ambiguous_bins;
      }
      previous_ambiguity_separation = ambiguous_separation;
    } else {
      ambiguous_bins = 0;
      previous_ambiguity_separation = 0.0;
    }

    samples.push_back(TrackCenterSample{forward, best.candidate.center_m});
    total_score += best.candidate.emission;
    previous_slope = have_slope ?
      0.5 * previous_slope + 0.5 * best.slope : best.slope;
    have_slope = true;
    last_supported_bin = bin;

    if (ambiguous_bins >= config_.branch_confirmation_bins) {
      samples.resize(ambiguity_start_size);
      result.branch_ambiguous = true;
      result.ambiguity_forward_m = ambiguity_forward;
      break;
    }
  }

  if (samples.size() < static_cast<std::size_t>(config_.min_support_bins)) {
    return result;
  }

  if (samples.size() >= 3U) {
    std::vector<TrackCenterSample> smoothed = samples;
    for (std::size_t index = 1; index + 1 < samples.size(); ++index) {
      smoothed[index].lateral_m = 0.25 * samples[index - 1].lateral_m +
        0.50 * samples[index].lateral_m +
        0.25 * samples[index + 1].lateral_m;
    }
    samples.swap(smoothed);
  }

  result.valid = true;
  result.centerline = std::move(samples);
  // Do not draw unsupported rails backwards from the first actual observation.
  result.min_forward_m = result.centerline.front().forward_m;
  result.support_bins = result.centerline.size();
  result.score = total_score;
  const double last_supported_forward = result.centerline.back().forward_m;
  result.max_forward_m = std::min(
    config_.max_forward_m,
    last_supported_forward + config_.max_extrapolation_m);
  if (result.branch_ambiguous) {
    result.max_forward_m = std::min(result.max_forward_m, result.ambiguity_forward_m);
  }
  return result;
}

bool is_near_rail(
  const Eigen::Vector3f & point,
  const TrackPath & path,
  double gauge_m,
  double lateral_tolerance_m,
  double minimum_z_m,
  double maximum_z_m)
{
  if (!path.valid || point.x() < path.min_forward_m || point.x() > path.max_forward_m ||
    point.z() < minimum_z_m || point.z() > maximum_z_m)
  {
    return false;
  }
  const double center = path.center_at(point.x());
  const double left_distance = std::abs(point.y() - (center + gauge_m / 2.0));
  const double right_distance = std::abs(point.y() - (center - gauge_m / 2.0));
  return std::min(left_distance, right_distance) <= lateral_tolerance_m;
}

double LongitudinalLane::offset_at(double forward_m) const
{
  if (offsets.empty()) {
    return 0.0;
  }
  const double clamped = std::clamp(forward_m, offsets.front().forward_m, offsets.back().forward_m);
  const auto upper = std::upper_bound(
    offsets.begin(), offsets.end(), clamped,
    [](double value, const TrackCenterSample & sample) {return value < sample.forward_m;});
  if (upper == offsets.begin()) {
    return upper->lateral_m;
  }
  if (upper == offsets.end()) {
    return offsets.back().lateral_m;
  }
  const auto & right = *upper;
  const auto & left = *(upper - 1);
  const double span = right.forward_m - left.forward_m;
  if (span <= std::numeric_limits<double>::epsilon()) {
    return left.lateral_m;
  }
  const double ratio = (clamped - left.forward_m) / span;
  return left.lateral_m + ratio * (right.lateral_m - left.lateral_m);
}

bool LongitudinalLane::covers(double forward_m) const
{
  return !offsets.empty() && forward_m >= min_forward_m && forward_m <= max_forward_m;
}

LongitudinalLane make_fixed_longitudinal_lane(const TrackPath & path, double offset_m)
{
  LongitudinalLane lane;
  if (!path.valid || path.max_forward_m <= path.min_forward_m) {
    return lane;
  }
  lane.offsets = {
    TrackCenterSample{path.min_forward_m, offset_m},
    TrackCenterSample{path.max_forward_m, offset_m}};
  lane.min_forward_m = path.min_forward_m;
  lane.max_forward_m = path.max_forward_m;
  lane.fixed = true;
  return lane;
}

std::vector<LongitudinalLane> find_longitudinal_lanes(
  const std::vector<Eigen::Vector3f> & points,
  const TrackPath & path,
  const LongitudinalMaskConfig & config)
{
  if (!path.valid || config.maximum_z_m <= config.minimum_z_m ||
    config.corridor_half_width_m <= 0.0 || config.forward_bin_size_m <= 0.0 ||
    config.lateral_cell_size_m <= 0.0 || config.lane_tolerance_m <= 0.0 ||
    config.max_lateral_step_m <= 0.0 || config.min_points_per_bin < 1 ||
    config.min_support_bins < 1 || config.max_gap_bins < 0 || config.max_lanes < 1)
  {
    return {};
  }

  const int forward_bins = std::max(
    1, static_cast<int>(std::ceil(
      (path.max_forward_m - path.min_forward_m) / config.forward_bin_size_m)));
  const int lateral_cells = std::max(
    3, static_cast<int>(std::ceil(
      2.0 * config.corridor_half_width_m / config.lateral_cell_size_m)) + 1);
  std::vector<int> counts(
    static_cast<std::size_t>(forward_bins * lateral_cells), 0);

  for (const auto & point : points) {
    if (point.x() < path.min_forward_m || point.x() >= path.max_forward_m ||
      point.z() < config.minimum_z_m || point.z() > config.maximum_z_m)
    {
      continue;
    }
    const double offset = point.y() - path.center_at(point.x());
    if (std::abs(offset) > config.corridor_half_width_m) {
      continue;
    }
    const int forward_index = static_cast<int>(
      (point.x() - path.min_forward_m) / config.forward_bin_size_m);
    const int lateral_index = rounded_index(
      (offset + config.corridor_half_width_m) / config.lateral_cell_size_m);
    if (forward_index >= 0 && forward_index < forward_bins &&
      lateral_index >= 0 && lateral_index < lateral_cells)
    {
      ++counts[static_cast<std::size_t>(forward_index * lateral_cells + lateral_index)];
    }
  }

  struct Peak
  {
    int bin;
    int cell;
    int points;
  };
  std::vector<Peak> peaks;
  std::vector<std::vector<int>> peaks_by_bin(static_cast<std::size_t>(forward_bins));
  for (int bin = 0; bin < forward_bins; ++bin) {
    for (int cell = 0; cell < lateral_cells; ++cell) {
      const int count = counts[static_cast<std::size_t>(bin * lateral_cells + cell)];
      if (count < config.min_points_per_bin) {
        continue;
      }
      bool local_maximum = true;
      for (int neighbour = std::max(0, cell - 2);
        neighbour <= std::min(lateral_cells - 1, cell + 2); ++neighbour)
      {
        if (counts[static_cast<std::size_t>(bin * lateral_cells + neighbour)] > count) {
          local_maximum = false;
          break;
        }
      }
      if (local_maximum) {
        peaks_by_bin[static_cast<std::size_t>(bin)].push_back(static_cast<int>(peaks.size()));
        peaks.push_back(Peak{bin, cell, count});
      }
    }
  }

  std::vector<LongitudinalLane> lanes;
  std::vector<bool> available(peaks.size(), true);
  const int maximum_gap = config.max_gap_bins + 1;
  const int removal_radius = std::max(
    1, static_cast<int>(std::floor(
      2.0 * config.lane_tolerance_m / config.lateral_cell_size_m)));

  while (lanes.size() < static_cast<std::size_t>(config.max_lanes)) {
    std::vector<int> support(peaks.size(), 0);
    std::vector<double> score(peaks.size(), 0.0);
    std::vector<int> predecessor(peaks.size(), -1);
    int best_index = -1;

    for (std::size_t index = 0; index < peaks.size(); ++index) {
      if (!available[index]) {
        continue;
      }
      support[index] = 1;
      score[index] = std::log1p(static_cast<double>(peaks[index].points));
      const int first_bin = std::max(0, peaks[index].bin - maximum_gap);
      for (int bin = first_bin; bin < peaks[index].bin; ++bin) {
        const int delta_bins = peaks[index].bin - bin;
        const double allowed_step = config.max_lateral_step_m * delta_bins;
        for (const int previous : peaks_by_bin[static_cast<std::size_t>(bin)]) {
          if (!available[static_cast<std::size_t>(previous)] || support[static_cast<std::size_t>(previous)] == 0) {
            continue;
          }
          const double lateral_change = config.lateral_cell_size_m * std::abs(
            peaks[index].cell - peaks[static_cast<std::size_t>(previous)].cell);
          if (lateral_change > allowed_step) {
            continue;
          }
          const int proposed_support = support[static_cast<std::size_t>(previous)] + 1;
          const double normalized_step = lateral_change / allowed_step;
          const double proposed_score = score[static_cast<std::size_t>(previous)] +
            std::log1p(static_cast<double>(peaks[index].points)) -
            0.35 * normalized_step * normalized_step - 0.25 * (delta_bins - 1);
          if (proposed_support > support[index] ||
            (proposed_support == support[index] && proposed_score > score[index]))
          {
            support[index] = proposed_support;
            score[index] = proposed_score;
            predecessor[index] = previous;
          }
        }
      }
      if (best_index < 0 || support[index] > support[static_cast<std::size_t>(best_index)] ||
        (support[index] == support[static_cast<std::size_t>(best_index)] &&
        score[index] > score[static_cast<std::size_t>(best_index)]))
      {
        best_index = static_cast<int>(index);
      }
    }

    if (best_index < 0 || support[static_cast<std::size_t>(best_index)] < config.min_support_bins) {
      break;
    }

    std::vector<int> chain;
    for (int index = best_index; index >= 0; index = predecessor[static_cast<std::size_t>(index)]) {
      chain.push_back(index);
    }
    std::reverse(chain.begin(), chain.end());

    LongitudinalLane lane;
    lane.support_bins = chain.size();
    for (const int index : chain) {
      const auto & peak = peaks[static_cast<std::size_t>(index)];
      lane.offsets.push_back(TrackCenterSample{
        path.min_forward_m + (peak.bin + 0.5) * config.forward_bin_size_m,
        -config.corridor_half_width_m + peak.cell * config.lateral_cell_size_m});
    }
    if (lane.offsets.size() >= 3U) {
      auto smoothed = lane.offsets;
      for (std::size_t index = 1; index + 1 < lane.offsets.size(); ++index) {
        smoothed[index].lateral_m = 0.25 * lane.offsets[index - 1].lateral_m +
          0.50 * lane.offsets[index].lateral_m +
          0.25 * lane.offsets[index + 1].lateral_m;
      }
      lane.offsets.swap(smoothed);
    }
    lane.min_forward_m = std::max(
      path.min_forward_m, lane.offsets.front().forward_m - 0.5 * config.forward_bin_size_m);
    lane.max_forward_m = std::min(
      path.max_forward_m, lane.offsets.back().forward_m + 0.5 * config.forward_bin_size_m);
    lanes.push_back(std::move(lane));

    for (const int selected : chain) {
      const auto & peak = peaks[static_cast<std::size_t>(selected)];
      for (const int nearby : peaks_by_bin[static_cast<std::size_t>(peak.bin)]) {
        if (std::abs(peaks[static_cast<std::size_t>(nearby)].cell - peak.cell) <= removal_radius) {
          available[static_cast<std::size_t>(nearby)] = false;
        }
      }
    }
  }
  return lanes;
}

bool is_near_longitudinal_lane(
  const Eigen::Vector3f & point,
  const TrackPath & path,
  const std::vector<LongitudinalLane> & lanes,
  double lateral_tolerance_m,
  double minimum_z_m,
  double maximum_z_m)
{
  if (!path.valid || point.z() < minimum_z_m || point.z() > maximum_z_m) {
    return false;
  }
  const double offset = point.y() - path.center_at(point.x());
  return std::any_of(
    lanes.begin(), lanes.end(),
    [&](const LongitudinalLane & lane) {
      return lane.covers(point.x()) &&
             std::abs(offset - lane.offset_at(point.x())) <= lateral_tolerance_m;
    });
}

}  // namespace metro_obstacle_detector
