#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <vector>

namespace metro_obstacle_detector
{
struct ObjectObservation
{
  Eigen::Vector3f center;
  Eigen::Vector3f size;
};
struct ObjectState
{
  int id{0};
  int hits{0};
  int misses{0};
  bool confirmed{false};
  ObjectObservation observation;
};

// Greedy global distance ordering with one-to-one assignment. Only observed
// tracks are returned: missing tracks retain identity, never stale distances.
class ObjectTracker
{
public:
  void reset() {tracks_.clear();}
  std::vector<ObjectState> update(
    const std::vector<ObjectObservation> & observations, int required, int clear,
    const Eigen::Matrix4f & previous_to_current = Eigen::Matrix4f::Identity())
  {
    for (auto & track : tracks_) {
      track.observation.center = (previous_to_current *
        track.observation.center.homogeneous()).head<3>();
    }
    struct Edge {float distance; std::size_t track; std::size_t observation;};
    std::vector<Edge> edges;
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      for (std::size_t j = 0; j < observations.size(); ++j) {
        const auto delta = observations[j].center - tracks_[i].observation.center;
        const auto size_delta = observations[j].size - tracks_[i].observation.size;
        // Residual gate after ego-motion correction; do not let separate nearby
        // posts transfer confirmation freely across the track direction.
        if (delta.head<2>().norm() <= 0.75F && std::abs(delta.z()) <= 0.6F &&
          size_delta.norm() <= 1.2F)
        {
          edges.push_back({delta.norm() + 0.25F * size_delta.norm(), i, j});
        }
      }
    }
    std::sort(edges.begin(), edges.end(), [](const Edge & a, const Edge & b) {
      return a.distance < b.distance;
    });
    std::vector<bool> used_track(tracks_.size(), false), used_obs(observations.size(), false);
    std::vector<ObjectState> result(observations.size());
    for (const auto & edge : edges) {
      if (used_track[edge.track] || used_obs[edge.observation]) {continue;}
      auto & t = tracks_[edge.track];
      t.hits = t.misses == 0 ? t.hits + 1 : 1;
      t.misses = 0;
      t.confirmed = t.confirmed || t.hits >= required;
      t.observation = observations[edge.observation];
      result[edge.observation] = t;
      used_track[edge.track] = used_obs[edge.observation] = true;
    }
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      if (!used_track[i]) {++tracks_[i].misses;}
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [clear](const auto & t) {
      return t.misses >= clear;
    }), tracks_.end());
    for (std::size_t j = 0; j < observations.size(); ++j) {
      if (!used_obs[j]) {
        ObjectState state{next_id_++, 1, 0, required <= 1, observations[j]};
        tracks_.push_back(state);
        result[j] = state;
      }
    }
    return result;
  }
private:
  std::vector<ObjectState> tracks_;
  int next_id_{1};
};
}  // namespace metro_obstacle_detector
