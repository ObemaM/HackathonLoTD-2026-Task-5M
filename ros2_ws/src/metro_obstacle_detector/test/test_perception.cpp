#include <gtest/gtest.h>
#include "metro_obstacle_detector/object_tracker.hpp"
#include "metro_obstacle_detector/cloud_processing.hpp"
#include <pcl/common/transforms.h>

using namespace metro_obstacle_detector;

ObjectObservation object(float x, float y = 0) {
  return {{x, y, 0}, {0.5F, 0.5F, 1.7F}};
}

TEST(ObjectTracker, DifferentObjectsDoNotConfirmEachOther) {
  ObjectTracker tracker;
  EXPECT_FALSE(tracker.update({object(10)}, 3, 2)[0].confirmed);
  EXPECT_FALSE(tracker.update({object(15)}, 3, 2)[0].confirmed);
  EXPECT_FALSE(tracker.update({object(20)}, 3, 2)[0].confirmed);
}
TEST(ObjectTracker, ConfirmsEachObjectAndIgnoresOrdering) {
  ObjectTracker tracker;
  auto a = tracker.update({object(10), object(20)}, 3, 2);
  tracker.update({object(20), object(10)}, 3, 2);
  auto b = tracker.update({object(20), object(10)}, 3, 2);
  EXPECT_TRUE(b[0].confirmed);
  EXPECT_TRUE(b[1].confirmed);
  EXPECT_EQ(a[0].id, b[1].id);
  EXPECT_EQ(a[1].id, b[0].id);
}
TEST(ObjectTracker, MissesDoNotPublishStaleTracksAndResetConsecutiveHits) {
  ObjectTracker tracker;
  tracker.update({object(10)}, 3, 2);
  tracker.update({object(10)}, 3, 2);
  EXPECT_TRUE(tracker.update({}, 3, 2).empty());
  EXPECT_FALSE(tracker.update({object(10)}, 3, 2)[0].confirmed);
  tracker.reset();
  EXPECT_EQ(tracker.update({object(10)}, 3, 2)[0].hits, 1);
}
TEST(ObjectTracker, EgoMotionPreservesIdentity) {
  ObjectTracker tracker;
  auto a = tracker.update({object(20)}, 2, 2);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0, 3) = -2;
  auto b = tracker.update({object(18)}, 2, 2, transform);
  EXPECT_EQ(a[0].id, b[0].id);
  EXPECT_TRUE(b[0].confirmed);
}
TEST(Clustering, JoinsSparseFarTargetWithoutJoiningSeparateObjects) {
  auto cloud = std::make_shared<ProcessingCloud>();
  for (float x : {49.9F, 52.0F}) {
    for (float z : {0.0F, 0.4F, 0.8F, 1.2F}) {
      pcl::PointXYZI p; p.x=x; p.y=0; p.z=z; cloud->push_back(p);
    }
  }
  EXPECT_TRUE(adaptive_clusters(cloud, .3, 0, .6, 3, 100).empty());
  EXPECT_EQ(adaptive_clusters(cloud, .3, .006, .6, 3, 100).size(), 2U);
}
TEST(Motion, EmptyCloudAndLargeTimeGapAreRejected) {
  auto cloud = std::make_shared<ProcessingCloud>();
  EXPECT_FALSE(estimate_motion(cloud, cloud, 0.1).valid);
  EXPECT_FALSE(estimate_motion(cloud, cloud, 1.0).valid);
}
TEST(Motion, RecoversSmallTranslationOnStaticInfrastructure) {
  auto previous = std::make_shared<ProcessingCloud>();
  for (int i=0; i<50; ++i) {
    for (int j=0; j<12; ++j) {
      pcl::PointXYZI p;
      p.x=3.0F + i * .53F; p.y=2.0F + .3F * std::sin(i * .6F);
      p.z=-1.0F + j * .47F; previous->push_back(p);
    }
  }
  auto current = std::make_shared<ProcessingCloud>();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0,3)=-.12F; transform(1,3)=.04F;
  pcl::transformPointCloud(*previous, *current, transform);
  auto motion = estimate_motion(previous, current, .1);
  ASSERT_TRUE(motion.valid);
  EXPECT_NEAR(motion.previous_to_current(0,3), -.12, .05);
  EXPECT_NEAR(motion.previous_to_current(1,3), .04, .05);
}

TEST(Separators, RequiresChainAndRejectsSinglePerson) {
  auto cloud = std::make_shared<ProcessingCloud>();
  auto add_post = [&](float x) {
    for (float z=-1; z<1.1F; z+=.1F) {
      pcl::PointXYZI p; p.x=x; p.y=2; p.z=z; cloud->push_back(p);
    }
  };
  add_post(10);
  EXPECT_TRUE(find_separators(cloud, -1.65).empty());
  add_post(18); add_post(26);
  auto boundaries = find_separators(cloud, -1.65);
  ASSERT_EQ(boundaries.size(), 1U);
  EXPECT_NEAR(boundaries[0].intercept_m, 2, .1);
  EXPECT_NEAR(boundaries[0].min_forward_m, 10, .1);
  EXPECT_NEAR(boundaries[0].max_forward_m, 26, .1);
}
