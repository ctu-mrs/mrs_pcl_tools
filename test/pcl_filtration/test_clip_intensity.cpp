#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <mrs_pcl_tools/pcl_filtration_core.h>


template <typename PointT>
bool isInvalidated(const PointT& p)
{
  return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
}

/* TEST(IntensityClipping, checkIntensityField) //{ */
TEST(IntensityClipping, checkIntensityField)
{
  auto [range_exists1, range_offset1] = mrs_pcl_tools::getFieldOffset<pcl::PointXYZI>("intensity");
  ASSERT_TRUE(range_exists1);

  auto [range_exists2, range_offset2] = mrs_pcl_tools::getFieldOffset<pcl::PointXYZ>("intensity");
  ASSERT_FALSE(range_exists2);
}
//}


/* TEST(IntensityClipping, removeLowFields) //{ */
TEST(IntensityClipping, removeLowFields)
{
  DummyLogger logger;
  mrs_pcl_tools::Lidar3DConfig cfg;
  cfg.intensity.use = true;
  cfg.intensity.threshold = 100.0;
  cfg.intensity.range_mm = 5000;
  cfg.intensity.range_sq = cfg.intensity.range_mm * cfg.intensity.range_mm;
  cfg.invalid_value = std::numeric_limits<float>::quiet_NaN();

  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);
  pcl_filtration_core.loadLidarParams(cfg);

  using PC = pcl::PointCloud<pcl::PointXYZI>;
  auto pc = std::make_shared<PC>();
  pc->points.resize(3);

  // Points: min-1, min, min+1
  pc->points[0].intensity = 99.0;
  pc->points[1].intensity = 100.0;
  pc->points[2].intensity = 101.0;

  auto removed = pcl_filtration_core.removeLowFields<PC>(pc, /*publish_removed*/ true);

  EXPECT_TRUE(isInvalidated(pc->points[0]));
  EXPECT_FALSE(isInvalidated(pc->points[1]));
  EXPECT_FALSE(isInvalidated(pc->points[2]));

  // removed pc should contain 1 points if both flags set
  EXPECT_EQ(removed->size(), 1u);
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  // initialize the random number generator
  /* srand(static_cast<unsigned int>(time(0))); */
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}