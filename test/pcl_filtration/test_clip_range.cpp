#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <mrs_pcl_tools/pcl_filtration_core.h>

struct PointXYZR
{
  PCL_ADD_POINT4D;
  uint32_t range;  // mm
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZR,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (uint32_t, range, range)
)
// clang-format on

template <typename PointT>
bool isInvalidated(const PointT& p)
{
  return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
}

/* TEST(RangeClipping, checkRangeField) //{ */
TEST(RangeClipping, checkRangeField)
{
  auto [range_exists1, range_offset1] = mrs_pcl_tools::getFieldOffset<PointXYZR>("range");
  ASSERT_TRUE(range_exists1);

  auto [range_exists2, range_offset2] = mrs_pcl_tools::getFieldOffset<pcl::PointXYZI>("range");
  ASSERT_FALSE(range_exists2);
}
//}

/* TEST(RangeClipping, checkRangeClip_StoreOnlyClose) //{ */
TEST(RangeClipping, checkRangeClip_StoreOnlyClose)
{
  DummyLogger logger;
  mrs_pcl_tools::Lidar3DConfig cfg;
  cfg.rangeclip.use = true;
  cfg.rangeclip.min_mm = 100;
  cfg.rangeclip.max_mm = 200;
  cfg.rangeclip.min_sq = 100.0f * 100.0f;
  cfg.rangeclip.max_sq = 200.0f * 200.0f;
  cfg.invalid_value = std::numeric_limits<float>::quiet_NaN();

  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);
  pcl_filtration_core.loadLidarParams(cfg);

  using PC = pcl::PointCloud<PointXYZR>;
  auto pc = std::make_shared<PC>();
  pc->points.resize(3);

  // Points: min-1, min, middle_range, max, max+1
  pc->points[0].range = 99;
  pc->points[1].range = 100;
  pc->points[2].range = 201;

  auto removed = pcl_filtration_core.removeCloseAndFar<PC>(pc, /*close*/ true, /*far*/ false);

  EXPECT_TRUE(isInvalidated(pc->points[0]));
  EXPECT_FALSE(isInvalidated(pc->points[1]));
  EXPECT_TRUE(isInvalidated(pc->points[2]));

  // removed pc should contain 1 points if both flags set
  EXPECT_EQ(removed->size(), 1u);
}
//}

/* TEST(RangeClipping, checkRangeClip_StoreOnlyFar) //{ */
TEST(RangeClipping, checkRangeClip_StoreOnlyFar)
{
  DummyLogger logger;
  mrs_pcl_tools::Lidar3DConfig cfg;
  cfg.rangeclip.use = true;
  cfg.rangeclip.min_mm = 100;
  cfg.rangeclip.max_mm = 200;
  cfg.rangeclip.min_sq = 100.0f * 100.0f;
  cfg.rangeclip.max_sq = 200.0f * 200.0f;
  cfg.invalid_value = std::numeric_limits<float>::quiet_NaN();

  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);
  pcl_filtration_core.loadLidarParams(cfg);

  using PC = pcl::PointCloud<PointXYZR>;
  auto pc = std::make_shared<PC>();
  pc->points.resize(3);

  // Points: min-1, min, middle_range, max, max+1
  pc->points[0].range = 99;
  pc->points[1].range = 100;
  pc->points[2].range = 201;

  auto removed = pcl_filtration_core.removeCloseAndFar<PC>(pc, /*close*/ false, /*far*/ true);

  EXPECT_TRUE(isInvalidated(pc->points[0]));
  EXPECT_FALSE(isInvalidated(pc->points[1]));
  EXPECT_TRUE(isInvalidated(pc->points[2]));

  // removed pc should contain 1 points if both flags set
  EXPECT_EQ(removed->size(), 1u);
}
//}

/* TEST(RangeClipping, checkRangeClip_StoreCloseFar) //{ */
TEST(RangeClipping, checkRangeClip_StoreCloseFar)
{
  DummyLogger logger;
  mrs_pcl_tools::Lidar3DConfig cfg;
  cfg.rangeclip.use = true;
  cfg.rangeclip.min_mm = 100;
  cfg.rangeclip.max_mm = 200;
  cfg.rangeclip.min_sq = 100.0f * 100.0f;
  cfg.rangeclip.max_sq = 200.0f * 200.0f;
  cfg.invalid_value = std::numeric_limits<float>::quiet_NaN();

  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);
  pcl_filtration_core.loadLidarParams(cfg);

  using PC = pcl::PointCloud<PointXYZR>;
  auto pc = std::make_shared<PC>();
  pc->points.resize(5);

  // Points: min-1, min, middle_range, max, max+1
  pc->points[0].range = 99;
  pc->points[1].range = 100;
  pc->points[2].range = 150;
  pc->points[3].range = 200;
  pc->points[4].range = 201;

  auto removed = pcl_filtration_core.removeCloseAndFar<PC>(pc, /*close*/ true, /*far*/ true);

  EXPECT_TRUE(isInvalidated(pc->points[0]));
  EXPECT_FALSE(isInvalidated(pc->points[1]));
  EXPECT_FALSE(isInvalidated(pc->points[2]));
  EXPECT_FALSE(isInvalidated(pc->points[3]));
  EXPECT_TRUE(isInvalidated(pc->points[4]));

  // removed pc should contain 2 points if both flags set
  EXPECT_EQ(removed->size(), 2u);
}
//}

/* TEST(RangeClipping, checkCloseAndFarAndLowFields_WithRangeField) //{ */
TEST(RangeClipping, checkCloseAndFarAndLowFields_WithRangeField)
{
  DummyLogger logger;
  mrs_pcl_tools::Lidar3DConfig cfg;
  cfg.rangeclip.use = true;
  cfg.rangeclip.min_mm = 100;
  cfg.rangeclip.max_mm = 200;
  cfg.rangeclip.min_sq = 100.0f * 100.0f;
  cfg.rangeclip.max_sq = 200.0f * 200.0f;
  cfg.invalid_value = std::numeric_limits<float>::quiet_NaN();

  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);
  pcl_filtration_core.loadLidarParams(cfg);

  using PC = pcl::PointCloud<PointXYZR>;
  auto pc = std::make_shared<PC>();
  pc->points.resize(5);

  // Points: min-1, min, middle_range, max, max+1
  pc->points[0].range = 99;
  pc->points[1].range = 100;
  pc->points[2].range = 150;
  pc->points[3].range = 200;
  pc->points[4].range = 201;

  auto removed = pcl_filtration_core.removeCloseAndFarAndLowFields<PC>(pc, /*close*/ true, /*far*/ true);

  EXPECT_TRUE(isInvalidated(pc->points[0]));
  EXPECT_FALSE(isInvalidated(pc->points[1]));
  EXPECT_FALSE(isInvalidated(pc->points[2]));
  EXPECT_FALSE(isInvalidated(pc->points[3]));
  EXPECT_TRUE(isInvalidated(pc->points[4]));

  // removed pc should contain 2 points if both flags set
  EXPECT_EQ(removed->size(), 2u);
}
//}

/* TEST(RangeClipping, checkCloseAndFarAndLowFields_WithoutRangeField) //{ */
TEST(RangeClipping, checkCloseAndFarAndLowFields_WithoutRangeField)
{
  DummyLogger logger;
  mrs_pcl_tools::Lidar3DConfig cfg;
  cfg.rangeclip.use = true;
  cfg.rangeclip.min_mm = 100;
  cfg.rangeclip.max_mm = 200;
  cfg.rangeclip.min_sq = 100.0f * 100.0f;
  cfg.rangeclip.max_sq = 200.0f * 200.0f;
  cfg.invalid_value = std::numeric_limits<float>::quiet_NaN();

  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);
  pcl_filtration_core.loadLidarParams(cfg);

  using PC = pcl::PointCloud<pcl::PointXYZI>;
  auto pc = std::make_shared<PC>();
  pc->points.resize(4);

  // clang-format off
  // Points at distances: 99, 100, 200, 201
  pc->points[0].x =  99.0f; pc->points[0].y = 0.0f; pc->points[0].z = 0.0f;
  pc->points[1].x = 100.0f; pc->points[1].y = 0.0f; pc->points[1].z = 0.0f;
  pc->points[2].x = 200.0f; pc->points[2].y = 0.0f; pc->points[2].z = 0.0f;
  pc->points[3].x = 201.0f; pc->points[3].y = 0.0f; pc->points[3].z = 0.0f;
  // clang-format on

  auto removed = pcl_filtration_core.removeCloseAndFarAndLowFields<PC>(pc, /*close*/ true, /*far*/ true);

  EXPECT_TRUE(isInvalidated(pc->points[0]));
  EXPECT_FALSE(isInvalidated(pc->points[1]));
  EXPECT_FALSE(isInvalidated(pc->points[2]));
  EXPECT_TRUE(isInvalidated(pc->points[3]));

  // removed pc should contain 2 points if both flags set
  EXPECT_EQ(removed->size(), 2u);
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