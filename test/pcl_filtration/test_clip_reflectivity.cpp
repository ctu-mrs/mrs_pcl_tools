#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <mrs_pcl_tools/pcl_filtration_core.h>


struct PointXYZR
{
  PCL_ADD_POINT4D;
  uint16_t reflectivity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZR,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (uint16_t, reflectivity, reflectivity)
)
// clang-format on

template <typename PointT>
bool isInvalidated(const PointT& p)
{
  return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
}

/* TEST(ReflectivityClipping, checkRangeField) //{ */
TEST(ReflectivityClipping, checkRangeField)
{
  auto [range_exists1, range_offset1] = mrs_pcl_tools::getFieldOffset<PointXYZR>("reflectivity");
  ASSERT_TRUE(range_exists1);

  auto [range_exists2, range_offset2] = mrs_pcl_tools::getFieldOffset<pcl::PointXYZI>("reflectivity");
  ASSERT_FALSE(range_exists2);
}
//}


/* TEST(ReflectivityClipping, removeLowFields) //{ */
TEST(ReflectivityClipping, removeLowFields)
{
  DummyLogger logger;
  mrs_pcl_tools::Lidar3DConfig cfg;
  cfg.reflectivity.use = true;
  cfg.reflectivity.threshold = 25;
  cfg.reflectivity.range_mm = 5.0;
  cfg.reflectivity.range_sq = cfg.reflectivity.range_mm * cfg.reflectivity.range_mm;
  cfg.invalid_value = std::numeric_limits<float>::quiet_NaN();

  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);
  pcl_filtration_core.loadLidarParams(cfg);

  using PC = pcl::PointCloud<PointXYZR>;
  auto pc = std::make_shared<PC>();
  pc->points.resize(3);

  // Points: min-1, min, min+1
  pc->points[0].reflectivity = 24;
  pc->points[1].reflectivity = 25;
  pc->points[2].reflectivity = 26;

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