#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <mrs_pcl_tools/utils/pointcloud_io.h>

/* TEST(TESTSuite, loadPcXYZ_success) //{ */
TEST(TESTSuite, loadPcXYZ_success)
{
  DummyLogger logger;

  std::string path = std::string(TEST_ASSETS_DIR) + "/lamppost.pcd";
  auto result = mrs_pcl_tools::loadPcXYZ(logger, path);

  ASSERT_TRUE(result.has_value());
  ASSERT_NE(result.value(), nullptr);
  EXPECT_GT(result.value()->points.size(), 0u);
}
//}

/* TEST(TESTSuite, loadPcXYZ_failure) //{ */
TEST(TESTSuite, loadPcXYZ_failure)
{
  DummyLogger logger;
  
  std::string path = std::string(TEST_ASSETS_DIR) + "/does_not_exist.pcd";
  auto result = mrs_pcl_tools::loadPcXYZ(logger, path);

  ASSERT_FALSE(result.has_value());
}
//}

/* TEST(TESTSuite, loadCloud_pcd) //{ */
TEST(TESTSuite, loadCloud_pcd)
{
  DummyLogger logger;

  std::string path = std::string(TEST_ASSETS_DIR) + "/lamppost.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  bool result = mrs_pcl_tools::loadCloud(logger, path, cloud, true);

  ASSERT_TRUE(result);
}
//}

/* TEST(TESTSuite, loadCloud_unknownType) //{ */
TEST(TESTSuite, loadCloud_unknownType)
{
  DummyLogger logger;
  
  std::string path = std::string(TEST_ASSETS_DIR) + "/lamppost.unknown";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  bool result = mrs_pcl_tools::loadCloud(logger, path, cloud, true);

  ASSERT_FALSE(result);
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