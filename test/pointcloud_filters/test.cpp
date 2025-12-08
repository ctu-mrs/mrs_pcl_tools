#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <mrs_pcl_tools/utils/pointcloud_filters.h>

/* TEST(TESTSuite, returnOriginal_VoxelGridFilter) //{ */
TEST(TESTSuite, returnOriginal_VoxelGridFilter)
{
  DummyLogger logger;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = 100;
  cloud->height = 100;
  cloud->resize(cloud->width * cloud->height);

  for (auto& point : *cloud)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  auto cloud_filtered = mrs_pcl_tools::filters::applyVoxelGridFilter(logger, cloud, 0.0f);

  EXPECT_EQ(cloud_filtered->size(), cloud->size());
}
//}

/* TEST(TESTSuite, downsample_VoxelGridFilter) //{ */
TEST(TESTSuite, downsample_VoxelGridFilter)
{
  DummyLogger logger;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = 100;
  cloud->height = 100;
  cloud->resize(cloud->width * cloud->height);

  for (auto& point : *cloud)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  auto cloud_filtered = mrs_pcl_tools::filters::applyVoxelGridFilter(logger, cloud, 0.1f);

  EXPECT_LT(cloud_filtered->size(), cloud->size());
  EXPECT_GT(cloud_filtered->size(), 0u);
}
//}

/* TEST(TESTSuite, applyRadiusOutlierFilter) //{ */
TEST(TESTSuite, applyRadiusOutlierFilter)
{
  DummyLogger logger;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 5;
  cloud->height = 1;
  cloud->resize(cloud->width * cloud->height);

  // 3 close points
  (*cloud)[0].x = 0.0f; (*cloud)[0].y = 0.0f; (*cloud)[0].z = 0.0f;
  (*cloud)[1].x = 0.1f; (*cloud)[1].y = 0.0f; (*cloud)[1].z = 0.0f;
  (*cloud)[2].x = 0.0f; (*cloud)[2].y = 0.1f; (*cloud)[2].z = 0.0f;

  // 2 isolated outliers
  (*cloud)[3].x = 10.0f; (*cloud)[3].y = 10.0f; (*cloud)[3].z = 10.0f;
  (*cloud)[4].x = -10.0f; (*cloud)[4].y = -10.0f; (*cloud)[4].z = -10.0f;

  const double radius = 0.5;
  const int min_neighbors = 2;
  const bool keep_organized = false;

  auto cloud_filtered = mrs_pcl_tools::filters::applyRadiusOutlierFilter(logger, cloud, radius, min_neighbors, keep_organized);
  EXPECT_EQ(cloud_filtered->size(), 3u);
}
//}


/* TEST(TESTSuite, applyMinimumGridFilter) //{ */
TEST(TESTSuite, applyMinimumGridFilter)
{
  DummyLogger logger;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 3;
  cloud->height = 1;
  cloud->resize(cloud->width * cloud->height);

  const float min_height = 0.1;
  (*cloud)[0].x = 0.0f; (*cloud)[0].y = 0.0f; (*cloud)[0].z = 0.2f;
  (*cloud)[1].x = 0.0f; (*cloud)[1].y = 0.0f; (*cloud)[1].z = min_height;
  (*cloud)[2].x = 0.0f; (*cloud)[2].y = 0.0f; (*cloud)[2].z = 0.3f;


  const float resolution = 0.1;
  auto cloud_filtered = mrs_pcl_tools::filters::applyMinimumGridFilter(logger, cloud, resolution);
  EXPECT_EQ(cloud_filtered->size(), 1u);
  EXPECT_FLOAT_EQ((*cloud_filtered)[0].z, min_height);
}
//}


// /* TEST(TESTSuite, applyBilateralFilter) //{ */
// TEST(TESTSuite, applyBilateralFilter)
// {
//   //TODO: add after figuring out the logger
// }
// //}


int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  // initialize the random number generator
  /* srand(static_cast<unsigned int>(time(0))); */
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}