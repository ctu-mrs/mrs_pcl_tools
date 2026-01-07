#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <mrs_pcl_tools/pcl_filtration_core.h>


/* struct makeOrganizedCloud //{ */
static std::shared_ptr<PC_I> makeOrganizedCloud(uint32_t w, uint32_t h)
{
  auto pc = std::make_shared<PC_I>(w, h);
  pc->is_dense = true;

  for (uint32_t j = 0; j < h; ++j)
  {
    for (uint32_t i = 0; i < w; ++i)
    {
      auto& p = pc->at(i, j);
      p.x = static_cast<float>(i);
      p.y = static_cast<float>(j);
      p.z = static_cast<float>(i + j);
    }
  }
  return pc;
}
//}

/* TEST(Downsample, whenStepsAreOne) //{ */
TEST(Downsample, whenStepsAreOne)
{
  DummyLogger logger;
  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);

  mrs_pcl_tools::DownsampleConfig dp;
  dp.row_step = 1;
  dp.col_step = 1;
  dp.dynamic_row_selection_enabled = false;
  dp.dynamic_row_offset = 0;

  auto pc = makeOrganizedCloud(2048, 128);
  const auto before_points = pc->size();

  pcl_filtration_core.downsample(pc, dp);

  EXPECT_EQ(pc->width, 2048u);
  EXPECT_EQ(pc->height, 128u);
  EXPECT_EQ(pc->size(), before_points);
}
//}

/* TEST(Downsample, reduceDimensionality) //{ */
TEST(Downsample, reduceDimensionality)
{
  DummyLogger logger;
  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);

  mrs_pcl_tools::DownsampleConfig dp;
  dp.row_step = 2;
  dp.col_step = 4;
  dp.dynamic_row_selection_enabled = false;
  dp.dynamic_row_offset = 0;

  auto pc = makeOrganizedCloud(2048, 128);

  pcl_filtration_core.downsample(pc, dp);

  EXPECT_EQ(pc->width, 512u);
  EXPECT_EQ(pc->height, 64u);
  EXPECT_EQ(pc->size(), static_cast<size_t>(512u * 64u));
}
//}

/* TEST(Downsample, reduceDimensionalityWithDynamicRow) //{ */
TEST(Downsample, reduceDimensionalityWithDynamicRow)
{
  DummyLogger logger;
  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);
  mrs_pcl_tools::DownsampleConfig dp;
  dp.row_step = 2;
  dp.col_step = 2;
  dp.dynamic_row_selection_enabled = true;

  auto base = makeOrganizedCloud(6, 6);

  // offset 0 -> sample rows 0,2,4
  {
    auto pc = std::make_shared<PC_I>(*base);
    dp.dynamic_row_offset = 0;
    pcl_filtration_core.downsample(pc, dp);

    EXPECT_EQ(pc->height, 3u);
    EXPECT_FLOAT_EQ(pc->at(0, 0).y, 0.f);
    EXPECT_FLOAT_EQ(pc->at(0, 1).y, 2.f);
    EXPECT_FLOAT_EQ(pc->at(0, 2).y, 4.f);
  }

  // offset 1 -> sample rows 1,3,5
  {
    auto pc = std::make_shared<PC_I>(*base);
    dp.dynamic_row_offset = 1;
    pcl_filtration_core.downsample(pc, dp);

    EXPECT_EQ(pc->height, 3u);
    EXPECT_FLOAT_EQ(pc->at(0, 0).y, 1.f);
    EXPECT_FLOAT_EQ(pc->at(0, 1).y, 3.f);
    EXPECT_FLOAT_EQ(pc->at(0, 2).y, 5.f);
  }
}
//}

/* TEST(Downsample, reduceDimensionalityWithDynamicRow) //{ */
TEST(Downsample, NonDivisibleWidth)
{
  DummyLogger logger;
  mrs_pcl_tools::PCLFiltrationCore pcl_filtration_core(logger);
  mrs_pcl_tools::DownsampleConfig dp;
  dp.row_step = 2;
  dp.col_step = 2;
  dp.dynamic_row_selection_enabled = true;
  dp.dynamic_row_offset = 0;  // row_offset = 0

  auto pc = makeOrganizedCloud(5, 5);  // intentionally not divisible
  pcl_filtration_core.downsample(pc, dp);

  // width_after  = ceil(5/2) = 3
  // height_after = ceil((5 - 0)/2) = 3
  EXPECT_EQ(pc->width, 3u);
  EXPECT_EQ(pc->height, 3u);
  EXPECT_EQ(pc->size(), static_cast<size_t>(3u * 3u));

  // Check some sampled content (top-left and last col)
  // Output (c=0,r=0) samples input (i=0,j=0)
  EXPECT_FLOAT_EQ(pc->at(0, 0).x, 0.f);
  EXPECT_FLOAT_EQ(pc->at(0, 0).y, 0.f);

  // Output last column (c=2,r=0) samples input i=4 (since 0,2,4)
  EXPECT_FLOAT_EQ(pc->at(2, 0).x, 4.f);
  EXPECT_FLOAT_EQ(pc->at(2, 0).y, 0.f);
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