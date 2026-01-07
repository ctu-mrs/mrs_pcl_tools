#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <mrs_pcl_tools/pcl_filtration_core.h>

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{

  // initialize the random number generator
  /* srand(static_cast<unsigned int>(time(0))); */
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}