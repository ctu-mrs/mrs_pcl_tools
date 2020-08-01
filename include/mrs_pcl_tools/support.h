#pragma once

#include "common_includes_and_typedefs.h"

namespace mrs_pcl_tools
{
class PCLHandler {

public:
  PCLHandler();

private:

};
}  // namespace mrs_pcl_tools

/*//{ class TicToc */

class TicToc {
public:
  TicToc() {
    tic();
  }

  void tic() {
    start = std::chrono::system_clock::now();
  }

  double toc() {
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
    return elapsed_seconds.count() * 1000;
  }

  void toc_print(const std::string text) {
    ROS_INFO("%s: %0.2f ms", text.c_str(), toc());
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start;
};

/*//}*/
