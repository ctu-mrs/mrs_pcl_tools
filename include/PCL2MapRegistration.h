#pragma once

/* includes //{ */
#include "common_includes_and_typedefs.h"

//}

namespace mrs_pcl_tools
{

/* class PCL2MapRegistration //{ */
class PCL2MapRegistration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized = false;

  PC::Ptr _pc_map; 
  PC::Ptr _pc_slam; 

  PC::Ptr load_pc(std::string path);

};
//}

}  // namespace mrs_pcl_tools
