# mrs_pcl_tools

Package grouping smaller nodes for processing, filtering, and general online/offline work with pointclouds.

## Dependencies
[ouster driver](https://mrs.felk.cvut.cz/gitlab/uav/drivers/ouster) for Ouster point type

## PCL cross-compilation
Point Cloud Library (PCL) is available in Debian repositories, however this package has disabled hardware-specific optimizations.
In order to enable hardware-specific optimizations (cmake flag `-march=native`), PCL has to be compiled manually on your machine.
The installation process has been tested on Ubuntu 20.04 and ROS Noetic (PCL version 1.10.1).

**Before installation**:

- make sure your machine has at least 28GB of RAM+SWAP memory (32GB is recommended, manual to increase SWAP memory is e.g., [here](https://askubuntu.com/a/1177939))
- change the PCL build flags/profile if needed in [`mrs_pcl_tools/installation/install_pcl.sh`](https://mrs.felk.cvut.cz/gitlab/uav/perception/mrs_pcl_tools/blob/master/installation/install_pcl.sh) (default profile: `Release`, profiles match `mrs_workspace`)

**Run installation** (can take up to 1 hour, compiler may fail internally so restart if that happens):

```bash
cd mrs_pcl_tools/installation
./install_pcl.sh
```

**After installation**:

- rebuild all your packages depending on PCL (including `mrs_lib`, `catkin clean && catkin build` is recommended on all your workspaces)
- add this snippet to `CMakeLists.txt` file of your PCL-dependent packages
```cmake
# Point Cloud Library: Override CXX flags inherited from catkin workspace, if precompiled PCL binaries from Debian repositories are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from Debian repositories. Disabling flag -march=native by overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using manually compiled PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()
```
