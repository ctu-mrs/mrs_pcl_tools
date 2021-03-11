cmake_minimum_required(VERSION 2.8.3)
project(mrs_pcl_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nodelet
    message_generation
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
    mrs_lib
    dynamic_reconfigure
    ouster_ros
    octomap_ros
    octomap_msgs
)

add_service_files(DIRECTORY srv/registration FILES
  SrvRegisterPointCloudByName.srv
)

find_package(PCL REQUIRED COMPONENTS)

# Include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# Compile with distcc when available (must be after find_package(catkin))
find_file(DISTCC_FULL_PATH distcc)
find_file(DISTCPP_FULL_PATH distg++)
IF (EXISTS ${DISTCC_FULL_PATH})
 IF (EXISTS ${DISTCPP_FULL_PATH})
   message("Found distcc and distg++, using distributed compilation")
   SET(CMAKE_C_COMPILER ${DISTCC_FULL_PATH})
   SET(CMAKE_CXX_COMPILER ${DISTCPP_FULL_PATH})
 ELSE()
   message("Found only distcc, but not distg++, not using distributed compilation")
 ENDIF()
ELSE()
  message("Not found distcc, not using distributed compilation")
ENDIF()

generate_dynamic_reconfigure_options(
  cfg/pcl_filtration_dynparam.cfg
  cfg/pcl2map_registration_dynparam.cfg
)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PCLSupportLib PCLFiltration PCL2MapRegistration
  CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs geometry_msgs pcl_ros mrs_lib
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# This causes adds the -march=native gcc flag that prevents ditributed compilation
add_definitions(
  ${PCL_DEFINITIONS}
  )

# Binary version of pcl on most of the linux distro is built with -march=native
# which cause a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using binary version
# add_definitions(-DPCL_NO_PRECOMPILE)

### LIBRARIES

# Library PCLSupportLib
add_library(PCLSupportLib
  src/support.cpp
  )
target_link_libraries(
  PCLSupportLib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES}
)

# Nodelet library PCLFiltration
add_library(PCLFiltration
  src/PCLFiltration.cpp
  )
add_dependencies(PCLFiltration mrs_pcl_tools_gencfg mrs_pcl_tools_generate_messages)
target_link_libraries(
  PCLFiltration PCLSupportLib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES}
)
target_include_directories(PCLFiltration SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

# Nodelet library PCL2MapRegistration
add_library(PCL2MapRegistration
  src/PCL2MapRegistration.cpp
  )
add_dependencies(PCL2MapRegistration mrs_pcl_tools_gencfg mrs_pcl_tools_generate_messages)
target_link_libraries(
  PCL2MapRegistration PCLSupportLib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES}
)
target_include_directories(PCL2MapRegistration SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

# Nodelet library PCLPublishPCDToNetwork
add_library(PCLPublishPCDToNetwork
  src/PCLPublishPCDToNetwork.cpp
  )
target_link_libraries(
  PCLPublishPCDToNetwork PCLSupportLib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES}
)
target_include_directories(PCLPublishPCDToNetwork SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

# ### EXECUTABLES
add_executable(rosbag_cloud_to_pcd src/executables/RosbagCloudToPCD.cpp)
target_link_libraries(rosbag_cloud_to_pcd
  PCLSupportLib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES}
)

add_executable(estimate_cloud_to_cloud_drift src/executables/EstimateCloudToCloudDrift.cpp)
target_link_libraries(estimate_cloud_to_cloud_drift
  PCLSupportLib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES}
)
