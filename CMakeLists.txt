cmake_minimum_required(VERSION 3.5)
project(mrs_pcl_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-O3)
add_definitions(-DCOMPILE_WITH_OUSTER)
# add_definitions(-g)

set(CATKIN_DEPENDENCIES
  cmake_modules
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  message_generation
  message_runtime
  mrs_lib
  mrs_msgs
  nodelet
  ouster_ros
  pcl_conversions
  pcl_ros
  rosconsole
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

add_service_files(DIRECTORY srv/registration FILES
  SrvRegisterPointCloudByName.srv
  SrvRegisterPointCloudOffline.srv
  )

find_package(PCL REQUIRED)

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

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs mrs_msgs
  )

###################################
## catkin specific configuration ##
###################################

set(LIBRARIES
  MrsPclTools_PCLSupportLib
  MrsPclTools_PCLFiltration
  MrsPclTools_PCL2MapRegistration
  MrsPclTools_GroundplaneDetector
  MrsPclTools_RemoveBelowGroundFilter
  MrsPclTools_PCLPublishCloudFileToNetwork
  )

set(EXECUTABLES
  estimate_cloud_to_cloud_drift
  estimate_lidar_slam_drift
  pcd_estimate_normals
  concatenate_lidar_scans
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
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
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
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

## | ------------------------ Libraries ----------------------- |

# PCLSupportLib

add_library(MrsPclTools_PCLSupportLib
  src/support.cpp
  )

add_dependencies(MrsPclTools_PCLSupportLib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(MrsPclTools_PCLSupportLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# PCLFiltration

add_library(MrsPclTools_PCLFiltration
  src/PCLFiltration.cpp
  src/groundplane_detector.cpp
  src/remove_below_ground_filter.cpp
  )

add_dependencies(MrsPclTools_PCLFiltration
  mrs_pcl_tools_gencfg
  mrs_pcl_tools_generate_messages
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(
  MrsPclTools_PCLFiltration
  MrsPclTools_PCLSupportLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

target_include_directories(MrsPclTools_PCLFiltration SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
  )

# PCL2MapRegistration

add_library(MrsPclTools_PCL2MapRegistration
  src/PCL2MapRegistration.cpp
  )

add_dependencies(MrsPclTools_PCL2MapRegistration
  mrs_pcl_tools_gencfg
  mrs_pcl_tools_generate_messages
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(MrsPclTools_PCL2MapRegistration
  MrsPclTools_PCLSupportLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

target_include_directories(MrsPclTools_PCL2MapRegistration
  SYSTEM PRIVATE ${PCL_INCLUDE_DIRS}
  )

# PCLPublishCloudFileToNetwork

add_library(MrsPclTools_PCLPublishCloudFileToNetwork
  src/PCLPublishCloudFileToNetwork.cpp
  )

add_dependencies(MrsPclTools_PCLPublishCloudFileToNetwork
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(
  MrsPclTools_PCLPublishCloudFileToNetwork
  MrsPclTools_PCLSupportLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

target_include_directories(MrsPclTools_PCLPublishCloudFileToNetwork SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
  )

# GroundplaneDetector

add_library(MrsPclTools_GroundplaneDetector
  src/groundplane_detector.cpp
  )

add_dependencies(MrsPclTools_GroundplaneDetector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(
  MrsPclTools_GroundplaneDetector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

target_include_directories(MrsPclTools_GroundplaneDetector SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
  )

# RemoveBelowGroundFilter

add_library(MrsPclTools_RemoveBelowGroundFilter
  src/remove_below_ground_filter.cpp
  )

add_dependencies(MrsPclTools_RemoveBelowGroundFilter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(
  MrsPclTools_RemoveBelowGroundFilter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

target_include_directories(MrsPclTools_RemoveBelowGroundFilter SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
  )

## | ----------------------- Executables ---------------------- |

add_executable(estimate_cloud_to_cloud_drift
  src/executables/EstimateCloudToCloudDrift.cpp
  )

add_dependencies(estimate_cloud_to_cloud_drift
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(estimate_cloud_to_cloud_drift
  MrsPclTools_PCLSupportLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# estimate_lidar_slam_drift

add_executable(estimate_lidar_slam_drift
  src/executables/EstimateLidarSlamDrift.cpp
  )

add_dependencies(estimate_lidar_slam_drift
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(estimate_lidar_slam_drift
  MrsPclTools_PCLSupportLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# pcd_estimate_normals

add_executable(pcd_estimate_normals
  src/executables/PcdEstimateNormals.cpp
  )

add_dependencies(pcd_estimate_normals
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(pcd_estimate_normals
  MrsPclTools_PCLSupportLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# concatenate_lidar_scans

add_executable(concatenate_lidar_scans
  src/executables/ConcatenateLidarScans.cpp
  )

add_dependencies(concatenate_lidar_scans
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(concatenate_lidar_scans
  MrsPclTools_PCLSupportLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                            Build                           |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/mrs_pcl_tools/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
