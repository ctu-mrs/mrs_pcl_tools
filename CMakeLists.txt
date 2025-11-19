cmake_minimum_required(VERSION 3.5)
project(mrs_pcl_tools)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-O3)
add_definitions(-DCOMPILE_WITH_OUSTER)
# add_definitions(-g)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # disables anoying googletest warning
# set(CATKIN_ENABLE_TESTING FALSE)

# # disables anoying PCL warnings
# set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  geometry_msgs
  # message_generation
  # message_runtime
  mrs_lib
  mrs_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  mrs_modules_msgs
  # nodelet
  ouster_ros
  pcl_conversions
  pcl_ros
  rclcppconsole
  )

# find_package(PCL REQUIRED COMPONENTS
#   common
#   )

# Include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()


# add_service_files(DIRECTORY srv/registration FILES
#   SrvRegisterPointCloudByName.srv
#   SrvRegisterPointCloudOffline.srv
#   )


###################################
## catkin specific configuration ##
###################################

# set(LIBRARIES
#   MrsPclTools_PCLSupportLib
#   MrsPclTools_PCLFiltration
#   MrsPclTools_PCL2MapRegistration
#   MrsPclTools_GroundplaneDetector
#   MrsPclTools_RemoveBelowGroundFilter
#   MrsPclTools_PCLPublishCloudFileToNetwork
#   )
set(LIBRARIES
    MrsPclTools_PCLFiltration
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

## | ------------------------ Libraries ----------------------- |

# PCLSupportLib

# add_library(MrsPclTools_PCLSupportLib
#   src/support.cpp
#   )

# add_dependencies(MrsPclTools_PCLSupportLib
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_gencfg
#   )

# target_link_libraries(MrsPclTools_PCLSupportLib
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   )

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

# component registration to make it discoverable in runtime.
rclcpp_components_register_nodes(MrsPclTools_PCLFiltration PLUGIN "MrsPclTools::MrsPclTools_PCLFiltration" EXECUTABLE MrsPclTools_PCLFiltration)

# PCL2MapRegistration

# add_library(MrsPclTools_PCL2MapRegistration
#   src/PCL2MapRegistration.cpp
#   )

# add_dependencies(MrsPclTools_PCL2MapRegistration
#   mrs_pcl_tools_gencfg
#   mrs_pcl_tools_generate_messages
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_gencfg
#   )

# target_link_libraries(MrsPclTools_PCL2MapRegistration
#   MrsPclTools_PCLSupportLib
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   )

# target_include_directories(MrsPclTools_PCL2MapRegistration
#   SYSTEM PRIVATE ${PCL_INCLUDE_DIRS}
#   )

# PCLPublishCloudFileToNetwork

# add_library(MrsPclTools_PCLPublishCloudFileToNetwork
#   src/PCLPublishCloudFileToNetwork.cpp
#   )

# add_dependencies(MrsPclTools_PCLPublishCloudFileToNetwork
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_gencfg
#   )

# target_link_libraries(
#   MrsPclTools_PCLPublishCloudFileToNetwork
#   MrsPclTools_PCLSupportLib
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   )

# target_include_directories(MrsPclTools_PCLPublishCloudFileToNetwork SYSTEM PRIVATE
#   ${PCL_INCLUDE_DIRS}
#   )



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
