get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

ament_add_gtest(test_${TEST_NAME}_downsample
  test_downsample.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_downsample
  MrsPclTools_PCLFiltration
)

ament_add_gtest(test_${TEST_NAME}_clip_intensity
  test_clip_intensity.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_clip_intensity
  MrsPclTools_PCLFiltration
)

ament_add_gtest(test_${TEST_NAME}_clip_range
  test_clip_range.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_clip_range
  MrsPclTools_PCLFiltration
)

ament_add_gtest(test_${TEST_NAME}_clip_reflectivity
  test_clip_reflectivity.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}_clip_reflectivity
  MrsPclTools_PCLFiltration
)