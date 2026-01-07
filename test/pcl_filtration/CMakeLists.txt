get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

ament_add_gtest(test_${TEST_NAME}_downsample
  test_downsample.cpp
  TIMEOUT 60
  )

target_link_libraries(test_${TEST_NAME}_downsample
  MrsPclTools_PCLFiltration
  )