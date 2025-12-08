get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

ament_add_gtest(test_${TEST_NAME}
  test.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}
  MrsPclTools_PCLSupportLib
)

target_compile_definitions(test_${TEST_NAME}
  PRIVATE
  TEST_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets"
)