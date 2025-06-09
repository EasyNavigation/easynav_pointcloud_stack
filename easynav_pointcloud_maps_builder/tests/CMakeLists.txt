ament_add_gtest(pointcloud_maps_builder_tests pointcloud_maps_builder_tests.cpp)
target_include_directories(pointcloud_maps_builder_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pointcloud_maps_builder_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(pointcloud_maps_builder_tests ${dependencies})


