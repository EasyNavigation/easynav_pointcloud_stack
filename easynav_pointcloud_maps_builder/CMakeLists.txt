cmake_minimum_required(VERSION 3.20)
project(easynav_pointcloud_maps_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(easynav_common REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  easynav_common
)

include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/easynav_pointcloud_maps_builder/PointcloudMapsBuilderNode.cpp

)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Executable
add_executable(pointcloud_maps_builder_main src/pointcloud_maps_builder_main.cpp)
ament_target_dependencies(pointcloud_maps_builder_main ${dependencies})
target_link_libraries(pointcloud_maps_builder_main ${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  pointcloud_maps_builder_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
