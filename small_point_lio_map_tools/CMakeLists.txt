cmake_minimum_required(VERSION 3.8)
project(small_point_lio_map_tools VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(terrain_mapping_node
  src/terrain_mapping_main.cpp
  src/terrain_mapping_node.cpp
)

target_include_directories(terrain_mapping_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(terrain_mapping_node
  rclcpp
  nav_msgs
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(terrain_mapping_node_test
    test/terrain_mapping_node_test.cpp
    src/terrain_mapping_node.cpp
  )
  if(TARGET terrain_mapping_node_test)
    target_include_directories(terrain_mapping_node_test
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(terrain_mapping_node_test
      rclcpp
      nav_msgs
      sensor_msgs
    )
  endif()
endif()

install(TARGETS
  terrain_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
