cmake_minimum_required(VERSION 3.8)
project(small_point_lio_pgo VERSION 0.1.0 LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(small_gicp CONFIG REQUIRED)
find_package(small_point_lio_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_library(G2O_CORE_LIBRARY g2o_core REQUIRED)
find_library(G2O_STUFF_LIBRARY g2o_stuff REQUIRED)
find_library(G2O_TYPES_SLAM3D_LIBRARY g2o_types_slam3d REQUIRED)
find_library(G2O_SOLVER_EIGEN_LIBRARY g2o_solver_eigen REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyFrame.msg"
  "msg/OptimizedKeyFrames.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

add_library(lidar_iris_vendor STATIC
  third_party/lidar_iris/LidarIris.cpp
  third_party/lidar_iris/fftm/fftm.cpp
)

target_include_directories(lidar_iris_vendor
  PUBLIC
    third_party/lidar_iris
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lidar_iris_vendor
  PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

add_library(small_point_lio_pgo_gicp STATIC
  src/gicp.cpp
)

target_include_directories(small_point_lio_pgo_gicp
  PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(small_point_lio_pgo_gicp
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    small_gicp::small_gicp
)

add_library(small_point_lio_pgo_pose_graph STATIC
  src/pose_graph.cpp
)

add_library(small_point_lio_pgo_cart_context STATIC
  src/cart_context.cpp
)

target_include_directories(small_point_lio_pgo_pose_graph
  PUBLIC
    include
)

target_link_libraries(small_point_lio_pgo_pose_graph
  PUBLIC
    Eigen3::Eigen
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_TYPES_SLAM3D_LIBRARY}
    ${G2O_SOLVER_EIGEN_LIBRARY}
)

target_include_directories(small_point_lio_pgo_cart_context
  PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(small_point_lio_pgo_cart_context
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

add_executable(loop_detector_node
  src/loop_detector_main.cpp
  src/loop_detector_node.cpp
)

add_executable(map_node
  src/map_node_main.cpp
  src/map_node.cpp
)

add_executable(keyframe_bridge_node
  src/small_point_lio_keyframe_bridge_main.cpp
  src/small_point_lio_keyframe_bridge.cpp
)

add_library(small_point_lio_pgo_local_feedback STATIC
  src/local_map_feedback_node.cpp
)

add_executable(local_map_feedback_node
  src/local_map_feedback_main.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

target_include_directories(loop_detector_node
  PRIVATE
    include
    third_party/lidar_iris
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(loop_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  pcl_conversions
)

target_include_directories(map_node
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
)

target_include_directories(keyframe_bridge_node
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
)

target_include_directories(small_point_lio_pgo_local_feedback
  PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(map_node
  rclcpp
  sensor_msgs
  std_srvs
  geometry_msgs
  pcl_conversions
)

ament_target_dependencies(keyframe_bridge_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
)

ament_target_dependencies(small_point_lio_pgo_local_feedback
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  small_point_lio_interfaces
)

target_link_libraries(loop_detector_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  small_point_lio_pgo_gicp
  small_point_lio_pgo_pose_graph
  small_point_lio_pgo_cart_context
  lidar_iris_vendor
  "${cpp_typesupport_target}"
)

target_link_libraries(map_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  "${cpp_typesupport_target}"
)

target_link_libraries(keyframe_bridge_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  "${cpp_typesupport_target}"
)

target_link_libraries(small_point_lio_pgo_local_feedback
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  small_point_lio_pgo_pose_graph
  "${cpp_typesupport_target}"
)

target_link_libraries(local_map_feedback_node
  small_point_lio_pgo_local_feedback
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(local_map_feedback_node_test
    test/local_map_feedback_node_test.cpp
  )
  if(TARGET local_map_feedback_node_test)
    target_include_directories(local_map_feedback_node_test
      PRIVATE
        include
        ${PCL_INCLUDE_DIRS}
    )
    ament_target_dependencies(local_map_feedback_node_test
      rclcpp
      sensor_msgs
      geometry_msgs
      pcl_conversions
      small_point_lio_interfaces
    )
    target_link_libraries(local_map_feedback_node_test
      small_point_lio_pgo_local_feedback
      "${cpp_typesupport_target}"
    )
  endif()

  ament_add_gtest(transform_utils_test
    test/transform_utils_test.cpp
  )
  if(TARGET transform_utils_test)
    target_include_directories(transform_utils_test
      PRIVATE
        include
    )
    target_link_libraries(transform_utils_test
      Eigen3::Eigen
    )
  endif()
endif()

install(TARGETS
  loop_detector_node
  map_node
  keyframe_bridge_node
  local_map_feedback_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz_cfg/
  DESTINATION share/${PROJECT_NAME}/rviz_cfg
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)

install(FILES third_party/lidar_iris/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses/lidar_iris
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
