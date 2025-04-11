cmake_minimum_required(VERSION 3.8)
project(graph_based_planning_pkg)
cmake_policy(SET CMP0074 NEW)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  )

  if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(clustering_node src/clustering_node.cpp src/marker_utils.cpp)
ament_target_dependencies(clustering_node rclcpp sensor_msgs visualization_msgs pcl_conversions nav_msgs geometry_msgs)
target_link_libraries(clustering_node ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(path_planning_node src/path_planning_node.cpp src/marker_utils.cpp)
ament_target_dependencies(path_planning_node rclcpp sensor_msgs visualization_msgs nav_msgs)
target_link_libraries(path_planning_node ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(row_detection_node src/row_detection_node.cpp src/marker_utils.cpp)
# ament_target_dependencies(row_detection_node rclcpp sensor_msgs visualization_msgs nav_msgs)
# target_link_libraries(row_detection_node ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(path_spline_node src/path_spline_node.cpp src/marker_utils.cpp)
ament_target_dependencies(path_spline_node rclcpp sensor_msgs visualization_msgs nav_msgs)
target_link_libraries(path_spline_node ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(row_detection_node src/row_detection_node.cpp)
ament_target_dependencies(
  row_detection_node 
  rclcpp std_msgs
  sensor_msgs
  pcl_conversions
  nav_msgs
  visualization_msgs
)
target_link_libraries(row_detection_node
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

install(TARGETS clustering_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS path_planning_node DESTINATION lib/${PROJECT_NAME})
# install(TARGETS row_detection_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS path_spline_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS row_detection_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
