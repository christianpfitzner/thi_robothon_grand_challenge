cmake_minimum_required(VERSION 3.8)
project(thi_robothon_grand_challenge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install( DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(example_opencv_colormask_node src/example_opencv_colormask.cpp)
ament_target_dependencies(example_opencv_colormask_node ${dependencies})
install(TARGETS example_opencv_colormask_node DESTINATION lib/${PROJECT_NAME})

add_executable(example_opencv_contours_node src/example_opencv_contours.cpp)
ament_target_dependencies(example_opencv_contours_node ${dependencies})
install(TARGETS example_opencv_contours_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
