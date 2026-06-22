cmake_minimum_required(VERSION 3.8)
project(ur5e_visual_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(aruco_detector src/aruco_detector.cpp)
ament_target_dependencies(aruco_detector
  rclcpp sensor_msgs geometry_msgs cv_bridge tf2 tf2_ros)
target_include_directories(aruco_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aruco_detector ${OpenCV_LIBS})

add_executable(pbvs_controller src/pbvs_controller.cpp)
ament_target_dependencies(pbvs_controller rclcpp geometry_msgs tf2)

install(TARGETS aruco_detector  pbvs_controller DESTINATION lib/${PROJECT_NAME}) # need to add pbvs controller oncei mplemented
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

ament_package()
