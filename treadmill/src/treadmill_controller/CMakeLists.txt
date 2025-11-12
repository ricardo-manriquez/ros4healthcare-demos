cmake_minimum_required(VERSION 3.8)
project(treadmill_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(treadmill src/treadmill.cpp)
add_executable(treadmill_ctrl src/treadmill_ctrl.cpp)
ament_target_dependencies(treadmill rclcpp std_msgs geometry_msgs tf2 tf2_ros std_srvs sensor_msgs nav_msgs)
ament_target_dependencies(treadmill_ctrl rclcpp std_msgs geometry_msgs tf2 tf2_ros std_srvs sensor_msgs nav_msgs)
target_link_libraries(treadmill hidapi-libusb m)
target_link_libraries(treadmill_ctrl hidapi-libusb m)

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

install(TARGETS
	treadmill
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	treadmill_ctrl
	DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf stl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
