cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)


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

add_executable(approach_service_server_real src/approach_service_server_real.cpp)
ament_target_dependencies(approach_service_server_real rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
rosidl_target_interfaces(approach_service_server_real ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(approach_service_server_sim src/approach_service_server_sim.cpp)
ament_target_dependencies(approach_service_server_sim rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
rosidl_target_interfaces(approach_service_server_sim ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  approach_service_server_real
  approach_service_server_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)


ament_package()