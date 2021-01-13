cmake_minimum_required(VERSION 3.5)
project(simple_arm)

# add compile options
add_compile_options(-Wall -Wextra)

# Compile as C++20
set(CMAKE_CXX_STANDARD 20)
# don't use gnu extensions
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Custom msg/srv 
rosidl_generate_interfaces(${PROJECT_NAME} "msg/EEF.msg")
ament_export_dependencies(rosidl_default_runtime)

add_library(arm_component SHARED
  src/arm_node.cpp)

# rclcpp_components_register_nodes(arm_component
#   PLUGIN "simple_arm::SimpleArm"
#   EXECUTABLE arm)

rclcpp_components_register_nodes(arm_component "simple_arm::SimpleArm")

ament_target_dependencies(arm_component rclcpp rclcpp_components sensor_msgs visualization_msgs)

rosidl_target_interfaces(arm_component ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  arm_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Install files.
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)

ament_package()
