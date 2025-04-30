cmake_minimum_required(VERSION 3.22.1)
project(px4_agent_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

#################
# Setup targets #
#################

# px4_agent_nav_node
add_executable(px4_agent_nav_node src/px4_agent_nav_node.cpp)
ament_target_dependencies(px4_agent_nav_node rclcpp px4_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

# px4_agent_search_approach_node
add_executable(px4_agent_search_approach_node src/px4_agent_search_approach_node.cpp)
ament_target_dependencies(px4_agent_search_approach_node rclcpp px4_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs)


############
# Install ##
############

install(TARGETS px4_agent_nav_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS px4_agent_search_approach_node DESTINATION lib/${PROJECT_NAME})
# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime px4_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

ament_export_include_directories(include)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()