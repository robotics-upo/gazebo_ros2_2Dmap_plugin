cmake_minimum_required(VERSION 3.8)
project(gazebo_ros2_2dmap_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find Gazebo
find_package(gazebo REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Build the plugin
add_library(gazebo_2Dmap_plugin SHARED 
  src/gazebo_2Dmap_plugin.cpp
)

ament_target_dependencies(gazebo_2Dmap_plugin
  rclcpp
  gazebo_dev
  gazebo_ros
  nav_msgs
  std_srvs
  geometry_msgs
)

target_link_libraries(gazebo_2Dmap_plugin
  ${GAZEBO_LIBRARIES}
)

# Install
install(TARGETS gazebo_2Dmap_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# install(DIRECTORY worlds/
#   DESTINATION share/${PROJECT_NAME}/worlds
# )

install(PROGRAMS scripts/generate_map.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
