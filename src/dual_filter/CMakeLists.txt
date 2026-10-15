cmake_minimum_required(VERSION 3.8)
project(lidar_filter_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters common)
find_package(pcl_conversions REQUIRED)

# Add Executable
add_executable(lidar_filter_node src/lidar_filter_node.cpp)

# Include Directories
target_include_directories(lidar_filter_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

# Link Libraries
target_link_libraries(lidar_filter_node
  # PRIVATE # may need to remove this line
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  ${PCL_LIBRARIES}
  # pcl_conversions::pcl_conversions
)
ament_target_dependencies(lidar_filter_node
  rclcpp
  # rclcpp_components
  sensor_msgs
  pcl_conversions
)


# Install rules
install(TARGETS
  lidar_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

