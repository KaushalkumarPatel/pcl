cmake_minimum_required(VERSION 3.8)
project(my_pcl_package)

# Default to C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add executable
add_executable(my_pcl_node src/my_pcl_node.cpp)

# Link libraries
target_link_libraries(my_pcl_node ${PCL_LIBRARIES})

# Add dependencies
ament_target_dependencies(my_pcl_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_msgs
)

# Install executables and libraries
install(TARGETS my_pcl_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()