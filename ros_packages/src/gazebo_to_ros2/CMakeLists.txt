cmake_minimum_required(VERSION 3.5)
project(gazebo_to_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gz-transport12 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

# lidar republisher
add_executable(lidar_republisher src/lidar_republisher.cpp)
ament_target_dependencies(lidar_republisher rclcpp geometry_msgs sensor_msgs)
target_link_libraries(lidar_republisher gz-transport${GZ_TRANSPORT_VER}::core ${PCL_LIBRARIES})

# lidar tf publisher
add_executable(lidar_tf_publisher src/lidar_tf_publisher.cpp)
ament_target_dependencies(lidar_tf_publisher rclcpp geometry_msgs tf2_ros)
target_link_libraries(lidar_tf_publisher gz-transport${GZ_TRANSPORT_VER}::core ${PCL_LIBRARIES})

# Install the executable
install(TARGETS lidar_republisher lidar_tf_publisher DESTINATION lib/${PROJECT_NAME})
# Build the package
ament_package()








