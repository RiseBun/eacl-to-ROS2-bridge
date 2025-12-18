cmake_minimum_required(VERSION 3.21)
project(vk_ros2_driver VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Define dependencies list
set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  EIGEN3
  stereo_msgs
)

# Create library
add_library(vk_ros2_driver SHARED src/vk_ros2_driver.cpp)
target_link_libraries(vk_ros2_driver
  PRIVATE /opt/vilota/lib/libvk_sdk.so)

# Set RPATH so the binary can find the library at runtime
set_target_properties(vk_ros2_driver PROPERTIES
  BUILD_RPATH "/opt/vilota/lib"
  INSTALL_RPATH "/opt/vilota/lib"
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)
  
target_include_directories(vk_ros2_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE /opt/vilota/include
  PRIVATE /opt/vilota/include/vk_sdk/capnp)
ament_target_dependencies(vk_ros2_driver PUBLIC ${dependencies})

# Register component node
rclcpp_components_register_node(vk_ros2_driver
  PLUGIN "vkc::VkRos2Driver" EXECUTABLE vk_ros2_driver_node
)

# Install targets and directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install (
  TARGETS vk_ros2_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
