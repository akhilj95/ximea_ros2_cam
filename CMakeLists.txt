cmake_minimum_required(VERSION 3.8)
project(ximea_ros2_cam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Position-independent code is required for dynamically loaded ROS 2 component libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Find Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# XIMEA xiAPI Pathing Definitions
set(XIMEA_INCLUDE_PATH /usr/include)
set(XIMEA_LIBRARY m3api)

# ---------------------------------------------------------------------------
# Generate Custom Messages & Interface Type Support
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XiImageInfo.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ---------------------------------------------------------------------------
# 1. Build the Shared Component Library
# ---------------------------------------------------------------------------
# We name the target target explicitly to maintain strict linking clarity
add_library(${PROJECT_NAME}_component SHARED src/ximea_ros2_cam.cpp)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${XIMEA_INCLUDE_PATH}
)

# Link hardware APIs and generated custom message type supports directly to the component
target_link_libraries(${PROJECT_NAME}_component 
  ${XIMEA_LIBRARY} 
  ${cpp_typesupport_target}
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp 
  rclcpp_components 
  sensor_msgs 
  std_msgs 
  image_transport 
  camera_info_manager
)

# Register the plugin so that standard ROS 2 Component containers can find it
rclcpp_components_register_nodes(${PROJECT_NAME}_component "ximea_ros2_cam::XimeaRosCam")

# ---------------------------------------------------------------------------
# 2. Build the Standalone Executable Node
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/ximea_ros2_cam_node.cpp)

# CRITICAL FIX: Explicitly bind the standalone executable wrapper with the component symbols
target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_component
)

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
)

# ---------------------------------------------------------------------------
# 3. Installation Rules
# ---------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_component ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Export targets so downstream dependencies can discover our library and tracking types
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  image_transport
  camera_info_manager
  rosidl_default_runtime
)

ament_package()