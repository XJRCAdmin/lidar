cmake_minimum_required(VERSION 3.5)
project(lidar_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensing_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
  vision_msgs
  sensing_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleDetection.msg"
  "msg/ObstacleDetectionArray.msg"
  # "msg/BallRelative.msg"
  # "msg/RobotState.msg"
  # "msg/BallState.msg"
  DEPENDENCIES vision_msgs std_msgs geometry_msgs
)

add_executable(obstacle_detector_node
  src/obstacle_detector.cpp
  src/utils.cpp
)
target_include_directories(obstacle_detector_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(obstacle_detector_node ${dependencies})
target_link_libraries(obstacle_detector_node ${PCL_LIBRARIES})
rosidl_target_interfaces(obstacle_detector_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

##
add_executable(odom_trans_node
  src/odom_to_baselink_transformer.cpp
  src/utils.cpp
)
target_include_directories(odom_trans_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(odom_trans_node ${dependencies})
target_link_libraries(odom_trans_node ${PCL_LIBRARIES})
rosidl_target_interfaces(odom_trans_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

## 
add_executable(obstacle_to_baselink_node
  src/obstacle_to_baselink_transformer.cpp
  src/utils.cpp
)
target_include_directories(obstacle_to_baselink_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(obstacle_to_baselink_node ${dependencies})
target_link_libraries(obstacle_to_baselink_node ${PCL_LIBRARIES})
rosidl_target_interfaces(obstacle_to_baselink_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

## 
add_executable(interface_node
  src/interface.cpp
)
target_include_directories(interface_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(interface_node ${dependencies})
target_link_libraries(interface_node ${PCL_LIBRARIES})
rosidl_target_interfaces(interface_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

## 
install(TARGETS
  obstacle_detector_node
  odom_trans_node
  interface_node
  obstacle_to_baselink_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch rviz scripts config DESTINATION share/${PROJECT_NAME})
# install(PROGRAMS
#   scripts/param_tuner.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()