cmake_minimum_required(VERSION 3.5)
project(m_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  include
)

add_executable(dynfilter src/dynfilter_with_odom.cpp src/DynObjFilter.cpp src/DynObjCluster.cpp)
ament_target_dependencies(dynfilter rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros pcl_ros)
target_link_libraries(dynfilter ${PCL_LIBRARIES} ${OpenCV_LIBS} TBB::tbb Python3::Python)
target_include_directories(dynfilter PRIVATE ${Python3_INCLUDE_DIRS})

add_executable(display_prediction src/display_prediction.cpp)
ament_target_dependencies(display_prediction rclcpp sensor_msgs pcl_ros)
target_link_libraries(display_prediction ${PCL_LIBRARIES} ${OpenCV_LIBS} TBB::tbb Python3::Python)
target_include_directories(display_prediction PRIVATE ${Python3_INCLUDE_DIRS})

add_executable(cal_recall src/cal_recall_multi.cpp)
ament_target_dependencies(cal_recall rclcpp)
target_link_libraries(cal_recall ${PCL_LIBRARIES} ${OpenCV_LIBS} TBB::tbb Python3::Python)
target_include_directories(cal_recall PRIVATE ${Python3_INCLUDE_DIRS})

install(TARGETS
  dynfilter
  display_prediction
  cal_recall
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
