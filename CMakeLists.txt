cmake_minimum_required(VERSION 3.8)
project(lidar_wheel_slam_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(gtsam_points REQUIRED)
# find_package(CUDAToolkit REQUIRED) # GPUあるときON

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/lidar_wheel_slam_demo_component.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "c107::LidarWheelOdomDemo")
target_compile_options(${PROJECT_NAME}_component PUBLIC -Wall)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_component
  gtsam
  gtsam_points::gtsam_points
  ${PCL_LIBRARIES}
)

ament_auto_add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
