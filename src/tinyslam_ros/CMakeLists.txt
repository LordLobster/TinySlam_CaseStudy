cmake_minimum_required(VERSION 3.8)
project(tinyslam_ros C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# tinySLAM map geometry. The original 2048 px @ 0.1 px/mm (10 mm/px) covers only
# 20.48 m; the ICTE4001 office is ~38.5 x 25.8 m, so use 25 mm/px -> 51.2 m square.
set(TINYSLAM_MAP_SIZE 2048 CACHE STRING "tinySLAM map width/height in pixels")
set(TINYSLAM_MAP_SCALE 0.04 CACHE STRING "tinySLAM map scale in pixels per mm")

add_library(coreslam STATIC
  core/CoreSLAM.c
  core/CoreSLAM_random.c
  core/CoreSLAM_ext.c
)
target_include_directories(coreslam PUBLIC core)
target_compile_definitions(coreslam PUBLIC
  TS_MAP_SIZE=${TINYSLAM_MAP_SIZE}
  TS_MAP_SCALE=${TINYSLAM_MAP_SCALE}
)
target_link_libraries(coreslam PUBLIC m)
set_target_properties(coreslam PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(tinyslam_node src/tinyslam_node.cpp)
target_compile_features(tinyslam_node PRIVATE cxx_std_17)
target_compile_options(tinyslam_node PRIVATE -Wall -Wextra)
target_link_libraries(tinyslam_node coreslam)
ament_target_dependencies(tinyslam_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS tinyslam_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
