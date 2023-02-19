cmake_minimum_required(VERSION 2.8.3)
project(detectAndLanding)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(landing_node 
  src/landing_test.cpp
)

add_executable(pub_pose 
  src/publish_tag_pose.cpp
)

target_link_libraries(landing_node
  ${catkin_LIBRARIES}
)
target_link_libraries(pub_pose
  ${catkin_LIBRARIES}
)
# catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
