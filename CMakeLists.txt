cmake_minimum_required(VERSION 2.8.3)
project(ahrs)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARIES})

catkin_package()

# executable node
add_executable(mahony
src/mahony_node.cpp
src/MahonyAHRS.cpp)
target_link_libraries(mahony ${catkin_LIBRARIES})

# executable node
add_executable(madgwick
src/madgwick_node.cpp
src/MadgwickAHRS.cpp)
target_link_libraries(madgwick ${catkin_LIBRARIES})

add_executable(test
src/test_node.cpp
src/MahonyAHRS.cpp
src/MadgwickAHRS.cpp)
target_link_libraries(test ${catkin_LIBRARIES})