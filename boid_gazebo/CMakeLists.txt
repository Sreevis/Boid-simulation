cmake_minimum_required(VERSION 2.8.3)
project(boid_gazebo)

#set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  gazebo_ros  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(boid src/boid.cpp)
target_link_libraries(boid ${catkin_LIBRARIES})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
