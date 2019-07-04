cmake_minimum_required(VERSION 2.8.3)
project(arti_gazebo_laser_livox)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  tf
  gazebo_ros
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES arti_gazebo_laser_livox
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs tf gazebo_ros
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(arti_gazebo_laser_livox src/arti_gazebo_laser_livox.cpp src/ray_data.cpp)
target_link_libraries(arti_gazebo_laser_livox ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} RayPlugin)

install(TARGETS
  arti_gazebo_laser_livox
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

