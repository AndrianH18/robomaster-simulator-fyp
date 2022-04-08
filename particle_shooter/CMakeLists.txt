cmake_minimum_required(VERSION 2.8.3)
project(particle_shooter)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  gazebo_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
    particle_shooter_plugin
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    gazebo_ros
    Boost
)

# For particle_shooter_plugin plugin
add_library(particle_shooter_plugin src/particle_shooter_plugin.cpp)
add_dependencies(
  particle_shooter_plugin
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(particle_shooter_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


# Declare a C++ executable
add_executable( collision_detector
  src/collision_detector.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(collision_detector
  ${catkin_LIBRARIES}
)


install(TARGETS particle_shooter_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    gazebo_msgs
)
