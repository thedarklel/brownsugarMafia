cmake_minimum_required(VERSION 2.8.3)
project(wpi_ur5)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    cmake_modules
    geometric_shapes
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(move_group_ur5 src/move_group_ur5.cpp)
target_link_libraries(move_group_ur5 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_ur5 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
