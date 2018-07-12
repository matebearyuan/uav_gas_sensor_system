cmake_minimum_required(VERSION 2.8.3)
project(trajectory_update_node)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/trajectory_sampling.cpp
  src/ros_conversions.cpp
  src/ros_visualization.cpp
)
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs mav_msgs roscpp sensor_msgs message_filters cmake_modules)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    mav_msgs
    roscpp
    sensor_msgs
    message_filters
    cmake_modules
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(trajectory_replanning_timeupdate  src/trajectory_replanning_timeupdate.cpp)
target_link_libraries(trajectory_replanning_timeupdate ${PROJECT_NAME})
add_dependencies(trajectory_replanning_timeupdate ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_replanning_master_for_three_drones  src/trajectory_replanning_master_for_three_drones.cpp)
target_link_libraries(trajectory_replanning_master_for_three_drones ${PROJECT_NAME})
add_dependencies(trajectory_replanning_master_for_three_drones ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_replanning_master_for_three_drones_vector  src/trajectory_replanning_master_for_three_drones_vector.cpp)
target_link_libraries(trajectory_replanning_master_for_three_drones_vector ${PROJECT_NAME})
add_dependencies(trajectory_replanning_master_for_three_drones_vector ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_replanning_master_for_five_drones_vector  src/trajectory_replanning_master_for_five_drones_vector.cpp)
target_link_libraries(trajectory_replanning_master_for_five_drones_vector ${PROJECT_NAME})
add_dependencies(trajectory_replanning_master_for_five_drones_vector ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_replanning_timeupdate_for_two_sides  src/trajectory_replanning_timeupdate_for_two_sides.cpp)
target_link_libraries(trajectory_replanning_timeupdate_for_two_sides ${PROJECT_NAME})
add_dependencies(trajectory_replanning_timeupdate_for_two_sides ${catkin_EXPORTED_TARGETS})
############
# BINARIES #
############
cs_add_executable(trajectory_sampler_node
  src/trajectory_sampler_node.cpp
)
target_link_libraries(trajectory_sampler_node ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()
