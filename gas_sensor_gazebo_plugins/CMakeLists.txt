cmake_minimum_required(VERSION 2.8.3)
project(gas_sensor_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs geometry_msgs nav_msgs tf dynamic_reconfigure message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetBias.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/SensorModel.cfg
  cfg/GNSS.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS gazebo
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf message_runtime
    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########


#======================================== gassource PLUGIN ==========================================//

  add_library(rotors_gazebo_gassource_plugin SHARED src/gazebo_gassource_plugin.cpp)
  target_link_libraries(rotors_gazebo_gassource_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_gassource_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install rotors_gazebo_gassource_plugin)

#======================================== gassensor PLUGIN ==========================================//

  add_library(rotors_gazebo_gassensor_plugin SHARED src/gazebo_gassensor_plugin.cpp)
  target_link_libraries(rotors_gazebo_gassensor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_gassensor_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install rotors_gazebo_gassensor_plugin)

