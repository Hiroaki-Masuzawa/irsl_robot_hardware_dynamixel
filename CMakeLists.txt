cmake_minimum_required(VERSION 3.0.0)

project(irsl_interface_dynamixel)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(irsl_shm_controller REQUIRED)
find_package(irsl_realtime_utils  REQUIRED)
find_package(irsl_common_utils  REQUIRED)
find_package(yaml-cpp)
find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   sensor_msgs
#   geometry_msgs
#   trajectory_msgs
#   dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
#   cmake_modules
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
#   ${YAML_CPP_INCLUDE_DIRS}
)

### debug print variables
##get_cmake_property(_variableNames CACHE_VARIABLES)
#get_cmake_property(_variableNames VARIABLES)
#message("VARIABLES>>>>")
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#  message("${_variableName}=${${_variableName}}")
#endforeach()
#message("VARIABLES<<<<")

add_executable(robot_control   src/robot_control.cpp)
add_executable(robot_hardware  src/robot_hardware.cpp)

# include_directories(
target_link_libraries(robot_control  ${YAML_CPP_LIBRARIES} irsl_common_utils irsl_shm_controller)
target_link_libraries(robot_hardware ${YAML_CPP_LIBRARIES} irsl_common_utils irsl_shm_controller ${catkin_LIBRARIES})
###target_link_libraries(test_host   irsl_shm_controller irsl_reamtime_utils)
###target_link_libraries(test_client irsl_shm_controller irsl_reamtime_utils)