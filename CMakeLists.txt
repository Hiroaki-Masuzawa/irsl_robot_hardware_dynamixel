cmake_minimum_required(VERSION 3.0.0)

project(irsl_interface_dynamixel)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(irsl_shm_controller REQUIRED)
find_package(irsl_realtime_utils  REQUIRED)
find_package(irsl_common_utils  REQUIRED)
find_package(yaml-cpp)
find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_toolbox
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(robot_hardware  src/robot_hardware.cpp src/DynamixelInterface.cpp )
target_link_libraries(robot_hardware ${YAML_CPP_LIBRARIES} irsl_common_utils irsl_shm_controller ${catkin_LIBRARIES})
