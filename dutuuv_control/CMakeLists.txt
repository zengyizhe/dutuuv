cmake_minimum_required(VERSION 2.8.3)
project(dutuuv_control)
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dutuuv_msgs
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
  actionlib
  actionlib_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


find_package(Ceres REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories($ {EIGEN3_INCLUDE_DIRS})

# add_action_files(
#   DIRECTORY actions
#   FILES 
#   ObjectTracking.action
#   DepthControl.action
# )

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs dutuuv_msgs
)


# generate_dynamic_reconfigure_options(
#  cfg/ObjectTracking.cfg
#  cfg/DepthController.cfg
#  cfg/DepthVelocityController.cfg
# )

catkin_package(
  INCLUDE_DIRS
     include
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
  CATKIN_DEPENDS actionlib_msgs
#  CATKIN_DEPENDS
#     dynamic_reconfigure
)

include_directories(include ${catkin_INCLUDE_DIRS}
 ${CERES_INCLUDE_DIRS}
 # ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
 ${EIGEN3_INCLUDE_DIR}
)


#add_executable(serial_to_mcu src/serial_to_mcu.cpp)
#target_link_libraries(serial_to_mcu
#   ${catkin_LIBRARIES}
#)
#add_dependencies(serial_to_mcu dutuuv_msgs_gencpp)

# add_executable(pwm_controller src/pwm_controller.cpp)
# target_link_libraries(pwm_controller
#    ${catkin_LIBRARIES}
#    yaml-cpp
# )
# add_dependencies(pwm_controller dutuuv_msgs_gencpp ${catkin_EXPORTED_TARGETS})

# add_executable(thruster_controller src/thruster_controller.cpp)
# target_link_libraries(thruster_controller ${catkin_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)
# add_dependencies(thruster_controller ${catkin_EXPORTED_TARGETS})
# add_dependencies(thruster_controller dutuuv_control_gencfg)
# add_dependencies(thruster_controller dutuuv_msgs_gencpp)


# add_executable(command_combinator src/command_combinator.cpp)
# target_link_libraries(command_combinator ${catkin_LIBRARIES})
# add_dependencies(command_combinator ${catkin_EXPORTED_TARGETS})
