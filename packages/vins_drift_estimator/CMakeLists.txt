cmake_minimum_required(VERSION 3.5)
project(vins_drift_estimator)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  nodelet
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Generate messages
add_message_files(
  FILES
  debug_msg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Generate dynamic parameters which are loaded by the dynamic reconfigure server
generate_dynamic_reconfigure_options(
  config/dynparam.cfg
  )

add_definitions(-Wall)
add_definitions(-Wextra)

set(LIBRARIES
  VinsDriftEstimator
  )

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  )


add_library(VinsDriftEstimator
  src/vins_drift_estimator.cpp
  )

## Add configure headers for dynamic reconfigure
add_dependencies(VinsDriftEstimator ${PROJECT_NAME}_gencfg)

add_dependencies(VinsDriftEstimator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(VinsDriftEstimator
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/remap_gt_to_gate_estimates.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

