cmake_minimum_required(VERSION 3.5)
project(corrected_vio_state_estimator)

set(CATKIN_DEPENDENCIES
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  message_generation
  message_runtime
  mrs_lib
  mrs_msgs
  mrs_uav_managers
  nav_msgs
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  mrs_uav_testing
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

set(LIBRARIES
  MrsUavStateEstimators_CorrectedVio
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS Eigen
  )

include_directories(
  include
  ${CMAKE_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

set(ROSLINT_CPP_OPTS "--filter=-whitespace/comments,-whitespace/braces,-legal/copyright,-readability/multiline_comment,-whitespace/blank_line,-whitespace/line_length,-readability/todo,-build/include_what_you_use,-runtime/references,-build/include")
roslint_cpp()


add_library(MrsUavStateEstimators_CorrectedVio
  src/corrected_vio.cpp
  )

target_link_libraries(MrsUavStateEstimators_CorrectedVio
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES estimator_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

