cmake_minimum_required(VERSION 3.5)
project(fsm_lo)

# ------------------------------------------------------------------------------
# FSM_LO_EXACT_FP=ON replaces -Ofast with optimisation settings that forbid the
# compiler from reassociating floating point arithmetic or contracting
# multiply-adds. Two compilers reassociate differently, so a reference build and
# a build under test only produce comparable numbers when this is on.
#
# FSM_LO_DETERMINISTIC_FFT=ON makes FFTW select its plan by heuristic rather
# than by timing itself. Plan choice affects rounding, and timing varies with
# machine load, so without this the same binary can produce different numbers on
# two consecutive runs.
#
# Both default to OFF, which is the configuration this package ships with.
# ------------------------------------------------------------------------------
option(FSM_LO_EXACT_FP "Forbid floating point reassociation and contraction" OFF)
option(FSM_LO_DETERMINISTIC_FFT "Select FFTW plans by heuristic, not by timing" OFF)

add_compile_options(-std=c++11)

if(FSM_LO_EXACT_FP)
  add_compile_options(-O2 -fno-fast-math -ffp-contract=off)
else()
  add_compile_options(-Ofast)
endif()

if(FSM_LO_DETERMINISTIC_FFT)
  add_definitions(-DFSM_LO_FFTW_ESTIMATE)
endif()

find_package(CGAL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  message_generation
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_runtime
  tf2
  tf2_msgs
  tf2_ros
  INCLUDE_DIRS include
)

INCLUDE_DIRECTORIES(include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3/)

# ------------------------------------------------------------------------------
# sm
add_library(fsm_lo src/fsm_lo.cpp)
target_link_libraries(fsm_lo
  fftw3
  CGAL::CGAL_Core
  ${catkin_LIBRARIES})
add_dependencies(fsm_lo
  ${catkin_EXPORTED_TARGETS})
# ------------------------------------------------------------------------------
# The executable
add_executable(fsm_lo_node
  src/fsm_lo_node.cpp)

target_link_libraries(fsm_lo_node
  fsm_lo
  ${catkin_LIBRARIES})

add_dependencies(fsm_lo_node
  ${catkin_EXPORTED_TARGETS}
)
# ------------------------------------------------------------------------------
# Tests
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_defects test/test_defects.cpp)
  target_link_libraries(test_defects
    fftw3
    CGAL::CGAL_Core
    ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  add_rostest(test/node_defects.test)
endif()
# ------------------------------------------------------------------------------
