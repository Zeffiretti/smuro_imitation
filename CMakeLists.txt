cmake_minimum_required(VERSION 3.0.2)
project(smuro_imitation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
  smuro_msgs smuro smuro_ft_control gazebo_msgs robotis_math control_toolbox roslib
  smuro_basic_traj smuro_exmpl_traj smuro_io)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
generate_messages(DEPENDENCIES
  smuro_msgs
  std_msgs
  )
#find_package(smuro_fk_ik REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)
#cs_add_library(ft_controller src/smuro_ft_control/ftcontroller.cpp)
#cs_add_library(motion_set src/motionset/motionset.cpp)
#cs_add_executable(motion_test_node src/motiontest.cpp)
#target_link_libraries(motion_test_node motion_set smuro_basic_traj reset_pose pullup pushdown gaomotion filemotion)
#cs_add_executable(traj_test_node src/trajtest.cpp)
#target_link_libraries(traj_test_node motion_set smuro_basic_traj reset_pose pullup pushdown filemotion)
#cs_add_executable(file_motion_node src/filemotiontest.cpp)
#target_link_libraries(file_motion_node motion_set smuro_basic_traj reset_pose pullup pushdown gaomotion filemotion)
#cs_add_executable(ui_control_node src/ui_control.cpp)
#target_link_libraries(ui_control_node motion_set smuro_basic_traj reset_pose pullup pushdown gaomotion filemotion)
#cs_add_executable(imitation_node src/imitation.cpp)
#target_link_libraries(imitation_node motion_set smuro_basic_traj reset_pose pullup pushdown gaomotion filemotion matreader)
# Executables: uncomment the following and edit arguments to compile new nodes
cs_add_library(synchro_controller src/synchrocontroller/synchrocontroller.cpp)
target_link_libraries(synchro_controller smuro_msgs_operators)
cs_add_executable(synchro_node src/synchronode.cpp)
target_link_libraries(synchro_node synchro_controller)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(example my_lib)

cs_install()
cs_export()