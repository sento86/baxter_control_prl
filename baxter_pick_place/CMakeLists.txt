cmake_minimum_required(VERSION 2.8.3)
project(baxter_pick_place)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  #moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools
  moveit_grasps
  moveit_simple_grasps
  #baxter_core_msgs
  #std_msgs
  #geometry_msgs
  #baxter_control
  roscpp
  cmake_modules
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
  gazebo_ros
  gazebo_msgs
)

#find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
  gazebo_ros
  gazebo_msgs
)

# Enable gdb to show line numbers
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
#set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Main executable
add_executable(block_pick_place src/block_pick_place.cpp src/baxter_utilities.cpp)
target_link_libraries(block_pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(baxter_control src/baxter_control.cpp src/baxter_utilities.cpp)
target_link_libraries(baxter_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

################

## Add C++14 support
if(CMAKE_COMPILER_IS_GNUCXX)
    # message("Setting C++14 and C++11 support..")
    set(CMAKE_CXX_FLAGS "-std=c++14")
    # message(${CMAKE_CXX_FLAGS})
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # message("Setting C++14 and C++11 support..")
    set(CMAKE_CXX_FLAGS "-std=c++14 -stdlib=libc++")
    # message(${CMAKE_CXX_FLAGS})
endif()

#############
## Install ##
#############

install(PROGRAMS
   scripts/ik_pick_and_place_demo_force_torque.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
   scripts/ik_pick_and_place_demo_force_torque_haptic.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

