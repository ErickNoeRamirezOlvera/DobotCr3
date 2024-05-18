cmake_minimum_required(VERSION 3.0.2)
project(dh_gripper_driver)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dh_gripper_msgs
)

catkin_package(
  INCLUDE_DIRS include include/dh_gripper_driver/include
  #LIBRARIES dh_gripper_driver
  #CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include()
  include/dh_gripper_driver/include
  include/dh_gripper_msgs/include
)
add_executable(dh_gripper_driver   
include/dh_gripper_driver/src/dh_device.cpp
include/dh_gripper_driver/src/dh_ag95_can.cpp 
include/dh_gripper_driver/src/dh_dh3_can.cpp 
include/dh_gripper_driver/src/dh_lagacy_gripper.cpp 
include/dh_gripper_driver/src/dh_modbus_gripper.cpp 
include/dh_gripper_driver/src/dh_rgi.cpp 
src/dh_gripper_driver.cpp)


add_dependencies(dh_gripper_driver ${catkin_EXPORTED_TARGETS})

target_link_libraries(dh_gripper_driver 
${catkin_LIBRARIES})

