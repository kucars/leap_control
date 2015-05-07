cmake_minimum_required(VERSION 2.8.3)
project(leap_control)


find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  geometry_msgs
  sensor_msgs
  std_msgs
  cmake_modules
  rospy
  message_generation
  visualization_msgs
)

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "/home/kuri/Downloads/LeapDeveloperKit_2.2.4+26750_linux/LeapSDK/include/"
  if( EXISTS $ENV{LEAP_SDK} $ENV{LEAP_SDK}/include)
)

add_message_files(
    FILES
    leapros.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs roscpp message_runtime rospy pcl_conversions pcl_ros
  DEPENDS system_lib
)


message ("Include dirs: ${include_directories} " )
add_executable(leap_control src/leap_control.cpp)
target_link_libraries(leap_control ${catkin_LIBRARIES})

if(EXISTS $ENV{LEAP_SDK})
  add_executable(leap_interface src/leap_interface.cpp)
  target_link_libraries(leap_interface ${PCL_LIBRARY_DIRS} ${catkin_LIBRARIES} $ENV{LEAP_SDK}/lib/x64/libLeap.so  )
  #target_include_directories(leap_interface PUBLIC $ENV{LEAP_SDK}/include)
endif()

