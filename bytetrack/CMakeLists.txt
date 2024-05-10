cmake_minimum_required(VERSION 3.0.2)
project(bytetrack)

set(Eigen3_DIR "/home/fbh/eigen-3.3.7/build")
set(cv_bridge_DIR "/home/fbh/Cv_Bridge/install/share/cv_bridge/cmake")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    Eigen3
    OpenCV
    cv_bridge
    image_transport
    message_filters
    std_msgs
    sensor_msgs
    geometry_msgs
    darknet_ros_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bytetrack
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(bytetrack
    src/main.cpp
    src/STrack.cpp
    src/BYTETracker.cpp
    src/utils.cpp
    src/BytekalmanFilter.cpp
    src/lapjv.cpp
)

target_link_libraries (bytetrack
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${cv_bridge_LIBRARIES}
)