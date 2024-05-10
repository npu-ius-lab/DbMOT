cmake_minimum_required(VERSION 3.0.2)
project(ucmctrack_sim)

set(Eigen3_DIR "/home/fbh/eigen-3.3.7/build")
set(cv_bridge_DIR "/home/fbh/Cv_Bridge/install/share/cv_bridge/cmake")

find_package(catkin REQUIRED COMPONENTS
    tf
    OpenCV
    message_filters
    darknet_ros_msgs
    geometry_msgs
    roscpp
    rospy
    Eigen3
    sensor_msgs
    std_msgs
    cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iusltrack_hungarian_eval
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ucmctrack_sim
    src/main.cpp
    src/track.cpp
    src/tracker.cpp
    src/kalmanfilter.cpp
    src/projection.cpp
    src/utils.cpp
    src/lapjv.cpp
)

target_link_libraries(ucmctrack_sim
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${cv_bridge_LIBRARIES}
)
