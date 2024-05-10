cmake_minimum_required(VERSION 3.0.2)
project(dbmot_3dmm_sim)

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
#  LIBRARIES dbmot_3dmm_sim
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dbmot_3dmm_sim
    src/main.cpp
    src/track.cpp
    src/iusltracker.cpp
    src/kalmanfilter3d.cpp
    src/projection.cpp
    src/Hungarian.cpp

    src/sot/tracker.cpp
    src/sot/trackerCSRT.cpp
    src/sot/trackerCSRTScaleEstimation.cpp
    src/sot/trackerCSRTSegmentation.cpp
    src/sot/trackerCSRTUtils.cpp
    src/sot/featureColorName.cpp
)

target_link_libraries(dbmot_3dmm_sim
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${cv_bridge_LIBRARIES}
)
