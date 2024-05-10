#pragma once

#include <opencv2/core.hpp>
// #include <darknet_ros_msgs/BoundingBox.h>
#include "dataType.h"

// using namespace darknet_ros_msgs;


class Detection
{
public:
    float score;
    cv::Rect2d bbx_tlwh;
    cv::Point2d state3d;
    // PROJ_RESULT distribution;
    int trackid = 0;
// public:
//     DETECTBOX to_xywh();
//     cv::Rect2d to_ltwh();
};