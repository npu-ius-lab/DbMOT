#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include "dataType.h"

class Projection
{
public:
    Projection();
    PROJ_RESULT pixel2world(cv::Rect2d detection, ros::Time stamp);
    cv::Point2d world2pixel(cv::Point2d point, ros::Time stamp);
     Eigen::Matrix3d _getMatrixA(ros::Time stamp);

private:
    tf::TransformListener _listener;
    Eigen::Matrix<double, 3, 4> _k_intr;

};