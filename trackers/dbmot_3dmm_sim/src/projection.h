#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>

class Projection
{
public:
    Projection();
    cv::Point2d pixel2world(cv::Rect2d detection, ros::Time stamp);
    cv::Point2d world2pixel(cv::Point2d point, ros::Time stamp);

private:
    tf::TransformListener _listener;
    Eigen::Matrix3d _k_intr;

};