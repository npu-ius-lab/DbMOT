
#pragma once


#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#define MINIMIZE_COST false //KM算法最小化代价
#define MAXIMIZE_UTIL true  //KM算法最大化收益

const float kRatio=0.5;
enum DETECTBOX_IDX {IDX_X = 0, IDX_Y, IDX_W, IDX_H };

typedef cv::Rect2d State2D;
typedef cv::Point2d State3D;
typedef cv::Mat Feature;

// Kalmanfilter
typedef Eigen::Matrix<float, 1, 2, Eigen::RowMajor> DETECTPOINT;
typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> KAL_MEAN;
typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> KAL_COVA;
typedef Eigen::Matrix<float, 1, 2, Eigen::RowMajor> KAL_HMEAN;
typedef Eigen::Matrix<float, 2, 2, Eigen::RowMajor> KAL_HCOVA;
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;

struct TrackingResult
{
    int id;
    int detidx;
    cv::Rect2d tlwh;
};

struct PROJ_RESULT {
    Eigen::Matrix2d cov_matrix;
    State3D mean;
};