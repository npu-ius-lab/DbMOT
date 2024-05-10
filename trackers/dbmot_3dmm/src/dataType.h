#ifndef DATATYPE_H
#define DATATYPE_H


#include <cstddef>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core/eigen.hpp>


typedef cv::Rect2d State2D;
typedef cv::Point2d State3D;
typedef cv::Mat Feature;

//projection
struct PROJ_RESULT {
    Eigen::Matrix2d cov_matrix;
    State3D mean;
};

struct DETECTION {
    double score;
    State2D boundingbox;
    PROJ_RESULT distribution;
    int trackid;
};

// Kalmanfilter
typedef Eigen::Matrix<float, 1, 2, Eigen::RowMajor> DETECTPOINT;
typedef Eigen::Matrix<float, -1, 4, Eigen::RowMajor> DETECTBOXSS;

typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> KAL_MEAN;
typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> KAL_COVA;
typedef Eigen::Matrix<float, 1, 2, Eigen::RowMajor> KAL_HMEAN;
typedef Eigen::Matrix<float, 2, 2, Eigen::RowMajor> KAL_HCOVA;
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;
using MATCH_DATA = std::pair<int, int>;
typedef struct t{
    std::vector<MATCH_DATA> matches;
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_detections;
}TRACHER_MATCHD;

//linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM;


#endif