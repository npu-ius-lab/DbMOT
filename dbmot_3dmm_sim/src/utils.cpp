// #include <Eigen/Dense>
// #include "projection.h"

// Eigen::Matrix4d Projection::to_inverse(const Eigen::Matrix4d matrix)
// {
//     Eigen::Matrix3d R = matrix.block<3,3>(0,0);
//     Eigen::Vector3d t = matrix.block<3,1>(0,3);
//     Eigen::Matrix3d R_T = R.transpose();
//     Eigen::Matrix4d matrix_inv = Eigen::Matrix4d::Identity();
//     matrix_inv.block<3,3>(0,0) = R_T;
//     matrix_inv.block<3,1>(0,3) = -R_T * t;
//     return matrix_inv;
// }


#include <opencv2/core.hpp>
#include "dataType.h"
#include "detection.h"

// DETECTBOX Detection::to_xywh()
// {//tlwh2xywh
//     DETECTBOX bbx_xywh;//centrex,centrey,width,height
//     bbx_xywh(0,IDX_X) = bbx_tlwh.x + kRatio*bbx_tlwh.width;
//     bbx_xywh(0,IDX_Y) = bbx_tlwh.y + kRatio*bbx_tlwh.height;
//     bbx_xywh(0,IDX_W) = bbx_tlwh.width;
//     bbx_xywh(0,IDX_H) = bbx_tlwh.height;
//     return bbx_xywh;
// }

// cv::Rect2d Detection::to_ltwh()
// {
//     cv::Rect2d bbx_tlwh;
//     bbx_tlwh.x = _bbx_tlbr.xmin;
//     bbx_tlwh.y = _bbx_tlbr.ymin;
//     bbx_tlwh.width = _bbx_tlbr.xmax - _bbx_tlbr.xmin;
//     bbx_tlwh.height = _bbx_tlbr.ymax - _bbx_tlbr.ymin;
//     this->bbx_tlwh = bbx_tlwh;
//     return bbx_tlwh;
// }

// #include "track.h"
// cv::Rect2d Track::to_tlwh()
// {//xywh2tlwh
//     DETECTBOX bbx_xywh = mean.leftCols(4);
//     cv::Rect2d bbx_tlwh;
//     bbx_tlwh.x = bbx_xywh(0,IDX_X) - kRatio*bbx_xywh(0,IDX_W);
//     bbx_tlwh.y = bbx_xywh(0,IDX_Y) - kRatio*bbx_xywh(0,IDX_H);
//     bbx_tlwh.width = bbx_xywh(0,IDX_W);
//     bbx_tlwh.height = bbx_xywh(0,IDX_H);
//     return bbx_tlwh;
// }