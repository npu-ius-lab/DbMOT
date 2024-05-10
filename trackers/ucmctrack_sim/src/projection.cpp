#include "projection.h"


Eigen::Matrix4d to_matrix(const Eigen::Vector3d T, const Eigen::Quaterniond R_)
{
    Eigen::Matrix3d R = R_.normalized().toRotationMatrix();
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3,3>(0,0) = R;
    matrix.block<3,1>(0,3) = T;
    // std::cout << matrix << std::endl;
    return matrix;
}

Eigen::Matrix4d to_inverse(const Eigen::Matrix4d matrix)
{
    Eigen::Matrix3d R = matrix.block<3,3>(0,0);
    Eigen::Vector3d t = matrix.block<3,1>(0,3);
    Eigen::Matrix3d R_T = R.transpose();
    Eigen::Matrix4d matrix_inv = Eigen::Matrix4d::Identity();
    matrix_inv.block<3,3>(0,0) = R_T;
    matrix_inv.block<3,1>(0,3) = -R_T * t;

    return matrix_inv;
}

Eigen::Matrix2d get_uv_error(State2D box)
{
    double u_error = 0.05 * box.height;
    double v_error = 0.05 * box.height; //0.05
    if (u_error > 13) 
        u_error = 13;
    else if (u_error < 2) 
        u_error - 2;
    if (v_error > 10)
        v_error = 10;
    else if (v_error < 2)
        v_error = 2;
    Eigen::Matrix2d errorMatrix = Eigen::Matrix2d::Identity();
    errorMatrix(0,0) = u_error * u_error;
    errorMatrix(1,1) = v_error * v_error;
    return errorMatrix;
}


Projection::Projection()
{
    _k_intr << 319.9988245765257,  0.000000000000000,  320.500, 0.0,
               0.000000000000000,  319.9988245765257,  240.500, 0.0,
               0.000000000000000,  0.000000000000000,  1.00000, 0.0;
}

PROJ_RESULT Projection::pixel2world(cv::Rect2d detection, ros::Time stamp)
{
    double centre_x = detection.x + detection.width / 2;
    double centre_y = detection.y + detection.height; 
    double fx = _k_intr(0,0);
    double fy = _k_intr(1,1);
    double u0 = _k_intr(0,2);
    double v0 = _k_intr(1,2);
    double nlz_x = (centre_x - u0) / fx;
    double nlz_y = (centre_y - v0) / fy;
    cv::Point3d p_nlz(nlz_x, nlz_y, 1);
    tf::StampedTransform tf_wc;
    _listener.waitForTransform("world", "front_cam_optical_frame", stamp, ros::Duration(1.5));
    _listener.lookupTransform("world", "front_cam_optical_frame", stamp, tf_wc);
    Eigen::Vector3d t_wc;
    t_wc << tf_wc.getOrigin().getX(), tf_wc.getOrigin().getY(), tf_wc.getOrigin().getZ();
    Eigen::Matrix3d r_wc;
    tf::Matrix3x3 r_wc_ = tf_wc.getBasis();
    r_wc << r_wc_.getRow(0).getX(), r_wc_.getRow(0).getY(), r_wc_.getRow(0).getZ(),
            r_wc_.getRow(1).getX(), r_wc_.getRow(1).getY(), r_wc_.getRow(1).getZ(),
            r_wc_.getRow(2).getX(), r_wc_.getRow(2).getY(), r_wc_.getRow(2).getZ();
    Eigen::Matrix4d rt_wc = Eigen::Matrix4d::Identity();
    rt_wc.block<3,3>(0,0) = r_wc;
    rt_wc.block<3,1>(0,3) = t_wc;
    double Zc = -t_wc[2] / (r_wc(2,0)*p_nlz.x + r_wc(2,1)*p_nlz.y + r_wc(2,2));
    Eigen::Vector4d p_cam;
    p_cam[0] = p_nlz.x * Zc;
    p_cam[1] = p_nlz.y * Zc;
    p_cam[2] = Zc;
    p_cam[3] = 1;
    Eigen::Vector4d p_world_;
    p_world_ = rt_wc * p_cam;
    cv::Point2d p_world;
    p_world.x = p_world_[0];
    p_world.y = p_world_[1];

    //* 计算投影噪声矩阵
    Eigen::Matrix3d A = _getMatrixA(stamp);
    Eigen::Matrix3d A_inv = A.inverse();
    Eigen::Matrix2d projErrorM;
    projErrorM(0,0) = Zc * A_inv(0,0) - A_inv(2,0) * Zc * p_world.x;
    projErrorM(0,1) = Zc * A_inv(0,1) - A_inv(2,1) * Zc * p_world.x;
    projErrorM(1,0) = Zc * A_inv(1,0) - A_inv(2,0) * Zc * p_world.y;
    projErrorM(1,1) = Zc * A_inv(1,1) - A_inv(2,1) * Zc * p_world.y;
    //* 过程噪声协方差矩阵
    Eigen::Matrix2d uvErrorM = get_uv_error(detection);
    Eigen::Matrix2d NoiseCovM = projErrorM * uvErrorM * projErrorM.transpose();

    PROJ_RESULT proj_res;
    proj_res.mean = p_world;
    proj_res.cov_matrix = NoiseCovM;
    return proj_res;
}

Eigen::Matrix3d Projection::_getMatrixA(ros::Time stamp)
{
    tf::StampedTransform tf_cw;
    _listener.waitForTransform("world", "front_cam_optical_frame", stamp, ros::Duration(1.5));
    _listener.lookupTransform("world", "front_cam_optical_frame", stamp, tf_cw);
    Eigen::Vector3d t_cw;
    t_cw << tf_cw.getOrigin().getX(), tf_cw.getOrigin().getY(), tf_cw.getOrigin().getZ();
    Eigen::Matrix3d r_cw;
    tf::Matrix3x3 r_cw_ = tf_cw.getBasis();
    r_cw << r_cw_.getRow(0).getX(), r_cw_.getRow(0).getY(), r_cw_.getRow(0).getZ(),
            r_cw_.getRow(1).getX(), r_cw_.getRow(1).getY(), r_cw_.getRow(1).getZ(),
            r_cw_.getRow(2).getX(), r_cw_.getRow(2).getY(), r_cw_.getRow(2).getZ();
    Eigen::Matrix4d rt_cw = Eigen::Matrix4d::Identity();
    rt_cw.block<3,3>(0,0) = r_cw;
    rt_cw.block<3,1>(0,3) = t_cw;

    Eigen::Matrix<double,3,4> temp = _k_intr * rt_cw;
    Eigen::Matrix3d A;
    A.block<3, 3>(0, 0) = temp.block<3, 3>(0, 0);
    A.block<3, 1>(0, 2) = temp.block<3, 1>(0, 3);
    return A;
}

cv::Point2d Projection::world2pixel(cv::Point2d p_world_, ros::Time stamp)
{
    tf::StampedTransform tf_cw;
    _listener.waitForTransform("front_cam_optical_frame", "world", stamp, ros::Duration(1.5));
    _listener.lookupTransform("front_cam_optical_frame", "world", stamp, tf_cw);
    Eigen::Vector3d t_cw;
    t_cw << tf_cw.getOrigin().getX(), tf_cw.getOrigin().getY(), tf_cw.getOrigin().getZ();
    Eigen::Matrix3d r_cw;
    tf::Matrix3x3 r_cw_ = tf_cw.getBasis();
    r_cw << r_cw_.getRow(0).getX(), r_cw_.getRow(0).getY(), r_cw_.getRow(0).getZ(),
            r_cw_.getRow(1).getX(), r_cw_.getRow(1).getY(), r_cw_.getRow(1).getZ(),
            r_cw_.getRow(2).getX(), r_cw_.getRow(2).getY(), r_cw_.getRow(2).getZ();
    Eigen::Matrix4d rt_cw = Eigen::Matrix4d::Identity();
    rt_cw.block<3,3>(0,0) = r_cw;
    rt_cw.block<3,1>(0,3) = t_cw;
    Eigen::Vector4d p_world;
    p_world << p_world_.x, p_world_.y, 0, 1;
    Eigen::Vector4d p_cam;
    p_cam = rt_cw * p_world;
    double fx = _k_intr(0,0);
    double fy = _k_intr(1,1);
    double u0 = _k_intr(0,2);
    double v0 = _k_intr(1,2);
    cv::Point2d p_pix;
    p_pix.x = fx * p_cam[0] / p_cam[2] + u0;
    p_pix.y = fy * p_cam[1] / p_cam[2] + v0;
    return p_pix;
}