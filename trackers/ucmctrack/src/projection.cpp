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

Projection::Projection(Eigen::Vector3d translation, Eigen::Quaterniond rotation)
{ 
    //* rmtt_01相机内参1217
    // K_in << 913.5808,  0.000000,  489.8049, 0.0,
    //         0.000000,  913.8495,  362.7804, 0.0,
    //         0.000000,  0.000000,  1.000000, 0.0;
    //* rmtt_02相机内参0324
    K_in << 919.4281,  0.000000, 468.9729, 0.0,
            0.000000,  919.0757, 365.6025, 0.0,
            0.000000,  0.000000, 1.000000, 0.0;
    //! 相机->机体
    // Eigen::Vector3d T(0.00420064, 0.0102048, -0.0238513);
    // Eigen::Quaterniond Q(0.381105, -0.520155, 0.597858, -0.476197);
    //* 0324
    Eigen::Vector3d T(0.0137623, 0.00222272, -0.0298866);
    Eigen::Quaterniond Q(-0.421837,  0.568055, -0.540446,  0.455285);
    this->K_ex_2 = to_matrix(T, Q);
    //! 机体->世界
    this->K_ex_1 = to_matrix(translation, rotation);

    //! 世界->相机
    Eigen::Matrix4d K_ex_1_inv = to_inverse(this->K_ex_1);
    Eigen::Matrix4d K_ex_2_inv = to_inverse(this->K_ex_2);
    this->K_ex = K_ex_2_inv * K_ex_1_inv;
    
    //todo: cal matrix A here:
    Eigen::Matrix<double,3,4> K = K_in * K_ex;
  
    this->A.block<3, 3>(0, 0) = K.block<3, 3>(0, 0);
    this->A.block<3, 1>(0, 2) = K.block<3, 1>(0, 3);

}

PROJ_RESULT
Projection::pixel2world(const State2D state2d)
{
    // 计算框底边中心
    double centreX = state2d.x + state2d.width / 2;
    double centreY = state2d.y + state2d.height;
    Eigen::Vector2d P_pix(centreX, centreY);
    double fx = K_in(0,0);
    double fy = K_in(1,1);
    double u0 = K_in(0,2);
    double v0 = K_in(1,2);
    double Xa = (P_pix[0] - u0) / fx;
    double Ya = (P_pix[1] - v0) / fy;
    Eigen::Vector3d Pa(Xa, Ya, 1);

    Eigen::Matrix4d K_ex_inv = to_inverse(this->K_ex);
    double Zc = -K_ex_inv(2,3) / (K_ex_inv(2,0) * Xa + K_ex_inv(2,1) * Ya + K_ex_inv(2,2));
    Eigen::Vector4d Pc(Zc * Xa, Zc * Ya, Zc, 1);

    // std::cout << Zc << std::endl;
    
    Eigen::Vector4d Pw = K_ex_inv * Pc;
    cv::Point2d res(Pw[0], Pw[1]);
    
    //* 计算投影噪声矩阵
    Eigen::Matrix3d A_inv = this->A.inverse();
    Eigen::Matrix2d projErrorM;
    projErrorM(0,0) = Zc * A_inv(0,0) - A_inv(2,0) * Zc * Pw[0];
    projErrorM(0,1) = Zc * A_inv(0,1) - A_inv(2,1) * Zc * Pw[0];
    projErrorM(1,0) = Zc * A_inv(1,0) - A_inv(2,0) * Zc * Pw[1];
    projErrorM(1,1) = Zc * A_inv(1,1) - A_inv(2,1) * Zc * Pw[1];
    //* 过程噪声协方差矩阵
    Eigen::Matrix2d uvErrorM = get_uv_error(state2d);
    Eigen::Matrix2d NoiseCovM = projErrorM * uvErrorM * projErrorM.transpose();

    PROJ_RESULT proj_res;
    proj_res.mean = res;
    proj_res.cov_matrix = NoiseCovM;
    return proj_res;
}


cv::Point2d
Projection::world2pixel(const cv::Point2d P_world_)
{
    //* {W} -> {C}
    Eigen::Vector4d Pw;
    Pw << P_world_.x, P_world_.y, 0, 1; //! z=0
    Eigen::Vector4d Pc;
    Pc = this->K_ex * Pw;
    //* {C} -> {A}
    Eigen::Vector3d Pa;
    Pa << Pc[0]/Pc[2], Pc[1]/Pc[2], 1;
    //* {A} -> {P}
    double fx = K_in(0,0);
    double fy = K_in(1,1);
    double u0 = K_in(0,2);
    double v0 = K_in(1,2);
    cv::Point2d P_pix;
    P_pix.x = fx * Pa[0] + u0;
    P_pix.y = fy * Pa[1] + v0;
    return P_pix;
}