#include "kalmanfilter.h"
#include <Eigen/Cholesky>

using namespace UCMC;

//sisyphus
const double KalmanFilter::chi2inv95[10] = {
    0,
    3.8415,
    5.9915,
    7.8147,
    9.4877,
    11.070,
    12.592,
    14.067,
    15.507,
    16.919
};

KalmanFilter::KalmanFilter()
{
    /**
     * _motion_mat: F_k
     * _update_mat: H_k
     * 
    */
    int ndim = 4;
    int fps = 10;
    double dt = 1.0 / fps;

    //* F_k
    _motion_mat = Eigen::MatrixXf::Identity(4, 4);//? 4x4
    _motion_mat << 1, dt,  0,  0,
                   0,  1,  0,  0,
                   0,  0,  1, dt,
                   0,  0,  0,  1;
    //* H_k
    _update_mat = Eigen::MatrixXf::Identity(2, 4);//? 2x4
    _update_mat << 1, 0, 0, 0,
                   0, 0, 1, 0;
    //* Q_k
    Eigen::Matrix<float,4,2> G;
    G.setZero();
    G(0,0) = 0.5 * dt * dt;
    G(1,0) = dt;
    G(2,1) = 0.5 * dt * dt;
    G(3,1) = dt;
    KAL_HCOVA Q0;

    int wx = 5;//0.5
    int wy = 5;//2.5
    Q0 << wx,  0,
           0, wy;
    _motion_cov = G * Q0 * G.transpose(); //* Q_k

}

KAL_DATA KalmanFilter::initiate(const DETECTPOINT &measurement)
{
    KAL_MEAN mean;
    mean(0,0) = measurement(0,0); //* x
    mean(0,1) = 0;                //* vx
    mean(0,2) = measurement(0,1); //* y
    mean(0,3) = 0;                //* vy
    //* P_k
    KAL_MEAN std;
    float vmax = 0.1;
    std(0) = 1;                     //sigma_x
    std(1) = (vmax * vmax) / 3.0;   //sigma_vx
    std(2) = 1;                     //sigma_y
    std(3) = (vmax * vmax) / 3.0;   //sigma_vy
    KAL_MEAN tmp = std.array().square(); //方差（标准差平方）
    KAL_COVA var = tmp.asDiagonal();// 误差协方差矩阵P(半正定)

    return std::make_pair(mean, var);
}

KAL_HMEAN KalmanFilter::predict(KAL_MEAN &mean, KAL_COVA &covariance)
{
    /** 
     * mean: x_k-1 (1x4)
     * covariance: P_k-1 (4x4)
     * 
     * mean1: x'k = F_k * x_k-1 (4x1)
     * covariance1: P'k = F_k * P_k-1 * F_k.T + Q_k (4x4)
     * 
     * motion_cov:  Q_k (4x4)
     * _motion_mat: F_k (4x4)
     * 
     */

    //? x'k = F_k * x_k-1
    KAL_MEAN mean1 = this->_motion_mat * mean.transpose();

    //? P'k = F_k * P_k-1 * F_k.T + Q_k
    KAL_COVA covariance1 = this->_motion_mat * covariance * (_motion_mat.transpose());
    covariance1 += this->_motion_cov;

    mean = mean1;
    covariance = covariance1;

    Eigen::Matrix<float,1,2> prediction;
    prediction(0,0) = mean(0,0);
    prediction(0,1) = mean(0,2);
    return prediction;
}

KAL_HDATA KalmanFilter::project(const KAL_MEAN &mean, const KAL_COVA &covariance, const KAL_HCOVA diag)
{
    /** 
     * mean: x_k-1 (1x4)
     * covariance: P_k-1 (4x4)
     * _update_mat: H_k (2x4)
     * 
     * mean1: H_k * x'k (1x2)
     * covariance1: H_k * P'k * H_k.T + R_k (2x2)
     * 
    */

    KAL_HMEAN mean1 = _update_mat * mean.transpose();//? H_k * x'k
    KAL_HCOVA covariance1 = _update_mat * covariance * (_update_mat.transpose());//? H_k * P'k * H_k.T
    covariance1 += diag; //? H_k * P'k * H_k.T + R_k

    return std::make_pair(mean1, covariance1);
}

KAL_DATA
KalmanFilter::update(
    const KAL_MEAN &mean,
    const KAL_COVA &covariance,
    const KAL_HCOVA diag,
    const DETECTPOINT &measurement)
{
    /** 
     * mean: x'k (1x4)
     * covariance: P'k (4x4)
     * measurement: z_k (1x2)
     * _update_mat: H_k (2x4)
     * diag: R_k (4x4)
     * 
     * projected_mean: H_k * x'_k (1x2)
     * projected_cov: H_k * P'k * H_k.T + R_k (2x2)
     * 
     * B: H_k * P'k.T (2x4)
     * kalman_gain: K_k (4x2)
     * innovation: z_k - H_k * x'_k (1x4)
     * 
     * new_mean: x_k (1x4)
     * new_covariance: P_k (4x4)
     * 
    */

    KAL_HDATA pa = project(mean, covariance, diag);
    KAL_HMEAN projected_mean = pa.first;
    KAL_HCOVA projected_cov = pa.second;

    Eigen::Matrix<float, 2, 4> B = (covariance * (_update_mat.transpose())).transpose();//? H_k * P'k.T

    //? K_k = P'k * H_k.T * (H_k * P'k * H_k.T + R_k)^-1
    Eigen::Matrix<float, 4, 2> kalman_gain = (projected_cov.llt().solve(B)).transpose(); // eg.4x2
    
    Eigen::Matrix<float, 1, 2> innovation = measurement - projected_mean;// eg.1x2 //? z_k - H_k * x'_k

    auto tmp = innovation * (kalman_gain.transpose());// eg.1x4

    KAL_MEAN new_mean = (mean.array() + tmp.array()).matrix();//? x'k + K_k * (z_k - H_k * x'k)

    //? P_k = P'k - K_k * (H_k * P'k * H_k.T + R_k) * K_k.T
    //* K_k.T = (H_k * P'k * H_k.T + R_k)^-1 * H_k.T * P'k.T
    //? P_k =  P'k - K_k * (H_k * P'k * H_k.T + R_k) * (H_k * P'k * H_k.T + R_k)^-1 * H_k.T * P'k.T
    //? P_k = P'k - K_k * H_k.T * P'k.T (P是半正定矩阵, P.T = P)
    KAL_COVA new_covariance = covariance - kalman_gain * projected_cov * (kalman_gain.transpose());

    return std::make_pair(new_mean, new_covariance);
}

float KalmanFilter::mahalanobis_distance(
    const KAL_MEAN mean, 
    const KAL_COVA covariance,
    const KAL_HMEAN measurement_mean, 
    const KAL_HCOVA measurement_covariance)
{
    Eigen::Matrix<float, 2, 1> diff = measurement_mean.transpose() - _update_mat * mean.transpose();
    KAL_HCOVA residual_cov = _update_mat * covariance * _update_mat.transpose() + measurement_covariance;
    Eigen::Matrix2f E = Eigen::Matrix2f::Identity();
    KAL_HCOVA residual_cov_inv = residual_cov.llt().solve(E);
    Eigen::Matrix<float, 1, 1> mahalanobis = diff.transpose() * residual_cov_inv * diff;
    float residual_cov_det = residual_cov.determinant();

    return mahalanobis.value() + log(residual_cov_det);
}