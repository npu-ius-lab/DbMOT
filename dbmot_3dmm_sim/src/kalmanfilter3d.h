#ifndef KALMANFILTER3D_H
#define KALMANFILTER3D_H

#include "dataType.h"

class KalmanFilter3D
{
public:
    static const double chi2inv95[10];

    KalmanFilter3D();

    KAL_DATA initiate(const DETECTPOINT& measurement);

    KAL_HMEAN predict(KAL_MEAN& mean, KAL_COVA& covariance);

    KAL_HDATA project(const KAL_MEAN& mean, const KAL_COVA& covariance);

    KAL_DATA update(const KAL_MEAN& mean,
                    const KAL_COVA& covariance,
                    // const KAL_HCOVA diag,
                    const DETECTPOINT& measurement);

private:
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> _motion_mat;
    Eigen::Matrix<float, 2, 4, Eigen::RowMajor> _update_mat;
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> _motion_cov;
    float _std_weight_position;
    float _std_weight_velocity;
};

#endif // KALMANFILTER_H
