#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "dataType.h"

namespace UCMC
{
class KalmanFilter
{
public:
    static const double chi2inv95[10];

    KalmanFilter();

    KAL_DATA initiate(const DETECTPOINT& measurement);

    KAL_HMEAN predict(KAL_MEAN& mean, KAL_COVA& covariance);

    KAL_HDATA project(const KAL_MEAN& mean, const KAL_COVA& covariance, const KAL_HCOVA diag);

    KAL_DATA update(const KAL_MEAN& mean,
                    const KAL_COVA& covariance,
                    const KAL_HCOVA diag,
                    const DETECTPOINT& measurement);

    float mahalanobis_distance(const KAL_MEAN mean, 
                               const KAL_COVA covariance,
                               const KAL_HMEAN measurement_mean, 
                               const KAL_HCOVA measurement_covariance);

private:
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> _motion_mat;
    Eigen::Matrix<float, 2, 4, Eigen::RowMajor> _update_mat;
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> _motion_cov;
    float _std_weight_position;
    float _std_weight_velocity;
};
}
#endif // KALMANFILTER_H
