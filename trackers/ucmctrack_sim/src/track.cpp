#include <iostream>
#include "track.h"

int Track::kf_count = 1; //! ID IS 1-BASED

Track::Track(KalmanFilter *kf, Detection detection)
{
    init_track(kf, detection);
    this->time_since_update = 0;//death_count
    this->hit_streak = 0;//birth_count
    this->age = 0;
    this->id = kf_count; //! ID IS 1-BASED
    this->kf_count++;

    this->detidx = -1;
    this->state = TrackState::Tentative;
}

void Track::init_track(KalmanFilter *kf, Detection detection)
{
    DETECTPOINT measurement;
    measurement << detection.distribution.mean.x, detection.distribution.mean.y;
    KAL_DATA data = kf->initiate(measurement);
    this->mean = data.first;
    this->covariance = data.second;
}

State3D Track::predict(KalmanFilter *kf)
{
	// if (this->time_since_update > 0) 
    //     this->hit_streak = 0;   
    // this->time_since_update += 1;
    this->age += 1;

    Eigen::Matrix<float,1,2> predict_pos;
    predict_pos = kf->predict(this->mean, this->covariance);

    cv::Point2d res;
    res.x = predict_pos(0,0);
    res.y = predict_pos(0,1);
	return res;
}

void Track::kf_update(KalmanFilter *const kf, State3D detection3d, Eigen::Matrix2d measurement_noise_)
{
    // this->hit_streak += 1;
    // this->time_since_update = 0;
    
    DETECTPOINT measurement;
    measurement(0,0) = detection3d.x;
    measurement(0,1) = detection3d.y;
    Eigen::Matrix2f measurement_noise = measurement_noise_.cast<float>();
    KAL_DATA data = kf->update(this->mean, this->covariance, measurement_noise, measurement);
    this->mean = data.first;
    this->covariance = data.second;
}

float Track::get_distance(KalmanFilter *kf, Detection detection)
{
    double x = detection.distribution.mean.x;
    double y = detection.distribution.mean.y;
    KAL_HMEAN measurement(x, y);

    Eigen::Matrix2d measurement_noise_ = detection.distribution.cov_matrix;
    KAL_HCOVA measurement_noise = measurement_noise_.cast<float>();
    
    float dis = kf->mahalanobis_distance(this->mean, this->covariance, measurement, measurement_noise);

    return dis;
}

//* track state
void Track::mark_active()
{
    this->state = TrackState::Active;
}

void Track::mark_deleted()
{
    this->state = TrackState::Coasted;
}

void Track::mark_tentative()
{
    this->state = TrackState::Tentative;
}

bool Track::is_active()
{
    return this->state == TrackState::Active;
}

bool Track::is_deleted()
{
    return this->state == TrackState::Coasted;
}

bool Track::is_tentative()
{
    return this->state == TrackState::Tentative;
}

//* track info
State3D Track::get_kf_state()
{
    return State3D(this->mean(0,0), this->mean(0,2));
}