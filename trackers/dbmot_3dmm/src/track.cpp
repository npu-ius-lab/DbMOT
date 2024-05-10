#include <iostream>
#include "track.h"

int Track::track_count = 1; 

Track::Track(KalmanFilter3D *kf, DETECTION detection, Feature image)
{
    init_track(kf, detection, image);
    this->time_since_update = 0;
    this->hit_streak = 1;
    this->age = 0;
    this->id = track_count; 
    this->track_count++;
    this->max_age = max_age;
    this->min_hits = min_hits;
    this->roi = detection.boundingbox;
}

void Track::init_track(KalmanFilter3D *kf, DETECTION detection, Feature image)
{
    //* csrt initialization
    cv::TrackerCSRT::Params params;
    params.scale_sigma_factor = 0.36f; 
    params.number_of_scales = 60; 
    this->csrt = cv::TrackerCSRT::create(params);
    this->csrt->init(image, detection.boundingbox);

    //* kalman filter initialization
    DETECTPOINT measurement;
    measurement << detection.distribution.mean.x, detection.distribution.mean.y;
    KAL_DATA data = kf->initiate(measurement);
    this->mean = data.first;
    this->covariance = data.second;
}

State3D Track::predict(KalmanFilter3D *kf)
{
    Eigen::Matrix<float,1,2> predict_pos;
    predict_pos = kf->predict(this->mean, this->covariance);
    this->age += 1;

	if (this->time_since_update > 0) 
        this->hit_streak = 0;   
    this->time_since_update += 1;

    cv::Point2d res;
    res.x = predict_pos(0,0);
    res.y = predict_pos(0,1);
	return res;
}

void Track::kf_update(KalmanFilter3D *const kf, State3D detection3d, Eigen::Matrix2d measurement_noise_)
{
    this->hit_streak += 1;
    this->time_since_update = 0;
    
    DETECTPOINT measurement;
    measurement(0,0) = detection3d.x;
    measurement(0,1) = detection3d.y;
    Eigen::Matrix2f measurement_noise = measurement_noise_.cast<float>();
    KAL_DATA data = kf->update(this->mean, this->covariance, measurement_noise, measurement);
    this->mean = data.first;
    this->covariance = data.second;
}

bool Track::csr_update(State2D &detection2d, Feature image)
{
    bool active = this->csrt->update(image, detection2d);
    if (active)
    {
        State2D trkResult = detection2d;
        this->roi = trkResult;   
    }
    else {
        // cout << "666666" << endl;
        this->roi = this->reference_box;
    }
    return active;
}

//* track info
State3D Track::get_kf_state()
{
    return State3D(this->mean(0,0), this->mean(0,2));
}

State2D Track::get_csr_state()
{
    // if (this->roi.x < 0 || this->roi.y < 0) {
    //     std::cout << "roi error!" << std:: endl;    
    // }
    
    return this->roi;
}

cv::Size2f Track::get_target_size()
{
    return csrt->get_target_size();
}

cv::Size2i Track::get_scale_size() //* 返回搜索区域的宽高
{
    return csrt->get_scale_size();
}

cv::Rect2d Track::get_search_zone()
{
    return csrt->get_search_zone();
}
