#include <iostream>
#include "track.h"

int Track::track_count = 0;

Track::Track(KalmanFilter3D *kf, cv::Rect2d initRect, cv::Point2d initPoint, 
             cv::Mat image, int min_hits, int max_age)
{
    init_track(kf, initRect, initPoint, image);
    this->time_since_update = 0;
    this->hit_streak = 1;
    this->id = track_count + 1;
    this->track_count++;
    this->state = TrackState::Tentative;
    
    this->_max_age = max_age;
    this->_min_hits = min_hits;
}

void Track::init_track(KalmanFilter3D *kf, cv::Rect2d detection2d, 
                       cv::Point2d detection3d, cv::Mat image)
{
    //* initialize csrt
    cv::TrackerCSRT::Params params;
    params.scale_sigma_factor = 0.36f; 
    params.number_of_scales = 60; 
    params.filter_lr = 0.05f;
    this->_roi = detection2d;
    this->csrt = cv::TrackerCSRT::create(params);
    this->csrt->init(image, detection2d);
    
    //* initialize kf
    DETECTPOINT measurement;
    measurement(0,0) = detection3d.x;
    measurement(0,1) = detection3d.y;
    KAL_DATA data = kf->initiate(measurement);
    this->mean = data.first;
    this->covariance = data.second;
}

cv::Point2d Track::predict(KalmanFilter3D *kf)
{
	if (this->time_since_update > 0) 
        this->hit_streak = 0;   
    this->time_since_update += 1;

    Eigen::Matrix<float,1,2> predict_pos;
    predict_pos = kf->predict(this->mean, this->covariance);

    cv::Point2d res;
    res.x = predict_pos(0,0);
    res.y = predict_pos(0,1);
	return res;
}

void Track::kf_update(KalmanFilter3D *const kf, cv::Point2d detection3d)
{
    this->hit_streak += 1;
    this->time_since_update = 0;
    
    DETECTPOINT measurement;
    measurement(0,0) = detection3d.x;
    measurement(0,1) = detection3d.y;
    // Eigen::Matrix2f diag = diag_.cast<float>();
    KAL_DATA data = kf->update(this->mean, this->covariance, measurement);
    this->mean = data.first;
    this->covariance = data.second;
}

bool Track::csr_update(cv::Rect2d &detection2d, cv::Mat image)
{
    bool active = this->csrt->update(image, detection2d);
    if (active)
    {
        cv::Rect2d trkResult = detection2d;
        this->_roi = trkResult;   
    }
    else {
        // detection2d = get_csr_state(); //* 未达到psr阈值时用csr上一帧的roi
        this->_roi = this->reference_box;
    }
    return active;
}

void Track::mark_missed()
{
    if (this->state == TrackState::Tentative)
    {
        this->state = TrackState::Deleted;
    }
    else if (this->time_since_update > this->_max_age)
    {
        this->state = TrackState::Deleted;
    }
}

//* track state
void Track::mark_active()
{
    this->state = TrackState::Active;
}

void Track::mark_deleted()
{
    this->state = TrackState::Deleted;
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
    return this->state == TrackState::Deleted;
}

bool Track::is_tentative()
{
    return this->state == TrackState::Tentative;
}

//* track info
cv::Point2d Track::get_kf_state()
{
    return cv::Point2d(this->mean(0,0), this->mean(0.2));
}

cv::Rect2d Track::get_csr_state()
{
    if (this->_roi.x <0)
        this->_roi.x = 0;
    else if (this->_roi.y <0)
        this->_roi.y =0;
    return this->_roi;
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