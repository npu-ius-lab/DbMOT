#pragma once

#include "dataType.h"
#include "kalmanfilter3d.h"
#include "sot/tracking.hpp"

using namespace std;



class Track
{
    enum TrackState {Tentative = 1, Active, Deleted}; 

public:   
    Track(KalmanFilter3D *kf, cv::Rect2d initRect, cv::Point2d initPoint, 
          cv::Mat image, int min_hits, int max_age);

    cv::Point2d predict(KalmanFilter3D *kf);
    void kf_update(KalmanFilter3D *const kf, cv::Point2d detection3d);
    bool csr_update(cv::Rect2d &csrState, cv::Mat image);
      cv::Rect2d to_tlwh();

public:
    void mark_missed();
    void mark_active();
    void mark_deleted();
    void mark_tentative();
    bool is_active();
    bool is_deleted();
    bool is_tentative();
	cv::Point2d get_kf_state();
    cv::Rect2d get_csr_state();
    cv::Size2f get_target_size();
    cv::Size2i get_scale_size();
    cv::Rect2d get_search_zone();

public:
    static int track_count;
	int time_since_update;
	int hit_streak;
	int id;
    cv::Rect2d search_zone;
    cv::Size2i scale_size;
    KAL_MEAN mean;
    KAL_COVA covariance;
    KAL_COVA diag;
    TrackState state;
    int detidx;
    cv::Point2d kf_state2d;
    bool update_success;
    cv::Rect2d reference_box;

private:
    void init_track(KalmanFilter3D *kf, cv::Rect2d detection2d, 
                    cv::Point2d detection3d, cv::Mat image);
    int _min_hits;
    int _max_age;
    cv::Ptr<cv::TrackerCSRT> csrt;
	cv::Rect2d _roi;
};
