#ifndef TRACK_H
#define TRACK_H

#include "dataType.h"
#include "kalmanfilter3d.h"
#include "sot/tracking.hpp"

using namespace std;


class Track
{ 
public:   
    Track(KalmanFilter3D *kf, DETECTION detection, Feature image);
    State3D predict(KalmanFilter3D *kf);
    void kf_update(KalmanFilter3D *const kf, State3D detection3d, Eigen::Matrix2d diag_);
    bool csr_update(State2D &csrState, Feature image);

    State3D get_kf_state();
    State2D get_csr_state();
    cv::Size2f get_target_size();
    cv::Size2i get_scale_size();
    cv::Rect2d get_search_zone();
	
public:
	int time_since_update;
	int hit_streak;
    int age;
    static int track_count;
	int id;

    KAL_MEAN mean;
    KAL_COVA covariance;
    KAL_COVA diag;

    int detidx;
    cv::Rect2d reference_box;
    cv::Point2d kf_state2d;
    cv::Point2d kf_prediction;

private:
    void init_track(KalmanFilter3D *kf, DETECTION detection, Feature image);
    int min_hits;
    int max_age;
    cv::Ptr<cv::TrackerCSRT> csrt;
	cv::Rect2d roi;
    
};

#endif // TRACK_H
