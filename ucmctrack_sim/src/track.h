#pragma once

#include "dataType.h"
#include "kalmanfilter.h"
#include "detection.h"
using namespace std;
using namespace UCMC;

 

class Track
{
    enum TrackState {Tentative = 1, Active, Coasted};
    
public:   
    Track(KalmanFilter *kf, Detection detection);

    State3D predict(KalmanFilter *kf);
    void kf_update(KalmanFilter *const kf, State3D detection3d, Eigen::Matrix2d diag_);
     void mark_missed();
     void mark_active();
     void mark_deleted();
     void mark_tentative();
     bool is_active();
     bool is_deleted();
     bool is_tentative();
    State3D get_kf_state();
    float get_distance(KalmanFilter *kf, Detection detection);
	
public:
    static int kf_count;
	int time_since_update;//death_count
	int hit_streak;//birth_count
	int age;
    int id;

    KAL_MEAN mean;
    KAL_COVA covariance;
    KAL_COVA diag;
    TrackState state;
    int detidx;

private:
    void init_track(KalmanFilter *kf, Detection detection);
};