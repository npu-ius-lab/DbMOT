#pragma once

#include <vector>
#include "dataType.h"
#include "track.h"
#include "kalmanfilter3d.h"
#include "projection.h"
#include "associate.h"

class myTracker
{
public:
    myTracker();    
    vector<Track> update(vector<DETECTION> &detections, Feature image, 
        Projection proj_func, int frame_count);

private:
    float _high_score = 0.75;
    int _max_age = 30;
    int _min_hits = 3;
    float _match_thresh = 0.1; //todo
     vector<Track> _trackers;
     vector<int> confirmed_idx;
     vector<int> coasted_idx;
     vector<int> tentative_idx;
     vector<int> detidx_remain;
    KalmanFilter3D *kf;
};