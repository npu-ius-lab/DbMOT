#pragma once

#include <vector>
#include "track.h"
#include "dataType.h"
#include "detection.h"
#include "projection.h"
#include "kalmanfilter3d.h"
#include "Hungarian.h"
using namespace cv;
using namespace std;

class IUSLTracker
{
public:
    IUSLTracker(int max_age, int min_hits, float match_thresh);
    vector<TrackingResult> update(vector<Detection> &detections, cv::Mat img, int frameCount, ros::Time stamp);

public:
    vector<pair<int,cv::Point2d>> _predictions; 
    vector<Track> _demo_trackers;
private:
    void _associate(vector<Detection> detections, vector<Rect2d> predictions,
        set<int> &unmatchedDets, set<int> &unmatchedTrks, vector<pair<int, int>> &matched);
    void _initiate_tracker(int detidx, Detection detection, cv::Mat img);
private:
     //match
     vector<int> active_idxes;
     vector<int> tentative_idxes;
     vector<int> deleted_idxes;
    int _max_age;
    int _min_hits;
    float _iou_thresh;
    float _euc_thresh = 20.0;
    vector<Track> _trackers;
    KalmanFilter3D *_kf;
    Projection *_myproj;
    
};