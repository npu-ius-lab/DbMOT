#include <iostream>
#include "tracker.h"


myTracker::myTracker()
{
    this->kf = new KalmanFilter3D();
}

vector<Track> myTracker::update(vector<DETECTION> &detections, Feature image, Projection proj_func, int frame_count)
{   
     vector<int> detidx_low;
     vector<int> detidx_high;
    for (int i=0; i < detections.size(); i++)
    {
        if (detections[i].score >= _high_score)
            detidx_high.push_back(i);
        else
            detidx_low.push_back(i);
    }
    //* prediction
    vector<cv::Rect2d> predict_boxes; 
    for (Track &track : _trackers)
    {
        track.detidx = -1;
        State3D pred_point3d = track.predict(this->kf);
        track.kf_prediction = pred_point3d;
        cv::Point2d pred_point2d = proj_func.world2pixel(pred_point3d);
        // if (pred_point2d.x >= 0 && pred_point2d.y >= 0)
        // {
            cv::Size2f target_size = track.get_target_size();

            State2D new_search_zone(pred_point2d.x - 0.5 * target_size.width, 
                pred_point2d.y - target_size.height, target_size.width, target_size.height);
                
            bool active = track.csr_update(new_search_zone, image);
            cv::Rect2d csr_pred = new_search_zone;
            predict_boxes.push_back(csr_pred);
            //* 反投影搜索区域
            track.kf_state2d = pred_point2d;
        // }
    }

    vector<cv::Rect2d> detect_boxes;
    for (int i = 0; i < detections.size(); i++) {
        detect_boxes.push_back(detections[i].boundingbox);
    }
    Associate assoc(detect_boxes, predict_boxes);
     vector<pair<int, int>> matches;
     set<int> u_track, u_detection;
    assoc.solve(u_detection, u_track, matches, _match_thresh);

    for (int idx = 0; idx < matches.size(); idx++) 
    {
        int detIdx = matches[idx].second;
        int trkIdx = matches[idx].first;
        _trackers[trkIdx].kf_update(this->kf, detections[detIdx].distribution.mean, detections[detIdx].distribution.cov_matrix);
        _trackers[trkIdx].detidx = detIdx;
        _trackers[trkIdx].reference_box = detect_boxes[detIdx];
    }
    for (int idx : u_detection) {
        Track new_track = Track(this->kf, detections[idx], image);
        new_track.detidx = idx;
        _trackers.push_back(new_track);
    }

    vector<Track> tracking_result;
    for (auto it = _trackers.begin(); it != _trackers.end();)
    {
        if (((*it).time_since_update < 1) &&
            ((*it).hit_streak >= _min_hits || frame_count <= _min_hits))
        {
            tracking_result.push_back((*it));
            it++;
        }
        else {
            it++;
        }
        //* remove dead tracklet
        if (it != _trackers.end() && (*it).time_since_update > _max_age)
            it = _trackers.erase(it);
    }
    return tracking_result;
}