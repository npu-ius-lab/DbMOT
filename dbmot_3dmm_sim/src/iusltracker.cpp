#include "iusltracker.h"

IUSLTracker::IUSLTracker(int max_age, int min_hits, float match_thresh)
{
    _max_age = max_age;
    _min_hits = min_hits;
    _iou_thresh = match_thresh;
    _kf = new KalmanFilter3D();
    _myproj = new Projection();
}
// #include <iostream>
// using namespace std;
// #include <opencv2/highgui/highgui.hpp>

vector<TrackingResult>
IUSLTracker::update(vector<Detection> &detections, cv::Mat img, int frameCount, ros::Time stamp)
{
    for (Track& track : _trackers)
    {
        track.detidx = -1;
    }

    _predictions.clear();
    vector<Rect2d> csr_tracks;
    for (Track& track : _trackers) 
    {
        cv::Point2d pred3d = track.predict(_kf);
         _predictions.push_back(make_pair(track.id, pred3d));
        
        cv::Point2d pred2d = _myproj->world2pixel(pred3d, stamp);
        
      ////cv
        //  cv::Point kfshow(pred2d.x, pred2d.y);
        //  cv::circle(img, kfshow, 5, cv::Scalar(0, 0, 255), -1);
      ////cv
         cv::Size2f targetSize = track.get_target_size();
         float target_w = targetSize.width;
         float target_h = targetSize.height;
        cv::Rect2d S1(pred2d.x - kRatio*target_w, pred2d.y - target_h, target_w, target_h);
        track.kf_state2d.x = pred2d.x;
        track.kf_state2d.y = pred2d.y - kRatio*target_h;
        bool update_success = track.csr_update(S1, img);
        track.update_success = update_success;
        csr_tracks.push_back(S1);
    }
    
    for (Detection& detection : detections)
    {
        cv::Point2d res = _myproj->pixel2world(detection.bbx_tlwh, stamp);
        detection.state3d = res;
    }

    set<int> u_track, u_detection;
    vector<pair<int,int>> matches;
    _associate(detections, csr_tracks, u_detection, u_track, matches);
    // cout << "m:" << matches.size() << " ud:" << u_detection.size() << " ut:" << u_track.size() << endl;

    for (int i : u_detection) {
        _initiate_tracker(i, detections[i], img);
    }
    for (int i : u_track) {
        _trackers[i].detidx = -1;
        _trackers[i].mark_deleted();
    }
    for (auto pa : matches)
    {
        int track_idx = pa.first;
        int detection_idx = pa.second;
        _trackers[track_idx].detidx = detection_idx;
        _trackers[track_idx].kf_update(_kf, detections[detection_idx].state3d);
        _trackers[track_idx].reference_box = detections[detection_idx].bbx_tlwh;
        if (_trackers[track_idx].is_deleted()) {
            _trackers[track_idx].mark_tentative();
        }
        if (_trackers[track_idx].is_tentative() && _trackers[track_idx].hit_streak >= _min_hits) {
            _trackers[track_idx].mark_active();
        }
    }

    vector<Track>::iterator it;
    for (it = _trackers.begin(); it != _trackers.end();)
    {
        Track& track = (*it);
        if ((track.is_deleted() && track.time_since_update >=_max_age) ||
            (track.is_tentative() && track.time_since_update >=_min_hits))

            it = _trackers.erase(it);
        else
            ++it;
    }

    vector<TrackingResult> ret;
    for (Track& track : _trackers) {
        if (track.is_active() || frameCount <= _min_hits) {
            TrackingResult res;
            res.id = track.id;
            res.detidx = track.detidx;
            res.tlwh = track.get_csr_state();
            res.search_zone = track.get_search_zone();
            cv::Rect2d proj_sz(track.kf_state2d.x - 0.5*track.get_scale_size().width,
                               track.kf_state2d.y - 0.5*track.get_scale_size().height,
                               track.get_scale_size().width,
                               track.get_scale_size().height);
            res.proj_sz = proj_sz;
            ret.push_back(res);
            // cout << "trackid: " << track.id << "    " << "success: " << track.update_success << endl;
        }
    }

    //* Update Status
    active_idxes.clear();
    deleted_idxes.clear();
    tentative_idxes.clear();
    for (int i=0; i<_trackers.size(); i++) 
    {
        Track& track = _trackers[i];
        if (track.is_active())
            active_idxes.push_back(i);

        else if (track.is_tentative())
            tentative_idxes.push_back(i);
            
        else if (track.is_deleted())
            deleted_idxes.push_back(i);
    }
    // cout << "active:" << active_idxes.size() << "  tentative:" << tentative_idxes.size() << "  deleted:" << deleted_idxes.size() << endl;

    return ret;
}

void IUSLTracker::_initiate_tracker(int detidx, Detection detection, cv::Mat img)
{
    Track new_track = Track(_kf, detection.bbx_tlwh, detection.state3d, img, _min_hits, _max_age);
    new_track.detidx = detidx;
    _trackers.push_back(new_track);
}

void IUSLTracker::_associate(vector<Detection> detections, vector<Rect2d> predictions,
    set<int> &unmatchedDets, set<int> &unmatchedTrks, vector<pair<int, int>> &matched)
{
    set<int> allDets, allTrks;
    set<int> matchedDets;

    if (detections.size() == 0) {
        for (int t=0; t < predictions.size(); t++) {
            unmatchedTrks.insert(t);
        }
    } 
    else if (predictions.size() == 0) { //for the begainning of tracking
        for (int d=0; d < detections.size(); d++)
        {
            unmatchedDets.insert(d);
        }
    }
    else 
    {   
        vector<int> assignment;
        HungarianAlgorithm Hungarian;
        vector<vector<double>> costMatrix;
        costMatrix.resize(predictions.size(), vector<double>(detections.size(), 0.0));

        for (int t = 0; t < predictions.size(); t++) {
            for (int d = 0; d < detections.size(); d++) {
                // double iou;
                // double in = (predictions[t] & detections[d].bbx_tlwh).area();           
                // double un = predictions[t].area() + detections[d].bbx_tlwh.area() - in; 
                // if (un < DBL_EPSILON)    
                // {
                //     iou = 0; 
                // }                      
                // iou = (double)(in / un);
                // costMatrix[t][d] = 1.0 - iou;
                
                cv::Point2d center1(predictions[t].x + kRatio * predictions[t].width, 
                    predictions[t].y + kRatio * predictions[t].height);
                
                cv::Point2d center2(detections[d].bbx_tlwh.x + kRatio * detections[d].bbx_tlwh.width, 
                    detections[d].bbx_tlwh.y + kRatio * detections[d].bbx_tlwh.height);
                
                double distance = cv::norm(center1 - center2);
                // double cost = 1.0 / (1.0 + std::exp(-(distance - 100))); 
                costMatrix[t][d] = distance;
                // cout << distance << " ";

            }
            // cout << "\n";
        }
        // cout << endl;

        if (std::min(costMatrix.size(), costMatrix[0].size()) > 0) {
            // assignment[i]表示与第i个track相匹配的检测索引
            Hungarian.Solve(costMatrix, assignment, MINIMIZE_COST); 
        }
        if (detections.size() > predictions.size())
        {
            for (unsigned int n = 0; n < detections.size(); n++) {
                allDets.insert(n);
            }
            for (unsigned int i = 0; i < predictions.size(); ++i) {
                matchedDets.insert(assignment[i]);
            }
            set_difference(allDets.begin(), allDets.end(),
                matchedDets.begin(), matchedDets.end(),
                insert_iterator<set<int>>(unmatchedDets, unmatchedDets.begin()));
        }
        else if (detections.size() < predictions.size())
        {
            for (unsigned int i = 0; i < predictions.size(); ++i) {
                if (assignment[i] == -1) {
                    unmatchedTrks.insert(i);
                }
            }
        }
        for (unsigned int i = 0; i < predictions.size(); ++i)
        {
            if (assignment[i] == -1) {// pass over invalid values
                continue;
            }
            // if (1 - costMatrix[i][assignment[i]] < _iou_thresh) {
            if (costMatrix[i][assignment[i]] > _euc_thresh) {
                unmatchedTrks.insert(i);
                unmatchedDets.insert(assignment[i]); 
            }
            else {
                matched.push_back(std::make_pair(i, assignment[i]));
            }
        }
    }
}