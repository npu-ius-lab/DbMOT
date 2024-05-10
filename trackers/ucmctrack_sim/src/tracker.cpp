#include <vector>
#include <iostream>
#include "dataType.h"
#include "track.h"
#include "tracker.h"

UCMCTrack::UCMCTrack(int _max_age, int _min_hits)
{
    this->max_age = _max_age;
    this->min_hits = _min_hits;
    this->kf = new UCMC::KalmanFilter();
    _myproj = new Projection();
}

void UCMCTrack::update(vector<Detection> &detections, int frameCount, ros::Time stamp)
{
    data_association(detections, stamp);
    associate_tentative(detections);
    initial_tentative(detections);
    delete_old_trackers();
    update_status(detections);
}

void UCMCTrack::data_association(vector<Detection> &detections, ros::Time stamp)
{   
     vector<int> detidx_low;
     vector<int> detidx_high;
    for (int i=0; i < detections.size(); i++)
    {
        if (detections[i].score >= high_score)
            detidx_high.push_back(i);
        else
            detidx_low.push_back(i);
    }
    for (Detection& detection : detections)
    {
        PROJ_RESULT res = _myproj->pixel2world(detection.bbx_tlwh, stamp);
        detection.distribution = res;
    }

    _predictions.clear();
    for (Track& track : trackers) 
    {
        State3D point = track.predict(this->kf);
        _predictions.push_back(point);
    }
    
    std::vector<int> trackidx(confirmed_idx.begin(), confirmed_idx.end());
    trackidx.insert(trackidx.end(), coasted_idx.begin(), coasted_idx.end());
    int num_det = detidx_high.size();
    int num_trk = trackidx.size();
    for (Track& track : trackers)
    {
        track.detidx = -1;
    }

    detidx_remain.clear();
     vector<int> trackidx_remain;
    if (num_det * num_trk > 0) 
    {   
         std::vector< std::vector<float> > cost_matrix;
        cost_matrix.resize(num_trk, vector<float>(num_det, 0.0f));
        for (int i=0; i<num_trk; i++) {
            int trkidx = trackidx[i];
            for (int j=0; j<num_det; j++) {
                int detidx = detidx_high[j];    
                cost_matrix[i][j] = trackers[trkidx].get_distance(this->kf, detections[detidx]);
                // cout << cost_matrix[i][j] << " ";
            }
            // cout << "\n";
        }
         std::vector< std::vector<int> > matches;
         std::vector<int> u_track, u_detection;
        linear_assignment(cost_matrix, num_trk, num_det, mahal_thresh, matches, u_track, u_detection);

        for (int idx : u_detection) {
            int detidx = detidx_high[idx];
            detidx_remain.push_back(detidx);
        }
        for (int idx : u_track) {
            int trkidx = trackidx[idx];
            trackidx_remain.push_back(trkidx);
        }
        for (int i=0; i<matches.size(); i++) 
        {
            int trkidx = trackidx[matches[i][0]];
            int detidx = detidx_high[matches[i][1]];
            Track& track = trackers[trkidx];
            Detection& detection = detections[detidx];
            track.kf_update(this->kf, detection.distribution.mean, detection.distribution.cov_matrix);
            track.time_since_update = 0;
            track.detidx = detidx;
            track.mark_active();
            detection.trackid = track.id;
        }
    }
    else {
        detidx_remain = detidx_high;
        trackidx_remain = trackidx;
    }

    num_det = detidx_low.size();
    num_trk = trackidx_remain.size();
    if (num_det * num_trk > 0) 
    {   
         std::vector< std::vector<float> > cost_matrix;
        cost_matrix.resize(num_trk, vector<float>(num_det, 0.0f));
        for (int i=0; i<num_trk; i++) {
            int trkidx = trackidx_remain[i];
            for (int j=0; j<num_det; j++) {
                int detidx = detidx_low[j];
                cost_matrix[i][j] = trackers[trkidx].get_distance(this->kf, detections[detidx]);
                // cout << cost_matrix[i][j] << " ";
            }
            // cout << "\n";
        }
         std::vector< std::vector<int> > matches;
         std::vector<int> u_track, u_detection;
        linear_assignment(cost_matrix, num_trk, num_det, mahal_thresh, matches, u_track, u_detection);

        for (int idx : u_track) {
            int trkidx = trackidx_remain[idx];
            trackers[trkidx].mark_deleted();
            trackers[trkidx].detidx = -1;
        }
        for (int i=0; i<matches.size(); i++)
        {
            int trkidx = trackidx_remain[matches[i][0]];
            int detidx = detidx_low[matches[i][1]];
            Track& track = trackers[trkidx];
            Detection& detection = detections[detidx];
            track.kf_update(this->kf, detection.distribution.mean, detection.distribution.cov_matrix);
            track.time_since_update = 0;
            track.detidx = detidx;
            track.mark_active();
            detection.trackid = track.id;
        }
        
    } 
}

void UCMCTrack::associate_tentative(vector<Detection> &detections)
{
    int num_det = detidx_remain.size();
    int num_trk = tentative_idx.size();
    
     std::vector< std::vector<float> > cost_matrix;
    cost_matrix.resize(num_trk, vector<float>(num_det, 0.0f));

    for (int i=0; i<num_trk; i++) {
        int trkidx = tentative_idx[i];
        for (int j=0; j<num_det; j++) {
            int detidx = detidx_remain[j];
            cost_matrix[i][j] = trackers[trkidx].get_distance(this->kf, detections[detidx]);
        }
    }
     std::vector< std::vector<int> > matches;
     std::vector<int> u_track, u_detection;
    linear_assignment(cost_matrix, num_trk, num_det, mahal_thresh, matches, u_track, u_detection);
    // cout << matches.size() << "  " << u_detection.size() << "  " << u_track.size() << endl;

    for (int i = 0; i < matches.size(); i++) 
    {
        int trkidx = tentative_idx[matches[i][0]];
        int detidx = detidx_remain[matches[i][1]];
        Track& track = trackers[trkidx];
        Detection& detection = detections[detidx];
        track.kf_update(this->kf, detection.distribution.mean, detection.distribution.cov_matrix);
        track.time_since_update = 0;
        track.hit_streak += 1;
        track.detidx = detidx;
        detection.trackid = track.id;

        if (track.hit_streak > min_hits) {
            track.hit_streak = 0;
            track.mark_active();
        }
    }
    for (int idx : u_track) {
        int trkidx = tentative_idx[idx];
        trackers[trkidx].detidx = -1;
    }
    vector<int> unmatched_detidx;
    for (int idx : u_detection) {
        int detidx = detidx_remain[idx];
        unmatched_detidx.push_back(idx);
    }
    detidx_remain = unmatched_detidx;
}

void UCMCTrack::initial_tentative(vector<Detection> &detections)
{
    for (int idx : detidx_remain) {
        Track new_track = Track(this->kf, detections[idx]);
        new_track.mark_tentative();
        new_track.detidx = idx;
        trackers.push_back(new_track);
    }
    detidx_remain.clear();
}

void UCMCTrack::delete_old_trackers()
{
    for (auto it=trackers.begin(); it!=trackers.end();)
    {
        Track& track = (*it);
        track.time_since_update += 1;
        bool is_missed = track.is_deleted() && track.time_since_update >= max_age;
        bool init_failed = track.is_tentative() && track.time_since_update >= min_hits;
        if (init_failed || is_missed) {
            // cout << "pop No." << track.id << " tracker" << "\n";
            it = trackers.erase(it);
        }
        else {
            it++;
        }
    }
}

void UCMCTrack::update_status(vector<Detection> &detections)
{
    confirmed_idx.clear();
    coasted_idx.clear();
    tentative_idx.clear();
    for (int i=0; i<trackers.size(); i++)
    {
        Track& track = trackers[i];
        int detidx = track.detidx;
        if (track.is_active()) {
            confirmed_idx.push_back(i);
        }
        else if (track.is_deleted()) {
            coasted_idx.push_back(i);
        }
        else if (track.is_tentative()) {
            tentative_idx.push_back(i);
        }
    }
    // cout << "confirmed:" << "  " << confirmed_idx.size() << "\n";
    // cout << "tentative:" << "  " << tentative_idx.size() << "\n";
    // cout << "coasterd:" << "  " << coasted_idx.size() << "\n";
}