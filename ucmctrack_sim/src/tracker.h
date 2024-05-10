#pragma once

#include <vector>
#include "dataType.h"
#include "track.h"
#include "kalmanfilter.h"
#include "detection.h"
#include "projection.h"

using namespace std;


class UCMCTrack
{
public:
    UCMCTrack(int max_age_, int min_hits_);
    
    void update(vector<Detection> &detections, int frameCount, ros::Time stamp);

    void linear_assignment( std::vector< std::vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		 std::vector< std::vector<int> > &matches,  std::vector<int> &unmatched_a,  std::vector<int> &unmatched_b);
    
    double lapjv(const std::vector< std::vector<float> > &cost, std::vector<int> &rowsol, std::vector<int> &colsol, 
		bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

    std::vector< std::vector<float> > get_cost_matrix(vector<int> trks_pool, vector<int> dets_pool, int &row_size, int &col_size);

public:
    vector<State3D> _predictions;
private:
    float high_score = 0.75;
    int max_age;
    int min_hits;
    int mahal_thresh = 100;
     vector<Track> trackers;
     vector<int> confirmed_idx;
     vector<int> coasted_idx;
     vector<int> tentative_idx;
     vector<int> detidx_remain;
    UCMC::KalmanFilter *kf;
    Projection *_myproj;

private:
    void data_association(vector<Detection> &detections, ros::Time stamp);
    void associate_tentative(vector<Detection> &detections);
    void initial_tentative(vector<Detection> &detections);
    void delete_old_trackers();
    void update_status(vector<Detection> &detections);
};