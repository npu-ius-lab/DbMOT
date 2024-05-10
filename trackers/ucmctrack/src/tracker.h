#ifndef TRACKER_H
#define TRACKER_H

#include <vector>
#include "dataType.h"
#include "track.h"
#include "kalmanfilter.h"
#include "detection.h"

using namespace std;

class UCMCTrack
{
public:
    UCMCTrack(int max_age_=30, int min_hits_=3);
    
    vector<track_data> update(vector<Detection> &detections, int frame_count);

    void linear_assignment( std::vector< std::vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		 std::vector< std::vector<int> > &matches,  std::vector<int> &unmatched_a,  std::vector<int> &unmatched_b);
    
    double lapjv(const std::vector< std::vector<float> > &cost, std::vector<int> &rowsol, std::vector<int> &colsol, 
		bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

    std::vector< std::vector<float> > get_cost_matrix(vector<int> trks_pool, vector<int> dets_pool, int &row_size, int &col_size);

private:
    float high_score = 0.75;
    int max_age = 30;
    int min_hits = 3;
    int mahal_thresh = 6000;
     vector<Track> trackers;
     vector<int> confirmed_idx;
     vector<int> coasted_idx;
     vector<int> tentative_idx;
     vector<int> detidx_remain;
    UCMC::KalmanFilter *kf;

private:
    void data_association(vector<Detection> &detections);
    void associate_tentative(vector<Detection> &detections);
    void initial_tentative(vector<Detection> &detections);
    void delete_old_trackers();
    void update_status(vector<Detection> &detections);
};


#endif // TRACKER_H