#include "associate.h"

double GetIoU(Rect2d bb_track, Rect2d bb_detect)
{
    double in = (bb_track & bb_detect).area();           
    double un = bb_track.area() + bb_detect.area() - in; 
    if (un < DBL_EPSILON)    
    {
        return 0; 
    }                      
    return (double)(in / un);
}

double GetEucDis(Rect2d rect1, Rect2d rect2)
{
    double center_x1 = rect1.x + 0.5 * rect1.width;
    double center_y1 = rect1.y + 0.5 * rect1.height;
    double center_x2 = rect2.x + 0.5 * rect2.width;
    double center_y2 = rect2.y + 0.5 * rect2.height;
    double distance = sqrt(pow(center_x1 - center_x2, 2) + pow(center_y1 - center_y2, 2));

    return distance;
}

Associate::Associate(vector<Rect2d> detections, vector<cv::Rect2d> predictions)
{   
    _dets.clear();
    _dets = detections;
    _preds.clear();
    _preds = predictions;

    _trkNum = _preds.size();
    _detNum = _dets.size();
}

void Associate::solve(set<int> &unmatchedDets, set<int> &unmatchedTrks, vector<pair<int, int>> &matched, const float match_thresh)
{
    set<int> allDets, allTrks;
    set<int> matchedDets;
    
    if (_detNum == 0) {
        for (int t=0; t < _trkNum; t++) {
            unmatchedTrks.insert(t);
        }
    } 
    else if (this->_trkNum == 0) { //for the begainning of tracking
        for (int d=0; d < _detNum; d++)
        {
            unmatchedDets.insert(d);
        }
    }
    else 
    {   
        vector<int> assignment;
        HungarianAlgorithm Hungarian;
        vector<vector<double>> costMatrix;
        costMatrix.resize(_trkNum, vector<double>(_detNum, 0.0));

        for (int t = 0; t < _trkNum; t++) {
            for (int d = 0; d < _detNum; d++) {
                costMatrix[t][d] = 1.0 - GetIoU(_preds[t], _dets[d]);
                // costMatrix[t][d] = GetEucDis(_preds[t], _dets[d]);
            }
        }

        if (std::min(costMatrix.size(), costMatrix[0].size()) > 0) {
            // assignment[i]表示与第i个track相匹配的检测索引
            Hungarian.Solve(costMatrix, assignment, false); 
        }
        if (_detNum > _trkNum)
        {
            for (unsigned int n = 0; n < _detNum; n++) {
                allDets.insert(n);
            }
            for (unsigned int i = 0; i < _trkNum; ++i) {
                matchedDets.insert(assignment[i]);
            }
            set_difference(allDets.begin(), allDets.end(),
                matchedDets.begin(), matchedDets.end(),
                insert_iterator<set<int>>(unmatchedDets, unmatchedDets.begin()));
        }
        else if (_detNum < _trkNum)
        {
            for (unsigned int i = 0; i < _trkNum; ++i) {
                if (assignment[i] == -1) {
                    unmatchedTrks.insert(i);
                }
            }
        }
        for (unsigned int i = 0; i < _trkNum; ++i)
        {
            if (assignment[i] == -1) {// pass over invalid values
                continue;
            }
            if (1 - costMatrix[i][assignment[i]] < match_thresh) {
            // if (costMatrix[i][assignment[i]] > match_thresh) {
                unmatchedTrks.insert(i);
                unmatchedDets.insert(assignment[i]); 
            }
            else {
                matched.push_back(std::make_pair(i, assignment[i]));
            }
        }
    }
}
