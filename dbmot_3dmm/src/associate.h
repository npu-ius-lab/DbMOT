#pragma once

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <set>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Hungarian.h"

using namespace cv;
using namespace std;

class Associate
{
public:
    Associate(vector<cv::Rect2d> detections, vector<cv::Rect2d> trackers);
    void solve(set<int> &unmatchedDets, set<int> &unmatchedTrks, vector<pair<int,int>> &matched, const float match_thresh);
    
private:
    int _detNum;
    int _trkNum;
    vector<Rect2d> _dets, _preds;

};