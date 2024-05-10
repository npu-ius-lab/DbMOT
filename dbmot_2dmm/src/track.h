#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/video/tracking.hpp"
#include "sot/tracking.hpp"

using namespace std;
using namespace cv;

#define StateType cv::Rect2d


class Track
{
public:
	Track(StateType initRect, cv::Mat image)
	{
		init_track(initRect, image);
		m_time_since_update = 0;
		m_hits = 0;
		m_hit_streak = 0;
		m_age = 0;
		m_id = track_count;
		track_count++;
	}

	~Track()
	{
		m_history.clear();
	}

	StateType predict();

	void kf_update(StateType stateMat);
	StateType get_kf_state();
	StateType get_rect_xysr(float cx, float cy, float s, float r);

    bool csr_update(StateType &stateMat, cv::Mat image);
	StateType get_csr_state();
    cv::Size2f get_target_size();
    cv::Size2i get_scale_size();
    cv::Rect2d get_search_zone();

    static int track_count;
	int m_time_since_update;
	int m_hits;
	int m_hit_streak;
	int m_age;
	int m_id;
     int detidx;
     cv::Point2d kf_state2d;

private:
	void init_track(StateType initRect, cv::Mat image);

	cv::KalmanFilter kf;
	cv::Mat measurement;
	std::vector<StateType> m_history;

    cv::Ptr<cv::TrackerCSRT> csrt;
    Rect2d csrt_roi;
    
};