
#include "track.h"


int Track::track_count = 0;


// initialize Kalman filter
void Track::init_track(StateType stateMat, cv::Mat image)
{
	int stateNum = 7;
	int measureNum = 4;
	kf = KalmanFilter(stateNum, measureNum, 0);

	measurement = Mat::zeros(measureNum, 1, CV_32F);
    //* F
	kf.transitionMatrix = (Mat_<float>(stateNum, stateNum) <<
		1, 0, 0, 0, 1, 0, 0,
		0, 1, 0, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 0, 1,
		0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 1);
    //* H
	// setIdentity(kf.measurementMatrix);
    kf.measurementMatrix = (Mat_<float>(measureNum, stateNum) <<
        1, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0);
    //* Q
	// setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
    kf.processNoiseCov = (Mat_<float>(stateNum, stateNum) <<
        1.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,
        0.0, 1.0, 0.0, 0.0,  0.0,  0.0,  0.0,
        0.0, 0.0, 1.0, 0.0,  0.0,  0.0,  0.0,
        0.0, 0.0, 0.0, 1.0,  0.0,  0.0,  0.0,
        0.0, 0.0, 0.0, 0.0, 0.01,  0.0,  0.0,
        0.0, 0.0, 0.0, 0.0,  0.0, 0.01,  0.0,
        0.0, 0.0, 0.0, 0.0,  0.0,  0.0, 0.0001);
    //* R
	// setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
    kf.measurementNoiseCov = (Mat_<float>(measureNum, measureNum) <<
        1, 0,  0,  0,
        0, 1,  0,  0,
        0, 0, 10,  0,
        0, 0,  0, 10);
    //* P
	// setIdentity(kf.errorCovPost, Scalar::all(1));
	kf.errorCovPost = (Mat_<float>(stateNum, stateNum) <<
        10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10000.0);

	// initialize state vector with bounding box in [cx,cy,s,r] style
	kf.statePost.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
	kf.statePost.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
	kf.statePost.at<float>(2, 0) = stateMat.area();
	kf.statePost.at<float>(3, 0) = stateMat.width / stateMat.height;

    // this->csrt = TrackerCSRT::create();
    TrackerCSRT::Params params;
    // params.scale_lr = 0.012f;   //0.025f
    // params.scale_step = 1.00f;
    // params.filter_lr = 0.04f;  //0.02f
    params.scale_sigma_factor = 0.36f; //0.36f
    params.number_of_scales = 60; //60
    
    this->csrt = TrackerCSRT::create(params);
    this->csrt->init(image, stateMat);
    this->csrt_roi = stateMat;
}


// Predict the estimated bounding box.
StateType Track::predict()
{
	// predict
	Mat p = kf.predict();
	m_age += 1;

	if (m_time_since_update > 0)
		m_hit_streak = 0;
	m_time_since_update += 1;

	StateType predictBox = get_rect_xysr(p.at<float>(0, 0), p.at<float>(1, 0), p.at<float>(2, 0), p.at<float>(3, 0));

	m_history.push_back(predictBox);
	return m_history.back();
}


// Update the state vector with observed bounding box.
void Track::kf_update(StateType stateMat)
{
	m_time_since_update = 0;
	m_history.clear();
	m_hits += 1;
	m_hit_streak += 1;

	// measurement
	measurement.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
	measurement.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
	measurement.at<float>(2, 0) = stateMat.area();
	measurement.at<float>(3, 0) = stateMat.width / stateMat.height;

	// update
	kf.correct(measurement);
}


// Return the current state vector
StateType Track::get_kf_state()
{
	Mat s = kf.statePost;
	return get_rect_xysr(s.at<float>(0, 0), s.at<float>(1, 0), s.at<float>(2, 0), s.at<float>(3, 0));
}


// Convert bounding box from [cx,cy,s,r] to [x,y,w,h] style.
StateType Track::get_rect_xysr(float cx, float cy, float s, float r)
{
	float w = sqrt(s * r);
	float h = s / w;
	float x = (cx - w / 2);
	float y = (cy - h / 2);

	if (x < 0 && cx > 0)
		x = 0;
	if (y < 0 && cy > 0)
		y = 0;

	return StateType(x, y, w, h);
}

bool Track::csr_update(StateType &stateMat, cv::Mat image)
{
    bool active = csrt->update(image, stateMat);
    if (active)
    {
        this->csrt_roi = stateMat;   
    }

    return active;
}

StateType Track::get_csr_state()
{
    return this->csrt_roi;
}

cv::Size2f Track::get_target_size()
{
    return csrt->get_target_size();
}

cv::Size2i Track::get_scale_size() //* 返回搜索区域的宽高
{
    return csrt->get_scale_size();
}

cv::Rect2d Track::get_search_zone()
{
    return csrt->get_search_zone();
}
