#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "associate.h"
#include "KalmanTracker.h"

using namespace std;
using namespace darknet_ros_msgs;
using namespace sensor_msgs;
using namespace message_filters;

Rect2d weighting(Rect2d detBox, Rect2d predBox)
{
    Rect2d weightedBox;
    weightedBox.x = 0.5 * detBox.x + 0.5 * predBox.x;
    weightedBox.y = 0.5 * detBox.y + 0.5 * predBox.y;
    weightedBox.width = 0.5 * detBox.width + 0.5 * predBox.width;
    weightedBox.height = 0.5 * detBox.height + 0.5 * predBox.height;
    return weightedBox;
}

cv::Rect2d 
tranScale(darknet_ros_msgs::BoundingBox &bbx_msg,
          const int std_w, const int std_h, const int ori_w, const int ori_h)
{
    // double w_ratio = (double)std_w / ori_w;
    // double h_ratio = (double)std_h / ori_h;
    double w_ratio = 1;
    double h_ratio = 1;
    int xmin = int(w_ratio * bbx_msg.xmin);
    int xmax = int(w_ratio * bbx_msg.xmax);
    int ymin = int(h_ratio * bbx_msg.ymin);
    int ymax = int(h_ratio * bbx_msg.ymax);
    cv::Rect2d bbx;
    bbx.x = xmin;
    bbx.y = ymin;
    bbx.width = xmax - xmin;
    bbx.height = ymax - ymin;
    return bbx;
}

class SubPub
{
public:
    SubPub(){};
    SubPub(ros::NodeHandle &n, int max_age, int min_hits);
    ~SubPub(){};
    void callback(const sensor_msgs::Image::ConstPtr &img_ptr,
                  const darknet_ros_msgs::BoundingBoxes::ConstPtr &det_msg);
private:
    int _minHits;
    int _maxAge;
    vector<cv::Rect2d> detections;
    vector<cv::Rect2d> predictedBoxes;
    vector<KalmanTracker> trackers;
    int frameCount = 1;
};


SubPub::SubPub(ros::NodeHandle &n, int max_age, int min_hits) : _maxAge(max_age), _minHits(min_hits)
{

}

void SubPub::callback(const sensor_msgs::Image::ConstPtr &img_ptr,
                    const darknet_ros_msgs::BoundingBoxes::ConstPtr &detect_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    
    detections.clear();
    for (int i = 0; i < detect_msg->bounding_boxes.size(); i++)
    {
        auto bbx_msg = detect_msg->bounding_boxes[i];
        if (bbx_msg.probability >= 0.6) {
            cv::Rect2d bbx = tranScale(bbx_msg, 960, 720, 640, 480);
            detections.push_back(bbx);
        }
    }

    predictedBoxes.clear();
    for (auto it = trackers.begin(); it != trackers.end();)
    {
        cv::Rect2d pBox = (*it).predict();
        if (pBox.x >= 0 && pBox.y >= 0)
        {
            predictedBoxes.push_back(pBox);
            it++;
        }
        else
        {
            it = trackers.erase(it);
        }
    }

    Associate assoc(detections, predictedBoxes);
     vector<pair<int, int>> matches;
     set<int> u_track, u_detection;
    assoc.solve(u_detection, u_track, matches, 0.3);

    for (int idx = 0; idx < matches.size(); idx++) 
    {
        int detIdx = matches[idx].second;
        int trkIdx = matches[idx].first;
        trackers[trkIdx].update(detections[detIdx]);
    }
    for (int idx : u_detection) {
        KalmanTracker tracker = KalmanTracker(detections[idx]);
        trackers.push_back(tracker);
    }

    for (auto it = trackers.begin(); it != trackers.end();)
    {
        if (((*it).m_time_since_update < 1) &&
            ((*it).m_hit_streak >= _minHits || frameCount <= _minHits))
        {
            Rect2d box = (*it).get_state();
            int id = (*it).m_id + 1;
            cv::rectangle(img, box, cv::Scalar(0,255,0), 2);
            cv::putText(img, cv::format("%d", id), cv::Point(
                box.x, box.y-5), 0, 0.6, cv::Scalar(255,255,0), 2, cv::LINE_AA);

            std::cout << frameCount << "," << id <<  "," << box.x << "," 
                << box.y << "," << box.width << "," << box.height << "," << "1,-1,-1,-1" << std::endl;
            
            it++;
        }
        else
            it++;

        // remove dead tracklet
        if (it != trackers.end() && (*it).m_time_since_update > _maxAge)
            it = trackers.erase(it);
    }
    // std::string topContent = "Frame: " + to_string(frame_count);
    // cv::Point topPosition(20, 40);
    // cv::putText(img, topContent, topPosition, FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,0), 2, LINE_AA);

    cv::imshow("tracking", img);
    cv::waitKey(1);
    frameCount++;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sort");
    ros::NodeHandle n;

    bool view_img;
    int max_age;
    int min_hits;
    ros::param::get("~max_age", max_age);
    ros::param::get("~min_hits", min_hits);

    SubPub subpub(n, max_age, min_hits);
    std::cout << "SORT is ready" << std::endl;

    Subscriber<Image> image_sub(n, "/yolo/img", 10);
    Subscriber<BoundingBoxes> detect_sub(n, "/yolo/bbx", 10);

    typedef sync_policies::ApproximateTime<Image, BoundingBoxes> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, detect_sub);
    sync.registerCallback(boost::bind(&SubPub::callback, &subpub, _1, _2));

    ros::spin();

    return 0;
}