#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "associate.h"
#include "track.h"

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
    vector<Track> trackers;
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
        (*it).detidx = -1;
        cv::Rect2d kfPred = (*it).predict();
        it->kf_state2d.x = kfPred.x + 0.5*kfPred.width;
        it->kf_state2d.y = kfPred.y + 0.5*kfPred.height;

        if (kfPred.x >= 0 && kfPred.y >= 0)
        {
            bool psrGate = (*it).csr_update(kfPred, img);
            if (psrGate) {
                cv::Rect2d csrPred = kfPred;
                predictedBoxes.push_back(csrPred);    
            }
            else {
                predictedBoxes.push_back(kfPred);
            }
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
    assoc.solve(u_detection, u_track, matches, 0.1);

    for (int idx = 0; idx < matches.size(); idx++) 
    {
        int detIdx = matches[idx].second;
        int trkIdx = matches[idx].first;
        trackers[trkIdx].kf_update(detections[detIdx]);
        trackers[trkIdx].detidx = detIdx;
    }
    for (int idx : u_detection) {
        Track new_tracker = Track(detections[idx], img);
        new_tracker.detidx = idx;
        trackers.push_back(new_tracker);
    }

    for (auto it = trackers.begin(); it != trackers.end();)
    {
        if (((*it).m_time_since_update < 1) &&
            ((*it).m_hit_streak >= _minHits || frameCount <= _minHits))
        {
            Rect2d box = detections[(*it).detidx];
            int id = (*it).m_id + 1;
            // cv::rectangle(img, box, cv::Scalar(0,255,0), 2);
            // cv::putText(img, cv::format("%d", id), cv::Point(
            //     box.x, box.y-5), 0, 0.6, cv::Scalar(255,255,0), 2, cv::LINE_AA);

            std::cout << frameCount << "," << id <<  "," << box.x << "," 
                << box.y << "," << box.width << "," << box.height << "," << "1,-1,-1,-1" << std::endl;
            
            //* csr search region
            // cv::Rect2d sz = it->get_search_zone();
            // rectangle(img, sz, Scalar(255,0,0), 2);
            //* kf search region
            // cv::Rect2d proj_sz (it->kf_state2d.x - 0.5*it->get_scale_size().width,
            //                     it->kf_state2d.y - 0.5*it->get_scale_size().height,
            //                     it->get_scale_size().width,
            //                     it->get_scale_size().height);
            // rectangle(img, proj_sz, Scalar(0,0,255), 2);


            it++;
        }
        else
            it++;

        // remove dead tracklet
        if (it != trackers.end() && (*it).m_time_since_update > _maxAge) {
            it = trackers.erase(it);
        }
            
    }
    // std::string topContent = "Frame: " + to_string(frame_count);
    // cv::Point topPosition(20, 40);
    // cv::putText(img, topContent, topPosition, FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,0), 2, LINE_AA);
    
    // std::string outputDirectory = "/home/fbh/img_329/2dmm_iou_fusion/";
    // std::string outputFilename = to_string(frameCount) + ".jpg";
    // std::string outputPath = outputDirectory + outputFilename;
    // bool success = cv::imwrite(outputPath, img);

    // cv::imshow("tracking", img);
    // cv::waitKey(1);
    frameCount++;
    

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dbmot_2dmm");
    ros::NodeHandle n;

    bool view_img;
    int max_age;
    int min_hits;
    ros::param::get("~max_age", max_age);
    ros::param::get("~min_hits", min_hits);

    SubPub subpub(n, max_age, min_hits);
    std::cout << "dbmot_2dmm is ready" << std::endl;

    Subscriber<Image> image_sub(n, "/yolo/img", 10);
    Subscriber<BoundingBoxes> detect_sub(n, "/yolo/bbx", 10);

    typedef sync_policies::ApproximateTime<Image, BoundingBoxes> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, detect_sub);
    sync.registerCallback(boost::bind(&SubPub::callback, &subpub, _1, _2));

    ros::spin();

    return 0;
}