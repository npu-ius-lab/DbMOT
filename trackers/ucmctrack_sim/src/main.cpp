#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "dataType.h"
#include "projection.h"
#include "track.h"
#include "tracker.h"
#include "detection.h"

using namespace std;
using namespace darknet_ros_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;


class SubPub
{
public:
    SubPub(ros::NodeHandle &n);

    void callback(const sensor_msgs::Image::ConstPtr &img_ptr,
                  const darknet_ros_msgs::BoundingBoxes::ConstPtr &det_msg);
private:
    int minHits=3;
    int maxAge =30;
    int frameCount = 1;
    vector<Track> trackers;
    vector<Detection> detections;
    UCMCTrack mytracker;
    ros::Publisher pub;
};

SubPub::SubPub(ros::NodeHandle &n) : mytracker(maxAge, minHits)
{
    pub = n.advertise<geometry_msgs::PoseArray>("ucmc_track/kf_predictions", 10);
}

void SubPub::callback(const Image::ConstPtr &img_ptr,
                      const BoundingBoxes::ConstPtr &detect_msg)
{
    ros::Time stamp = img_ptr->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    detections.clear();
    for (int i = 0; i < detect_msg->bounding_boxes.size(); i++)
    {
        auto bbx_msg = detect_msg->bounding_boxes[i];
        if (bbx_msg.probability >= 0.5) {
            cv::Rect2d bbx;
            bbx.x = bbx_msg.xmin;
            bbx.y = bbx_msg.ymin;
            bbx.width = bbx_msg.xmax - bbx_msg.xmin;
            bbx.height = bbx_msg.ymax - bbx_msg.ymin;
            Detection detection;
            detection.score = bbx_msg.probability;
            detection.bbx_tlwh = bbx;
            detections.push_back(detection);
        }
    }
    
    mytracker.update(detections, frameCount, stamp);
    // mytracker.update(detections);
    vector<State3D> predictions = mytracker._predictions;

    geometry_msgs::PoseArray pred_msg;
    pred_msg.header.frame_id = "world";
    pred_msg.header.stamp = detect_msg->header.stamp;
    for (State3D& pred : predictions) {
        Pose pose;
        pose.position.x = pred.x;
        pose.position.y = pred.y;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        pred_msg.poses.push_back(pose);
    }
    pub.publish(pred_msg);

    for (Detection& detection : detections) {
        if (detection.trackid > 0) {
            cv::rectangle(img, detection.bbx_tlwh, cv::Scalar(0,255,0), 2);
            cv::putText(img, cv::format("%d", detection.trackid), cv::Point(
                detection.bbx_tlwh.x, detection.bbx_tlwh.y - 5), 0, 0.6, cv::Scalar(255,255,0), 2, cv::LINE_AA);
                
            std::cout << frameCount << "," << detection.trackid << "," << detection.bbx_tlwh.x << "," 
                << detection.bbx_tlwh.y << "," << detection.bbx_tlwh.width << "," << detection.bbx_tlwh.height <<
                ",1,-1,-1,-1"<< std::endl;
        }
    }
    cv::imshow("tracking", img);
    cv::waitKey(1);
    frameCount++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Tracking");
    ros::NodeHandle n;

    SubPub subpub(n);
    std::cout << "UCMCTrack is ready!" << std::endl;

    Subscriber<Image> image_sub(n, "/yolo/img", 10, ros::TransportHints().tcpNoDelay());
    Subscriber<BoundingBoxes> detect_sub(n, "/yolo/bbx", 10, ros::TransportHints().tcpNoDelay());
    typedef sync_policies::ApproximateTime<Image, BoundingBoxes> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, detect_sub);
    sync.registerCallback(boost::bind(&SubPub::callback, &subpub, _1, _2));

    ros::spin();

    return 0;
}