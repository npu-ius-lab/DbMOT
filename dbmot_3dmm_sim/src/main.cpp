#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/highgui/highgui.hpp>
#include "dataType.h"
#include "detection.h"
#include "iusltracker.h"
#include "track.h"

using namespace std;
using namespace darknet_ros_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;
using namespace visualization_msgs;

class SubPub
{
public:
    SubPub(ros::NodeHandle &n);

    void callback(const sensor_msgs::Image::ConstPtr &img_ptr,
                  const darknet_ros_msgs::BoundingBoxes::ConstPtr &det_msg);
private:
    int frameCount = 1;
    int maxAge = 30;
    int minHits = 3;
    float matchThresh = 0.3;
    vector<Detection> detections;
    IUSLTracker mytracker;
    ros::Publisher pub;
    ros::Publisher pub1;
};

SubPub::SubPub(ros::NodeHandle &n) : mytracker(maxAge, minHits, matchThresh)
{
    
    // pub = n.advertise<geometry_msgs::PoseArray>("iusl_trackers/detection3d", 10);
    pub1 = n.advertise<visualization_msgs::MarkerArray>( "tracking/targets", 10);
}

void SubPub::callback(const Image::ConstPtr &img_ptr,
                      const BoundingBoxes::ConstPtr &detect_msg)
{
    // std::cout << "==========" << frameCount << "===========" << std::endl;
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
            // rectangle(img, bbx, Scalar(0,0,255), 2); //blue
        }
    }
    
    // cout << "det size: " << detections.size() << endl;

    vector<TrackingResult> trkres = mytracker.update(detections, img, frameCount, stamp);

    // geometry_msgs::PoseArray pre_msg;
    // pre_msg.header.frame_id = "world";
    // pre_msg.header.stamp = stamp;
    // for (Detection& p : detections) {
    //     Pose pose;
    //     pose.position.x = p.state3d.x;
    //     pose.position.y = p.state3d.y;
    //     pose.position.z = 0;
    //     pose.orientation.x = 0.0;
    //     pose.orientation.y = 0.0;
    //     pose.orientation.z = 0.0;
    //     pose.orientation.w = 1.0;
    //     pre_msg.poses.push_back(pose);
    // }
    // pub.publish(pre_msg);

    // visualization_msgs::MarkerArray marray;
    // for (pair<int,Point2d>& p : mytracker._predictions) {
    //     visualization_msgs::Marker m;
    //     m.header.frame_id = "world";
    //     m.header.stamp = stamp;
    //     m.id = p.first;
    //     m.type = Marker::TEXT_VIEW_FACING;
    //     m.pose.position.x = p.second.x;
    //     m.pose.position.y = p.second.y;
    //     m.pose.position.z = 0;
    //     m.pose.orientation.x = 0.0;
    //     m.pose.orientation.y = 0.0;
    //     m.pose.orientation.z = 0.0;
    //     m.pose.orientation.w = 1.0;
    //     m.scale.x = 0.3;
    //     m.scale.y = 0.3;
    //     m.scale.z = 0.3;
    //     m.color.r = 0.0f;
    //     m.color.g = 1.0f;
    //     m.color.b = 0.8f;
    //     m.color.a = 1;
    //     m.text = "id=" + to_string(p.first);
    //     m.lifetime = ros::Duration(0.15);
    //     marray.markers.push_back(m);

    //     visualization_msgs::Marker n;
    //     n.header.frame_id = "world";
    //     n.header.stamp = stamp;
    //     n.ns = "target_pos" + to_string(p.first);
    //     n.id = p.first;
    //     n.type = Marker::ARROW;
    //     n.pose.position.x = p.second.x;
    //     n.pose.position.y = p.second.y;
    //     n.pose.position.z = 0;
    //     n.pose.orientation.x = 0.0;
    //     n.pose.orientation.y = 0.0;
    //     n.pose.orientation.z = 0.0;
    //     n.pose.orientation.w = 1.0;
    //     n.scale.x = 0.4;
    //     n.scale.y = 0.04;
    //     n.scale.z = 0.04;
    //     n.color.r = 0.0f;
    //     n.color.g = 1.0f;
    //     n.color.b = 0.0f;
    //     n.color.a = 1;
    //     n.lifetime = ros::Duration(0.15);
    //     marray.markers.push_back(n);
    // }
    // pub1.publish(marray);

    //* vis
    for (int i = 0; i < trkres.size(); i++)
    {
        cv::Rect2d res = detections[trkres[i].detidx].bbx_tlwh;
        cv::Rect2d sz = trkres[i].search_zone;
        cv::Rect2d projsz = trkres[i].proj_sz;
        // rectangle(img, res, Scalar(0,255,0), 2); 
        // putText(img, "id="+to_string(trkres[i].id), cv::Point(res.x, res.y-3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,0),2);

        rectangle(img, sz, Scalar(255,0,0), 2); //blue
        rectangle(img, projsz, Scalar(0,0,255), 2); //red
        rectangle(img, res, Scalar(0,255,0), 1); 
        putText(img, to_string(trkres[i].id), cv::Point(res.x, res.y-3), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,0), 2);
        putText(img, "Frame: "+to_string(frameCount), cv::Point(40, 440), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);

        std::cout << frameCount << "," << trkres[i].id << "," << res.x << "," << res.y << "," 
            << res.width << "," << res.height << ",1,-1,-1,-1" << std::endl;
    } 
    cv::imshow("1", img);
    cv::waitKey(1);
    frameCount++;

    

    // std::string outputDirectory = "";
    // std::string outputFilename = to_string(frameCount) + ".jpg";
    // std::string outputPath = outputDirectory + outputFilename;
    // bool success = cv::imwrite(outputPath, img);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dbmot_3dmm_sim");
    ros::NodeHandle n;
    SubPub subpub(n);
    std::cout << "DbMOT 3D Motion Model for Simulation is Ready!" << std::endl;
    // Subscriber<Odometry> pose_sub(n, "/ground_truth/state", 10);
    Subscriber<Image> image_sub(n, "/yolo/img", 10, ros::TransportHints().tcpNoDelay());
    Subscriber<BoundingBoxes> detect_sub(n, "/yolo/bbx", 10, ros::TransportHints().tcpNoDelay());
    typedef sync_policies::ApproximateTime<Image, BoundingBoxes> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, detect_sub);
    sync.registerCallback(boost::bind(&SubPub::callback, &subpub, _1, _2));

    ros::spin();

    return 0;
}