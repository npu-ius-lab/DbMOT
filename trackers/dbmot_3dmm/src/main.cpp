#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "dataType.h"
#include "track.h"
#include "tracker.h"
#include "projection.h"

using namespace std;
using namespace darknet_ros_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;
using namespace visualization_msgs;


cv::Rect2d 
tranScale(darknet_ros_msgs::BoundingBox &bbx_msg,
        const int std_w, const int std_h, const int ori_w, const int ori_h)
{
    double w_ratio = (double)std_w / ori_w;
    double h_ratio = (double)std_h / ori_h;
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
    SubPub(ros::NodeHandle &n);
    ~SubPub(){};
    void callback(const geometry_msgs::PoseStamped::ConstPtr &drone_pose, 
                  const sensor_msgs::Image::ConstPtr &img_ptr,
                  const darknet_ros_msgs::BoundingBoxes::ConstPtr &det_msg);
private:

    int frame_count = 0;
    vector<DETECTION> detections;
    myTracker mytracker;
    ros::Publisher pub;
    ros::Publisher img_pub;
};

SubPub::SubPub(ros::NodeHandle &n)
{
    std::cout << "dbmot_3dmm is ready!" << std::endl;
    myTracker mytracker();
    pub = n.advertise<visualization_msgs::MarkerArray>( "kalman_predictions", 10);
    img_pub = n.advertise<sensor_msgs::Image>("tracking_result", 10);
}

void SubPub::callback(const PoseStamped::ConstPtr &pose_msg,
                        const Image::ConstPtr &img_ptr,
                      const BoundingBoxes::ConstPtr &detect_msg)
{
    frame_count++;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    Eigen::Quaterniond q_ex_1(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x,
                              pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
    Eigen::Vector3d t_ex_1(pose_msg->pose.position.x, pose_msg->pose.position.y,
                           pose_msg->pose.position.z);
    Projection proj_func(t_ex_1, q_ex_1);

    detections.clear();
    for (int i = 0; i < detect_msg->bounding_boxes.size(); i++)
    {
        auto bbx_msg = detect_msg->bounding_boxes[i];
        cv::Rect2d bbx = tranScale(bbx_msg, 960, 720, 640, 480); 
        PROJ_RESULT proj_res = proj_func.pixel2world(bbx);
        DETECTION detection;
        detection.score = bbx_msg.probability;
        detection.boundingbox = bbx;
        detection.distribution = proj_res;
        detections.push_back(detection);
        rectangle(img, bbx, cv::Scalar(0,255,255), 2);
    }

    vector<Track> results;
    results = mytracker.update(detections, img, proj_func, frame_count);
    
    for (Track& track : results) {
        int& id = track.id;
        int detidx = track.detidx;
        State2D det_bbx = detections[detidx].boundingbox;

        cv::rectangle(img, det_bbx, cv::Scalar(0,255,255), 2);
        cv::putText(img, cv::format("%d", id), cv::Point(
            det_bbx.x, det_bbx.y-5), 0, 0.6, cv::Scalar(255,255,0), 2, cv::LINE_AA);

        std::cout << frame_count << "," << id << "," << det_bbx.x << "," 
            << det_bbx.y << "," << det_bbx.width << "," << det_bbx.height <<",1,-1,-1,-1"<< std::endl;
    
        //* csr search region
        cv::Rect2d sz = track.get_search_zone();
        rectangle(img, sz, Scalar(255,0,0), 2);
        // * kf search region
        cv::Rect2d proj_sz (track.kf_state2d.x - 0.5*track.get_scale_size().width,
                            track.kf_state2d.y - 0.5*track.get_scale_size().height,
                            track.get_scale_size().width,
                            track.get_scale_size().height);
        rectangle(img, proj_sz, Scalar(0,0,255), 2);

        putText(img, "Frame: "+to_string(frame_count), cv::Point(60, 660), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
    }
    
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = img_ptr->header.stamp;
    cv_image.encoding = "bgr8";
    cv_image.image = img;
    img_pub.publish(cv_image.toImageMsg());

    //* kf prediction demonstration
    visualization_msgs::MarkerArray marray;
    for (Track& track : results) 
    {
        cv::Point2d kf_pred = track.kf_prediction;

        visualization_msgs::Marker m;
        m.header.frame_id = "world";
        m.header.stamp = img_ptr->header.stamp;
        m.id = track.id;
        m.type = Marker::TEXT_VIEW_FACING;
        m.pose.position.x = kf_pred.x;
        m.pose.position.y = kf_pred.y;
        m.pose.position.z = 0;
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.3;
        m.scale.y = 0.3;
        m.scale.z = 0.3;
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.8f;
        m.color.a = 1;
        m.text = "id=" + to_string(track.id);
        m.lifetime = ros::Duration(0.15);
        marray.markers.push_back(m);

        visualization_msgs::Marker n;
        n.header.frame_id = "world";
        n.header.stamp = img_ptr->header.stamp;
        n.ns = "target_pos" + to_string(track.id);
        n.id = track.id;
        n.type = Marker::ARROW;
        n.pose.position.x = kf_pred.x;
        n.pose.position.y = kf_pred.y;
        n.pose.position.z = 0;
        n.pose.orientation.x = 0.0;
        n.pose.orientation.y = 0.0;
        n.pose.orientation.z = 0.0;
        n.pose.orientation.w = 1.0;
        n.scale.x = 0.6;
        n.scale.y = 0.06;
        n.scale.z = 0.06;
        n.color.r = 0.0f;
        n.color.g = 1.0f;
        n.color.b = 0.0f;
        n.color.a = 1;
        n.lifetime = ros::Duration(0.15);
        marray.markers.push_back(n);
    }
    pub.publish(marray);

    // std::string outputDirectory = "";
    // std::string outputFilename = to_string(frame_count) + ".jpg";
    // std::string outputPath = outputDirectory + outputFilename;
    // bool success = cv::imwrite(outputPath, img);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dbmot_3dmm");
    ros::NodeHandle n;

    SubPub subpub(n);
    Subscriber<PoseStamped> pose_sub(n, "/vrpn_client_node/rmtt_02/pose", 10, ros::TransportHints().tcpNoDelay());
    Subscriber<Image> image_sub(n, "/yolo/img", 10, ros::TransportHints().tcpNoDelay());
    Subscriber<BoundingBoxes> detect_sub(n, "/yolo/bbx", 10, ros::TransportHints().tcpNoDelay());

    typedef sync_policies::ApproximateTime<PoseStamped, Image, BoundingBoxes> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), pose_sub, image_sub, detect_sub);
    sync.registerCallback(boost::bind(&SubPub::callback, &subpub, _1, _2, _3));

    ros::spin();

    return 0;
}