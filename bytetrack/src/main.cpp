#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "BYTETracker.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace darknet_ros_msgs;


cv::Rect_<float>
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
    cv::Rect_<float> bbx;
    bbx.x = xmin;
    bbx.y = ymin;
    bbx.width = xmax - xmin;
    bbx.height = ymax - ymin;
    return bbx;
}


class SubPub
{
public:
    SubPub(ros::NodeHandle &n, bool view_img, int max_age, int min_hits);
    ~SubPub(){};
    void callback(const sensor_msgs::Image::ConstPtr &img_ptr,
                  const darknet_ros_msgs::BoundingBoxes::ConstPtr &det_msg);

private:
    ros::Publisher pub;
    bool viewImg;
    int minHits;
    int maxAge;
    int frame_count = 1;    
    BYTETracker mytracker;
};


SubPub::SubPub(ros::NodeHandle &n, bool view_img, int max_age, int min_hits) : viewImg(view_img), maxAge(max_age), minHits(min_hits)
{
    BYTETracker mytracker(maxAge);
}    

void SubPub::callback(const sensor_msgs::Image::ConstPtr &img_ptr,
                    const darknet_ros_msgs::BoundingBoxes::ConstPtr &detect_msg)
{
    ros::Time timeStamp = std::min(detect_msg->header.stamp, img_ptr->header.stamp);
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    
    std::vector<detect_result> objects;
    for (int i = 0; i < detect_msg->bounding_boxes.size(); i++)
    {
        auto bbx_msg = detect_msg->bounding_boxes[i];
        if (bbx_msg.probability >= 0.5) {
            cv::Rect_<float> bbx = tranScale(bbx_msg, 960, 720, 640, 480);
            detect_result detres;
            detres.confidence = bbx_msg.probability;
            detres.box.x = bbx.x;
            detres.box.y = bbx.y;
            detres.box.width = bbx.width;
            detres.box.height = bbx.height;
            objects.push_back(detres);
        }
    }
    std::vector<STrack> output_stracks = mytracker.update(objects);
    for (unsigned long i = 0; i < output_stracks.size(); i++)
    {
        std::vector<float> tlwh = output_stracks[i].tlwh;
        bool vertical = tlwh[2] / tlwh[3] > 1.6;
        if (tlwh[2] * tlwh[3] > 20 && !vertical)
        {
            int id = output_stracks[i].track_id;
            // cv::Scalar s = mytracker.get_color(output_stracks[i].track_id);
            cv::putText(img, cv::format("%d", id), cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6, cv::Scalar(255,255,0), 2, cv::LINE_AA);
            cv::rectangle(img, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), cv::Scalar(0,255,0), 2);
            std::cout << frame_count << "," << id << "," << tlwh[0] 
                << "," << tlwh[1] << "," << tlwh[2] << "," << tlwh[3] << "," << "1,-1,-1,-1"  << std::endl;
        }
    }

    // std::string topContent = "Frame: " + to_string(frame_count);
    // cv::Point topPosition(20, 40);
    // cv::putText(img, topContent, topPosition, cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,0), 2, cv::LINE_AA);

    cv::imshow("tracking", img);
    cv::waitKey(1);
    frame_count++;

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "sort");
    ros::NodeHandle n;

    bool view_img;
    int max_age;
    int min_hits;
    ros::param::get("~view_image", view_img);
    ros::param::get("~max_age", max_age);
    ros::param::get("~min_hits", min_hits);

    SubPub subpub(n, view_img, max_age, min_hits);
    std::cout << "ByteTrack is ready" << std::endl;

    Subscriber<Image> image_sub(n, "/yolo/img", 10);
    Subscriber<BoundingBoxes> detect_sub(n, "/yolo/bbx", 10);

    typedef sync_policies::ApproximateTime<Image, BoundingBoxes> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, detect_sub);
    sync.registerCallback(boost::bind(&SubPub::callback, &subpub, _1, _2));

    ros::spin();

    return 0;
}
