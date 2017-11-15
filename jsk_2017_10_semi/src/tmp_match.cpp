#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PointStamped.h"
#include "opencv2/highgui/highgui.hpp"

//pointcloud_screenpoint.launch を利用?

class TmpMatch
{
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher point_pub_;
  cv::Rect roi;
  cv::Mat template_img;
  
  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv::Mat in_img = cv_bridge::toCvCopy(msg,msg->encoding)->image;
    cv::Mat src_img = in_img.clone();
    //テンプレートマッチングの実装
    cv::Mat result;
    cv::matchTemplate(src_img,template_img,result,CV_TM_CCORR_NORMED);
    cv::Point max_pt;
    double max_val;
    cv::minMaxLoc(result,0,&max_val,0,&max_pt);
    roi = cv::Rect(cv::Point(max_pt.x,max_pt.y),
		   cv::Point(std::min(static_cast<int>(max_pt.x+template_img.cols),src_img.size().width-1),
			     std::min(static_cast<int>(max_pt.y+template_img.raws),src_img.size().height-1)));
    
    cv::rectangle(src_img,roi,cv::Scalar(0,255,255),2,8,0);
    cv::imshow("result of template matching",src_img);
    cv::waitkey(1);
  }
  
public:
  TmpMatch(ros::NodeHandle nh = ros::NodeHandle()) : it_(nh)
  {
    img_sub_ = it_.subscribe("image",3,&TmpMatch::imageCallback,this);//subscriber
    point_pub_ = it_.advertise<geometry_msgs/PointStamped>("point",3,this);//publisher
    template_img = cv::imread();//read template_img
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tmp_match_node");
  TmpMatch tm;
  ros::spin();
}
