#ifndef bevfusion_node_h
#define bevfusion_node_h


#include "bevfusion_plugin.hpp"

#include <ros/ros.h>
// message_filters消息同步器
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h> // 时间接近

// 图像
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

// 点云
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h>


class RosNode : BEVFusionNode
{ 
  ros::NodeHandle n;
  ros::Publisher pub_img;

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud; 
  message_filters::Subscriber<sensor_msgs::Image> sub_f_img; 
  message_filters::Subscriber<sensor_msgs::Image> sub_fl_img; 
  message_filters::Subscriber<sensor_msgs::Image> sub_fr_img; 
  message_filters::Subscriber<sensor_msgs::Image> sub_b_img; 
  message_filters::Subscriber<sensor_msgs::Image> sub_bl_img; 
  message_filters::Subscriber<sensor_msgs::Image> sub_br_img; 

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, 
    sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image,
    sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;
  
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
	std::shared_ptr<Sync> sync;
  
 public:
  RosNode()
  { 
    pub_img = n.advertise<sensor_msgs::Image>("/bevfusion/image_raw", 10);
    sub_cloud.subscribe(n,"/lidar_top", 10);
    sub_f_img.subscribe(n,"/cam_front/raw", 10);
    sub_b_img.subscribe(n,"/cam_back/raw", 10);

    sub_fl_img.subscribe(n,"/cam_front_left/raw", 10);
    sub_fr_img.subscribe(n,"/cam_front_right/raw", 10);
    
    sub_bl_img.subscribe(n,"/cam_back_left/raw", 10);
    sub_br_img.subscribe(n,"/cam_back_right/raw", 10);
    
    sync = std::make_shared<Sync>( MySyncPolicy(10), sub_cloud, 
      sub_f_img,  sub_fl_img, sub_fr_img,
      sub_b_img ,sub_bl_img, sub_br_img); 
    
    sync->registerCallback(boost::bind(&RosNode::callback,this, _1, _2,_3, _4, _5, _6,_7)); // 绑定回调函数
    
  };
  ~RosNode(){};
  void callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud, 
    const sensor_msgs::ImageConstPtr& msg_f_img,
    const sensor_msgs::ImageConstPtr& msg_fl_img,
    const sensor_msgs::ImageConstPtr& msg_fr_img,
    const sensor_msgs::ImageConstPtr& msg_b_img,
    const sensor_msgs::ImageConstPtr& msg_bl_img,
    const sensor_msgs::ImageConstPtr& msg_br_img);

  
};



void  RosNode::callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud, 
  const sensor_msgs::ImageConstPtr& msg_f_img,
  const sensor_msgs::ImageConstPtr& msg_fl_img,
  const sensor_msgs::ImageConstPtr& msg_fr_img,
  const sensor_msgs::ImageConstPtr& msg_b_img,
  const sensor_msgs::ImageConstPtr& msg_bl_img,
  const sensor_msgs::ImageConstPtr& msg_br_img)
{
  
  cv::Mat f_img, fl_img, fr_img, b_img, bl_img, br_img;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  
  pcl::fromROSMsg(*msg_cloud, *cloud_ptr);
  f_img  = cv_bridge::toCvShare(msg_f_img , "bgr8")->image;
  fl_img = cv_bridge::toCvShare(msg_fl_img, "bgr8")->image;
  fr_img = cv_bridge::toCvShare(msg_fr_img, "bgr8")->image;
  b_img  = cv_bridge::toCvShare(msg_b_img , "bgr8")->image;
  bl_img = cv_bridge::toCvShare(msg_bl_img, "bgr8")->image;
  br_img = cv_bridge::toCvShare(msg_br_img, "bgr8")->image;
  
  // 这里可能是ros包中前向左右图搞反了, 所以交换fr_img, fl_img的位置
  std::vector<unsigned char *> images = load_images(f_img, fr_img, fl_img, b_img, bl_img, br_img);
  
  // printf("size: %ld \n", cloud_ptr->points.size());
  
  int lidar_num = cloud_ptr->points.size();
  float lidar_arr[lidar_num * 5];
  for(size_t i = 0; i < cloud_ptr->points.size(); ++i )
  {
    long index = i * 5;
    lidar_arr[index]     = cloud_ptr->points[i].x;
    lidar_arr[index + 1] = cloud_ptr->points[i].y;
    lidar_arr[index + 2] = cloud_ptr->points[i].z;
    lidar_arr[index + 3] = cloud_ptr->points[i].intensity;
    // lidar_arr[index + 4] = cloud->points[i].time;
    lidar_arr[index + 4] = 0;
  }
  Inference(images, lidar_arr, cloud_ptr->points.size());

  cv::Mat img = cv::imread((pkg_path + "/configs/cuda-bevfusion.jpg").c_str());
  cv::resize(img, img, cv::Size(img.size().width /2, img.size().height /2));
  sensor_msgs::Image::Ptr msg_img_new; 
  msg_img_new = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  pub_img.publish(msg_img_new);
}

#endif