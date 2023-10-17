#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION  // 预处理器宏定义

#include "bevfusion_ros.hpp"

RosNode::RosNode(const std::string model_name, const std::string  precision)
  : model_name_(model_name), precition_(precision)
{ 
  
  bevfusion_node_.reset(new BEVFusionNode(model_name_, precition_));
  getTopicName();
  pub_img_ = n_.advertise<sensor_msgs::Image>("/bevfusion/image_raw", 10);
  sub_cloud_.subscribe(n_, topic_cloud_, 10);
  sub_f_img_.subscribe(n_, topic_img_f_, 10);
  sub_b_img_.subscribe(n_, topic_img_b_, 10);

  sub_fl_img_.subscribe(n_,topic_img_fl_, 10);
  sub_fr_img_.subscribe(n_,topic_img_fr_, 10);
  
  sub_bl_img_.subscribe(n_,topic_img_bl_, 10);
  sub_br_img_.subscribe(n_,topic_img_br_, 10);
  
  sync_ = std::make_shared<Sync>( MySyncPolicy(10), sub_cloud_, 
    sub_f_img_, sub_fl_img_, sub_fr_img_,
    sub_b_img_ ,sub_bl_img_, sub_br_img_); 
  
  sync_->registerCallback(boost::bind(&RosNode::callback,this, _1, _2,_3, _4, _5, _6,_7)); // 绑定回调函数
  
  }

void RosNode::getTopicName()
{
  n_.param<std::string>("topic_cloud", topic_cloud_, "/lidar_top");
  
  n_.param<std::string>("topic_img_f", topic_img_f_, "/cam_front/raw");
  n_.param<std::string>("topic_img_b", topic_img_b_, "/cam_back/raw");
  
  n_.param<std::string>("topic_img_fl", topic_img_fl_, "/cam_front_left/raw");
  n_.param<std::string>("topic_img_fr", topic_img_fr_, "/cam_front_right/raw");
  
  n_.param<std::string>("topic_img_bl", topic_img_bl_, "/cam_back_left/raw");
  n_.param<std::string>("topic_img_br", topic_img_br_, "/cam_back_right/raw");
}


void RosNode::callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud, 
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
  bevfusion_node_->Inference(images, lidar_arr, cloud_ptr->points.size());

  cv::Mat img = cv::imread((pkg_path + "/configs/cuda-bevfusion.jpg").c_str());
  cv::resize(img, img, cv::Size(img.size().width /2, img.size().height /2));
  sensor_msgs::Image::Ptr msg_img_new; 
  msg_img_new = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  pub_img_.publish(msg_img_new);
}
