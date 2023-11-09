#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION  // 预处理器宏定义

#include "bevfusion_node.hpp"

RosNode::RosNode(const std::string &model_name, const std::string &precision, const std::string &pkg_path) 
: Node("bevfusion_node"), pkg_path_(pkg_path), model_name_(model_name), precision_(precision)
{ 
  
  bevfusion_plugin_.reset(new BEVFusionNode(model_name_, precision_, pkg_path_));

  pub_img_ = this->create_publisher<ImageMsg>("/result_image", 10);

  sub_img_f_  = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/cam_front/raw");
  sub_img_fl_ = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/cam_front_left/raw");
  sub_img_fr_ = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/cam_front_right/raw");
  sub_img_b_  = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/cam_back/raw");
  sub_img_bl_ = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/cam_back_left/raw");
  sub_img_br_ = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/cam_back_left/raw");
  sub_cloud_  = std::make_shared<message_filters::Subscriber<PCMsg> >(this, "/lidar_top"); 

  sync_ = std::make_shared<Sync>(MySyncPolicy(10), *sub_img_f_, *sub_img_fl_, *sub_img_fr_ ,*sub_img_b_, *sub_img_bl_, *sub_img_br_, *sub_cloud_);
  sync_->registerCallback(&RosNode::callback, this);
}

void RosNode::callback(
    const ImageMsg::SharedPtr msg_img_f,
    const ImageMsg::SharedPtr msg_img_fl,
    const ImageMsg::SharedPtr msg_img_fr,
    const ImageMsg::SharedPtr msg_img_b,
    const ImageMsg::SharedPtr msg_img_bl,
    const ImageMsg::SharedPtr msg_img_br,
    const PCMsg msg_cloud)
{
  // RCLCPP_INFO(this->get_logger(), "start");
  cv::Mat img_f, img_fl, img_fr, img_b, img_bl, img_br;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  img_f  = cv_bridge::toCvShare(msg_img_f,  "bgr8")->image;
  img_fl = cv_bridge::toCvShare(msg_img_fl, "bgr8")->image;
  img_fr = cv_bridge::toCvShare(msg_img_fr, "bgr8")->image;
  img_b  = cv_bridge::toCvShare(msg_img_b,  "bgr8")->image;
  img_bl = cv_bridge::toCvShare(msg_img_bl, "bgr8")->image;
  img_br = cv_bridge::toCvShare(msg_img_br, "bgr8")->image;


  pcl::fromROSMsg(msg_cloud, *cloud);
  // std::cout <<  img_br.size() << std::endl;
  // std::cout << cloud->points.size() << std::endl;

  // 这里可能是ros包中前向左右图搞反了, 所以交换fr_img, fl_img的位置
  std::vector<unsigned char *> images = load_images(img_f, img_fr, img_fl, img_b, img_bl, img_br);
  
  int lidar_num = cloud->points.size();
  float lidar_arr[lidar_num * 5];
  for(size_t i = 0; i < cloud->points.size(); ++i )
  {
    long index = i * 5;
    lidar_arr[index]     = cloud->points[i].x;
    lidar_arr[index + 1] = cloud->points[i].y;
    lidar_arr[index + 2] = cloud->points[i].z;
    lidar_arr[index + 3] = cloud->points[i].intensity;
    // lidar_arr[index + 4] = cloud->points[i].time;
    lidar_arr[index + 4] = 0;
  }

  bevfusion_plugin_->Inference(images, lidar_arr, cloud->points.size());

  cv::Mat img = cv::imread((pkg_path_ + "/configs/cuda-bevfusion.jpg").c_str());
  cv::resize(img, img, cv::Size(img.size().width /2, img.size().height /2));
  ImageMsg::SharedPtr msg_img_new; 
  
  msg_img_new = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  
  pub_img_->publish(*msg_img_new);
}