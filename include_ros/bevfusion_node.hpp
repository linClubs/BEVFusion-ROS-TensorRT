#ifndef bevfusion_node_h
#define bevfusion_node_h

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// message_filters消息同步器
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h> // 时间接近

// 图像
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// 点云
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include "bevfusion_plugin.hpp"

class RosNode : public rclcpp::Node
{ 
  
  std::string model_name_, precision_, pkg_path_;
  
  // 定义一个BEVFusion检测插件
  std::shared_ptr<BEVFusionNode> bevfusion_plugin_;

  using ImageMsg = sensor_msgs::msg::Image;
  using PCMsg = sensor_msgs::msg::PointCloud2;

  typedef message_filters::sync_policies::ApproximateTime<
  ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, PCMsg> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;
    
  std::shared_ptr<message_filters::Subscriber<ImageMsg> > sub_img_f_;
  std::shared_ptr<message_filters::Subscriber<ImageMsg> > sub_img_fl_;
  std::shared_ptr<message_filters::Subscriber<ImageMsg> > sub_img_fr_;
  std::shared_ptr<message_filters::Subscriber<ImageMsg> > sub_img_b_;
  std::shared_ptr<message_filters::Subscriber<ImageMsg> > sub_img_bl_;
  std::shared_ptr<message_filters::Subscriber<ImageMsg> > sub_img_br_;
  
  std::shared_ptr<message_filters::Subscriber<PCMsg> > sub_cloud_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
  

 public:
  RosNode(const std::string &model_name, const std::string &precision, const std::string &pkg_path);
  ~RosNode(){};
  void callback (
    const ImageMsg::SharedPtr msg_img_f,
    const ImageMsg::SharedPtr msg_img_fl,
    const ImageMsg::SharedPtr msg_img_fr,
    const ImageMsg::SharedPtr msg_img_b,
    const ImageMsg::SharedPtr msg_img_bl,
    const ImageMsg::SharedPtr msg_img_br,
    const PCMsg msg_cloud);

};
#endif