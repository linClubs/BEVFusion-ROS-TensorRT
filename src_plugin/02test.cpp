
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION  // 预处理器宏定义

#include "bevfusion_plugin.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) 
{ 
  std::string model_name = "resnet50int8";
  std::string precision  = "int8"; 

  std::string pkg_path;
  std::string data_path, cloud_path;
  // 数据集路径
  pkg_path = "/home/lin/ros2_code/bevfusion_ws/src/BEVFusion-ROS-TensorRT";
  data_path = pkg_path + "/example-data";
  cloud_path = pkg_path + "/example-data/test_pcd.pcd";


  // 1. opencv读取本地图像数据 Load image and lidar to host
  cv::Mat f_img, fl_img, fr_img, b_img, bl_img, br_img;
  f_img = cv::imread(data_path + "/0-FRONT.jpg", cv::IMREAD_UNCHANGED);
  fl_img = cv::imread(data_path + "/1-FRONT_RIGHT.jpg", cv::IMREAD_UNCHANGED);
  fr_img = cv::imread(data_path + "/2-FRONT_LEFT.jpg", cv::IMREAD_UNCHANGED);
  b_img = cv::imread(data_path + "/3-BACK.jpg", cv::IMREAD_UNCHANGED);
  bl_img = cv::imread(data_path + "/4-BACK_LEFT.jpg", cv::IMREAD_UNCHANGED);
  br_img = cv::imread(data_path + "/5-BACK_RIGHT.jpg", cv::IMREAD_UNCHANGED);
  std::vector<unsigned char *> images = load_images(f_img, fl_img, fr_img, b_img, bl_img, br_img);

  // 2. pcl本地读取雷达数据
  pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>());
  pcl::io::loadPCDFile<PointT>(cloud_path, *cloud_ptr);

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

  printf("Start Detect...\n");
  // 3. 推理
  auto bevfusion_node = std::make_shared<BEVFusionNode>(model_name, precision, pkg_path);
  bevfusion_node->Inference(images, lidar_arr, lidar_num);
  
  return 0;

}