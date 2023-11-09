#include "bevfusion_node.hpp"

int main(int argc, char** argv) 
{  
  
  rclcpp::init(argc, argv);
  auto param_node = std::make_shared<rclcpp::Node>("param_node");
  
  std::string model_name, precision, pkg_path; 
  
  // model_name ;// = "resnet50";
  // precision  ;// = "fp16"; 
  pkg_path = "src/BEVFusion-ROS-TensorRT";

  // 声明参数
  param_node->declare_parameter<std::string>("model_name", "resnet50");
  
  // 从参数列表更新参数
  param_node->get_parameter("model_name", model_name);

  param_node->declare_parameter<std::string>("precision", "fp16");
  param_node->get_parameter("precision", precision);

  pkg_path = ament_index_cpp::get_package_share_directory("bevfusion") + "/../../../../src/BEVFusion-ROS-TensorRT";  

  std::cout << "\033[1;32m--pkg_path: "   << pkg_path   << "\033[0m" << std::endl;
  std::cout << "\033[1;32m--model_name: " << model_name << "\033[0m" << std::endl;
  std::cout << "\033[1;32m--precision : " << precision  << "\033[0m" << std::endl;

  auto bevfusion_node = std::make_shared<RosNode>(model_name, precision, pkg_path);
  rclcpp::spin(bevfusion_node);
  rclcpp::shutdown();
  return 0;
}