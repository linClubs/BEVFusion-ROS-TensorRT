#include "bevfusion_ros.hpp"

int main(int argc, char** argv) 
{  
  ros::init(argc, argv, "bevfusion_node");
  ros::NodeHandle n;
  std::string model_name;
  std::string  precision; 
    
  n.param<std::string>("model_name", model_name, "resnet50int8");
  n.param<std::string>("precision",  precision, "int8");
  
  std::cout << "\033[1;32m--model_name: " << model_name << "\033[0m" << std::endl;
  std::cout << "\033[1;32m--precision : " << precision << "\033[0m" << std::endl;

  auto bevfusion_node = std::make_shared<RosNode>(model_name, precision);
  ros::spin();
  return 0;
}