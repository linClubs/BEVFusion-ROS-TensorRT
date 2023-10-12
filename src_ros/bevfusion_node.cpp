#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION  // 预处理器宏定义

#include "bevfusion_node.hpp"

int main(int argc, char** argv) 
{  
  ros::init(argc, argv, "bevfusion_node");
  ros::NodeHandle n;
  auto bevfusion_node = std::make_shared<RosNode>();
  ros::spin();
  return 0;
}