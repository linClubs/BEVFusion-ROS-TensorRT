# `BEVFusion-ROS-TensorRT-CPP`

This repository contains source code and models for BEVFusion online real-time inference using CUDA, TensorRT & ROS.

![](https://github.com/linClubs/BEVFusion-ROS-TensorRT/blob/main/build/cuda-bevfusion.gif)


# 1 依赖安装

+ **`ubuntu-20.04,noetic,cuda-11.3, cudnn-8.6.0, TensorRT-8.5`**

1. 默认已安装`noetic, cuda, cudnn`, 已下载`TensorRT`源码

2. `ros`依赖

~~~python
# 1. 建立ros工作空间
mkdir -p bevfusion_ws/src

# 2. 进入bevfusion_ws/src目录，拉取源码
cd bevfusion_ws/src
git clone https://github.com/linClubs/BEVFusion-ROS-TensorRT.git

# 3. 进入bevfusion_ws工作空间一键安装功能包需要ros依赖
cd ../ 
rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
~~~

3. [模型下载参考](https://github.com/linClubs/BEVFusion-ROS-TensorRT/blob/main/model/readme.md)

4. [模型导出参考](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/master/CUDA-BEVFusion/qat/README.md)

5. `ros`包准备
+ `bevfusion`官方提供了已训练好的`nuscenes`模型参数
+ `nuscenes`传感器之间的参数已给出,无需标定 

如果需接真实的传感器进行场景测试,需提前完成**训练**和**标定**工作

[`nuscenes2rosbag`](https://github.com/linClubs/nuscenes2rosbag)

# 2 编译运行

1. 编译前需要修改`CMakeLists.txt`中`TensorRT`和`CUDA`路径,修改如下

~~~python
...
# cuda
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.3) # CUDA修改这一行
set(CUDA_INSTALL_TARGET_DIR targets/x86_64-linux)
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)
set(CUDA_LIBS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/lib)

# TENSORRT
set(TensorRT_ROOT /home/lin/software/TensorRT-8.5.3.1)  # TensorRT修改这一行
# set(TensorRT_ROOT ~/share/TensorRT-8.5.3.1)           
set(TensorRT_INCLUDE_DIRS ${TensorRT_ROOT}/include)
set(TensorRT_LIBS ${TensorRT_ROOT}/lib/)
...
~~~

2. 编译运行
~~~python
# 1. 编译
catkin_make

# 2. source工作空间
source devel/setup.bash

# 3. 运行bevfusion_node
rosrun bevfusion bevfusion_node
~~~

---

# `BEVFusion-ROS-TensorRT`部分函数介绍

1. 点云`float32`转`float16`

`"points.tensor"` 是 `float16` 数据格式. `c++`浮点数一般用`float`4个字节

~~~c
typedef unsigned short half;
static inline half __internal_float2half(const float f);

// msg转half精度在换成nv::DataType::Float16
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ROI_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // msg转成pcl
    pcl::fromROSMsg(*msg, *ROI_cloud);

    std::cout << "ROI_cloud->points.size()   " << ROI_cloud->points.size() << std::endl;
    
    half *points = new half[ROI_cloud->points.size() * 5];
    
    for (int i = 0; i < ROI_cloud->points.size(); i++)
    {
        points[i * 5 + 0] = __internal_float2half(ROI_cloud->points[i].x);
        points[i * 5 + 1] = __internal_float2half(ROI_cloud->points[i].y);
        points[i * 5 + 2] = __internal_float2half(ROI_cloud->points[i].z);
        points[i * 5 + 3] = __internal_float2half(1);
        points[i * 5 + 4] = __internal_float2half(0);
    }
    vector<int32_t> shape{ROI_cloud->points.size(), 5};

    // Tensor Tensor::from_data_reference(void *data, vector<int32_t> shape, DataType dtype, bool device)

    // float32转float16
    lidar_point_cloud_tensor = nv::Tensor::from_data_reference(points, shape, nv::DataType::Float16, false);
}
~~~

2. opencv读取图像转stb库支持的格式

~~~c
unsigned char* cv2stb(std::string img_path)
{
  cv::Mat image = cv::imread(img_path.c_str(), cv::IMREAD_UNCHANGED);
  int width = image.cols;  // 宽x
  int height = image.rows; // 高y
  int channels = image.channels(); // 通道

  std::vector<unsigned char> buffer;    // 创建一个char类型的数组buffer用来存储图像的data域
  cv::imencode(".jpg", image, buffer); // 编码格式 ""参数可添 .jpg、.png

  // 使用stbi_load函数加载图像数据 width * height * channels = buffer.size()
  unsigned char* stbi_data = stbi_load_from_memory(buffer.data(), buffer.size(), &width, &height, &channels, 0);
  return stbi_data;
}
~~~

3. 运行报错`tool/simhei.ttf`找不到, 全局搜索`tool/simhei.ttf`或者`UseFont`关键字

在`/src/common/visualize.cu`中修改`UseFont`的值即可,改成`simhei.ttf`正确的路径即可

---

# References

+ [bevfusion](https://github.com/mit-han-lab/bevfusion)
+ [Lidar_AI_Solution](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution)
