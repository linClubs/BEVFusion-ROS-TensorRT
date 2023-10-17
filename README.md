# `BEVFusion-ROS-TensorRT-CPP`

This repository contains source code and models for BEVFusion online real-time inference using CUDA, TensorRT & ROS.

![](configs/cuda-bevfusion.gif)


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
cd .. 
rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
~~~

3. [模型下载参考](https://github.com/linClubs/BEVFusion-ROS-TensorRT/blob/main/model/readme.md)

4. [模型导出参考](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/master/CUDA-BEVFusion/qat/README.md)

+ 修改`./tool/environment.sh`中`cuda tensorrt cudnn`的路径, 运行`./tool/build_trt_engine.sh`生成tensorrt推理模型

~~~python
./tool/build_trt_engine.sh
~~~


5. `ros`包准备

+ `bevfusion`官方提供了已训练好的`nuscenes`模型参数

+ rosbag数据转换参考[`nuscenes2rosbag`功能包](https://github.com/linClubs/nuscenes2rosbag)

+ `nuscenes`传感器之间的参数已给出,无需标定 

如果需接真实的传感器进行场景测试,需提前完成**训练**和**标定**工作

[传感器标定参考](https://github.com/linClubs/Calibration-Is-All-You-Need)


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

+ `bevfusion_node.launch`修改`model_name`与`precision`参数值

`model_name: resnet50/resnet50int8/swint`
`precision:  fp16/int8`
`swint + int8`模式不能工作

~~~python
# 1. 编译
catkin_make

# 2. source工作空间
source devel/setup.bash

# 3. 运行bevfusion_node
roslaunch bevfusion bevfusion_node.launch

# 4. 播放数据集
 rosbag play 103.bag 
~~~

3. 运行报错`tool/simhei.ttf`找不到, 全局搜索`tool/simhei.ttf`或者`UseFont`关键字

在`/src/common/visualize.cu`中修改`UseFont`的值即可,改成`simhei.ttf`正确的路径即可

---

# References

+ [bevfusion](https://github.com/mit-han-lab/bevfusion)
+ [Lidar_AI_Solution](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution)


+ bev感知交流群-472648720, 欢迎各位小伙伴进群一起学习讨论bev相关知识！！！^_^
