# `BEVFusion-ROS-TensorRT-CPP`

This repository contains source code and models for BEVFusion online real-time inference using CUDA, TensorRT & ROS.

**Supports ROS2**. please switch to the [galactic-devel branch](https://github.com/linClubs/BEVFusion-ROS-TensorRT/tree/galactic-devel)


![](configs/cuda-bevfusion.gif)


# 1 依赖安装

+ **`ubuntu-22.04,ROS 2 humble,cuda-11.3, cudnn-8.6.0, TensorRT-8.5`**

0. build old version of protobuf
   
+ Ubuntu 22.04 inherently uses protobuf 3.12.4, while current version of BEVFusion needs 3.6.1
+ so build the protobuf v3.6.1 by following the instructions in this repo [protobuf 3.6.1](https://github.com/protocolbuffers/protobuf/tree/v3.6.1)
+ then update the [path_of_the_protobuf_built] of line 41 of this repo's [Cmakefile](https://github.com/thirdcat/BEVFusion-ROS-TensorRT/blob/humble-devel/CMakeLists.txt)

1. 默认已安装`humble, cuda, cudnn`, 已下载`TensorRT`源码

2. `ros`依赖

~~~python
# 1. 建立ros工作空间
mkdir -p bevfusion_ws/src

# 2. 进入bevfusion_ws/src目录，拉取源码
cd bevfusion_ws/src
git clone https://github.com/linClubs/BEVFusion-ROS-TensorRT.git 

# 3. 切换galactic-devel分支
git branch humble-devel

# 4. 进入bevfusion_ws工作空间一键安装功能包需要ros依赖
cd ../ 
rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
~~~

3. [模型下载参考](https://github.com/linClubs/BEVFusion-ROS-TensorRT/blob/main/model/readme.md)

4. [模型导出参考](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/blob/master/CUDA-BEVFusion/qat/README.md)

5. `rosbag`准备

+ `bevfusion`官方提供了已训练好的`nuscenes`模型参数
+ `nuscenes`传感器之间的参数已给出,无需标定 

如果需接真实的传感器进行场景测试,需提前完成**训练**和**标定**工作

[传感器标定参考](https://github.com/linClubs/Calibration-Is-All-You-Need)


[`nuscenes2rosbag`](https://github.com/linClubs/nuscenes2rosbag)

+ 上面生成的是`ros1`的`bag`，直接使用`rosbags`转化成`ros2`的`bag`

~~~python
# 1. install rosbags
pip install rosbags

# 2. ros1 bag convert to ros2 bag, result will be "nuscenes-103/"
rosbags-convert nuscenes-103.bag
~~~



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
set(TensorRT_ROOT ~/software/TensorRT-8.5.3.1)  # TensorRT修改这一行
# set(TensorRT_ROOT ~/share/TensorRT-8.5.3.1)           
set(TensorRT_INCLUDE_DIRS ${TensorRT_ROOT}/include)
set(TensorRT_LIBS ${TensorRT_ROOT}/lib/)
...
~~~

2. 编译运行

+ `bevfusion.launch.py`修改`model_name`与`precision`参数值
~~~python
# model_name: resnet50/resnet50int8/swint
# precision:  fp16/int8
# swint + int8模式不能工作
parameters=[
			{'model_name': 'resnet50'},
			{'precision' : 'int16'}
		]
~~~

~~~python
# 1. 编译
colcon build --symlink-install

# 2. source工作空间
source install/setup.bash

# 3. 运行bevfusion_node
ros2 launch bevfusion bevfusion.launch.py

# 4. 播放数据集
 ros2 bag play nuscenes-103.db3

# 5 rviz2结果显示
rviz2 -d src/BEVFusion-ROS-TensorRT/launch/view.rviz
~~~

---

3. 错误修改
+ 报错1 运行报错`tool/simhei.ttf`找不到, 全局搜索`tool/simhei.ttf`或者`UseFont`关键字

修改：在`/src/common/visualize.cu`中修改`UseFont`的值即可,改成`simhei.ttf`正确的路径即可


+ 报错2 运行`ros2 run bevfusion bevfusion_node` 
`error while loading shared libraries: libspconv.so: cannot open shared object file: No such file or directory`

修改：
~~~python
# 查看LD_LIBRARY_PATH路径
echo $LD_LIBRARY_PATH

# 如果没有libspconv.so路径就按照下面代码添加即可
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/third_party/3DSparseConvolution/libspconv/lib/x86_64
~~~

---

# References

+ [bevfusion](https://github.com/mit-han-lab/bevfusion)
+ [Lidar_AI_Solution](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution)


+ bev感知交流群-472648720, 欢迎各位小伙伴进群一起学习讨论bev相关知识！！！^_^
