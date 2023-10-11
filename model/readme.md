# 模型文件下载

[文档参考](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/master/CUDA-BEVFusion)
- 下载模型  ( [Google Drive](https://drive.google.com/file/d/1bPt3D07yyVuSuzRAHySZVR2N15RqGHHN/view?usp=sharing) ) or ( [Baidu Drive](https://pan.baidu.com/s/1_6IJTzKlJ8H62W5cUPiSbA?pwd=g6b4) )
- 下载测试数据集 ( [Google Drive](https://drive.google.com/file/d/1RO493RSWyXbyS12yWk5ZzrixAeZQSnL8/view?usp=sharing) ) or ( [Baidu Drive](https://pan.baidu.com/s/1ED6eospSIF8oIQ2unU9WIQ?pwd=mtvt) )


1. 模型下载解压后复制到`BEVFusion-ROS-TensorRT`目录中，目录结果如下：

~~~c
(base) lin@PC:src/BEVFusion-ROS-TensorRT/model$ tree
.
├── resnet50
│   ├── bevfusion-det.pth
│   ├── camera.backbone.onnx
│   ├── camera.vtransform.onnx
│   ├── default.yaml
│   ├── fuser.onnx
│   ├── head.bbox.onnx
│   └── lidar.backbone.xyz.onnx
├── resnet50int8
│   ├── bevfusion_ptq.pth
│   ├── camera.backbone.onnx
│   ├── camera.vtransform.onnx
│   ├── default.yaml
│   ├── fuser.onnx
│   ├── head.bbox.onnx
│   └── lidar.backbone.xyz.onnx
└── swint
    ├── camera.backbone.onnx
    ├── camera.vtransform.onnx
    ├── default.yaml
    ├── fuser.onnx
    ├── head.bbox.onnx
    └── lidar.backbone.xyz.onnx
~~~
