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
        points[i * 5 + 4] = __internal_float2half(0);    // 实时激光雷达没有第五维数据可直接赋值0
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