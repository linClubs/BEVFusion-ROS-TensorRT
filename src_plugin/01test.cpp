
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION  // 预处理器宏定义

#include "bevfusion_plugin.hpp"

int main(int argc, char** argv) 
{
  
  // 测试数据的目录，模型, 模型预测的精度
  std::string data_path = pkg_path + "/example-data";
  const char* model     = "resnet50int8";
  const char* precision = "int8";

  auto core = create_core(model, precision);
  
  if (core == nullptr) 
  {
    printf("Core has been failed.\n");
    return -1;
  }

  // 创建cudaStream
  cudaStream_t stream;
  cudaStreamCreate(&stream);
 
  core->print();
  core->set_timer(true);

  // data是路径
  // Load matrix to host 加载传感器内外参数程序
  auto camera2lidar = nv::Tensor::load(data_path + "/camera2lidar.tensor", false);
  auto camera_intrinsics = nv::Tensor::load(data_path + "/camera_intrinsics.tensor", false);
  auto lidar2image = nv::Tensor::load(data_path + "/lidar2image.tensor", false);
  auto img_aug_matrix = nv::Tensor::load(data_path + "/img_aug_matrix.tensor", false);
  
  // 1 打印参数 nv::Tensor::print函数 "Tensor" 偏移, 每行多少个输入 输入多少行 每个内参16个数 总共6个相机
  camera2lidar.print();
  lidar2image.print();
  img_aug_matrix.print();
  camera_intrinsics.print();
  
  // camera_intrinsics.print("Tensor", 80UL, 4UL, 4UL);
  for(int i = 0 ; i < 4; ++i)
    printf("%ld  ", camera_intrinsics.size(i));
  printf("\n\n");

  // 2 使用数组替代Tensor读取参数 camera_intrinsics.ptr<float>()是一个float的数组
  float cam_intrinsics[96];
  float cam2lidar[96];
  float lidar2img[96];
  float img_aug_mat[96];

  for(int i = 0; i < 96; ++i)
  { 
    cam2lidar[i] = camera2lidar.ptr<float>()[i];
    lidar2img[i] = lidar2image.ptr<float>()[i];
    img_aug_mat[i] = img_aug_matrix.ptr<float>()[i];
    cam_intrinsics[i] = camera_intrinsics.ptr<float>()[i];
  }

  printParam(cam2lidar, 0, 16);
  printParam(lidar2img, 0, 16);
  printParam(img_aug_mat, 0, 16);
  printParam(cam_intrinsics, 0, 16);

  // 3 传感器参数分配内存
  // core->update(camera2lidar.ptr<float>(), camera_intrinsics.ptr<float>(), lidar2image.ptr<float>(), img_aug_matrix.ptr<float>(), stream);
  core->update( cam2lidar, cam_intrinsics, lidar2img, img_aug_mat, stream);

  core->free_excess_memory();  // 可以注释掉这句

  // 4 加载图像和激光数据 Load image and lidar to host加载图像和雷达数据 std::vector<unsigned char *> images
  auto images = load_images(data_path);
  auto lidar_points = nv::Tensor::load(data_path + "/points.tensor", false);
  
  printf("lidar_points.size(0) = %ld\n\n",lidar_points.size(0));
  printf("lidar_points.shape: ");
  
  // 4.1 统计一帧点云的个数
  unsigned long data_num = 1;
  for (auto s : lidar_points.shape)
  {
    data_num *= s;
    // printf(" %d  ", s);
  }

  // 4.2 点云转成float32类型 
  // 只有gpu上的才能转float16->float32, c++指针数组又是在内存上操作,
  // 下面表示 cpu_float16 -> gpu_float16 -> gpu_float32 -> cpu_float32
  auto lidar_cpu = lidar_points.to_device().to_float().to_host();
  // lidar_cpu.print("Tensor", 0, 5, 4);  // 打印前4行 * 5列数据

  // lidar_cpu.print("Tensor", data_num -10, 5UL, 4UL);
  // 创建一个Tensor， 默认值全部为0, 类型Float32,
  // std::vector<int64_t> lidar_shape{2, 5};
  // nv::Tensor lidar_raw(lidar_shape, nv::DataType::Float32, false);
  
  //  4.3 将float32类型点云存成pcl格式, 注意这里是5维pcl::PointXYZI是4维
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  std::string cloud_path = data_path + "/test_pcd.pcd";
  
  // 4.3.1 遍历float32类型点云,都赋值给pcl对应的点云变量
  for(long i = 0; i < lidar_points.shape[0]; ++i)
  {
    PointT p; 
    long index = i * 5;
    p.x         = lidar_cpu.ptr<float>()[index];
    p.y         = lidar_cpu.ptr<float>()[index + 1];
    p.z         = lidar_cpu.ptr<float>()[index + 2];
    p.intensity = lidar_cpu.ptr<float>()[index + 3];
    p.time      = lidar_cpu.ptr<float>()[index + 4];
    cloud->points.emplace_back(p);
  }

  cloud->height = 1;
  cloud->width = cloud->points.size();
  // 保存为pcd文件的点云
  pcl::io::savePCDFileASCII<PointT>(cloud_path, *cloud);
  // 加载pcd文件点云
  pcl::io::loadPCDFile<PointT>(cloud_path, *cloud);
  
  // 4.3.2 用float数组存储pcl点云
  data_num = cloud->points.size() * 5;

  float lidar_arr[data_num];
  
  for(size_t i = 0; i < cloud->points.size(); ++i )
  {
    long index = i * 5;
    lidar_arr[index]     = cloud->points[i].x;
    lidar_arr[index + 1] = cloud->points[i].y;
    lidar_arr[index + 2] = cloud->points[i].z;
    lidar_arr[index + 3] = cloud->points[i].intensity;
    lidar_arr[index + 4] = cloud->points[i].time;
  }
  
  std::vector<int64_t> lidar_shape{int64_t(cloud->points.size()), 5};
  
  // 4.3.3 数组数据转成Tensor ； from_data_reference( (void *)强转, shape, DataType, false) 函数根据DataType去内存中取值
  auto lidar_data3 = nv::Tensor::from_data_reference((void *)lidar_arr, lidar_shape, nv::DataType::Float32, false);
  
  // lidar_data3.print("Tensor", 0, 5, 4);

  // 4.3.4 float32数据转float16
  auto lidar_data4 = lidar_data3.to_device().to_half().to_host();

  // lidar_data4.print("Tensor", 0, 5, 4);

  // warmup 热身 推理
  // auto bboxes =
  //     core->forward((const unsigned char**)images.data(), lidar_points.ptr<nvtype::half>(), lidar_points.size(0), stream);

  // 5 推理
  printf("Start Detect...\n");
  
  auto bboxes =
      core->forward((const unsigned char**)images.data(), lidar_data4.ptr<nvtype::half>(), lidar_points.size(0), stream);

  // evaluate inference time  评价推理时间
  // for (int i = 0; i < 5; ++i) 
  // {
  //   core->forward((const unsigned char**)images.data(), lidar_points.ptr<nvtype::half>(), lidar_points.size(0), stream);
  // }

  // 6 可视化 visualize and save to jpg 
  visualize(bboxes, lidar_points, images, lidar2image, "cuda-bevfusion.jpg", stream);

  // destroy memory
  free_images(images);   // 释放内存
  checkRuntime(cudaStreamDestroy(stream));
  return 0;

}