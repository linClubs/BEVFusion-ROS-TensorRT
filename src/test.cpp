#include <iostream>

#include <cuda_runtime.h>
#include <string.h>

#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "bevfusion/bevfusion.hpp"
#include "common/check.hpp"
#include "common/tensor.hpp"
#include "common/timer.hpp"
#include "common/visualize.hpp"

#include <opencv2/opencv.hpp>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 定义激光雷达XYZIT五维数据的格式
#include <pcl/register_point_struct.h>
struct PointXYZIT
{
	PCL_ADD_POINT4D
	PCL_ADD_INTENSITY;
	float time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (
PointXYZIT,
(float, x, x) (float, y, y) (float, z, z) 
(float, intensity, intensity)
(float, time, time)
)

typedef PointXYZIT PointT; // 取别名

// opencv读取图像并转成stb格式的buffer
unsigned char* cv2stb(std::string img_path)
{
  cv::Mat image = cv::imread(img_path, cv::IMREAD_UNCHANGED);
  int width = image.cols;  // 宽x
  int height = image.rows; // 高y
  int channels = image.channels(); // 通道

  std::vector<unsigned char> buffer;    // 创建一个char类型的数组buffer用来存储图像的data域
  cv::imencode(".jpg", image, buffer); // 编码格式 ""参数可添 .jpg、.png

  // 使用stbi_load函数加载图像数据 width * height * channels = buffer.size()
  unsigned char* stbi_data = stbi_load_from_memory(buffer.data(), buffer.size(), &width, &height, &channels, 0);
  return stbi_data;
}

// 打印4*4的参数
void printParam(float *parm, int const begin, int const end)
{   
  for(int i = begin; i < end; ++i)
  {
   // 打印
    if(i > 0 && i % 4 == 0)  // 每4行输出 4*4的矩阵
      printf("\n");
    if(i > 0 && i % 16 == 0) // 没16个数字空一行，16个数为一组参数
      printf("\n");
      printf("%.3f  ", parm[i]);
  }
  printf("\n\n");
}

static std::vector<unsigned char*> load_images(const std::string& root) 
{
  const char* file_names[] = {"0-FRONT.jpg", "1-FRONT_RIGHT.jpg", "2-FRONT_LEFT.jpg",
                              "3-BACK.jpg",  "4-BACK_LEFT.jpg",   "5-BACK_RIGHT.jpg"};

  std::vector<unsigned char*> _images;

  for (int i = 0; i < 6; ++i) 
  {
    char path[200];
    sprintf(path, "%s/%s", root.c_str(), file_names[i]);
    std::cout << path << std::endl;
  
    int width, height, channels;

    // 1 原版方法
    // unsigned char* img = stbi_load(path, &width, &height, &channels, 0);
    // _images.push_back(img);
    
    // 2 使用opencv读取图像转 unsigned char*
    _images.push_back(cv2stb(path));
  }
  return _images;
}

static void free_images(std::vector<unsigned char*>& images) 
{
  for (size_t i = 0; i < images.size(); ++i) 
    stbi_image_free(images[i]);
  images.clear();
}

static void visualize(const std::vector<bevfusion::head::transbbox::BoundingBox>& bboxes, const nv::Tensor& lidar_points,
                      const std::vector<unsigned char*> images, const nv::Tensor& lidar2image, const std::string& save_path,
                      cudaStream_t stream) 
                      {
  std::vector<nv::Prediction> predictions(bboxes.size());
  memcpy(predictions.data(), bboxes.data(), bboxes.size() * sizeof(nv::Prediction));

  int padding = 300;
  int lidar_size = 1024;
  int content_width = lidar_size + padding * 3;
  int content_height = 1080;
  nv::SceneArtistParameter scene_artist_param;
  scene_artist_param.width = content_width;
  scene_artist_param.height = content_height;
  scene_artist_param.stride = scene_artist_param.width * 3;

  nv::Tensor scene_device_image(std::vector<int>{scene_artist_param.height, scene_artist_param.width, 3}, nv::DataType::UInt8);
  scene_device_image.memset(0x00, stream);

  scene_artist_param.image_device = scene_device_image.ptr<unsigned char>();
  auto scene = nv::create_scene_artist(scene_artist_param);

  nv::BEVArtistParameter bev_artist_param;
  bev_artist_param.image_width = content_width;
  bev_artist_param.image_height = content_height;
  bev_artist_param.rotate_x = 70.0f;
  bev_artist_param.norm_size = lidar_size * 0.5f;
  bev_artist_param.cx = content_width * 0.5f;
  bev_artist_param.cy = content_height * 0.5f;
  bev_artist_param.image_stride = scene_artist_param.stride;

  auto points = lidar_points.to_device();
  auto bev_visualizer = nv::create_bev_artist(bev_artist_param);
  bev_visualizer->draw_lidar_points(points.ptr<nvtype::half>(), points.size(0));
  bev_visualizer->draw_prediction(predictions, false);
  bev_visualizer->draw_ego();
  bev_visualizer->apply(scene_device_image.ptr<unsigned char>(), stream);

  nv::ImageArtistParameter image_artist_param;
  image_artist_param.num_camera = images.size();
  image_artist_param.image_width = 1600;
  image_artist_param.image_height = 900;
  image_artist_param.image_stride = image_artist_param.image_width * 3;
  image_artist_param.viewport_nx4x4.resize(images.size() * 4 * 4);
  memcpy(image_artist_param.viewport_nx4x4.data(), lidar2image.ptr<float>(),
         sizeof(float) * image_artist_param.viewport_nx4x4.size());

  int gap = 0;
  int camera_width = 500;
  int camera_height = static_cast<float>(camera_width / (float)image_artist_param.image_width * image_artist_param.image_height);
  int offset_cameras[][3] = {
      {-camera_width / 2, -content_height / 2 + gap, 0},
      {content_width / 2 - camera_width - gap, -content_height / 2 + camera_height / 2, 0},
      {-content_width / 2 + gap, -content_height / 2 + camera_height / 2, 0},
      {-camera_width / 2, +content_height / 2 - camera_height - gap, 1},
      {-content_width / 2 + gap, +content_height / 2 - camera_height - camera_height / 2, 0},
      {content_width / 2 - camera_width - gap, +content_height / 2 - camera_height - camera_height / 2, 1}};

  auto visualizer = nv::create_image_artist(image_artist_param);
  for (size_t icamera = 0; icamera < images.size(); ++icamera) 
  {
    int ox = offset_cameras[icamera][0] + content_width / 2;
    int oy = offset_cameras[icamera][1] + content_height / 2;
    bool xflip = static_cast<bool>(offset_cameras[icamera][2]);
    visualizer->draw_prediction(icamera, predictions, xflip);

    nv::Tensor device_image(std::vector<int>{900, 1600, 3}, nv::DataType::UInt8);
    
    device_image.copy_from_host(images[icamera], stream);

    if (xflip) {
      auto clone = device_image.clone(stream);
      scene->flipx(clone.ptr<unsigned char>(), clone.size(1), clone.size(1) * 3, clone.size(0), device_image.ptr<unsigned char>(),
                   device_image.size(1) * 3, stream);
      checkRuntime(cudaStreamSynchronize(stream));
    }
    visualizer->apply(device_image.ptr<unsigned char>(), stream);

    scene->resize_to(device_image.ptr<unsigned char>(), ox, oy, ox + camera_width, oy + camera_height, device_image.size(1),
                     device_image.size(1) * 3, device_image.size(0), 0.8f, stream);
    checkRuntime(cudaStreamSynchronize(stream));
  }

  // 保存图像
  printf("Save to %s\n", save_path.c_str());
  stbi_write_jpg(save_path.c_str(), scene_device_image.size(1), scene_device_image.size(0), 3,
                 scene_device_image.to_host(stream).ptr(), 100);


}

std::shared_ptr<bevfusion::Core> create_core(const std::string& model, const std::string& precision) {

  printf("Create by %s, %s\n", model.c_str(), precision.c_str());
  bevfusion::camera::NormalizationParameter normalization;
  normalization.image_width = 1600;
  normalization.image_height = 900;
  normalization.output_width = 704;
  normalization.output_height = 256;
  normalization.num_camera = 6;
  normalization.resize_lim = 0.48f;
  normalization.interpolation = bevfusion::camera::Interpolation::Bilinear;

  float mean[3] = {0.485, 0.456, 0.406};
  float std[3] = {0.229, 0.224, 0.225};
  normalization.method = bevfusion::camera::NormMethod::mean_std(mean, std, 1 / 255.0f, 0.0f);

  bevfusion::lidar::VoxelizationParameter voxelization;
  voxelization.min_range = nvtype::Float3(-54.0f, -54.0f, -5.0);
  voxelization.max_range = nvtype::Float3(+54.0f, +54.0f, +3.0);
  voxelization.voxel_size = nvtype::Float3(0.075f, 0.075f, 0.2f);
  voxelization.grid_size =
      voxelization.compute_grid_size(voxelization.max_range, voxelization.min_range, voxelization.voxel_size);
  voxelization.max_points_per_voxel = 10;
  voxelization.max_points = 300000;
  voxelization.max_voxels = 160000;
  voxelization.num_feature = 5;

  bevfusion::lidar::SCNParameter scn;
  scn.voxelization = voxelization;
  scn.model = nv::format("../model/%s/lidar.backbone.xyz.onnx", model.c_str());
  scn.order = bevfusion::lidar::CoordinateOrder::XYZ;

  if (precision == "int8") 
  {
    scn.precision = bevfusion::lidar::Precision::Int8;
  } else {
    scn.precision = bevfusion::lidar::Precision::Float16;
  }

  bevfusion::camera::GeometryParameter geometry;
  geometry.xbound = nvtype::Float3(-54.0f, 54.0f, 0.3f);
  geometry.ybound = nvtype::Float3(-54.0f, 54.0f, 0.3f);
  geometry.zbound = nvtype::Float3(-10.0f, 10.0f, 20.0f);
  geometry.dbound = nvtype::Float3(1.0, 60.0f, 0.5f);
  geometry.image_width = 704;
  geometry.image_height = 256;
  geometry.feat_width = 88;
  geometry.feat_height = 32;
  geometry.num_camera = 6;
  geometry.geometry_dim = nvtype::Int3(360, 360, 80);

  bevfusion::head::transbbox::TransBBoxParameter transbbox;
  transbbox.out_size_factor = 8;
  transbbox.pc_range = {-54.0f, -54.0f};
  transbbox.post_center_range_start = {-61.2, -61.2, -10.0};
  transbbox.post_center_range_end = {61.2, 61.2, 10.0};
  transbbox.voxel_size = {0.075, 0.075};
  transbbox.model = nv::format("../model/%s/build/head.bbox.plan", model.c_str());
  transbbox.confidence_threshold = 0.12f;
  transbbox.sorted_bboxes = true;

  bevfusion::CoreParameter param;
  param.camera_model = nv::format("../model/%s/build/camera.backbone.plan", model.c_str());
  param.normalize = normalization;
  param.lidar_scn = scn;
  param.geometry = geometry;
  param.transfusion = nv::format("../model/%s/build/fuser.plan", model.c_str());
  param.transbbox = transbbox;
  param.camera_vtransform = nv::format("../model/%s/build/camera.vtransform.plan", model.c_str());
  return bevfusion::create_core(param);
}

int main(int argc, char** argv) 
{
  
  // 测试数据的目录，模型, 模型预测的精度
  
  
  const char* data      = "../example-data";
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
  auto camera2lidar = nv::Tensor::load(nv::format("%s/camera2lidar.tensor", data), false);
  auto camera_intrinsics = nv::Tensor::load(nv::format("%s/camera_intrinsics.tensor", data), false);
  auto lidar2image = nv::Tensor::load(nv::format("%s/lidar2image.tensor", data), false);
  auto img_aug_matrix = nv::Tensor::load(nv::format("%s/img_aug_matrix.tensor", data), false);
  
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
  auto images = load_images(data);
  auto lidar_points = nv::Tensor::load(nv::format("%s/points.tensor", data), false);
  
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
  std::string cloud_path = "../example-data/test_pcd.pcd";
  
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