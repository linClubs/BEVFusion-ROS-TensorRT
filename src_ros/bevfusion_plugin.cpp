#include "bevfusion_plugin.hpp"

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

float *pcl2arr(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  int lidar_num = cloud->points.size();
  float lidar_arr[lidar_num * 5];
  for(size_t i = 0; i < cloud->points.size(); ++i )
  {
    long index = i * 5;
    lidar_arr[index]     = cloud->points[i].x;
    lidar_arr[index + 1] = cloud->points[i].y;
    lidar_arr[index + 2] = cloud->points[i].z;
    lidar_arr[index + 3] = cloud->points[i].intensity;
    // lidar_arr[index + 4] = cloud->points[i].time;
    lidar_arr[index + 4] = 0;
  }
  return lidar_arr;
}

// opencv读取图像并转成stb格式的buffer
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

unsigned char* cv2stb(const cv::Mat &image)
{
  int width = image.cols;  // 宽x
  int height = image.rows; // 高y
  int channels = image.channels(); // 通道

  std::vector<unsigned char> buffer;    // 创建一个char类型的数组buffer用来存储图像的data域
  cv::imencode(".jpg", image, buffer); // 编码格式 ""参数可添 .jpg、.png

  // 使用stbi_load函数加载图像数据 width * height * channels = buffer.size()
  unsigned char* stbi_data = stbi_load_from_memory(buffer.data(), buffer.size(), &width, &height, &channels, 0);
  return stbi_data;
}

std::vector<unsigned char*> load_images(const std::string& data_path) 
{
  const char* file_names[] = {"0-FRONT.jpg", "1-FRONT_RIGHT.jpg", "2-FRONT_LEFT.jpg",
                              "3-BACK.jpg",  "4-BACK_LEFT.jpg",   "5-BACK_RIGHT.jpg"};

  std::vector<unsigned char*> _images;

  for (int i = 0; i < 6; ++i) 
  {
    std::string path = data_path +"/" + file_names[i];
    int width, height, channels;

    // 1 原版方法
    // unsigned char* img = stbi_load(path.c_str(), &width, &height, &channels, 0);
    // _images.push_back(img);
    
    // 2 使用opencv读取图像转 unsigned char*
    // BEVFusionNode bev1;
    _images.push_back(cv2stb(path));
  }
  return _images; 
}

// 直接读取6张图
std::vector<unsigned char*> load_images(
  const cv::Mat &f_img, const cv::Mat &fl_img, const cv::Mat &fr_img,
  const cv::Mat &b_img, const cv::Mat &bl_img, const cv::Mat &br_img) 
{
  std::vector<unsigned char*> _images;
  _images.push_back(cv2stb( f_img));
  _images.push_back(cv2stb(fl_img));
  _images.push_back(cv2stb(fr_img));
  _images.push_back(cv2stb( b_img));
  _images.push_back(cv2stb(bl_img));
  _images.push_back(cv2stb(br_img));
  return _images; 
}

void free_images(std::vector<unsigned char*>& images) 
{
  for (size_t i = 0; i < images.size(); ++i) 
    stbi_image_free(images[i]);
  images.clear();
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

  scn.model = pkg_path + nv::format("/model/%s/lidar.backbone.xyz.onnx", model.c_str());

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
  transbbox.model = pkg_path + nv::format("/model/%s/build/head.bbox.plan", model.c_str());
  transbbox.confidence_threshold = 0.12f;
  transbbox.sorted_bboxes = true;

  bevfusion::CoreParameter param;
  param.camera_model = pkg_path + nv::format("/model/%s/build/camera.backbone.plan", model.c_str());
  param.normalize = normalization;
  param.lidar_scn = scn;
  param.geometry = geometry;
  param.transfusion = pkg_path + nv::format("/model/%s/build/fuser.plan", model.c_str());
  param.transbbox = transbbox;
  param.camera_vtransform = pkg_path + nv::format("/model/%s/build/camera.vtransform.plan", model.c_str());
  return bevfusion::create_core(param);
}


BEVFusionNode::BEVFusionNode()
{ 
  

  cloud_.reset(new pcl::PointCloud<PointT>());
  core = create_core(model, precision);

  if (core == nullptr) 
  {
    printf("Core has been failed.\n");
  }

  cudaStreamCreate(&stream);
  core->print();
  core->set_timer(true);

  camera2lidar = nv::Tensor::load(config_path + "/camera2lidar.tensor", false);
  camera_intrinsics = nv::Tensor::load(config_path + "/camera_intrinsics.tensor", false);
  lidar2image = nv::Tensor::load(config_path + "/lidar2image.tensor", false);
  img_aug_matrix = nv::Tensor::load(config_path + "/img_aug_matrix.tensor", false);
  
  // camera2lidar.print();
  // lidar2image.print();
  // img_aug_matrix.print();
  // camera_intrinsics.print();

  for(int i = 0; i < 96; ++i)
  { 
    cam2lidar[i] = camera2lidar.ptr<float>()[i];
    lidar2img[i] = lidar2image.ptr<float>()[i];
    img_aug_mat[i] = img_aug_matrix.ptr<float>()[i];
    cam_intrinsics[i] = camera_intrinsics.ptr<float>()[i];
  }

  // printParam(cam2lidar, 0, 16);
  // printParam(lidar2img, 0, 16);
  // printParam(img_aug_mat, 0, 16);
  // printParam(cam_intrinsics, 0, 16);

  core->update( cam2lidar, cam_intrinsics, lidar2img, img_aug_mat, stream);
  core->free_excess_memory();  // 可以注释掉这句

}


void BEVFusionNode::Inference(const std::vector<unsigned char *>& images_data, float *lidar_arr, int lidar_num)
{ 
  
  std::vector<int64_t> lidar_shape{lidar_num, 5};
  // Tensor类型的点云
  auto lidar_data = nv::Tensor::from_data_reference((void *)lidar_arr, lidar_shape, nv::DataType::Float32, false);
  
  // float32数据转float16
  auto lidar_half = lidar_data.to_device().to_half().to_host();

  // 推理
  auto bboxes =
    core->forward((const unsigned char**)images_data.data(), lidar_half.ptr<nvtype::half>(), lidar_data.size(0), stream);
  
  visualize(bboxes, lidar_half, images_data, lidar2image, "cuda-bevfusion.jpg", stream);
}

void BEVFusionNode::visualize(const std::vector<bevfusion::head::transbbox::BoundingBox>& bboxes, const nv::Tensor& lidar_points,
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
  printf("Save to %s\n", (pkg_path + "/build/" + save_path).c_str());
  stbi_write_jpg((pkg_path + "/build/" + save_path).c_str(), scene_device_image.size(1), scene_device_image.size(0), 3,
                 scene_device_image.to_host(stream).ptr(), 100);

  // cv::Mat img = cv::imread(save_path.c_str());
  // cv::resize(img, img, cv::Size(img.size().width /2, img.size().height /2));

}



BEVFusionNode::~BEVFusionNode()
{ 
  free_images(images_data_);   // 释放内存
  checkRuntime(cudaStreamDestroy(stream));
}

