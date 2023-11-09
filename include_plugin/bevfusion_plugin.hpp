#ifndef bevfusion_plugin_h
#define bevfusion_plugin_h

#include <iostream>
#include <string.h>
#include <vector>
#include <cuda_runtime.h>

#include "bevfusion/bevfusion.hpp"
#include "common/check.hpp"
#include "common/tensor.hpp"
#include "common/timer.hpp"
#include "common/visualize.hpp"

#include <stb_image.h>
#include <stb_image_write.h>

// --------------
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

// typedef PointXYZIT PointT;
typedef pcl::PointXYZI PointT;

void printParam(float *parm, int const begin, int const end);
unsigned char* cv2stb(std::string img_path);
unsigned char* cv2stb(const cv::Mat &image);

float *pcl2arr(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

std::vector<unsigned char*> load_images(const std::string& root);
std::vector<unsigned char*> load_images(
  const cv::Mat &f_img, const cv::Mat &fl_img, const cv::Mat &fr_img,
  const cv::Mat &b_img, const cv::Mat &bl_img, const cv::Mat &br_img);


class BEVFusionNode
{
 private:
	 // 测试数据的目录，模型, 模型预测的精度
	
	std::string pkg_path_;
	std::string data_path_ = pkg_path_ + "/example-data";
	
	std::string model_name_;
	std::string precision_;
	std::string config_path_;
	
	// Tensor类型内外参数
	nv::Tensor camera2lidar; 
	nv::Tensor camera_intrinsics; 
	nv::Tensor lidar2image; 
	nv::Tensor img_aug_matrix; 

	// float类型内外参数 本地读取采用这个
	float cam_intrinsics[96];
	float cam2lidar[96];
	float lidar2img[96];
	float img_aug_mat[96];

	pcl::PointCloud<PointT>::Ptr cloud_;
	std::vector<unsigned char *> images_data_;

	std::shared_ptr<bevfusion::Core> core;
	cudaStream_t stream;
	
 public:
	BEVFusionNode(const std::string& model, const std::string& precision, const std::string& pkg_path);
	
	//  释放图像和cuda的内存
	~BEVFusionNode();
	
	std::shared_ptr<bevfusion::Core> create_core(const std::string& model, const std::string& precision);

	// 推理
	// void Inference(const std::vector<unsigned char *>& images_data, const pcl::PointCloud<PointT>::Ptr &cloud);
	void Inference(const std::vector<unsigned char *>& images_data, float *lidar_arr, int lidar_num);
	
	// 可视化
	void visualize(const std::vector<bevfusion::head::transbbox::BoundingBox>& bboxes, const nv::Tensor& lidar_points,
                      const std::vector<unsigned char*> images, const nv::Tensor& lidar2image, const std::string& save_path,
                      cudaStream_t stream);

	// 释放图像
	void free_images(std::vector<unsigned char*>& images);
};

#endif