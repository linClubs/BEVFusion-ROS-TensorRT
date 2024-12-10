cmake_minimum_required(VERSION 3.5)
project(bevfusion)

add_definitions(-w)
# set(CMAKE_BUILD_TYPE "Release")   # pcl会报错 使用debug模式
# set(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-W)
add_compile_options(-std=c++17)

set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -g -DENABLE_TEXT_BACKEND_STB")
#set(CMAKE_CXX_FLAGS "-O2 -g -ggdb -fPIC" )

# set(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -Wextra -Wall -Wno-missing-field-initializers -Wno-deprecated-declarations -O3 -DENABLE_TEXT_BACKEND_STB")
# set(CMAKE_CXX_FLAGS_DEBUG   "-std=c++14 -O0 -g -DENABLE_TEXT_BACKEND_STB")

set(CUDA_NVCC_FLAGS_RELEASE "-Werror=all-warnings -Xcompiler -std=c++17,-Wextra,-Wall,-Wno-deprecated-declarations,-O3 -DENABLE_TEXT_BACKEND_STB")
set(CUDA_NVCC_FLAGS_DEBUG   "-std=c++17 -O0 -g -DENABLE_TEXT_BACKEND_STB")

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} 
  # -gencode arch=compute_$ENV{CUDASM},code=compute_$ENV{CUDASM}
  -gencode arch=compute_86,code=compute_86
)

# cuda
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CUDA_INSTALL_TARGET_DIR targets/x86_64-linux)

set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)
set(CUDA_LIBS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/lib)

# TENSORRT
set(TensorRT_ROOT /usr/local/TensorRT)
# set(TensorRT_ROOT ~/share/software/TensorRT-8.5.3.1)
set(TensorRT_INCLUDE_DIRS ${TensorRT_ROOT}/include)
set(TensorRT_LIBS ${TensorRT_ROOT}/lib/)

find_package(CUDA REQUIRED)
find_package(Protobuf PATHS [path_of_the_protobuf_built] REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

# ros依赖 find dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()
ament_package()

include_directories(
  ${TensorRT_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

  third_party/stb
  third_party/cuOSD/src
  third_party/3DSparseConvolution/libspconv/include
  src/common
  src/
  
  include_plugin
  include_ros
)



link_directories(
  ${TensorRT_LIBS}
  ${CUDA_LIBS}
  third_party/3DSparseConvolution/libspconv/lib/${arch}
)

file(GLOB_RECURSE CORE_FILES 
  src/bevfusion/*.cu 
  src/bevfusion/*.cpp
  src/common/tensor.cu
  src/common/tensorrt.cpp
)

file(GLOB_RECURSE VISUALIZE_FILES 
  third_party/cuOSD/src/*.cpp
  third_party/cuOSD/src/*.cu
  src/common/visualize.cu
)


####################### bevfusion program #############################

cuda_add_executable(bevfusion_node
  src_ros/main.cpp 
  src_ros/bevfusion_node.cpp 
  src_plugin/bevfusion_plugin.cpp
  
  ${VISUALIZE_FILES}
  ${CORE_FILES}
)

target_link_libraries(bevfusion_node
  libnvinfer.so
  libspconv.so
  ${Protobuf_LIBRARIES}
  libnvinfer_plugin.so

  libcudart.so
  libcublasLt.so
  
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(bevfusion_node
	rclcpp
	std_msgs
	sensor_msgs
	cv_bridge
  message_filters
  pcl_conversions
)


install(TARGETS
  bevfusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
