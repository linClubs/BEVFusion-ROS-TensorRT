cmake_minimum_required(VERSION 2.8.12)
project(bevfusion)

add_definitions(-w)
# set(CMAKE_BUILD_TYPE "Release")   # pcl会报错 使用debug模式
# set(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-W)
add_compile_options(-std=c++14)

set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

set(CMAKE_CXX_FLAGS "-std=c++14 -O3 -g -DENABLE_TEXT_BACKEND_STB")

# set(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -Wextra -Wall -Wno-missing-field-initializers -Wno-deprecated-declarations -O3 -DENABLE_TEXT_BACKEND_STB")
# set(CMAKE_CXX_FLAGS_DEBUG   "-std=c++14 -O0 -g -DENABLE_TEXT_BACKEND_STB")

set(CUDA_NVCC_FLAGS_RELEASE "-Werror=all-warnings -Xcompiler -std=c++14,-Wextra,-Wall,-Wno-deprecated-declarations,-O3 -DENABLE_TEXT_BACKEND_STB")
set(CUDA_NVCC_FLAGS_DEBUG   "-std=c++14 -O0 -g -DENABLE_TEXT_BACKEND_STB")

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
set(TensorRT_ROOT /home/lin/software/TensorRT-8.5.3.1)
# set(TensorRT_ROOT ~/share/software/TensorRT-8.5.3.1)
set(TensorRT_INCLUDE_DIRS ${TensorRT_ROOT}/include)
set(TensorRT_LIBS ${TensorRT_ROOT}/lib/)


find_package(CUDA REQUIRED)
find_package(Protobuf REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


include_directories(
  third_party/stb
  third_party/pybind11/include
  third_party/cuOSD/src
  third_party/3DSparseConvolution/libspconv/include
  
  ${TensorRT_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  include_plugin

  src/common
  src/

)


link_directories(
  third_party/3DSparseConvolution/libspconv/lib/${arch}
  build
  /usr/lib/aarch64-linux-gnu
  /usr/aarch64-linux-gnu/lib
  ${TensorRT_LIBS}
  ${CUDA_LIBS}
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

####################### libbevfusion_core.so ##########################
cuda_add_library(bevfusion_core SHARED 
  ${CORE_FILES}
)

target_link_libraries(bevfusion_core
  libcudart.so
  libcublasLt.so
  libnvinfer.so
  libspconv.so
  ${Protobuf_LIBRARIES}
  libnvinfer_plugin.so
)

####################### bevfusion program #############################
cuda_add_executable(demo
  src_plugin/02test.cpp 
  src_plugin/bevfusion_plugin.cpp
  ${VISUALIZE_FILES}
)

target_link_libraries(demo
  bevfusion_core
  libnvinfer.so
  libspconv.so
  ${Protobuf_LIBRARIES}
  libnvinfer_plugin.so
  
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
 
)

# bevfusion_node
# cuda_add_executable(bevfusion_node
#   src_ros/bevfusion_node.cpp src_ros/bevfusion_plugin.cpp
#   ${VISUALIZE_FILES}
# )

# target_link_libraries(bevfusion_node
#   bevfusion_core
#   libnvinfer.so
#   libspconv.so
#   ${Protobuf_LIBRARIES}
#   libnvinfer_plugin.so
  
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
#   ${catkin_LIBRARIES}
# )