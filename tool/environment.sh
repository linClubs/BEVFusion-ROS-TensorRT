
# 1 配置 CUDA TensorRT cudnn路径
# ----------------x86------------------
export TensorRT_Lib=/home/lin/software/TensorRT-8.5.3.1/lib
export TensorRT_Inc=/home/lin/software/TensorRT-8.5.3.1/include
export TensorRT_Bin=/home/lin/software/TensorRT-8.5.3.1/bin

export CUDA_Lib=/usr/local/cuda/lib64
export CUDA_Inc=/usr/local/cuda/include
export CUDA_Bin=/usr/local/cuda/bin
export CUDA_HOME=/usr/local/cuda

export CUDNN_Lib=/usr/local/cuda/lib64
# -----------------------------------------------------

# ----------------orin--------------------
# # tensorrt
# export TensorRT_Inc=/usr/include
# export TensorRT_Lib=/usr/lib/aarch64-linux-gnu
# export TensorRT_Bin=/usr/src/tensorrt/bin

# # cuda
# export CUDA_Lib=/usr/local/cuda-11.4/targets/aarch64-linux/lib
# export CUDA_Inc=/usr/local/cuda-11.4/include
# export CUDA_Bin=/usr/local/cuda/bin
# export CUDA_HOME=/usr/local/cuda

# # cudnn
# export CUDNN_Lib=/usr/lib/aarch64-linux-gnu
# -------------------------------------


# 2 选择模型3种：resnet50/resnet50int8/swint

export DEBUG_MODEL=resnet50int8
# export DEBUG_MODEL=resnet50
# export DEBUG_MODEL=swint

# 模型精度2种：fp16/int8
# export DEBUG_PRECISION=fp16
export DEBUG_PRECISION=int8
# ----------------


export DEBUG_DATA=example-data
export USE_Python=OFF

# check the configuration path
# clean the configuration status
export ConfigurationStatus=Failed
if [ ! -f "${TensorRT_Bin}/trtexec" ]; then
    echo "Can not find ${TensorRT_Bin}/trtexec, there may be a mistake in the directory you configured."
    return
fi

if [ ! -f "${CUDA_Bin}/nvcc" ]; then
    echo "Can not find ${CUDA_Bin}/nvcc, there may be a mistake in the directory you configured."
    return
fi

echo "=========================================================="
echo "||  MODEL: $DEBUG_MODEL"
echo "||  PRECISION: $DEBUG_PRECISION"
echo "||  DATA: $DEBUG_DATA"
echo "||  USEPython: $USE_Python"
echo "||"
echo "||  TensorRT: $TensorRT_Lib"
echo "||  CUDA: $CUDA_HOME"
echo "||  CUDNN: $CUDNN_Lib"
echo "=========================================================="

BuildDirectory=`pwd`/build

if [ "$USE_Python" == "ON" ]; then
    export Python_Inc=`python3 -c "import sysconfig;print(sysconfig.get_path('include'))"`
    export Python_Lib=`python3 -c "import sysconfig;print(sysconfig.get_config_var('LIBDIR'))"`
    export Python_Soname=`python3 -c "import sysconfig;import re;print(re.sub('.a', '.so', sysconfig.get_config_var('LIBRARY')))"`
    echo Find Python_Inc: $Python_Inc
    echo Find Python_Lib: $Python_Lib
    echo Find Python_Soname: $Python_Soname
fi

export PATH=$TensorRT_Bin:$CUDA_Bin:$PATH
export LD_LIBRARY_PATH=$TensorRT_Lib:$CUDA_Lib:$CUDNN_Lib:$BuildDirectory:$LD_LIBRARY_PATH
export PYTHONPATH=$BuildDirectory:$PYTHONPATH
export ConfigurationStatus=Success

if [ -f "tool/cudasm.sh" ]; then
    echo "Try to get the current device SM"
    . "tool/cudasm.sh"

    echo "Current CUDA SM: $cudasm"
fi

export CUDASM=$cudasm
export CUDASM=86

echo Configuration done!