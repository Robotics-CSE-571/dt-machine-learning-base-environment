#!/bin/bash

set -e

apt update
apt install -y libopenblas-base libopenmpi-dev
rm -rf /var/lib/apt/lists/*

# download PyTorch
echo "Downloading PyTorch v${PYTORCH_VERSION}..."
PYTORCH_WHEEL_NAME="torch-${PYTORCH_VERSION}.cuda.cudnn-cp38-cp38-linux_aarch64.whl"
WHEEL_URL="https://duckietown-public-storage.s3.amazonaws.com/assets/python/wheels/${PYTORCH_WHEEL_NAME}"
wget -q "${WHEEL_URL}" -O "/tmp/${PYTORCH_WHEEL_NAME}"
# install PyTorch
echo "Installing PyTorch v${PYTORCH_VERSION}..."
pip3 install "/tmp/${PYTORCH_WHEEL_NAME}"
rm "/tmp/${PYTORCH_WHEEL_NAME}"

# download TensorRT
echo "Downloading TensorRT v${TENSORRT_VERSION}..."
TENSORRT_WHEEL_NAME=tensorrt-${TENSORRT_VERSION}-cp38-cp38-linux_aarch64.whl
WHEEL_URL="https://duckietown-public-storage.s3.amazonaws.com/assets/python/wheels/${TENSORRT_WHEEL_NAME}"
wget -q "${WHEEL_URL}" -O "/tmp/${TENSORRT_WHEEL_NAME}"
# install TensorRT
echo "Installing TensorRT v${TENSORRT_VERSION}..."
pip3 install "/tmp/${TENSORRT_WHEEL_NAME}"
rm "/tmp/${TENSORRT_WHEEL_NAME}"

# download PyCUDA
echo "Downloading PyCUDA v${PYCUDA_VERSION}..."
TENSORRT_WHEEL_NAME=pycuda-${PYCUDA_VERSION}-cp38-cp38-linux_aarch64.whl
WHEEL_URL="https://duckietown-public-storage.s3.amazonaws.com/assets/python/wheels/${TENSORRT_WHEEL_NAME}"
wget -q "${WHEEL_URL}" -O "/tmp/${TENSORRT_WHEEL_NAME}"
# install PyCUDA
echo "Installing PyCUDA v${PYCUDA_VERSION}..."
pip3 install "/tmp/${TENSORRT_WHEEL_NAME}"
rm "/tmp/${TENSORRT_WHEEL_NAME}"

# clean
pip3 uninstall -y dataclasses

apt-get update && apt-get install gcc-8 g++-8 unzip -y
rm /usr/bin/gcc /usr/bin/g++
ln -s /usr/bin/gcc-8 /usr/bin/gcc
ln -s /usr/bin/g++-8 /usr/bin/g++

# torchvision
git clone --depth 1 --branch v0.8.0 https://github.com/pytorch/vision /tmp/torchvision
pip3 install /tmp/torchvision
rm -rf /tmp/torchvision

#TARGET="https://pypi.python.org/simple/pycuda/"
#PATTERN="pycuda-2021.1.tar.gz"

#wget --recursive --no-directories --accept=$PATTERN $TARGET

#wget https://files.pythonhosted.org/packages/5a/56/4682a5118a234d15aa1c8768a528aac4858c7b04d2674e18d586d3dfda04/pycuda-2021.1.tar.gz
#tar -xfv pycuda.tar.gz -C ./pycuda_build

#CUDA_INC_DIR=/usr/local/cuda-10.2 pip3 install pycuda