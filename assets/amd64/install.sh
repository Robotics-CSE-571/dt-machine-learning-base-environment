#!/bin/bash

set -ex

# setup nvidia repo
sudo apt-key adv --fetch-keys \
    https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" \
    > /etc/apt/sources.list.d/cuda.list
echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" \
    > /etc/apt/sources.list.d/nvidia-ml.list 

# install CUDA 11.1
apt-get update
apt-get install -y \
    cuda-cudart-$CUDA_PKG_VERSION-1 \
    cuda-compat-$CUDA_PKG \
    cuda-libraries-$CUDA_PKG \
    cuda-nvtx-$CUDA_PKG \
    libcublas-$CUDA_PKG \
    libnccl2 \
    libcudnn8=$CUDNN_VERSION-1+cuda11.1
apt-mark hold libnccl2 libcudnn8 cuda-compat-$CUDA_PKG

# TODO Install Tensor RT here
# >>>...

# Clean up
rm -rf /usr/src/cudnn_samples_v8 && rm -rf /var/lib/apt/lists/*

# install PyTorch
pip3 install torch==1.8.1+cu111 torchvision==0.9.1+cu111 torchaudio==0.8.1 -f https://download.pytorch.org/whl/torch_stable.html

# clean
pip3 uninstall -y dataclasses
