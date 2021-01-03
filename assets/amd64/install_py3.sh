#!/bin/bash
# remove dataclass
pip3 uninstall dataclass

# Pytorch: 
pip install torch==1.7.0+cu101 torchvision==0.8.1+cu101 torchaudio==0.7.0 -f https://download.pytorch.org/whl/torch_stable.html

# Tensorflow:
pip3 install tensorflow>=2.3.1 cupy-cuda101

pip3 cache purge