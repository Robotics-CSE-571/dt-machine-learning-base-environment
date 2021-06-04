# Template for DNN with duckiebot and a template ROS Node

This is based on [this repo](https://github.com/duckietown/dt-machine-learning-base-environment) from the duckiebot.

I recommend to run this image on your laptop/workstation with GPU since
building it on Jetson is really time consuming.

If you want to use GPU on your laptop/workstation, you need to first install nvidia docker runtime as
described on [this page](https://github.com/NVIDIA/nvidia-container-runtime#ubuntu-distributions)
on your laptop/workstation.

Of course you need to have the nvidia driver for linux first.
I do not recommend anything on how to get it or download it. Since
it is kind of difficult for ubuntu 16.04 but they made it
super easy for ubuntu 20.04 which they can be installed from
apt.

## install nvidia requirements 

If you want to run the image on your laptop/workstaion follow these, Otherwise you can skip it.

```bash
sudo apt-get install nvidia-container-runtime
```

and then run the following

```bash
sudo tee /etc/docker/daemon.json <<EOF
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
EOF
sudo pkill -SIGHUP dockerd
```

Now you have the nvidia runtime available. You can test it with

```bash
docker run --rm --runtime=nvidia  nvidia/cuda:11.0-base nvidia-smi
```

Which should list your GPU devices which is available within the docker image.
Otherwise there is some issue in your setup.

## Build and run the image on Laptop

It is going to be relatively time consuming

```bash
dts devel build  -f
```

change the robot name and ip according to yours.

```bash
ROBOT_NAME="yousofsduckie"
ROBOT_IP="${ROBOT_NAME}.local"
dts devel run -f --net host -- -e ROS_MASTER_URI=http://${ROBOT_IP}:11311 -e VEHICLE_NAME=${ROBOT_NAME} --runtime nvidia
```

## Build and run the image for Jetson

Note building the image for Jetson is taking very long time but the nvidia runtime stuff should 
be already available on jetson and no extra requirements are need.

To build the image for Jetson

```bash
ROBOT_NAME="yousofsduckie"
dts devel build -f -H ${ROBOT_NAME}.local --ncpus 4
```

To run the image on Jetson

```bash
dts devel run -H ${ROBOT_NAME}.local
```

## Test

The current version should print somewhere in the terminal as the following

```bash
torch is imported successfully
cuda available? True
```

## Troubleshooting

### "Not enough space" on Jetson during building the image

run the following to delete the dangling images.

```bash
docker -H ${ROBOT_NAME}.local image prune
```

Run `docker -H ${ROBOT_NAME}.local image ls` and see all the images. You can remove the previous images
that you built and you do not need for example for previous project

to remove an image:

```bash
docker -H ${ROBOT_NAME}.local image rm <image-name>:<image-tag>

# for example
docker -H ${ROBOT_NAME}.local image rm duckietown/cse571-sp21-project-1:main-arm64v8
```

## Template ROS Node

You can find the template ROS node that imports pytorch in `packages/my_dnn/src/my_dnn_node.py`.
