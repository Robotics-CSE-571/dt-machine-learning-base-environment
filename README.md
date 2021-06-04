# Template for DNN with duckiebot

This is based on [this repo](https://github.com/duckietown/dt-machine-learning-base-environment) from the duckiebot.

I recommend to run this image on your laptop/workstation with GPU since
building it on Jetson is really time consuming since it needs
to compile the pandas on Jetson which takes a long time.
And actually I was not yet successful on building it on Jetson.

If you want to use GPU, you need to first install nvidia docker runtime as
described on [this page](https://github.com/NVIDIA/nvidia-container-runtime#ubuntu-distributions)
on your laptop/workstation. I do not know how to do it on Jetson though.

Of course you need to have the nvidia driver for linux first.
I do not recommend anything how to get it or download it. Since
it is kind of difficult for ubuntu 16.04 but they made it
super easy for ubuntu 20.04 which they can be installed from
apt.

## install nvidia requirements

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

## Build the image for laptop

It is going to be relatively time consuming

```bash
dts devel build  -f
```

## Run the image on laptop but connected to roscore on the robot

change the robot name and ip according to yours.

```bash
ROBOT_NAME="yousofsduckie"
ROBOT_IP="${ROBOT_NAME}.local"
dts devel run -f --net host -- -e ROS_MASTER_URI=http://${ROBOT_IP}:11311 -e VEHICLE_NAME=${ROBOT_NAME} --runtime nvidia
```

The current version should print somewhere in the terminal as the following

```bash
torch is imported successfully
cuda available? True
```
