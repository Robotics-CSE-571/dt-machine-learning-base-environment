# Template for DNN with duckiebot

I recommend run it on your laptop since building it on jetson is really time conuming since it needs
to compile the pandas on jetson which takes a long time.

If you want to use GPU, you need to first insall nvidia docker runtime as
described on [this page](https://github.com/NVIDIA/nvidia-container-runtime#ubuntu-distributions)
on the laptop. I do not know how to id on jetson though.

## install nvidia requirements
Of course you need to have the nvidia driver first.

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

Which should list your GPU evices which is available within the docker image.
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
