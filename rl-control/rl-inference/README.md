# rl-inference

## Introduction
This repository is an extension of the project `model-control`. It takes as input a neural network model file trained from `rl-robotics` project and runs a cpp runtime for real-world deployment. The project now supports `onnx` and `om` models for Lite3 and A1, while `rl-robotics` python project supports the `pt` model.


## Software architecture
The architecture is similar to the project`model-control`, except for a new kind of locomotion state namaed as `qrFSMStateRLLocomotion`.


## Prepare environment
Appropriately sized neural networks are able to be deployed on CPU, Ascend NPU or GPU onboard, determined by the computational hardware you have.
-  For those having Ascend NPU, `om` model is recommended. 
1. Follow the offical [guidance](https://www.hiascend.com/document/detail/zh/canncommercial/62RC1/overview/index.html) to set up the enviroment for Atlas 200I A2 device if you have a fresh SD-card.
2. Since Unbuntu 22 does not support ROS1 noetic via apt tool, one needs to compile it from sources. This [tutorial](https://blog.csdn.net/Drknown/article/details/128701624) introduces ROS1 installation.
3. Download the pre-trained models into `${PROJECT_DIR}/rl-control/rl-inference/` from the two URLs:
    
    [om model for Lite3](https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/models/23.0.RC2/lite3_320.om)

    [pt model for Lite3](https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/models/23.0.RC2/lite3_320.pt)

-  For those only having CPU(x86 or arm) device, the `onnx` model is runnable. You can follow this [website](https://stackoverflow.com/questions/63420533/setting-up-onnx-runtime-on-ubuntu-20-04-c-api) to install ONNX-Runtime firstly.

Note that the `onnx` model is converted from the `pt` model and the `om` model is further converted from the `onnx` model using [ATC](https://www.hiascend.com/document/detail/zh/canncommercial/62RC1/inferapplicationdev/atctool/atctool_000014.html) command.

# Usage
### Run inference on Ascend Atlas 200I (NPU) in the real-world. 
First of all, go into the directory `rl-inference`.

Take Lite3 for example. To compile the source for om inference, one should turn on the cmake variable `USE_NPU` in the file `rl-inference/src/quadruped/CMakeLists.txt`, as well as trun off the variable `USE_ONNX`. Set the value of `network_path` in the configuration file `src/quadruped/config/lite3/main.yaml` to the right model path. Then compile the code:
```
catkin_make --only-pkg-with-deps quadruped examples
```

As with `model-control`,
```
source devel/setup.bash 
rosrun examples example_lite3_real
``` 
After launching the control process, you need to wait until the quadruped stands up, followed by pressing B key on Logitech F710 gamepad to switch to toque-stance mode. Then press the X key to switch to RL-locomotion mode with network inference.

### Joy Control (Only for Lite3)
If you do not have Logitech F710 gamepad, a lite3 gamepad can be used to control the quadruped at runtime. The detailed command logic is coded in the function `qrDesiredStateCommand::RecvSocket`.
