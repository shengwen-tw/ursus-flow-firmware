# Ursus-flow firmware

## Demo videos

[![flow_rviz](https://github.com/shengwen1997/ursus-flow-firmware/blob/master/materials/flow_demo.png?raw=true)](https://www.youtube.com/watch?v=TZlbP051b0A)

## On-board LEDs

| Color  | Function                    | Description                                                                                                                                                         |
|--------|-----------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| blue   | optical flow update frequency | blinking rate is equal to optical flow update rate|
| yellow | flow detecting state        | the led lights up while the flow is detected                                                                                                                                                                   |
| red    | device initialization state | on: hardware initialization succeed<br />off: hardware initialization failed                                                                                                       |

## USB calibration monitor

The calibration monitor was written in C++ and using ROS (Robotic Operating System),
make sure you have already installed ROS before building the program.

**1. Install dependencies**

```
sudo apt install ros-kinetic-jsk-visualization ros-kinetic-serial
```

**2. Build**

```
cd tools/ros_ws
make
```

**3. Add USB premission for Ursus-flow board**

1. open or create new linux udev rule file:

```
sudo vi /etc/udev/rules.d/libusb.rules
```

2. paste

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666"

```


**4. Run**

```
cd tools/ros_ws
. devel/setup.bash
roslaunch ursusflow monitor.launch
```

### Camera calibration

```
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.031 image:=/ursusflow/flow_image --no-service-check --fix-aspect-ratio
```
