# Ursus-flow firmware

## On-board LEDs

| Color  | Function                    | Description                                                                                                                                                         |
|--------|-----------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| blue   | optical flow task frequency | default: blink in 250hz<br />calibration mode: blink in low frequency |
| yellow | -                           | -                                                                                                                                                                   |
| red    | device initialization state | on: everything is good<br />off: hardware initialization failed                                                                                                       |

## USB calibration monitor

The calibration monitor was written in C++ and using ROS (Robotic Operating System),
make sure you have already installed ROS before building the program.

**Dependencies**

```
sudo apt install ros-kinetic-jsk-visualization
```

**Build**

```
cd tools/ros_ws
make
```

**Run**

```
cd tools/ros_ws
. devel/setup.bash
roslaunch ursusflow monitor.launch
```

**Camera calibration**

```
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.031 image:=/ursusflow/flow_image --no-service-check --fix-aspect-ratio
```

**Fix USB "Permission denied" error**

First, open or create the linux udev rule file:

```
sudo vi /etc/udev/rules.d/libusb.rules
```

then paste

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666"

```
