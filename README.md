# Ursus-flow firmware

## On-board LEDs

| Color  | Function                    | Description                                                                                                                                                         |
|--------|-----------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| blue   | optical flow task frequency | default: blink in 250hz<br />calibration mode: blink in low frequency |
| yellow | -                           | -                                                                                                                                                                   |
| red    | device initialization state | on: everything is good<br />off: hardware initialization is failed                                                                                                       |

## USB calibration monitor

The calibration monitor was written in C++ and using ROS (Robotic Operating System),
remember install ROS before building the program.

Build

```
cd tools/ros_ws
make
```

Run

```
cd tools/ros_ws
. devel/setup.bash
rosrun ursusflow monitor
```

Fix USB "Permission denied" error


```
sudo vi /etc/udev/rules.d/libusb.rules
```

and paste

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666"

```
