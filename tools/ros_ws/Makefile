all:
	catkin_make -j4

clean:
	catkin_make clean
run_usb:
	roslaunch ursusflow monitor.launch

run_serial:
	roslaunch ursusflow_serial monitor.launch

astyle:
	astyle -r --style=linux --indent=tab=8 *.cpp *.hpp
