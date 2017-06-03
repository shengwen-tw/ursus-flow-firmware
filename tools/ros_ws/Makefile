all:
	catkin_make -j4

clean:
	catkin_make clean

run_usb_sim:
	roslaunch ursusflow_usb_sim monitor.launch

run_usb:
	roslaunch ursusflow_usb_monitor monitor.launch

run_serial:
	roslaunch ursusflow_serial monitor.launch

astyle:
	astyle -r --style=linux --indent=tab=8 *.cpp *.hpp
