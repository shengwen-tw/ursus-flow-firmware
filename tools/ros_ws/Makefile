all:
	catkin_make -j4

clean:
	catkin_make clean
run:
	roslaunch ursusflow monitor.launch

astyle:
	astyle -r --style=linux --indent=tab=8 *.cpp *.hpp
