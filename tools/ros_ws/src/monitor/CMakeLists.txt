cmake_minimum_required(VERSION 2.8.3)
project(ursusflow)

set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS
	"-std=c++0x \
	-Ofast -g \
	-static-libstdc++ \
	-fsanitize=address -fno-omit-frame-pointer \
	-static-libasan"
)

find_package(
    catkin REQUIRED COMPONENTS
    cv_bridge
    sensor_msgs
    std_msgs
    roscpp
    message_generation
)

find_package(OpenCV REQUIRED)
 
#add_message_files(
#    FILES
#)

#add_service_files(
#    FILES
#    SendHsv.srv
#)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime
)


########################################################################################

set(
    INC
    ./monitor
)

set(
    SRC
    ./main.cpp
    ./flow_simulate.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${INC})
add_executable(monitor ${SRC})
target_link_libraries (monitor ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} -lusb-1.0)
add_dependencies(monitor ${PROJECT_NAME}_generate_messages_cpp)
