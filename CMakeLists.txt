cmake_minimum_required(VERSION 2.8.3)
project(yesense_imu)

find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	message_generation
)
find_package(serial REQUIRED)
find_library(SERIAL  serial)
 
catkin_package(
  	INCLUDE_DIRS
  	LIBRARIES sensor_imu
  	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  	DEPENDS system_lib
)

include_directories( ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${serial_INCLUDE_DIRS})

add_executable( yesense_node src/yesense_node.cpp )
target_link_libraries( yesense_node ${catkin_LIBRARIES} ${serial_LIBRARIES})








