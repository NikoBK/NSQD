cmake_minimum_required(VERSION 2.8.3)
project(flight_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	# Add any other dependencies here
)

catkin_package(
	LIBRARIES flight_control
	CATKIN_DEPENDS roscpp
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(flight_control src/controller.cpp src/socketserver.cpp src/debug.cpp)
target_link_libraries(flight_control ${catkin_LIBRARIES})
