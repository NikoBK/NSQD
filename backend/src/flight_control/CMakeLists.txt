cmake_minimum_required(VERSION 2.8.3)
project(flight_control)

# Compile as C++11 because we use nullptr.
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

catkin_package(
	LIBRARIES flight_control
	CATKIN_DEPENDS roscpp
)

include_directories(
	${catkin_INCLUDE_DIRS}
	/usr/include
)

add_executable(flight_control src/Main.cpp src/Message.hpp src/Server.cpp src/Server.h src/Matrice100.cpp)
target_link_libraries(flight_control ${catkin_LIBRARIES})
