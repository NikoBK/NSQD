cmake_minimum_required(VERSION 2.8.3)
project(flight_control)

# Compile as C++11 because we use nullptr.
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	OpenCV
	#tinyxml2_catkin
	#tinyxml2
)

#add_subdirectory(#
#	external
#)


#find_library(#
#	TINYXML_LIBRARIES NAMES tinyxml PATHS /usr/local/lib
#)


catkin_package(
	LIBRARIES flight_control #tinyxml2_catkin
	CATKIN_DEPENDS roscpp
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${TINYXML2_INCLUDE_DIRS}
	/usr/include
)

add_executable(flight_control src/tinyxml2.cpp src/Main.cpp src/Message.hpp src/Server.cpp src/Server.h src/Matrice100.cpp)
target_link_libraries(flight_control ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS} ${TINYXML_LIBRARIES})
