#include "Server.h"
#include <iostream>
#include <fstream>
#include "../include/Matrice100.hpp"
#include "../include/Main.hpp"
#include "Message.hpp"
#include <opencv2/opencv.hpp>

// DEFINITIONS
#define REFRESH_RATE_HZ 50

// ORIENTATION & POSITIONS
// Data structs to save roll pitch adn yaw as well as gps data (declared in Matrice100.hpp)
RPY rpy;
TargetRPY targetRPY;
GPS_Data gps_data;
TargetGPS targetGPSData;

// DEBUG & LOG
Error errorData;
std::ofstream csvFile;
int imuSample = 0; // Used to write imu data to csv file.
float target = 0;
bool _captureDeviceReady = false;

// REFERENCES & POINTERS
Matrice100 *drone;

// ROS
ros::Time begin;

// OPENCV
cv::VideoCapture cap;

// DATA
//Used for storing target thrust to be written to csv file
float thrust = 0;
float targetThrust = 0;
float targetYaw = 0;
int state = GROUNDED_STATE;

/** Return true or false based on whether or not
 *	the IMU is ready.
 */
bool imuReady(int sample, int rate) {
	if (sample % rate == 0) {
		return true;
	} else {
		return false;
	}
}

// Write imu data to csv file
void csvWrite(ros::Time begin, int sampleRate)
{	
	if (imuReady(imuSample, sampleRate)) {
		// Get the time difference between now and begin.
		ros::Duration timeDiff = ros::Time::now() - begin;
		
		// write to file
  		csvFile << timeDiff 
  		<< " , "<< rpy.roll 
  		<< " , " << rpy.pitch 
  		<< " , " << rpy.yaw 
  		<< " , " << gps_data.latitude 
  		<< " , " << gps_data.longitude 
  		<< " , " << gps_data.altitude 
  		<< " , " << thrust << "\n";
	}
	imuSample += 1;
}

// Write imu data to csv file
void csvWritePathLog(ros::Time begin, int sampleRate)
{	
	if (imuReady(imuSample, sampleRate)) {
		// Get the time difference between now and begin
		ros::Duration timeDiff = ros::Time::now() - begin;
		
		// write to file
  		csvFile << timeDiff 
  		<< " , " << rpy.roll 
  		<< " , " << rpy.pitch 
  		<< " , " << rpy.yaw 
  		<< " , " << gps_data.latitude 
  		<< " , " << gps_data.longitude 
  		<< " , " << gps_data.altitude 
  		<< " , " << thrust
        << " , " << targetRPY.roll 
        << " , " << targetRPY.pitch 
        << " , " << targetRPY.yaw 
        << " , " << targetGPSData.latitude 
        << " , " << targetGPSData.longitude 
        << " , " << targetGPSData.altitude 
        << " , " << targetThrust
        << " , " << errorData.errorLat 
        << " , " << errorData.errorLon 
        << " , " << errorData.errorAlt 
        << " , " << errorData.errorYaw 
        << "\n";
	}
	imuSample += 1;
}

/** Stops the application.
*/
void terminate() {
	cap.release();
	std::cout << "Press any key to stop the computer vision capture" << std::endl;
	cv::waitKey(0);
	cv::destroyAllWindows();
}

/** Capture the current frame and process it.
*/
bool captureFrame() {
	cv::Mat frame;
	return true;
}

/** Initialize computer vision related things.
*/
bool initCV() {
	std::cout << "Initializing drone vision..." << std::endl;
	int id = 2;

	// Create a capture instance and set the device id.
	cap = cv::VideoCapture(id);
	
	// Check if the capture device is accessible.
	if (!cap.isOpened()) {
		std::cerr << "Error: Unable to open camera (" << id << ")" << std::endl;
		return false;
	}
	
	std::cout << "Drone vision succesfully initialized" << std::endl;
	return true;
}

/** Initialize ROS components.
*/
bool initROS(int argc, char **argv) {
	std::cout << "Initializing ROS..." << std::endl;
	
	ros::init(argc, argv, "flight_control_node");
	ros::NodeHandle nh;
	
	// Initialise the Matrice100 object (calling its constructor)
    drone = new Matrice100(&nh);
	
	std::cout << "ROS succesfully initialized" << std::endl;
	return true;
}

int main(int argc, char **argv)
{
	// ROS must be ready, otherwise we won't run the program.
	if (!initROS(argc, argv)) {
		std::cerr << "ROS could not be initialized" << std::endl;
		return 1;
	}
	
	// Camera must be open, otherwise we won't run the program.
	_captureDeviceReady = initCV();
	if (!_captureDeviceReady) {
		std::cerr << "Drone vision could not be initialized" << std::endl;
		return 1;
	}

	// Start the TCP socketserver.
    Server server(8888, drone, &csvFile);

    // Set ROS refresh rate.
    // TODO: Would be nice to have this in initROS()
    ros::Rate rate(REFRESH_RATE_HZ);

	// Superloop - runs at 50Hz (ROS)
    while (ros::ok()) {
    	// TODO: Capture a frame here when the networking works.

		// Update drone position and orientation on client.
		// TODO: Bring this in when tested.
		// drone->getRPY(&rpy);
    	// drone->getGPSData(&gps_data);
    	
		// Instantiate tick messages.
		UpdateMessage updMsg;
		// TODO: Instantiate image message here when the networking works.
	

        // if not connected
        if (!server.connected()) {
	    	std::cout << "waiting for connection..." << std::endl;
        	server.AcceptConnection(); // Accept any incoming connection.
        }
        else 
        {
        	// std::cout << "handling a connection..." << std::endl;
	    	// handle the current connection and update state
        	server.HandleConnection(&state);
        	
        	// TODO: Why 5??
        	if (imuReady(imuSample, 5)) 
        	{
        		// TODO: Set the update message content
        		// here and send the message through the server.
        		// The contents should be the following:
        		updMsg.roll = 1;	//rpy.roll;
	    		updMsg.pitch = 2;	//rpy.pitch;
				updMsg.yaw = 3;	//rpy.yaw;
				updMsg.thrust = 4;	//drone->getTargetThrust();
				updMsg.lat = 5;	//(float)gps_data.latitude;
				updMsg.lon = 6;	//(float)gps_data.longitude;
				updMsg.alt = 10; 	//(float)gps_data.altitude;
			    updMsg.state = 11;	//state;
				server.Send(updMsg);
				std::cout << "Message sent!" << std::endl;
        	}
    	}
		// TODO: Implement finite state machine here.
		
		// Update properties and restart the loop.
		imuSample++;
		ros::spinOnce();
    	rate.sleep();
    }

	// Stop the program when the while loop is broken.
	terminate();
}
