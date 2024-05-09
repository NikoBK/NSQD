#include "Server.h"
#include <iostream>
#include <fstream>
#include "../include/Matrice100.hpp"
#include "../include/Constants.hpp"
#include "Message.hpp"
#include <opencv2/opencv.hpp>

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
Matrice100* drone;

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
  		<< " , " << rpy.roll 
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
bool initROS(int argc, char** argv) {
	std::cout << "Initializing ROS..." << std::endl;
	
	ros::init(argc, argv, "flight_control_node");
	ros::NodeHandle nh;
	
	// Initialise the Matrice100 object (calling its constructor)
    drone = new Matrice100(&nh);
	
	std::cout << "ROS succesfully initialized" << std::endl;
	return true;
}

void updateState() {
	switch(state)
	{
		case GROUNDED_STATE: {
			//std::cout << "Grounded State currently does nothing..." << std::endl;
			break;
		}
		case ARMED_STATE: {
			//std::cout << "Armed State currently does nothing..." << std::endl;
			break;
		}
		case HOVER_STATE: {
			//std::cout << "Hover State currently does nothing..." << std::endl;
			break;
		}
		case PUBLISH_ANGLE_STATE: {
			drone->pubTargetValues();
			break;
		}
		case START_TEST_STATE: {
			// Timestamp beginning
			begin = ros::Time::now();
			std::cout << "Writing headers to csv" << "\n";

			// Write imu data to csv file
			csvFile << "    Time , " 
				<< "ImuRoll , " 
				<< "ImuPitch , " 
				<< "ImuYaw , " 
				<< "Latitude , " 
				<< "Longitude , " 
				<< "Altitude , " 
				<< "Thrust" 
				<< "\n";

			// Publish target values
			drone->pubTargetValues();
			state = RUNNING_TEST_STATE;
			break;
		}
		case RUNNING_TEST_STATE: {
			// Log the test
			csvWrite(begin, 3);

			// update target values
			drone->pubTargetValues();
			break;
		}
		case STOP_TEST_STATE: {
			std::cout << "closing csv file" << "\n";
			csvFile.close();
			state = HOVER_STATE;
			break;
		}
		case INITIALISE_ENROUTE_STATE: {
			// Timestmap the beginning of the state
			begin = ros::Time::now();
			imuSample = 0;

			// Log to csv file
			csvFile << "Time , " 
				<< "ImuRoll , " 
				<< "ImuPitch , " 
				<< "ImuYaw , " 
				<< "Latitude , " 
				<< "Longitude , " 
				<< "Altitude , " 
				<< "Thrust , "
			    << "TargetRoll , " 
				<< "targetPitch , " 
				<< "targetYaw , " 
				<< "targetLatitude , " 
				<< "targetLongitude , " 
				<< "targetAltitude , " 
				<< "targetThrust , "
				<< "errorLatitude , " 
				<< "errorLongitude , " 
				<< "errorAltitude , " 
				<< "errorYaw" 
				<< "\n";

			drone->startMission();
			drone->updateTargetYaw();
			drone->updateTargetPoints();

			state = ENROUTE_TURN_STATE;
			break;
		}
		case ENROUTE_STATE: {
			drone->updateTargetPoints();
			drone->calculateError();
			drone->getError(&errorData);
			drone->runPIDController(); // Function that has PID implemented and updates a controlData structure.
			drone->pubTargetValues();

			if (drone->getTrackState() == 1) {
				drone->updateTargetYaw();
				state = ENROUTE_TURN_STATE;
			}
			else if (drone->getTrackState() == 2) {
				state = ENROUTE_STOPPED_STATE;
			}
			csvWritePathLog(begin, 1);
			break;
		}
		case ENROUTE_TURN_STATE: {
			drone->calculateError();
			drone->getError(&errorData);
			drone->runPIDController();

			targetYaw = drone->getTargetYaw();

			float precision = 0.052; //rad

			if (rpy.yaw - precision < targetYaw && targetYaw < rpy.yaw + precision) {
				state = ENROUTE_STATE;
			}

			csvWritePathLog(begin, 1);
			break;
		}
		case ENROUTE_STOPPED_STATE: {
			csvFile.close();
			state = HOVER_STATE;
			break;
		}
		case START_HOVER_TEST_STATE: {
			std::cout << "Initialise hover test log file" << std::endl;
			imuSample = 0;
			begin = ros::Time::now();

			// Log to csv file
			csvFile << "Time , " 
				<< "ImuRoll , " 
				<< "ImuPitch , " 
				<< "ImuYaw , " 
				<< "Latitude , " 
				<< "Longitude , " 
				<< "Altitude , " 
				<< "Thrust , "
			    << "TargetRoll , " 
				<< "targetPitch , " 
				<< "targetYaw , " 
				<< "targetLatitude , " 
				<< "targetLongitude , " 
				<< "targetAltitude , " 
				<< "targetThrust , "
				<< "errorLatitude , " 
				<< "errorLongitude , " 
				<< "errorAltitude , " 
				<< "errorYaw" 
				<< "\n";
		
			drone->setTargetValues(0, 0, 45, rpy.yaw*180/3.14, 35);
			
			state = HOVER_LOGGING_STATE;
			break;
		}
		case HOVER_LOGGING_STATE: {
			drone->updateTargetLatLon();
			drone->calculateError();
			drone->getError(&errorData);
			
			drone->runPIDController();

			//targetYaw = drone->getTargetYaw();
			targetThrust = drone->getTargetThrust();
			drone->getTargetGPS(&targetGPSData);
			drone->getTargetRPY(&targetRPY);
			
			csvWritePathLog(begin, 1);
			
			drone->pubTargetValues();
			
			//if (imuReady(imuSample, 2000)) {
				//state = STOP_HOVER_LOGGING_STATE;
			//}
			break;
		}
		case STOP_HOVER_LOGGING_STATE: {
			std::cout << "Closing hover test log file" << std::endl;
			csvFile.close();
			state = HOVER_STATE;
			break;
		}
	}
}

int main(int argc, char** argv)
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
    Server server(SERVER_PORT, drone, &csvFile);

    // Set ROS refresh rate.
    // TODO: Would be nice to have this in initROS()
    ros::Rate rate(REFRESH_RATE_HZ);
    
    drone->initPIDValues();

	// Superloop - runs at 50Hz (ROS)
    while (ros::ok()) 
	{
		// Update drone position and orientation on client.
		drone->getRPY(&rpy);
    	drone->getGPSData(&gps_data);

        // if not connected
        if (!server.connected()) {
	    	//std::cout << "waiting for connection..." << std::endl;
        	server.AcceptConnection(); // Accept any incoming connection.
        }
        else 
        {
        	// std::cout << "handling a connection..." << std::endl;
	    	// handle the current connection and update state
        	server.HandleConnection(&state);
        	
			// Instantiate and send the update message.
        	// TODO: Why 5??
        	if (imuReady(imuSample, 20)) {
				UpdateMessage updMsg;
        		updMsg.roll = rpy.roll;
	    		updMsg.pitch = rpy.pitch;
				updMsg.yaw = rpy.yaw;
				updMsg.thrust = drone->getTargetThrust();
				updMsg.lat = (float)gps_data.latitude;
				updMsg.lon = (float)gps_data.longitude;
				updMsg.alt = (float)gps_data.altitude;
			    updMsg.state = state;
				server.Send(updMsg);
				
        	}
    	}
		// TODO: Implement finite state machine here.
		updateState();
		
		// Update properties and restart the loop.
		imuSample++;
		ros::spinOnce();
    	rate.sleep();
    }

	// Stop the program when the while loop is broken.
	terminate();
}
