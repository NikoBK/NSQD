#include "Server.h"
#include <iostream>
#include <fstream>
#include "../include/Matrice100.hpp"
#include "../include/Constants.hpp"
#include "Message.hpp"
#include <opencv2/opencv.hpp>
//#include <opencv2/imgcodecs.hpp>

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
int imgNum = 0;

// DATA
//Used for storing target thrust to be written to csv file
float initial_alt = 0;
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
  		<< " , " << std::fixed << std::setprecision(7) << gps_data.latitude 
  		<< " , " << std::fixed << std::setprecision(7) << gps_data.longitude 
  		<< " , " << gps_data.altitude 
  		<< " , " << thrust
        << " , " << targetRPY.roll 
        << " , " << targetRPY.pitch 
        << " , " << targetRPY.yaw 
        << " , " << std::fixed << std::setprecision(7) << targetGPSData.latitude 
        << " , " << std::fixed << std::setprecision(7) << targetGPSData.longitude 
        << " , " << targetGPSData.altitude 
        << " , " << targetThrust
        << " , " << std::fixed << std::setprecision(7) << errorData.errorLat 
        << " , " << std::fixed << std::setprecision(7) << errorData.errorLon 
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

/*
bool captureFrame() {
	cv::Mat frame;
	
	cap >> frame;
	
	std::string dir = "images";
	std::string prefix = "IMG";
	std::string filepath;
	std::string filename;
	std::string extension = ".jpg";
	
	do {
		std::ostringstream oss;
		filename = oss << prefix << std::setfill('0') << std::setw(6) << std::rand() % 100000 << extension;
		filepath = "flight_footage/" + filename;
	} while (fs::exists(filepath));
	
	cv::imwrite(filepath, frame);
	
	imgNum++;
	return true;
}
*/

/** Capture the current frame and process it.
*/
bool captureFrame() {
	cv::Mat frame;
	
	cap >> frame;
	
	const std::string filename = "flight_footage/image_" + std::to_string(imgNum) + ".png"; 
	
	cv::imwrite(filename, frame); 
	
	imgNum++;
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
			//drone->updateTargetYaw();
			//targetYaw = drone->getTargetYaw();
			//std::cout << "Target yaw: " << targetYaw << '\n';
			
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
		case CAPTURE_IMAGES_STATE: {
			
			if (imuReady(imuSample, 100)) {     //imuReady(imuSample, 99)) {
				std::cout << "Writing frame" << "\n";
				captureFrame();
			}
			
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
			//drone->setTargetAltitude(10);
			drone->updateTargetPoints();

			state = ENROUTE_TURN_STATE;
			std::cout << "Turn state" << '\n';
			break;
		}
		case ENROUTE_STATE: {
			drone->updateTargetPoints();
			drone->calculateError();
			drone->getError(&errorData);
			drone->runPIDController(); // Function that has PID implemented and updates a controlData structure.
			drone->pubTargetValues();
			
			targetThrust = drone->getTargetThrust();
			drone->getTargetRPY(&targetRPY);
			drone->getTargetGPS(&targetGPSData);
			
			if (drone->getTrackState() == 1) {
				drone->updateTargetYaw();
				state = ENROUTE_TURN_STATE;
				std::cout << "Turn state" << '\n';
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
			drone->pubTargetValues();
			
			targetThrust = drone->getTargetThrust();
			drone->getTargetRPY(&targetRPY);
			targetYaw = drone->getTargetYaw();
			drone->getTargetGPS(&targetGPSData);
		
			std::cout << "Target Yaw: " << targetYaw << '\n';
			std::cout << "Imu Yaw: " << rpy.yaw << '\n';
			std::cout << "Target Alt: " << targetGPSData.altitude << '\n';
			std::cout << "GPS Alt: " << gps_data.altitude << '\n';
			
			std::cout << "Target lat: " << targetGPSData.latitude << '\n';
			std::cout << "Current lat: " << gps_data.latitude << '\n';
			std::cout << "Target lon: " << targetGPSData.longitude << '\n';
			std::cout << "Current lon: " << gps_data.longitude << '\n';
			
			float precision = 0.052; //rad
			float precisionAlt = 1; //m
			float precisionLati = 0.00001; //
			float precisionLongi = 0.00001; // 
			double avgPrevLati = 0;
			double avgPrevLongi = 0;
				
			drone->prevLati.push_back(gps_data.latitude);
			drone->prevLongi.push_back(gps_data.longitude);
			
			if(drone->prevLati.size() >= 50) {
				drone->prevLati.erase(drone->prevLati.begin());
				drone->prevLongi.erase(drone->prevLongi.begin());
			}
		
			for(int i = 0; i < drone->prevLati.size(); i++) {
				avgPrevLati += drone->prevLati[i];
				avgPrevLongi += drone->prevLongi[i];
			}
			
			avgPrevLati = avgPrevLati/drone->prevLati.size();
			avgPrevLongi = avgPrevLongi/drone->prevLongi.size();
			
			if (gps_data.altitude - precisionAlt < targetGPSData.altitude && targetGPSData.altitude < gps_data.altitude + precisionAlt 
				&& rpy.yaw - precision < targetYaw && targetYaw < rpy.yaw + precision
				&& avgPrevLati - precisionLati < targetGPSData.latitude && targetGPSData.latitude < avgPrevLati + precisionLati
				&& avgPrevLongi - precisionLongi < targetGPSData.longitude && targetGPSData.longitude < avgPrevLongi + precisionLongi) {
				
				state = ENROUTE_STATE;
				std::cout << "On route" << '\n';
			}
			
			//std::cout << "First entry in lati" << drone->prevLati[0] << '\n';

			csvWritePathLog(begin, 1);
			break;
		}
		case ENROUTE_STOPPED_STATE: {
			std::cout << "stop" << '\n';
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
		
			drone->setTargetValues(0, 0, 45, rpy.yaw*180/3.14, 33);
			drone->updateTargetLatLon();
			
			state = HOVER_LOGGING_STATE;
			break;
		}
		case HOVER_LOGGING_STATE: {
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
    ros::spinOnce();
    
    drone->getGPSData(&gps_data);
    initial_alt = gps_data.altitude;    
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
        	if (imuReady(imuSample, 20)) {
				UpdateMessage updMsg;
        		updMsg.roll = 1.2; //rpy.roll;
	    		updMsg.pitch = 1.5; //rpy.pitch;
				updMsg.yaw = rpy.yaw;
				updMsg.thrust = drone->getTargetThrust();
				updMsg.lat = (float)gps_data.latitude;
				updMsg.lon = (float)gps_data.longitude;
				updMsg.alt = std::fabs(gps_data.altitude-initial_alt);
			    updMsg.state = state;
			    
				server.Send(updMsg);
				
        	}
    	}
		updateState();
		
		// Update properties and restart the loop.
		imuSample++;
		ros::spinOnce();
    	rate.sleep();
    }

	// Stop the program when the while loop is broken.
	terminate();
}
