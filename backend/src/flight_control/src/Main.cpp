#include "Server.h"
#include <iostream>
#include <fstream>
#include "../include/Matrice100.hpp"
#include "../include/Main.hpp"
#include "Message.hpp"
#include <opencv2/opencv.hpp>


#define REFRESH_RATE_HZ 50


// Data structs to save roll pitch adn yaw as well as gps data
// Declared in Matrice100.hpp
RPY rpy;
TargetRPY targetRPY;
GPS_Data gps_data;
TargetGPS targetGPSData;
Error errorData;


// Make the matrice100 object globally available by making a pointer to the object
// Notice that the object is first initialised in the main function using the "new" command
Matrice100* drone;

// Used to write imu data to csv file.
int imuSample = 0;
float target = 0;

// FileHandler.
std::ofstream csvFile;

std::string message;

ros::Time begin;

//Used for storing target thrust to be written to csv file
float thrust = 0;
float targetThrust = 0;
float targetYaw = 0;

// Used to capture images
bool _captureDeviceReady = false;


// Write imu data to csv file
void csvWrite(ros::Time begin, int sampleRate)
{	
	if (imuSample % sampleRate == 0)
	{
		ros::Duration timeDiff = ros::Time::now() - begin;
  		csvFile << timeDiff << " , "<< rpy.roll << " , " << rpy.pitch << " , " << rpy.yaw << " , " << gps_data.latitude << " , " << gps_data.longitude << " , " << gps_data.altitude << " , " << thrust << "\n";
	}
  
	imuSample += 1;
}

// Write imu data to csv file
void csvWritePathLog(ros::Time begin, int sampleRate)
{	
	if (imuSample % sampleRate == 0)
	{
		ros::Duration timeDiff = ros::Time::now() - begin;
  		csvFile << timeDiff << " , "<< rpy.roll << " , " << rpy.pitch << " , " << rpy.yaw << " , " << gps_data.latitude << " , " << gps_data.longitude << " , " << gps_data.altitude << " , " << thrust
                            << " , "<< targetRPY.roll << " , " << targetRPY.pitch << " , " << targetRPY.yaw << " , " << targetGPSData.latitude << " , " << targetGPSData.longitude << " , " << targetGPSData.altitude << " , " << targetThrust
                            << " , " << errorData.errorLat << " , " << errorData.errorLon << " , " << errorData.errorAlt << " , " << errorData.errorYaw << "\n";
	}

	imuSample += 1;
}

//Inspired from finite statemachines in PLC workshop the code runs in states
int state = GROUNDED_STATE;



int main(int argc, char **argv)
{

    // Starting ros
    ros::init(argc, argv, "flight_control_node");
    ros::NodeHandle nh;

    // Initialise the Matrice100 object (calling its constructor)
    drone = new Matrice100(&nh);

    Server server(8888, drone, &csvFile);

    ros::Rate rate(REFRESH_RATE_HZ);

    std::cout << "Starting drone camera initialization..." << std::endl;
    int id = 2;
    cv::VideoCapture cap(id);

    if (!cap.isOpened()) {
	std::cerr << "Error: Unable to open camera (" << std::to_string(id) << ")" << std::endl;
	_captureDeviceReady = false;
    } else {
	std::cout << "Capture device is open" << std::endl;
	_captureDeviceReady = true;
    }
    cv::Mat frame;

    //Main loop running at given frequency until ROS is closed
    while (ros::ok()) 
    {
	if (_captureDeviceReady) {
		// Grab the latest frame from a VideoCapture device.
		cap >> frame;
		if (frame.empty()) {
			std::cerr << "Error: Unable to capture frame" << std::endl;
		}
	}
	if (_captureDeviceReady) {
		// Grab the latest frame from a VideoCapture device.
		cap >> frame;
		if (frame.empty()) {
			std::cerr << "Error: Unable to capture frame" << std::endl;
		}
	}

	// Update drone position and orientation on client.
	drone->getRPY(&rpy);
    	drone->getGPSData(&gps_data);
	// Instantiate tick messages.
	UpdateMessage updMsg;
	ImageMessage imgMsg;
	

        // if not connected
        if (!server.connected()) 
        {
	    std::cout << "waiting for connection..." << std::endl;
            // see if theres a connection
            server.AcceptConnection();
        }
        else 
        {
	    // handle the current connection and update state
            server.HandleConnection(&state);
	    
	    // Convert frame to byte array and populate image message
	    /*if (!frame.empty()) {
		std::cout << "pre-yoink!" << std::endl;
		std::vector<unsigned char> imgBytesVec;
		cv::imencode(".jpg", frame, imgBytesVec);
		unsigned char* imgBytes = imgBytesVec.data();
		imgMsg.imgBytes = imgBytes;
		imgMsg.len = imgBytesVec.size();
		server.Send(imgMsg);
		std::cout << "yoink!" << std::endl;
	    }
	    else {
		std::cerr << "Error: Unable to capture frame" << std::endl;
	    }*/
		
	    
	if (imuSample % 5 == 0)
	{

    	    // Populate update message
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

        //Act based on current state of operation
        switch (state) 
        {
        case GROUNDED_STATE: 
        {
            //message = "Waiting for command from client";
	    
            //std::cout << message << std::endl;
            break;
        }
        case ARMED_STATE:
        {
            //message = "Drone armed and ready for action";

            //std::cout << message << std::endl;
            break;
        }
        case HOVER_STATE:
        {
            //message = "Drone hovering and waiting for instructions";

            //std::cout << message << std::endl;
            break;
        }
        case PUBLISH_ANGLE_STATE:
        {
            //Publish target values at each loop iteration
            //Targets values are set using drone->setTargetValues(...)
            drone->pubTargetValues();
            break;
        }
        case START_TEST_STATE:
        {
            //csvFile has been opened on serverside and can no be written to

            // Time at start, used for csv logging of imu data
            begin = ros::Time::now();
            csvFile << "    Time , " << "ImuRoll , " << "ImuPitch , " << "ImuYaw , " << "Latitude , " << "Longitude , " << "Altitude , " << "Thrust" << "\n";

            //Publish target values at each loop iteration
            //Targets values are set using drone->setTargetValues(...)
            drone->pubTargetValues();

            state = RUNNING_TEST_STATE;

            break;
        }
        case RUNNING_TEST_STATE:
        {
            //Write to csvfile
            csvWrite(begin, 3);

            //Publish target values at each loop iteration
            //Targets values are set using drone->setTargetValues(...)
            drone->pubTargetValues();
            break;
        }
        case STOP_TEST_STATE:
        {
            //Close test file
            csvFile.close();
            state = HOVER_STATE;
            break;
        }

	    case INITIALISE_ENROUTE_STATE:
	{
		begin = ros::Time::now();

		csvFile << "Time , " << "ImuRoll , " << "ImuPitch , " << "ImuYaw" << "Latitude , " << "Longitude , " << "Altitude , " << "Thrust , "
			                << "TargetRoll , " << "targetPitch , " << "targetYaw , " << "targetLatitude , " << "targetLongitude , " << "targetAltitude , " << "targetThrust , "
					<< "errorLatitude , " << "errorLongitude , " << "errorAltitude , " << "errorYaw" << "\n";

	
		//drone->loadRouteFromGPX(filePath)
		//drone->calculateInterpolations(desired_vel, update_frequency)
		drone->startMission();
	
		state = ENROUTE_STATE;

		break;
	}

	    case ENROUTE_STATE:
	{
		drone->updateTargetPoints(); 
		drone->calculateError();

		drone->getError(&errorData);

		drone->runPIDController(); //function that has PID implemented and updates controlData struct

		//Publish new data
		drone->pubTargetValues();
	
		if (drone->getTrackState() == 1) {
		    state = ENROUTE_TURN_STATE;
		} 
		else if (drone->getTrackState() == 2) {
		    state = ENROUTE_STOPPED_STATE;
		}

		csvWritePathLog(begin, 1);
		
		break;
	}

	    case ENROUTE_TURN_STATE:
	{
		drone->calculateError();
		drone->getError(&errorData);

		drone->runPIDController();

		targetYaw = drone->getTargetYaw();

		if (rpy.yaw-0.052 < targetYaw && targetYaw < rpy.yaw+0.052) {
		    state = ENROUTE_STATE;
		}

		csvWritePathLog(begin, 1);
		
		break;
	}
	    case ENROUTE_STOPPED_STATE:
	{
		csvFile.close();
		state = HOVER_STATE;

            	break;
	}
        //}
        }
	
	imuSample++;

	//Update all topics and services
	ros::spinOnce();
	
    	rate.sleep();

	
    }

	cap.release();
	std::cout << "Press any key to stop the computer vision capture" << std::endl;
	cv::waitKey(0);
	cv::destroyAllWindows();
}
