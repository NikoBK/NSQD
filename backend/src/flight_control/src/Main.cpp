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
GPS_Data gps_data;

// Make the matrice100 object globally available by making a pointer to the object
// Notice that the object is first initialised in the main function using the "new" command
Matrice100* drone;

// Used to write imu data to csv file.
int imuSample = 0;

// FileHandler.
std::ofstream csvFile;

std::string message;

ros::Time begin;

//Used for storing target thrust to be written to csv file
float thrust = 0;

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

	// Update drone position and orientation on client.
	drone->getRPY(&rpy);
    	drone->getGPSData(&gps_data);

	// Instantiate tick messages.
	UpdateMessage updMsg;
	ImageMessage imgMsg;	

        //Update all topics and services
	ros::spinOnce();

        // if not connected
        if (!server.connected()) 
        {
            // see if theres a connection
            server.AcceptConnection();
        }
        else 
        {
	    // handle the current connection and update state
            server.HandleConnection(&state);
	    
	    // Convert frame to byte array and populate image message
	    if (_captureDeviceReady && !frame.empty()) {
		std::vector<unsigned char> imgBytes;
		cv::imencode(".jpg", frame, imgBytes);
		imgMsg.imgBytes = imgBytes;
		imgMsg.len = imgBytes.size();
		server.Send(imgMsg);
	    }
	    else {
		std::cerr << "Error: Unable to capture frame" << std::endl;
	    }

            // handle the current connection and update state
            server.HandleConnection(&state);

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
            csvFile << "    Time , " << "ImuRoll , " << "ImuPitch , " << "ImuYaw" << "Latitude , " << "Longitude , " << "Altitude , " << "Thrust" << "\n";

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
        case ENROUTE_STATE:
        {
            //The code that will try to follow the uploaded route
            //Something like (pseudocode)
            /*
            case INITIALISE_ENROUTE_STATE:
                drone->loadRouteFromGPX(filePath)
                drone->calculateInterpolations(desired_vel, update_frequency)
                drone->startMission()
                
                state = ENROUTE_STATE

            case ENROUTE_STATE:
                drone->calculateError()
                error = drone->getError()
                csvFile.write(error)

                drone->updateTargetValues() //function that has PID implmented and updates controlData struct

                //Publish new data
                drone->pubTargetValues()

                //Somewhere we must take pictures as well...

            case ENROUTE_STOPPED_STATE:
                drone->calculateError()
                error = drone->getError()
                csvFile.write(error)

                drone->updateTargetValues() //function that has PID implmented and updates controlData struct

                //Publish new data
                drone->pubTargetValues()
            
            */

            break;
        }
        }
    	rate.sleep();
    }
	cap.release();
	std::cout << "Press any key to stop the computer vision capture" << std::endl;
	cv::waitKey(0);
	cv::destroyAllWindows();
}
