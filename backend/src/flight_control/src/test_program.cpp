#include "../include/Matrice100.hpp"
#include "../include/netserver.hpp"

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#define REFRESH_RATE_HZ 50

// Used to write imu data to csv file.
int imuSample = 0;

// Imu and Gps data.
std::ofstream csvFile;

// Data structs to save roll pitch adn yaw as well as gps data
// Declared in Matrice100.hpp
RPY rpy;
GPS_Data gps_data;


// Make the matrice100 object globally available by making a pointer to the object
// Notice that the object is first initialised in the main function using the "new" command
Matrice100* drone;

//Control input (The angles we send to flight control)
float roll = 0;
float pitch = 0;
float yaw = 0;
float thrust = 0;



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


// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   ROS_INFO("The function runs");
	
   drone->arm(0);
   // Terminate program
   exit(signum);
}



int main(int argc, char **argv)
{
	// Starting ros
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;

	// Initialise the Matrice100 object (calling its constructor)
	drone = new Matrice100(&nh);

	ros::Rate rate(REFRESH_RATE_HZ);

	// Initiliasing stuff to catch CTRL+C action in terminal
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = signal_callback_handler;
   	sigemptyset(&sigIntHandler.sa_mask);
   	sigIntHandler.sa_flags = 0;
   	sigaction(SIGINT, &sigIntHandler, NULL);

	// Sanity check	
	if (argc != 5) 
	{	
	    std::cout << "Error : test needs 4 arguments : Roll/Pitch angle Thrust fileName\n";
	}
	else
	{
	    csvFile.open(argv[4]);

		
	    //Set desired orientation value
	    if (std::string(argv[1]) == "roll") {
	        roll = std::stof(argv[2]) * 3.14/180; 
	        ROS_INFO("roll set to [%f]", roll);
	    } else if (std::string(argv[1]) == "pitch") {
	        pitch = std::stof(argv[2]) * 3.14/180; 
		ROS_INFO("pitch set to [%f]", pitch);
	    } else if (std::string(argv[1]) == "yaw") {
	        yaw = std::stof(argv[2]) * 3.14/180; 
		ROS_INFO("yaw set to [%f]", yaw);
	    }
            
	    thrust = std::stof(argv[3]);
        }

	// Connect to ground client and initialise server thread
	//startserver();


	// Time at start, used for csv logging of imu data
	ros::Time begin = ros::Time::now();
	csvFile << "    Time , " << "ImuRoll , " << "ImuPitch , " << "ImuYaw" << "Latitude , " << "Longitude , " << "Altitude , " << "Thrust" << "\n";


	// Tick loop time 20 ms (50 hz)
	while(ros::ok())
  	{
		ros::spinOnce();

		drone->getRPY(&rpy);
		drone->getGPSData(&gps_data);

		ROS_INFO("imu roll [%f]", rpy.roll);
		ROS_INFO("imu pitch [%f]", rpy.pitch);
		ROS_INFO("imu yaw [%f]", rpy.yaw);

		//ROS_INFO("roll [%f]", roll);
		//ROS_INFO("pitch [%f]", pitch);
		//ROS_INFO("yaw [%f]", yaw);

		// Write every 3 imu data to csv file. (begin time, sample rate)
 		csvWrite(begin, 3);

		// Depending on axis set the other to be the same as what the IMU reads
		// In the test bench it can only correct its orientation in one axis
        	if (std::string(argv[1])=="roll"){
            		pitch = rpy.pitch;
			yaw = rpy.yaw; 
        	} else if (std::string(argv[1])=="pitch"){
            		roll = rpy.roll;
			yaw = rpy.yaw; 
		} else if (std::string(argv[1])=="yaw"){
            		roll = rpy.roll;
			pitch = rpy.pitch;
		}

		// Publish desired angles            
		drone->pubAngle(roll,pitch,thrust,yaw, 33); //(Flag 35 for stable mode, 34 unstable)

		// Sleep, so while loop reach desired hertz.
		rate.sleep();
  	}
}



