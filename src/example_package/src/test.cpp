// Local files
#include "ros/ros.h"

// Sensor mesgs
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

// Misc
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

// Service client
ros::ServiceClient localServiceClient;

// Topic publisher
ros::Publisher move;

// Used to write imu data to csv file.
int imuSample = 0;

// Run mainloop at this hz
#define REFRESH_RATE_HZ 50

// Imu and Gps data.
std::ofstream csvFile;
float imuRoll = 0;
float imuPitch = 0;
float imuYaw = 0;

//Control input (The angles we send to flight control)
float roll = 0;
float pitch = 0;
float yaw = 0;
float thrust = 0;


//Callback event for dji_sdk/imu subscription event.
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	// Imu data in Quaternions.
    float x = msg->orientation.x;
	float y = msg->orientation.y;
	float z = msg->orientation.z;
	float w = msg->orientation.w;

	// Convert quaternions to roll,pitch and yaw.
	// Roll (x-axis)
	imuRoll = atan2( 2 * (w * x + y * z) , 1 - 2 * (x * x + y * y) );
	
	// Pitch (y-axis)
	imuPitch = 2 * atan2( sqrt(1 + 2 * (w * y - x * z)) , sqrt(1 - 2 * (w * y - x * z)) ) - M_PI / 2; 

	// Yaw (z-axis)
	imuYaw = atan2( 2 * (w * z + x * y) , 1 - 2 * (y * y + z * z) );
}

// Publish message of type sensor::Joy drone orientation/angles (roll,pitch, thrust and yaw) with a given flag. 
void pubAngle(int roll,int pitch,int thrust,int yaw,int flag) {
	sensor_msgs::Joy controlData;
	controlData.axes.push_back(roll);
	controlData.axes.push_back(pitch);
	controlData.axes.push_back(thrust);
	controlData.axes.push_back(yaw);
	controlData.axes.push_back(flag);

	move.publish(controlData);
}


// Write imu data to csv file
void csvWrite(ros::Time begin, int sampleRate)
{	
	if (imuSample % sampleRate == 0)
	{
		ros::Duration timeDiff = ros::Time::now() - begin;
  		csvFile << timeDiff << " , "<< imuRoll << " , " << imuPitch << " , " << imuYaw << "\n";
	}
  
	imuSample += 1;
}

// argc : argument count, number of arguments passed to the node
// argv : argument vector, an array contaning the individual arguments passed to node. 
int main(int argc, char **argv)
{
	// Starting ros
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;

    if (argc != 5) 
    {	
        std::cout << "Error : test needs 4 arguments : Roll/Pitch angle Thrust fileName\n";
    }
    else
    {
        csvFile.open(argv[4]);

        if (argv[1] == "roll") {
            roll = std::stof(argv[2]); 
        } else if (argv[1] == "pitch") {
            pitch = std::stof(argv[2]); 
        }
            
        thrust = std::stof(argv[3]);
    }
  
	// Telling ros master that move is a message of type sensor_msgs/Joy and is released on topic ..generic
	move = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);

	// Subcribing to sensor data 
	ros::Subscriber imuDataSub = nh.subscribe("dji_sdk/imu", 1, imuCallback); //Imu

	// Set rate of whileloop. controlData needs to be published at 50 hertz (Need source for this).
	ros::Rate rate(REFRESH_RATE_HZ);

	// Time at start, used for csv logging of imu data
	ros::Time begin = ros::Time::now();
	csvFile << "    Time , " << "ImuRoll , " << "ImuPitch , " << "ImuYaw" << "\n";
  
	// Tick loop time 20 ms (50 hz)
	while(ros::ok())
  	{
		ros::spinOnce();
	
		ROS_INFO("imu roll [%f]", imuRoll);
		ROS_INFO("imu pitch [%f]", imuPitch);
		ROS_INFO("imu yaw [%f]", imuYaw);
	
		// Write every 3 imu data to csv file. (begin time, sample rate, file object to write to)
 		csvWrite(begin, 3);

        if (argv[1]=="roll"){
            pitch = imuPitch; 
        } else if (argv[1]=="pitch")
            roll = imuRoll; 

        yaw = imuYaw; 
            
		pubAngle(roll,pitch,thrust,yaw,35); //(Flag 35 for stable mode, 34 unstable)

		// Sleep, so while loop reach desired hertz.
		rate.sleep();
  	}

	return 0;
}