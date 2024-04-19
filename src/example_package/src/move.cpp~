// Local files
#include "ros/ros.h"
//#include "../include/xmlparser.hpp"

// Sensor mesgs
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

// Service
#include <dji_sdk/SetLocalPosRef.h>
#include <geometry_msgs/PointStamped.h>

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
std::ofstream csvFile("ImuCsvTest.csv");
int imuSample = 0;

// Run mainloop at this hz
#define REFRESH_RATE_HZ 50

// Imu and Gps data.
float imuRoll = 0;
float imuPitch = 0;
float imuYaw = 0;

double latitude = 0;
double longitude = 0;
double altitude = 0;

// Local ENU (East,North,Up) position in meters.
float localPosX = 0;
float localPosY = 0;
float localPosZ = 0;

// Gps health (Amount of connected satellites)
int gpsHealth = 0;

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


//Callback event for dji_sdk/gps_position subscription event.
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	// Gps position.
        latitude = msg->latitude;
	longitude = msg->longitude;
	altitude = msg->altitude;
}


// Callback event for dji_sdk/local_position. Get the current local position.
void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 
	// Local ENU (East,North,Up) position in meters. Calculated realative on user defined ref point. 
	localPosX = msg->point.x; 
	localPosY = msg->point.y;
	localPosZ = msg->point.z;
}


// Callback event for dji_sdk/gps_healt. Get amount of connected satellites. 
void gpsHealthCallback(const std_msgs::UInt8::ConstPtr& msg)
{ 
	gpsHealth = msg->data;
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


// Call dji_sdk/set_local_pos_ref service, to set the local reference point.
bool setLocalPosition()
{
	dji_sdk::SetLocalPosRef localPosReferenceSetter;
 
	localServiceClient.call(localPosReferenceSetter);

	return (bool)localPosReferenceSetter.response.result;
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

int wayPointStatus(float accept) 
{
	// temp placholder values
	float longi = 56.1;
	float lati = 9.94;

	// Calculate if the drone is within the boundary box
	
		

	// Check if the drone has reached the waypoint
 	if ( abs(longi - longitude) < accept & abs(lati - latitude) < accept) 
	{

	}

	// Calculate heading to next waypoint 
	int heading = atan2(lati - latitude, longi - longitude);

	return heading;
}

void log (const std::string& text) {
	std::cout << text << std::endl;
}

// argc : argument count, number of arguments passed to the node
// argv : argument vector, an array contaning the individual arguments passed to node. 
int main(int argc, char **argv)
{
	log("Starting XML parser...");
	//initXMLParser("Hello World");

	// Starting ros
	ros::init(argc, argv, "move_node");
	ros::NodeHandle nh;
  
	// Check if node recived right amount of input
	if (argc != 5) 
	{	
  	std::cout << "Error : move needs 4 arguments : Roll Pitch Yaw Thrust\n" ;
	}
	else
	{
		// Cast node input as float
  		roll = std::stoi(argv[1]); 
  		pitch = std::stoi(argv[2]);
  		yaw = std::stoi(argv[3]);
  		thrust = std::stoi(argv[4]); 
	}

	// Telling ros master that move is a message of type sensor_msgs/Joy and is released on topic ..generic
	move = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);

	// Subcribing to sensor data 
	ros::Subscriber imuDataSub = nh.subscribe("dji_sdk/imu", 1, imuCallback); //Imu
	ros::Subscriber gpsDataSub = nh.subscribe("dji_sdk/gps_position", 1, gpsCallback); //Gps
	ros::Subscriber gpsHealthSub = nh.subscribe("dji_sdk/gps_health", 1, gpsHealthCallback); //Gps Health
	ros::Subscriber localPositionSub = nh.subscribe("dji_sdk/local_position", 1, localPositionCallback);

	// Clients
	localServiceClient = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

	// Keep attempting to set local ref point until success.
	ros::Rate satRate(1);
	while(ros::ok() & !setLocalPosition())
  	{
		ros::spinOnce();

  		ROS_INFO("Gps health : %i , need atleast 4 to continue", gpsHealth);

		satRate.sleep();
	}

	// Set rate of whileloop. controlData needs to be published at 50 hertz (Need source for this).
	ros::Rate rate(REFRESH_RATE_HZ);

	// Time at start, used for csv logging of imu data
	ros::Time begin = ros::Time::now();
	csvFile << "    Time , " << "ImuRoll , " << "ImuPitch , " << "ImuYaw" << "\n";
  
	// Tick loop time 20 ms (50 hz)
	while(ros::ok())
  	{
		ros::spinOnce();
		
		ROS_INFO("Local x [%f]", localPosX);
		ROS_INFO("Local y [%f]", localPosY);
		ROS_INFO("Local z [%f]", localPosZ);

		//ROS_INFO("Latitude: [%f]",latitude);
		//ROS_INFO("Longitude: [%f]",longitude);
		//ROS_INFO("Altitude: [%f]",altitude);
	
		//ROS_INFO("imu roll [%f]", imuRoll);
		//ROS_INFO("imu pitch [%f]", imuPitch);
		//ROS_INFO("imu yaw [%f]", imuYaw);
		
		// Write every 3 imu data to csv file. (begin time, sample rate)
 		// csvWrite(begin,3);
 	
		pubAngle(roll,pitch,thrust,yaw,35); //(Flag 35 for stable mode, 34 unstable)

		// Sleep, so while loop reach desired hertz.
		rate.sleep();
  	}

	return 0;
}



