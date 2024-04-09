#include "ros/ros.h"

// Sensor mesgs
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>

// Service
#include <dji_sdk/SetLocalPosRef.h>

// Misc
#include <string>
#include <vector>
#include <cmath>

// Service client
ros::ServiceClient set_local_pos_reference;

// Topic publisher
ros::Publisher move;

#define REFRESH_RATE_HZ 40

// Global Imu and Gps data.
float imuRoll = 0.0;
float imuPitch = 0.0;
float imuYaw = 0.0;

float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;

// Local pos variable msg
geometry_msgs::PointStamped local_position;

/*
* Callback event for dji_sdk/imu subscription event (pub/sub).
* NOTE: This has to be here because NodeHandler.Subscribe requires a callback handler.
*/
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	// ROS_INFO("[Orientation] x: %f, y: %f, z: %f, w: %f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w); // In Quaternions
	// Imu data in Quaternions.
        float x = msg->orientation.x;
	float y = msg->orientation.y;
	float z = msg->orientation.z;
	float w = msg->orientation.w;

	// Convert to roll,pitch and yaw.
	// Roll (x-axis)
	imuRoll = atan2( 2 * (w * x + y + z) , 1 - 2 * (x * x + y * y) );
	
	// Pitch (y-axis)
	imuPitch = 2 * atan2( sqrt(1 + 2 * (w * y - x * z)) , sqrt(1 - 2 * (w * y - x * z)) ) - M_PI / 2; 

	// Yaw (z-axis)
	imuYaw = atan2( 2 * (w * z + x * y) , 1 - 2 * (y * y + z * z) );
}

/*
* Callback event for dji_sdk/gps_position subscription event (pub/sub)
*/
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	// Gps position in ??? unit-
        latitude = msg->latitude;
	longitude = msg->longitude;
	altitude = msg->altitude;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  local_position = *msg;
}

/*
* Publish message of type sensor::Joy drone orientation/angles (roll,pitch,thrust and yaw) 
* with a given flag. 
*/
void pubAngle(int roll,int pitch,int thrust,int yaw,int flag) {
	// Make a sensor_msgs/Joy message, populate it, and publish.
	sensor_msgs::Joy controlData;
	controlData.axes.push_back(roll);
	controlData.axes.push_back(pitch);
	controlData.axes.push_back(thrust);
	controlData.axes.push_back(yaw);
	controlData.axes.push_back(flag);

	move.publish(controlData);
}

// argc : argument count, number of arguments passed to the node
// argv : argument vector, an array contaning the individual arguments passed to node. 
int main(int argc, char **argv)
{
  // Starting ros
  ros::init(argc, argv, "move_node");
  ros::NodeHandle nh;
  
  // Telling ros master that move is a message of type sensor_msgs/Joy and is released on topic ..generic
  move = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  // Subcribing to sensor data 
  ros::Subscriber imuData = nh.subscribe("dji_sdk/imu", 1, imuCallback); //Imu
  ros::Subscriber gpsData = nh.subscribe("dji_sdk/gps_position", 1, gpsCallback); //Gps
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 1, &local_position_callback);

  // Clients
  set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  // Set rate of whileloop. controlData needs to be published at 40-50 hertz (Need source for this).
  ros::Rate rate(REFRESH_RATE_HZ);
  
  // Tick loop time 25 ms (40 hz)
  while(ros::ok())
  {
	// Update subscribers	
	ros::spinOnce();

	ROS_INFO("Latitude: [%f]",latitude);
	ROS_INFO("Longitude: [%f]",longitude);
	ROS_INFO("Altitude: [%f]",altitude);
	
	//ROS_INFO("imu roll [%f]", imuRoll);
	//ROS_INFO("imu pitch [%f]", imuPitch);
	//ROS_INFO("imu yaw [%f]", imuYaw);

	pubAngle(0,0,1,0,35); // (Roll,Pitch,Thrust,Yaw,Flag) (Flag 35 for stable mode, 34 unstable)

	// Sleep, so while loop reach desired hertz.
	rate.sleep();
  }
  
  
  return 0;
}



