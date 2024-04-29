// Misc
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include "../include/Matrice100.hpp"



//Constructor
Matrice100::Matrice100(ros::NodeHandle *nodehandle) : _nh(*nodehandle) {
	
	//Initialise subscribers, publishers and services
	arm_client = _nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
	permission_client = _nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
	taskControl_client = _nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

	//Notice that the callback reference is different when pointing to a member function instead of a normal function
	//Read more at https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
	imuDataSub = _nh.subscribe("dji_sdk/imu", 1, &Matrice100::imuCallback, this);
	gpsDataSub = _nh.subscribe("dji_sdk/gps_position", 1, &Matrice100::gpsCallback, this);
	flightStatusSub = _nh.subscribe("dji_sdk/flight_status", 1, &Matrice100::flightStatusCallback, this);
	BatteryVoltageSub = _nh.subscribe("dji_sdk/battery_state", 1, &Matrice100::batteryVoltageCallback, this);
	velocitySub = _nh.subscribe("dji_sdk/velocity", 1, &Matrice100::velocityCallback, this);

	move = _nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);
	
}


//Callback event for dji_sdk/imu subscription event.
void Matrice100::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
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
void Matrice100::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	// Gps position.
	latitude = msg->latitude;
	longitude = msg->longitude;
	altitude = msg->altitude;
}

//Callback event for dji_sdk/gps_position subscription event.
void Matrice100::flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg) {
	// Drone flight status.
	flightStatus = msg->data;
}

void Matrice100::batteryVoltageCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
	batteryVoltage = msg->voltage;
}

void Matrice100::velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	x_vel = msg->vector.x;
	y_vel = msg->vector.y;
	z_vel = msg->vector.z;
}

// Publish message of type sensor::Joy drone orientation/angles (roll,pitch, thrust and yaw) with a given flag.
void Matrice100::pubAngle(float roll,float pitch, float thrust, float yaw,int flag) {

	// Push to struct
	controlData.axes.push_back(roll);
	controlData.axes.push_back(pitch);
	controlData.axes.push_back(thrust);
	controlData.axes.push_back(yaw);
	controlData.axes.push_back(flag);

	// Publish to flight topic
	move.publish(controlData);
}

void Matrice100::getRPY(RPY* rpy_struct) {
	rpy_struct->roll = imuRoll;
	rpy_struct->pitch = imuPitch;
	rpy_struct->yaw = imuYaw;
}

void Matrice100::getGPSData(GPS_Data* gps_struct) {
	gps_struct->latitude = latitude;
	gps_struct->longitude = longitude;
	gps_struct->altitude = altitude;
}

int Matrice100::getFlightStatus() {
	return flightStatus;
}

float Matrice100::getBatteryVoltage() {
	return batteryVoltage;
}

void Matrice100::getVel(VEL* vel_struct) {
	vel_struct->x = x_vel;
	vel_struct->y = y_vel;
	vel_struct->z = z_vel;
}

int  Matrice100::arm(int arm_drone) {
	arm_srv.request.arm = arm_drone;	

	if (arm_client.call(arm_srv))
	{
	  ROS_INFO("Sum: %ld", (long int)arm_srv.response.result);
	  return 1;
	}
	else
	{
	  ROS_ERROR("Failed to call service arm");
	  return 0;
	}
}

int Matrice100::request_permission(int permission) {
	permission_srv.request.control_enable = permission;

	if (permission_client.call(permission_srv))
	{
	  ROS_INFO("Sum: %ld", (long int)permission_srv.response.result);
	  return 1;
	}
	else
	{
	  ROS_ERROR("Failed to call sdk authority service");
	  return 0;
	}

	
}

int Matrice100::takeOff() {
	task_srv.request.task = 4;

	if (taskControl_client.call(task_srv))
	{
	  ROS_INFO("Sum: %ld", (long int)task_srv.response.result);
	  return 1;
	}
	else
	{
	  ROS_ERROR("Failed to call flight control");
	  return 0;
	}

}

int Matrice100::land() {
	task_srv.request.task = 6;

	if (taskControl_client.call(task_srv))
	{
	  ROS_INFO("Sum: %ld", (long int)task_srv.response.result);
	  return 1;
	}
	else
	{
	  ROS_ERROR("Failed to call flight control");
	  return 0;
	}

}

int Matrice100::goHome() {
	task_srv.request.task = 1;

	if (taskControl_client.call(task_srv))
	{
	  ROS_INFO("Sum: %ld", (long int)task_srv.response.result);
	  return 1;
	}
	else
	{
	  ROS_ERROR("Failed to call flight control");
	  return 0;
	}

}







