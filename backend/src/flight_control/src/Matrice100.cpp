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

	//Resize controlData to use idx instead of .pushback(value)
	controlData.axes.resize(5);
	
}


//Callback event for dji_sdk/imu subscription event.
void Matrice100::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	// Imu data in Quaternions.
    float x = msg->orientation.x;
	float y = msg->orientation.y;
	float z = msg->orientation.z;
	float w = msg->orientation.w;

	//makes it possible to make sensor fusion
	imuAccX = msg->linear_acceleration.x;
	imuAccY = msg->linear_acceleration.y;
	imuAccZ = msg->linear_acceleration.z;

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

void Matrice100::startMission() {
	// Start mission
	trackState = 0; //0 = regular flight
	lineStep = 0;
	pointStep = 0;
	errorLat = 0;
	errorLon = 0;
	errorAlt = 0;
	errorYaw = 0;
	integralLat = 0;
	integralLon = 0;
	integralAlt = 0;
	integralYaw = 0;
	derivativeLat = 0;
	derivativeLon = 0;
	derivativeAlt = 0;
	derivativeYaw = 0;
}

void Matrice100::setPIDValues(float kp, float ki, float kd, int type) {
	// Set PID values for a given axis
	pidParamsArray[type][0] = kp;
	pidParamsArray[type][1] = ki;
	pidParamsArray[type][2] = kd;
}

// Calculate PID control signal
float calculateError() {
    errorLat = targetLat - latitude;
	errorLon = targetLon - longitude;
	errorAlt = targetAlt - altitude;
	errorYaw = targetYaw - imuYaw;

	// REMEMBER TO SET TO 0!!!
	integralLat += errorLat; 
	integralLon += errorLon;
	integralAlt += errorAlt;
	integralYaw += errorYaw;

    derivativeLat = errorLat - prevErrorLat;
    prevErrorLat = errorLat;
	derivativeLon = errorLon - prevErrorLon;
	prevErrorLon = errorLon;
	derivativeAlt = errorAlt - prevErrorAlt;
	prevErrorAlt = errorAlt;
	derivativeYaw = errorYaw - prevErrorYaw;
	prevErrorYaw = errorYaw;
	
}

float Matrice100::getTargetYaw() {
	return targetYaw;
}

void Matrice100::runPIDController() {    
    // Calculate control signal
	controlData.axes[0] += pidParamsArray[0][0] * errorLat + pidParamsArray[0][1] * integralLat + pidParamsArray[0][2] * derivativeLat;
	controlData.axes[1] += pidParamsArray[1][0] * errorLon + pidParamsArray[1][1] * integralLon + pidParamsArray[1][2] * derivativeLon; 	
	controlData.axes[2] += pidParamsArray[2][0] * errorAlt + pidParamsArray[2][1] * integralAlt + pidParamsArray[2][2] * derivativeAlt;	
	controlData.axes[3] += pidParamsArray[3][0] * errorYaw + pidParamsArray[3][1] * integralYaw + pidParamsArray[3][2] * derivativeYaw;
}

void Matrice100::updateTargetPoints() {
	// Check if all points in line has been reached
	if (pointStep == track[lineStep].size() - 1) {
		if (lineStep == track.size() - 1) {
			// All lines have been reached
			trackState = 2; //2 = track finished
		}
		else {
			pointStep = 0;
			lineStep++;
			updateTargetYaw();
			trackState = 1; //1 = new line
		}
	}
	else {
		// Update target values
		targetLat = 0; //track[lineStep][pointStep][0];
		targetLon = 0; //track[lineStep][pointStep][1];
		targetAlt = 0; //track[lineStep][pointStep][2];
		pointStep++;
		trackState = 0; //0 = new point
	}
}

void Matrice100::updateTargetYaw() {
	lat1 = track[linestep][0][0];
	lon1 = track[linestep][0][1];
	lat2 = track[linestep][track[lineStep].size() - 1][0];
	lon2 = track[linestep][track[lineStep].size() - 1][1];
	targetVector = {lat2 - lat1, lon2 - lon1};
	refVector = {0, 1};
	dotProduct = targetVector[0] * refVector[0] + targetVector[1] * refVector[1];
	magTarget = sqrt(pow(targetVector[0], 2) + pow(targetVector[1], 2));
	magRef = sqrt(pow(refVector[0], 2) + pow(refVector[1], 2));
	targetYaw = acos(dotProduct / (magTarget * magRef));
}

// Publish message of type sensor::Joy drone orientation/angles (roll,pitch, thrust and yaw) with a given flag.
void Matrice100::pubTargetValues() {

	// Push to struct if new values given
	//controlData.axes.push_back(roll);
	//controlData.axes.push_back(pitch);
	//controlData.axes.push_back(thrust);
	//controlData.axes.push_back(yaw);
	//controlData.axes.push_back(flag);

	// Publish to flight topic
	move.publish(controlData);
}

// Publish message of type sensor::Joy drone orientation/angles (roll,pitch, thrust and yaw) with a given flag.
void Matrice100::setTargetValues(float roll,float pitch, float thrust, float yaw, int flag) {

	//Update controlData if any parameters are given otherwise keep same values
	controlData.axes[0] = roll; 
	controlData.axes[1] = pitch; 	
	controlData.axes[2] = thrust;	
	controlData.axes[3] = yaw; 		
	controlData.axes[4] = flag; 	
}

void Matrice100::getTargetGPS(TargetGPS* targetGPS_struct) {
	targetGPS_struct->latitude = targetLat;
	targetGPS_struct->longitude = targetLon;
	targetGPS_struct->altitude = targetAlt;
}

void Matrice100::getTargetRPY(TargetRPY* targetRPY_struct) {
	targetRPY_struct->roll = controlData.axes[0];
	targetRPY_struct->pitch = controlData.axes[1];
	targetRPY_struct->yaw = controlData.axes[3];
}

void Matrice100::getError(Error* error_struct) {
		error_struct->errorLat = errorLat;
		error_struct->errorLon = errorLon;
		error_struct->errorAlt = errorAlt;
		error_struct->errorYaw = errorYaw;
}

void Matrice100::getRPY(RPY* rpy_struct, bool fusion_data) {
	//default gets data based on fusion between gyro and accelerometer using a simple complementary filter
	//https://vanhunteradams.com/Pico/ReactionWheel/Complementary_Filters.html
	if (fusion_data) {
		//Replace with fusion calculations
		rpy_struct->roll = imuRoll;
		rpy_struct->pitch = imuPitch;
		rpy_struct->yaw = imuYaw;
	} else {
		rpy_struct->roll = imuRoll;
		rpy_struct->pitch = imuPitch;
		rpy_struct->yaw = imuYaw;
	}
	
}

void Matrice100::getGPSData(GPS_Data* gps_struct) {
	gps_struct->latitude = latitude;
	gps_struct->longitude = longitude;
	gps_struct->altitude = altitude;
}

int Matrice100::getTrackState() {
	return trackState;
}

int Matrice100::getFlightStatus() {
	return flightStatus;
}

int Matrice100::getTargetThrust() {
	return controlData.axes[2];
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







