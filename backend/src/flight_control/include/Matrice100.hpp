#ifndef MATRICE100_HPP
#define MATRICE100_HPP

#include "ros/ros.h"

// Sensor mesgs
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>
#include "dji_sdk/DroneArmControl.h"
#include "dji_sdk/SDKControlAuthority.h"
#include "dji_sdk/DroneTaskControl.h"


struct RPY {
	float roll;
	float pitch;
	float yaw;
};

struct TargetRPY {
	float roll;
	float pitch;
	float yaw;
};

struct GPS_Data {
	double latitude;
	double longitude;
	double altitude;
};

struct TargetGPS {
	double latitude;
	double longitude;
	double altitude;
};

struct VEL {
	double x;
	double y;
	double z;
};

struct Error {
	float errorLat;
	float errorLon;
	float errorAlt;
	float errorYaw;
};


class Matrice100 {
	private:
		float imuRoll, imuPitch, imuYaw;
		float pidParamsArray[4][3];    // = {{kp_roll, ki_roll, kd_roll}, {kp_pitch, ki_pitch, kd_pitch}, {kp_alt, ki_alt, kd_alt}, {kp_yaw, ki_yaw, kd_yaw}};
		float targetLat, targetLon, targetAlt, targetYaw;
		float errorLat, errorLon, errorAlt, errorYaw;
		float integralLat, integralLon, integralAlt, integralYaw;
		float derivativeLat, derivativeLon, derivativeAlt, derivativeYaw;
		float prevErrorLat, prevErrorLon, prevErrorAlt, prevErrorYaw;
		float imuAccX, imuAccY , imuAccZ;
		float batteryVoltage;
		float latitude;
		float longitude;
		float altitude;
		double x_vel;
		double y_vel;
		double z_vel;
		int trackState;
		int flightStatus;
		int lineStep;
		int pointStep; //aka time step

		std::vector<std::vector<std::vector<double>>> track;
		std::vector<std::vector<double>> photoPoints;
		

		//Nodehandler
		ros::NodeHandle _nh;

		dji_sdk::DroneArmControl arm_srv;
		dji_sdk::SDKControlAuthority permission_srv;
		dji_sdk::DroneTaskControl task_srv;
		sensor_msgs::Joy controlData;

		// Topic publishers adn subscribers
		ros::Publisher move;
		ros::Subscriber imuDataSub;
		ros::Subscriber gpsDataSub;
		ros::Subscriber flightStatusSub;
		ros::Subscriber BatteryVoltageSub;
		ros::Subscriber velocitySub;		

		// Service client
		ros::ServiceClient arm_client;
		ros::ServiceClient permission_client;
		ros::ServiceClient taskControl_client;


		void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

		void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

		void flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg);

		void batteryVoltageCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

		void velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

	public:
		void setPIDValues(float kp, float ki, float kd, int type);
		void setTargetValues(float roll,float pitch, float thrust, float yaw,int flag);
		void pubTargetValues();
		void getRPY(RPY* rpy_struct, bool fusion_data = true);
		void getTargetRPY(TargetRPY* targetRPY_struct);
		void getGPSData(GPS_Data* gps_struct);
		void getTargetGPS(TargetGPS* targetGPS_struct);
		void getVel(VEL* vel_struct);
		void getError(Error* error_struct);
		void runPIDController();
		void updateTargetYaw();	
		void updateTargetPoints();
		void startMission();
		void calculateError();
		void initPIDValues();
		
		//For testing
		void setTargetAltitude(float altitude);
		void updateTargetLatLon();
		
		void loadPathFromString(const char* xmlContent);
		void interpolatePath(float desired_vel, float accGain, int update_frequency, float altitude);

		float getBatteryVoltage();
		float getTargetYaw();
		float getTargetThrust();
		
		int getTrackState();
		int getFlightStatus();
		int arm(int arm_drone);
		int request_permission(int permission = 1);
		int takeOff();
		int land();
		int goHome();
		

		//Constructor
		Matrice100(ros::NodeHandle *nodehandle);


};

#endif
