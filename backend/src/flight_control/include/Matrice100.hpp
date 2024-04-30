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


struct GPS_Data {
	double latitude;
	double longitude;
	double altitude;
};

struct VEL {
	double x;
	double y;
	double z;
};

class Matrice100 {
	private:
		float imuRoll, imuPitch, ImuYaw;
		float K_p, K_i, K_d;
		float imuAccX, imuAccY , imuAccZ;
		float batteryVoltage;
		double latitude;
		double longitude;
		double altitude;
		double x_vel;
		double y_vel;
		double z_vel;
		int flightStatus;

 
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
		void setPIDValues(float kp, float ki, float kd);
		void setTargetValues(float roll,float pitch, float thrust, float yaw,int flag);
		void pubTargetValues();
		void getRPY(RPY* rpy_struct, bool fusion_data = true);
		void getGPSData(GPS_Data* gps_struct);
		void getVel(VEL* vel_struct);
		int getFlightStatus();
		float getBatteryVoltage();
		
		int getTargetThrust();
		int arm(int arm_drone);
		int request_permission(int permission = 1);
		int takeOff();
		int land();
		int goHome();
		

		//Constructor
		Matrice100(ros::NodeHandle *nodehandle);


};

#endif
