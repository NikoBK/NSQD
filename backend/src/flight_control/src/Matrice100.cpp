// Misc
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>
//#include "tinyxml_catkin2/tinyxml2.h"
#include "../include/tinyxml2.h"

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


void Matrice100::loadPathFromString(const char* xmlContent) {
	
	photoPoints.clear();
	
	// Create xmldocument in memory
	tinyxml2::XMLDocument doc;
	
	// Parse string to xml
    doc.Parse(xmlContent);
    
	// Catch error
	if(doc.Error()) {
		std::cout << "Error parsing XML string: " << doc.ErrorStr() << '\n';
	}

	tinyxml2::XMLElement* gpx = doc.RootElement();

	tinyxml2::XMLElement* trk = gpx->FirstChildElement("trk");

	// Loop through tracks 
	std::vector<double> point;
	for(const tinyxml2::XMLElement* trkpt = trk->FirstChildElement("trkpt"); trkpt != 0; trkpt = trkpt->NextSiblingElement("trkpt")) {
		// Read trkpoint attribute
		
		const char* lat = trkpt->Attribute("lat");
		const char* lon = trkpt->Attribute("lon");
		
		// Append Latitude 
		point.push_back(std::stod(lat));
		
		// Append Longitude
		point.push_back(std::stod(lon));

		photoPoints.push_back(point);
		point.clear();
	}
	
	std::cout << "Photopoint: " << photoPoints[0][0] << " lon: " << photoPoints[0][1] << '\n';
}

void Matrice100::interpolatePath(float desiredVel, float accGain, float updateHz,float alti){
	
	// Clear former track
	track.clear();
		
	// Position and heading
	double latDifC = 0;
    double lonDifC = 0;
    double latDifN = 0;
    double lonDifN = 0;
    double dotProduct = 0;
    double magTarget = 0;
    double magRef = 0;
    double heading = 0;

	// Distance between photopoints, in meters.
    double latDifMeter = 0;
    double lonDifMeter = 0;
    float magMeters = 0;
    
    // Line travel time.
    float tf = 0; // s
    double step = 0; 

    // Latitude parameters
    double aLat = 0;
    double tbLat = 0;
    double thetaBLat = 0;

    // Longitude parameters
    double aLon = 0;
    double tbLon = 0;
    double thetaBLon = 0;
		
    std::vector<std::vector<double>> line = {};
    std::vector<double> point = {};

	// Vector contaning photopoints reduced to endpoints.
	std::vector<std::vector<double>> endPoints = {};
	
	// Add starting point to endpoints
	point.push_back(latitude);
	point.push_back(longitude);
	endPoints.push_back(point);
    point.clear();
    
   
    alti = altitude+alti; 
    float stepTime = 1/updateHz; // ms	
	
	// Convert PhotoPoints to Endpoints 
    for(int i = 0 ; i < photoPoints.size(); i++) {

        // Append first point
        if(i == 0) {
            point.push_back(photoPoints[i][0]);
            point.push_back(photoPoints[i][1]);
            endPoints.push_back(point);
            
            point.clear();
        }

        // If the end has been reached, append the last point.
        if(i == (photoPoints.size()-2)){
            point.push_back(photoPoints[i+1][0]);
            point.push_back(photoPoints[i+1][1]);
            endPoints.push_back(point);
            point.clear();
            break;
        }
        
        // Calculate heading between next to points. If heading is not 0, then we have reached a end point.
        latDifC = photoPoints[i][0]-photoPoints[i+1][0];
		lonDifC = photoPoints[i][1]-photoPoints[i+1][1]; 
        latDifN = photoPoints[i+1][0]-photoPoints[i+2][0];
		lonDifN = photoPoints[i+1][1]-photoPoints[i+2][1]; 

        // Calculate heading between current photopoint and next photopoint
        dotProduct = latDifC * latDifN + lonDifC * lonDifN;
	    magTarget = sqrt(pow(latDifC, 2) + pow(lonDifC, 2));
	    magRef = sqrt(pow(latDifN, 2) + pow(lonDifN, 2));
	    heading = acos(dotProduct / (magTarget * magRef));

        if(!heading == 0) {
            //std::cout << "heading: " << heading << '\n';
            point.push_back(photoPoints[i+1][0]);
            point.push_back(photoPoints[i+1][1]);
            endPoints.push_back(point);
            point.clear();
        }    
    }
	
	std::cout << "Path reduced to following endpoints" << '\n';
    for(int i = 0; i < endPoints.size(); i++) {
        std::cout << "Endpoints lat: " << endPoints[i][0] << " lon: " << endPoints[i][1] << '\n';
    }
	
	// Loop through end points.
	for(int i = 0 ; i <= endPoints.size()-2; i++) {
		
        // Calculate distance and convert to meters
        latDifC = endPoints[i+1][0]-endPoints[i][0];
		lonDifC = endPoints[i+1][1]-endPoints[i][1]; 
        latDifMeter = latDifC * 111320;
        lonDifMeter = lonDifC * ((40075000 * cos(latDifC) / 360));
        magMeters = sqrt(pow(latDifMeter,2)+pow(lonDifMeter,2));

        // Line travel time
        tf = magMeters/desiredVel;

        // Latitude acceleration
        aLat = ((4*latDifC)/(tf*tf)) * accGain;
        // Calculate time before shift to lineare piece, and latitude at that time
        tbLat = tf / 2 - fabs((sqrt(fabs((aLat*aLat)*(tf*tf)-4*aLat*(latDifC)))) / (2 * aLat));
        if(tbLat != tbLat) {
            tbLat = 0;
        }
        thetaBLat = 0.5 * (aLat*tbLat*tbLat) + endPoints[i][0];

        // Longitude acceleration
        aLon = (4*(lonDifC)/(tf*tf)) * accGain;
        // Calculate time before shift to lineare piece, and longitude at that time.
        tbLon = tf / 2 - fabs((sqrt(fabs((aLon*aLon)*(tf*tf)-4*aLon*lonDifC))) / (2 * aLon));
        if(tbLon != tbLon) {
            tbLon = 0;
        }
        thetaBLon = 0.5*(aLon*tbLon*tbLon) + endPoints[i][1];     

		
        // Interpolate latitude
        int s = 0;
        // Acceleration
        for(; s*stepTime <= tbLat; s++ ) {
            // Calculate step and append 
            step = endPoints[i][0] + 0.5 * aLat * pow(s*stepTime,2);
            point.push_back(step);

            // Longitude temp 0
            point.push_back(0);

            // Append altitude
            point.push_back(alti);
            
            line.push_back(point);
            point.clear();
        }

        // Lineare piece
        for(; (s*stepTime) <= (tf-tbLat); s++) {
            step = thetaBLat + aLat * tbLat * (s*stepTime - tbLat);
            point.push_back(step);

            point.push_back(0);

            point.push_back(alti);
            
            line.push_back(point);
            point.clear();
        }

        // Deacceleration
        for(; (s*stepTime) <= tf; s++) {
            step = endPoints[i+1][0] - 0.5 * aLat * pow((tf-s*stepTime),2);
            point.push_back(step);

            point.push_back(0);

            point.push_back(alti);
            
            line.push_back(point);
            point.clear();
        }

        // Interpolate longitude
        s = 0;
        // Acceleration
        for(; s*stepTime <= tbLon; s++ ) {
            step = endPoints[i][1] + 0.5 * aLon * pow(s*stepTime,2);
            line[s][1] = step;
        }

        // Lineare piece
        for(; (s*stepTime) <= (tf-tbLon); s++) {
            step = thetaBLon + aLon * tbLon * (s*stepTime - tbLon);
            line[s][1] = step;
        }

        // Deacceleration
        for(; (s*stepTime) <= tf; s++) {
            step = endPoints[i+1][1] - 0.5 * aLon * pow((tf-s*stepTime),2);
            line[s][1] = step;
        }

        // Print line for debugging     
		/*
        for(int l = 0; l < line.size(); l++) {
            std::cout <<"{" << line[l][0] << "," << line[l][1] << "}," << '\n';
        }
        */
		
        track.push_back(line);
        line.clear();
	}
	/*
	std::cout <<"Track size: " << track.size()  << '\n';
	for(int l = 0; l <= track.size()-1; l++) {
                       
            std::cout <<"Line size: " << track[l].size()  << '\n';
            
            for(int t= 0; t <= track[l].size()-1; t++) {
            	
            	std::cout <<"Point lat: " << track[l][t][0]  << '\n';
            	std::cout <<"Point lon: " << track[l][t][1]  << '\n';
            	std::cout <<"Point alt: " << track[l][t][2]  << '\n';
            }     
    }
    
    std::cout <<"Track size: " << track.size()  << '\n';
    */
}

//Callback event for dji_sdk/gps_position subscription event.
void Matrice100::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	// Gps position.
	/*
	latitude = msg->latitude;
	longitude = msg->longitude;
	*/
	longitude = msg->latitude;
	latitude = msg->longitude;
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
	controlData.axes[4] = 33;
}

void Matrice100::initPIDValues() {
	// Set PID values for a given axis
	setPIDValues(0.08, 0, 1, 0);
	setPIDValues(0.08, 0, 1, 1);
	setPIDValues(0.8, 0.001, 1.6, 2);
	//setPIDValues(0, 0, 0, 3);
	
}

void Matrice100::setPIDValues(float kp, float ki, float kd, int type) {
	// Set PID values for a given axis
	pidParamsArray[type][0] = kp;
	pidParamsArray[type][1] = ki;
	pidParamsArray[type][2] = kd;
	
	std::cout << "kp: " << std::to_string(kp) << std::endl;
	std::cout << "ki: " << std::to_string(ki) << std::endl;
	std::cout << "kd: " << std::to_string(kd) << std::endl;
}

// Calculate PID control signal
void Matrice100::calculateError() {
    errorLat = targetLat - latitude;
	errorLon = targetLon - longitude;
	errorAlt = targetAlt - altitude;
	errorYaw = targetYaw - imuYaw;

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
	//Thrust input that is slightly below the gravitational pull
	float gravityConst = 33; // % thrust   

	//Constants to convert from latitude and longitude to meters
	//Approximated for a local scenario
	static const float lat_to_m = 111320;
	static const float lon_to_m = 109617.422;


    // Calculate control signal
	controlData.axes[0] = lat_to_m * (pidParamsArray[0][0] * errorLat + pidParamsArray[0][1] * integralLat + pidParamsArray[0][2] * derivativeLat); //roll
	controlData.axes[1] = lon_to_m * (pidParamsArray[1][0] * errorLon + pidParamsArray[1][1] * integralLon + pidParamsArray[1][2] * derivativeLon); //pitch
	controlData.axes[2] = gravityConst + pidParamsArray[2][0] * errorAlt + pidParamsArray[2][1] * integralAlt + pidParamsArray[2][2] * derivativeAlt; //thrust
	
	//Check for limits
	if (controlData.axes[0] > 0.61){
		controlData.axes[0] = 0.61;
	} else if (controlData.axes[0] < -0.61) {
		controlData.axes[0] = -0.61;
	}

	if (controlData.axes[1] > 0.61){
		controlData.axes[1] = 0.61;
	} else if (controlData.axes[1] < -0.61) {
		controlData.axes[1] = -0.61;
	}

	if (controlData.axes[2] > 70){
		controlData.axes[2] = 70;
	} else if (controlData.axes[2] < 0) {
		controlData.axes[2] = 5;
	}
	
	std::cout << "roll " << std::to_string(controlData.axes[0]) << std::endl;
	std::cout << "pitch " << std::to_string(controlData.axes[1]) << std::endl;
	std::cout << "thrust " << std::to_string(controlData.axes[2]) << std::endl;
	
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
			trackState = 1; //1 = new line
		}
	}
	else {
		// Update target values
		targetLat = track[lineStep][pointStep][0];
		targetLon = track[lineStep][pointStep][1];
		targetAlt = track[lineStep][pointStep][2];
		pointStep++;
		trackState = 0; //0 = new point
	}
}

//For testing hover
void Matrice100::setTargetAltitude(float new_altitude) {
	// Update target values
	targetAlt = new_altitude+altitude;
}

//For testing hover
void Matrice100::updateTargetLatLon() {
	// Update target values
	targetLat = latitude;
	targetLon = longitude;
	targetYaw = imuYaw;
}


void Matrice100::updateTargetYaw() {
 	
	float lat1 = track[lineStep][0][0];
	float lon1 = track[lineStep][0][1];
	float lat2 = track[lineStep][track[lineStep].size() - 1][0];
	float lon2 = track[lineStep][track[lineStep].size() - 1][1];
	

	std::vector<float> targetVector = {lat2 - lat1, lon2 - lon1};
	std::vector<float> refVector = {1, 0};
	
	// targetVector = {lat2 - lat1, lon2 - lon1};
	// refVector = {0, 1};
	float dotProduct = targetVector[0] * refVector[0] + targetVector[1] * refVector[1];
	float magTarget = sqrt(pow(targetVector[0], 2) + pow(targetVector[1], 2));
	float magRef = sqrt(pow(refVector[0], 2) + pow(refVector[1], 2));
	
	targetYaw = acos(dotProduct / (magTarget * magRef));
	
	if(targetVector[1] < 0) {
		targetYaw = targetYaw *-1;
		controlData.axes[3] = targetYaw;
		
	}
}

// Publish message of type sensor::Joy drone orientation/angles (roll,pitch, thrust and yaw) with a given flag.
void Matrice100::pubTargetValues() {
	// Publish to flight topic
	std::cout << "Published thrust " << std::to_string(controlData.axes[2]) << std::endl;
	move.publish(controlData);
}

// Update message of type sensor::Joy drone orientation/angles (roll,pitch, thrust and yaw) with a given flag.
void Matrice100::setTargetValues(float roll,float pitch, float thrust, float yaw, int flag) {

	//Update controlData if any parameters are given otherwise keep same values
	controlData.axes[0] = roll * 3.14/180; 
	controlData.axes[1] = pitch * 3.14/180; 	
	controlData.axes[2] = thrust;	
	controlData.axes[3] = yaw * 3.14/180; 		
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

float Matrice100::getTargetThrust() {
	return (float)controlData.axes[2];
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







