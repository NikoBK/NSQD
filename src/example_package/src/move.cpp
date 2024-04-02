#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <vector>

ros::Publisher move;

// This has to be here because NodeHandler.Subscribe requires
// a callback handler.
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	ROS_INFO("Received Imu message, callback instantiated.");
	ROS_INFO("[Orientation] x: %f, y: %f, z: %f, w: %f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

// argc : argument count, number of arguments passed to the node
// argv : argument vector, an array contaning the individual arguments passed to program. 
int main(int argc, char **argv)
{
  // Starting ros
  ros::init(argc, argv, "move_node");
  ros::NodeHandle nh;
  
  // Telling ros master that move is a message of type sensor_msgs and is released on topic ..generic
  move = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  // Subscribing to imu data
  ros::Subscriber imuData = nh.subscribe("dji_sdk/imu", 1, imuCallback);
  
  // 35 stable mode, 34 unstable
  uint8_t flag = 35;

  // Set rate of whileloop. controlData needs to be published at 40 hertz (Need source for this).
  ros::Rate rate(40);

  // Tick loop time 25 ms (40 hz)
  while(ros::ok())
  {
	ros::spinOnce();

	// Make a sensor_msgs message, populate it, and publish.
	sensor_msgs::Joy controlData;
	controlData.axes.push_back(0);
	controlData.axes.push_back(0);
	controlData.axes.push_back(0);
	controlData.axes.push_back(0);
	controlData.axes.push_back(flag);

	move.publish(controlData);
	ROS_INFO("This is a new test");
	// ROS_INFO("NodeHandler sub stamp value: %s",imuData->frame_id.c_str());

	rate.sleep();
  }

  
  
  
  return 0;
}



