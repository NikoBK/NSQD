#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

ros::Publisher move;

// argc : argument count, number of arguments passed to the node
// argv : argument vector, an array contaning the individual arguments passed to program. 
int main(int argc, char **argv)
{
  // Starting ros
  ros::init(argc, argv, "move_node");
  ros::NodeHandle nh;
  
  // Telling ros master that move is a message of type sensor_msgs and is released on topic ..generic
  move = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // 35 stable mode, 34 unstable
  uint8_t flag = 35;

  // Set rate of whileloop. controlData needs to be published at 40 hertz (Need source for this).
  ross::Rate rate(40);

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
  //ROS_INFO("Test print");

  rate.sleep();
  }

  
  
  return 0;
}



