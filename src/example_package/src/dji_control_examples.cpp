#include "ros/ros.h"
// #include "dji_sdk/dji_sdk.h" <-- Critial Warning - No file or directory
// #include <djiosdk/dji_vehicle.hpp>
#include "dji_sdk/SDKControlAuthority.h"
#include "dji_sdk/DroneArmControl.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_drone");

  ros::NodeHandle n;

  ros::ServiceClient arm_client = n.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
  ros::ServiceClient permission_client = n.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");

  dji_sdk::DroneArmControl arm_srv;
  dji_sdk::SDKControlAuthority permission_srv;
 
  permission_srv.request.control_enable = 1;
  arm_srv.request.arm = 1;

  if (permission_client.call(permission_srv))
  {
    ROS_INFO("Sum: %ld", (long int)permission_srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service control");
    return 1;
  }

  if (arm_client.call(arm_srv))
  {
    ROS_INFO("Sum: %ld", (long int)arm_srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service arm");
    return 1;
  }

  return 0;
}
