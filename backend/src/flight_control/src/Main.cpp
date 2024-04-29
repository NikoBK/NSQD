#include "Server.h"
#include "../include/Matrice100.hpp"


#define REFRESH_RATE_HZ 50


// Data structs to save roll pitch adn yaw as well as gps data
// Declared in Matrice100.hpp
RPY rpy;
GPS_Data gps_data;

// Make the matrice100 object globally available by making a pointer to the object
// Notice that the object is first initialised in the main function using the "new" command
Matrice100* drone;


//Control input (The angles we send to flight control)
float roll = 0;
float pitch = 0;
float yaw = 0;
float thrust = 0;




int main(int argc, char **argv)
{

    // Starting ros
    ros::init(argc, argv, "flight_control_node");
    ros::NodeHandle nh;

    // Initialise the Matrice100 object (calling its constructor)
    drone = new Matrice100(&nh);

    Server server(8888, drone);

    ros::Rate rate(REFRESH_RATE_HZ);

    while (ros::ok()) 
    {	
	ros::spinOnce();

        // if not connected
        if (!server.connected()) 
        {
            // see if theres a connection
            server.AcceptConnection();
        }
        else 
        {
            // handle the current connection
            server.HandleConnection();
        }

	rate.sleep();
    }
}
