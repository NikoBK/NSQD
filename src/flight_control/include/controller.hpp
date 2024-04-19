/* controller.hpp
 * author: nikobk
 * created: apr 8 2024
*/
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <iostream>

// Basic configuration container for the drone.
struct DroneConfig {
	std::string SvrAddr;
	int Port;
};

/**
 * Load the drone's configuration from an XML config file using TinyXML.
 */
void loadConfig(const char* path);

#endif
