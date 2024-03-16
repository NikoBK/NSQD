/* config.hpp
 * author: nikobk
 * created: mar 15 2024
*/

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <iostream>
#include <string>
#include "tinyxml2.h"

// Basic configuration container for the drone.
struct DroneConfig {
		int CaptureDeviceID;
		std::string CaptureOutputName;
		std::string Extension;
};

/**
 * Load the drone's configuration from an XML config file using TinyXML2.
 */
void loadConfig(const char* path);

/**
 * Parse config file content from XML to a config container. See:
 * <see="../include/config.hpp.DroneConfig"/>.
 */
void parseXML(tinyxml2::XMLDocument& xml); 

/**
 * Apply the config container vaules and initialize
 * the drone components.
 */
void initDrone(const DroneConfig& cfg);

/**
 * Print a line using the standard library's
 * console output function call.
 */
void log(const std::string& text);

#endif
