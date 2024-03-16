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

void loadConfig(const char* path);

void parseXML(tinyxml2::XMLDocument& xml); 

void initDrone(const DroneConfig& cfg);

void log(const std::string& text);

#endif
