/* config.hpp
 * author: nikobk
 * created: mar 15 2024
*/

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

// Basic configuration container for the drone.
struct DroneConfig {
		int CaptureDeviceID;
		std::string CaptureOutputName;
		std::string Extension;
};

void log(const std::string& text);

#endif
