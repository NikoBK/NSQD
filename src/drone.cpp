/* drone.cpp
 * author: nikobk
 * created: mar 15 2024
*/
#include "../include/config.hpp"
#include "../include/camera.hpp"
#include <unistd.h>
#include <fstream>

int main() {
		loadConfig("config.xml");
		return 0;
}

void loadConfig(const char* path) {
		log("Loading XML config files...");

		// Open the file
		std::ifstream file(path);
		if (!file.is_open()) {
				std::cerr << "Error: Failed to open XML file: " << path << std::endl;
				return;
		}

		// Read the file contents into a string
		std::string xmlString((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());

		// Close the file
		file.close();

		// Parse the XML from the string
		tinyxml2::XMLDocument xml;
		if (xml.Parse(xmlString.c_str()) != tinyxml2::XML_SUCCESS) {
				std::cerr << "Error: Failed to parse XML file: " << path << std::endl;
				return;
		}
		log("Success: Loaded XML file");
		parseXML(xml);
}

void parseXML(tinyxml2::XMLDocument& xml) {
		log("Parsing config XML file...");
		DroneConfig cfg;

		// Get root element
		tinyxml2::XMLElement* root = xml.FirstChildElement("DroneConfig");
		if (!root) {
				std::cerr << "Error: Root element 'DroneConfig' not found in XML config file." << std::endl;
				return;
		}
		
		std::string deviceIdStr = "";
		if (auto deviceIdElem = root->FirstChildElement("CaptureDeviceID")) {
				deviceIdStr = deviceIdElem->GetText();
		}
		int deviceId = std::stoi(deviceIdStr);
		cfg.CaptureDeviceID = deviceId;

		if (auto outputNameElem = root->FirstChildElement("CaptureOutputName")) {
				cfg.CaptureOutputName = outputNameElem->GetText();
		}
		if (auto extensionElem = root->FirstChildElement("Extension")) {
				cfg.Extension = extensionElem->GetText();
		}

		// Log setup.
		log("Success: Config XML file parsed.");
		log("CaptureDeviceID: " + std::to_string(cfg.CaptureDeviceID));
		log("CaptureOutputName: " + cfg.CaptureOutputName);
		log("Extension: " + cfg.Extension);

		// Start drone init.
		initDrone(cfg);
}

void initDrone(const DroneConfig& cfg) {
		log("Initializing drone...");
		initCamera(cfg.CaptureDeviceID, cfg.CaptureOutputName, cfg.Extension);
		// TODO: Init telemetry (?)
		// TODO: Init gyroscope (?)
		// TODO: Init GPS
}

void initCamera(const int& id, const std::string& output, const std::string& ext) {
		log("Initializing drone camera unit...");
		cv::VideoCapture cap(id);

		// Check if camera opened succesfully.
		if (!cap.isOpened()) {
				std::cerr << "Error: Unable to open camera (" << std::to_string(id) << ")" << std::endl;
				log("Abandoned camera setup.");
				return;
		}

		// Capture a single frame (image)
		cv::Mat frame;
		cap >> frame;
		if (frame.empty()) {
				std::cerr << "Error: Unable to capture frame" << std::endl;
		}

		std::string imgName = output + ""/*unix_epoch_time*/ + "." + ext;
		cv::imwrite("images/" + imgName, frame);
		cap.release();

		log("Image captured: " + imgName);
		log("Camera initialized succesfully.");
}

void log(const std::string& text) {
		std::cout << text << std::endl;
}
