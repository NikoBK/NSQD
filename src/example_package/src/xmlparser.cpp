#include "../include/xmlparser.hpp"

void initXMLParser(const std::string& text) {
	std::cout << "Initializing XML Parser..." << std::endl;

	// Open the file
	std::ifstream file(path);
	if (!file.is_open()) {
		std::cerr << "Error: Failed to open XML file: " << path << std::endl;
		return;
	}

	// Read the file contents into a string
	std::string xmlString((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

	// Close the file
	file.close();

	// Parse the XML from the string
	tinyxml2::XMLDocument xml;
	if (xml.Parse(xmlString.c_str()) != tinyxml2::XML_SUCCESS) {
		std::cerr << "Error: Failed to parse XML file: " << path << std::endl;
		return;
	}

	std::cout << "Succesfully loaded XML file." << std::endl;
	parseXML(xml);
}

void parseXML(tinyxml2::XMLDocument& xml) {
	// TODO: Add parsing.
}
