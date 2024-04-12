#ifndef XMLPARSER_HPP
#define XMLPARSER_HPP

#include <iostream>
#include <string>
#include <fstream>
#include "tinyxml2.h"

/*
 * Contains lat, long, elevation and timestamp for
 * a track point as part of a track segment.
 * see <cref="xmlparser.hpp::trackSegment"/>.
*/
struct trackPoint {
	double lat;
	double lon;
	int elevation;
	std::string time;
}

/*
 * Segment/collection of multiple track points.
*/
struct trackSegment {
	std::vector<trackPoint> trackPoints;
}

/*
 * Contains a track name as well aswell as a
 * collection of track segments that groups multiple
 * track points together.
*/
struct gpxTrack {
	std::string name;
	std::vector<trackSegment> trackSegments;
}

/*
 * All data the drone receives on startup.
*/
struct droneData {
	std::string creator;
	float gpxVersion;
	std::vector<gpxTrack> tracks;
}

/*
 * Initialize the XMLParser.
*/
void initXMLParser(const std::string& text);

/* 
 * Parse XML into some struct or w/e.
*/
void parseXML(tinyxml2::XMLDocument& xml);

#endif
