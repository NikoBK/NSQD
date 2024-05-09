// gui.h

#ifndef GUI_H
#define GUI_H

#include <string>
#include <vector>

//#include "../include/TCPSocket.h"
#include "../src/Message.hpp"

// Update variables (sent on every server tick)
struct UpdateVariables {
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    float thrust = 0;
    float lat = 0;
    float lon = 0;
    float alt = 0;
    int state = 0;
};

extern std::vector<std::string> logs; // Forward declaration of logs vector

void log(std::string text, std::string prefix = "INFO");

void updateProps(UpdateVariables* updVars, UpdateMessage* msg);

#endif
