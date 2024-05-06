// gui.h

#ifndef GUI_H
#define GUI_H

#include <string>
#include <vector>

#include "../include/TCPSocket.h"

extern std::vector<std::string> logs; // Forward declaration of logs vector

void log(std::string text, std::string prefix = "INFO");

void updateCameraFrame(unsigned char byteArray[]);

#endif
