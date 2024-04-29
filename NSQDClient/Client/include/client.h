// client.h

#ifndef CLIENT_H
#define CLIENT_H

#include <string>
#include <vector>

#include "../include/TCPSocket.h"

extern std::vector<std::string> logs; // Forward declaration of logs vector

void log(std::string text, std::string prefix = "INFO");

#endif
