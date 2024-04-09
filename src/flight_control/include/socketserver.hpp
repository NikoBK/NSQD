/* socketserver.hpp
 * author: nikobk
 * created: apr 8 2024
*/
#ifndef SOCKETSERVER_HPP
#define SOCKETSERVER_HPP

#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

/**
 * Create a socket listening for incoming connections on
 * a given port and address. Should ALWAYS be called from
 * the controller.
*/
void startServer(int port);

/**
 * Eliminate the socket and force-stop all connected clients.
*/
void stopServer();

#endif
