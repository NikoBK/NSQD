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

struct Message {
		uint8_t id; // message id (1 byte)
		std::string msg;
}

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

/**
 * Receive and process a message sent from the client
 * over TCP.
 */
void handleMessage(const Message& msg);

#endif
