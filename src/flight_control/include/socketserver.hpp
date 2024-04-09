/* socketserver.hpp
 * author: nikobk
 * created: apr 8 2024
*/
#ifndef SOCKETSERVER_HPP
#define SOCKETSERVER_HPP

/**
 * Create a socket listening for incoming connections on
 * a given port and address. Should ALWAYS be called from
 * the controller.
*/
void startServer();

/**
 * Eliminate the socket and force-stop all connected clients.
*/
void stopServer();

#endif
