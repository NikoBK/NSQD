/* socketserver.cpp
 * author: nikobk
 * created: apr 8 2024
*/
#include "../include/socketserver.hpp"
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include "../include/debug.hpp"

using namespace std;

void startServer() {
	log("starting socketserver...");
	// TODO: Implement something here.
	log("socketserver started");
	stopServer(); // TODO: Remove this eventually.
}

void stopServer() {
	log("stopping socketserver...");
	// TODO: Implement something here.
	log("stopped socketserver");
}
