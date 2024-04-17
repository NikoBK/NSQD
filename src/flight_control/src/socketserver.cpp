/* socketserver.cpp
 * author: nikobk
 * created: apr 8 2024
*/
#include "../include/socketserver.hpp"
#include "../include/debug.hpp"

using namespace std;

void startServer(int port) {
	log("starting socketserver...");

	// Create a socket
	int svrSkt = socket(AF_INET, SOCK_STREAM, 0);

	// Specify the address
	sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(port); // TODO: Change this port
	serverAddress.sin_addr.s_addr = INADDR_ANY;

	// Binding sockets
	bind(svrSkt, (struct sockaddr*)&serverAddress, sizeof(serverAddress));

	// Listen to the socket
	listen(svrSkt, 5);

	// Accepting connection request
	int cliSkt = accept(svrSkt, nullptr, nullptr);

	while (true) {
		Message msg;

		// Receive message
		int bytesReceived = recv(cliSkt, &msg, sizeof(msg), 0);
		if (bytesReceived <= 0) {
				// TODO: Handle error or connection closed
				break;
		}

		// Handle message
		handleMessage(msg);
	}

	// Process incoming data
	// char buffer[1024] = { 0 };
	// recv(cliSkt, buffer, sizeof(buffer), 0);
	// cout << "Message from client: " << buffer << endl;

	// Close socket
	close(svrSkt);

	log("socketserver started");
	stopServer(); // TODO: Remove this eventually.
}

void handleMessage(const Message& msg) {
		switch(msg.id) {
				case 1:
						log("Received test message!");
						std::cout << "testMsg.msg: " << msg.msg << std::endl;
						break;
				default:
						// Unknown msgId..
						std::cout << "Received unknwo message id: " << msg.id << std::endl;
						break;
}

void stopServer() {
	log("stopping socketserver...");
	// TODO: Implement something here.
	log("stopped socketserver");
}
