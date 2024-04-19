/* socketserver.cpp
 * author: nikobk
 * created: apr 8 2024
*/
#include "../include/socketserver.hpp"
#include "../include/debug.hpp"

using namespace std;

void startServer(int port, bool debugMode) {
	log("starting socketserver...");

	// Create a stream-oriented, reliable, two-way byte stream communication socket.
	int svrSkt = socket(AF_INET, SOCK_STREAM, 0);

	// Socket address for ipv4 communication.
	sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET; // family for ipv4 internet protocol.
	serverAddress.sin_port = htons(port); // host to network short. Convert 16bit value from host to network byte order.
	serverAddress.sin_addr.s_addr = INADDR_ANY;

	// Binding sockets.
	// NOTE: the bind function requires a pointer to a struct of type sockaddr.
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

		// Reply with debug message if in debug mode.
		if (debugMode) {
			Message debugMsg;
			debugMsg.id = 1;
			// TODO: Populate the debug message with variables.
			debugMsg.msg = "lat:{lat}, lon:{lon}, alt:{alt}, fps:{fps}, res:{res}"
		}
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
				case 2:
					log("Received initialize message!");
					// TODO: Set control authority and arm control.
					break;
				case 3:
					log("Received takeoff message!");
					break;
				case 4:
					log("Received land command message!");
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
