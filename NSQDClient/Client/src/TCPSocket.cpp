#include "../include/TCPSocket.h"
#include "../include/gui.h"
#include "Message.hpp"
#include <iostream>


TCPSocket::TCPSocket() : _connected(false), _socket(-1), _currentSize(0)
{ }

TCPSocket::~TCPSocket()
{ }

bool TCPSocket::Connect(const std::string& host, int port)
{
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		log("Failed to initialize WSAStartup", "ERROR");
		return false;
	}

	// Create socket
	_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (_socket == INVALID_SOCKET) {
		int lastError = WSAGetLastError();
		std::string error = "Failed to create socket: " + std::to_string(lastError);
		log(error, "ERROR");
		WSACleanup();
		return false;
	}

	// Server address
	struct sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(port); // Port should match the server port
	if (inet_pton(AF_INET, host.c_str(), &serverAddr.sin_addr) != 1) {
		log("Invalid address", "ERROR");
		closesocket(_socket);
		WSACleanup();
		return false;
	}

	// Connect to server
	if (connect(_socket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
		int lastError = WSAGetLastError();
		std::string error = "Failed to connect to server: SOCKET_ERROR Code (" + std::to_string(lastError) + ")";
		log(error, "ERROR");
		closesocket(_socket);
		WSACleanup();
		return false;
	}

	// set socket to reusable
	int yes = 1;
	if (setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(int)) == SOCKET_ERROR) {
		std::string error = "Failed to set reuse address (" + std::to_string(WSAGetLastError()) + ")";
		log(error, "ERROR");
		closesocket(_socket);
		WSACleanup();
		return false;
	}

	// set the socket to non-blocking
	u_long mode = 1;  // 1 enables non-blocking mode
	if (ioctlsocket(_socket, FIONBIO, &mode) == SOCKET_ERROR) {
		std::string error = "Failed to set non-blocking (" + std::to_string(WSAGetLastError()) + ")";
		log(error, "ERROR");
		closesocket(_socket);
		WSACleanup();
		return false;
	}

	std::string success = "Connected to: " + host + ":" + std::to_string(port);
	log(success, "INFO");
	_connected = true;
	return true;
}

void TCPSocket::HandleReceive()
{
	if (_currentSize == 0)
	{
		int bytesReadable = 0;
		int result = recv(_socket, reinterpret_cast<char*>(&bytesReadable), sizeof(int), MSG_PEEK);

		if (result <= 0) {
			int err = WSAGetLastError();
			if (err == WSAEWOULDBLOCK) {
				return;
			}

			// returning 0 or WSAECONNRESET means closed by host
			if (result == 0 || err == WSAECONNRESET) {
				Disconnect("");
			}
			else
			{
				// everything else is error
				std::string message = "FATAL ERROR: Recv Error: " + std::to_string(err);
				Disconnect(message);
			}
			return;
		}

		if (result < sizeof(int)) {
			return;
		}

		log("First result: "+std::to_string(result), "INFO");

		_currentSize = ntohl(bytesReadable);

		if (_currentSize <= 0 || _currentSize > 8192) { // 8192) {
			std::string message = "Size under or overflow: " + std::to_string(_currentSize);
			Disconnect(message);
			return;
		}
	}

	std::vector<char> b(_currentSize);
	//int result = recv(_socket, b.data(), _currentSize, 0);
	int result = recv(_socket, b.data(), _currentSize, 0);

	log("Second result: " + std::to_string(result), "INFO");

	if (result <= 0) {
		int err = WSAGetLastError();
		if (err == WSAEWOULDBLOCK) {
			return;
		}

		// returning 0 or WSAECONNRESET means closed by host
		if (result == 0 || err == WSAECONNRESET) {
			Disconnect("");
		}
		else
		{
			// everything else is error
			std::string message = "FATAL ERROR: Recv Error: " + std::to_string(err);
			Disconnect(message);
		}
		return;
	}

	// + 4 as thats offset to what we already read
	Decoder decoder(b.data() + 4, _currentSize);


	//Debug
	log("Current Size: " + std::to_string(_currentSize), "INFO");

	unsigned char messageId;
	decoder.ReadByte(&messageId);

	std::string message = "Message ID: " + std::to_string(messageId); // + " bytes";
	log(message, "INFO");

	switch ((int)messageId)
	{
		case UPDATE_MSG_ID:
		{
			UpdateMessage m;
			m.decode(decoder);

			std::cout << "roll: " << (float)m.roll << std::endl;
			std::cout << "pitc: " << (float)m.pitch << std::endl;
			std::cout << "yaw: " << (float)m.yaw << std::endl;
			std::cout << "thrust: " << (float)m.thrust << std::endl;
			std::cout << "lat: " << (float)m.lat << std::endl;
			std::cout << "lon: " << (float)m.lon << std::endl;
			std::cout << "alt: " << (float)m.alt << std::endl;
			std::cout << "state: " << (float)m.state << std::endl;
			/*message += "roll: " + std::to_string((int)m.roll) + "\n";
			message += "pitch: " + std::to_string((int)m.pitch) + "\n";
			message += "yaw: " + std::to_string((int)m.yaw) + "\n";
			message += "thrust: " + std::to_string((int)m.thrust) + "\n";
			message += "x (lat): " + std::to_string((int)m.lat) + "\n";
			message += "y (lon): " + std::to_string((int)m.lon) + "\n";
			message += "z (alt): " + std::to_string((int)m.alt) + "\n";
			message += "state: " + std::to_string(m.state) + "\n";*/

			//log(message, "INFO");
			break;
		}
	}

	_currentSize = 0;
}

void TCPSocket::Send(Message& message)
{
	Encoder encoder;
	message.encode(encoder);

	const char* buffer = encoder.buffer();
	int size = encoder.size();

	int result = send(_socket, buffer, size, 0);
	if (result <= 0)
	{
		int err = WSAGetLastError();
		if (err == WSAEWOULDBLOCK) {
			return;
		}

		// returning 0 or WSAECONNRESET means closed by host
		if (result == 0 || err == WSAECONNRESET) {
			Disconnect("");
		}
		else
		{
			// everything else is error
			std::string m = "FATAL ERROR: send Error: " + std::to_string(err);
			Disconnect(m);
		}
		return;
	}

	// std::string m = "Sent: " + std::to_string(result) + " bytes";
	// log(m, "WARNING");
}

void TCPSocket::Disconnect(const std::string& reason)
{
	if (!_connected) {
		return;
	}
	_connected = false;

	if (!reason.empty()) {
		std::string message = "Disconnected: " + reason;
		log(message, "WARNING");
	}
	
	closesocket(_socket);
	WSACleanup();
}

bool TCPSocket::connected() const {
	return _connected;
}
