#include "../include/TCPSocket.h"
#include "../include/gui.h"
#include "Message.hpp"

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

		_currentSize = ntohl(bytesReadable);
		if (_currentSize <= 0 || _currentSize > 8192) {
			std::string message = "Size under or overflow: " + _currentSize;
			Disconnect(message);
			return;
		}
	}

	std::vector<char> b(_currentSize);
	int result = recv(_socket, b.data(), _currentSize, 0);

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

	unsigned char messageId;
	decoder.ReadByte(&messageId);

	std::string message = "Message: " + std::to_string(messageId) + " bytes";
	log(message, "INFO");

	switch ((int)messageId)
	{
		case TEST_MESSAGE_ID:
		{
			TestMessage m;
			m.decode(decoder);

			std::string a = (m.a == true) ? "true\n" : "false\n";
			message += "a: " + a;
			message += "b: " + std::to_string(m.b) + "\n";
			message += "c: " + std::to_string(m.c) + "\n";
			message += "d: " + std::to_string(m.d) + "\n";
			message += "e: " + std::to_string(m.e) + "\n";
			message += "f: " + m.f;

			log(message, "INFO");
			break;
		}
		case RPY_MESSAGE_ID:
		{
			RPYMessage m;
			m.decode(decoder);

			std::string message = "RPYMessage: \n";
			message += "Roll: " + std::to_string(m.roll) + "\n";
			message += "Pitch: " + std::to_string(m.pitch) + "\n";
			message += "Yaw: " + std::to_string(m.yaw);

			log(message, "INFO");
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
