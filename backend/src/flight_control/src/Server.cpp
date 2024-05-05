#include "Server.h"
#include <vector>
#include "Message.hpp"
#include "../include/Matrice100.hpp"
#include "../include/Main.hpp"

Server::Server(int port, Matrice100 *drone, std::ofstream *csvFile) : _socket(0), _connected(false), _currentSize(0), _drone(drone), _csvFile(csvFile)
{
    // define TCP socket
    _serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (_serverSocket == -1) {
        std::cerr << "Failed to create socket: " << errno << std::endl;
        exit(-1);
    }

    // set socket to reusable
    int yes = 1;
    if (setsockopt(_serverSocket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
        std::cerr << "Failed to set reuse address: " << errno << std::endl;
        close(_serverSocket);
        exit(-1);
    }

    // get current flags
    int flags = fcntl(_serverSocket, F_GETFL, 0);
    if (flags == -1) {
        std::cerr << "Failed to get current flags: " << errno << std::endl;
        close(_serverSocket);
        exit(-1);
    }

    // enable non blocking flag
    //flags |= O_NONBLOCK;
    //if (fcntl(_serverSocket, F_SETFL, flags) == -1) {
    if (fcntl(_serverSocket, F_SETFL, fcntl(_serverSocket, F_GETFL) | O_NONBLOCK) == -1) {
        std::cerr << "Failed to set non-blocking: " << errno << std::endl;
        close(_serverSocket);
        exit(-1);
    }

    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(port);

    // bind to address and port
    if (bind(_serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Failed to bind socket." << std::endl;
        close(_serverSocket);
        exit(-1);
    }

    // backlog of 5
    if (listen(_serverSocket, 5) == -1) {
        std::cerr << "Failed to listen" << std::endl;
        close(_serverSocket);
        exit(-1);
    }
    std::cout << "Server Running on port: " << port << std::endl;
}

Server::~Server() {}

void Server::AcceptConnection()
{
    struct sockaddr_in addr;
    socklen_t addrLen = sizeof(addr);

    int socket = accept(_serverSocket, (struct sockaddr*)&addr, &addrLen);
    if (socket == -1) {
        int errorCode = errno;
        if (errorCode != EWOULDBLOCK) {
            std::string message = "FATAL ERROR: Failed to accept: " + errorCode;
            Disconnect(message);
            exit(-1);
        }
        return;
    }

    // get current flags
    int flags = fcntl(socket, F_GETFL, 0);
    if (flags == -1) {
        std::cerr << "Failed to get current flags: " << errno << std::endl;
        close(socket);
        return;
    }

    // enable non blocking flag
    flags |= O_NONBLOCK;
    if (fcntl(socket, F_SETFL, flags) == -1) {
        std::cerr << "Failed to set non-blocking: " << errno << std::endl;
        close(socket);
        return;
    }

    std::cout << "Client Connection Established" << std::endl;
    _connected = true;
    _socket = socket;

}

void Server::HandleConnection(int *state) 
{
	// std::cout << "[DEBUG] Enter HandleConnection" << std::endl;
    if (_currentSize == 0)
    {
        int bytesReadable = 0;
        int result = recv(_socket, reinterpret_cast<char*>(&bytesReadable), sizeof(int), MSG_PEEK);

        if (result <= 0) 
        {
            int err = errno;
            if (err == EWOULDBLOCK) {
            	std::cout << "[DEBUG] No content - non blocking return" << std::endl;
                return;
            }

            // returning 0 or WSAECONNRESET means closed by host
            if (result == 0 || err == ECONNRESET) {
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

        _currentSize = ntohl(bytesReadable); // - 4 as thats original size
        if (_currentSize <= 0 || _currentSize > 8192) {
            std::string message = "Size under or overflow: " + _currentSize;
            Disconnect(message);
            return;
        }
    }

    std::vector<char> b(_currentSize);
    int result = recv(_socket, b.data(), _currentSize, 0);
    if (result <= 0) {
        int err = errno;
        if (err == EWOULDBLOCK) {
            return;
        }

        // returning 0 or WSAECONNRESET means closed by host
        if (result == 0 || err == ECONNRESET) {
            Disconnect("");
        }
        else {
            // everything else is error
            std::string message = "FATAL ERROR: Recv Error: " + std::to_string(err);
            Disconnect(message);
        }
        return;
    }

    // + 4 as thats offset to what we already read
    Decoder decoder(b.data() + 4, _currentSize);

	// Check the first byte (identifier)
    unsigned char messageId;
    decoder.ReadByte(&messageId);
    std::cout << "Received message (ID): " << (int)messageId << '\n';
    
    // Handle the message
    switch ((int)messageId) 
    {
		case TEST_MSG_ID: {
		    TestMessage m;
		    m.decode(decoder);
		    std::string message = "TestMessage: \n";

		    std::string a = (m.a == true) ? "true\n" : "false\n";
		    message += "a: " + a;
		    message += "b: " + std::to_string(m.b) + "\n";
		    message += "c: " + std::to_string(m.c) + "\n";
		    message += "d: " + std::to_string(m.d) + "\n";
		    message += "e: " + std::to_string(m.e) + "\n";
		    message += "f: " + m.f;

		    std::cout << message << std::endl;
		    break;
		}
		default: {
			std::cerr << "Unrecognized message id: " << (int)messageId << std::endl;
			break;
		}
    }
    _currentSize = 0;
}

void Server::Send(Message& message)
{
    Encoder encoder;
    message.encode(encoder);

    const char* buffer = encoder.buffer();
    int size = encoder.size();

    int result = send(_socket, buffer, size, 0);
    if (result <= 0) {
        int err = errno;
        if (err == EWOULDBLOCK) {
            return;
        }

        // returning 0 or WSAECONNRESET means closed by host
        if (result == 0 || err == ECONNRESET) {
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
    //std::cout << "Sent: " << result << " bytes";
}

void Server::Disconnect(const std::string& reason)
{
    if (!_connected){
        return;
    }

    _socket = -1;
    _connected = false;
    _currentSize = 0;

    if (!reason.empty()) {
        std::cout << "Client Disconnected: " << reason << std::endl;
    }
    close(_socket);
}
