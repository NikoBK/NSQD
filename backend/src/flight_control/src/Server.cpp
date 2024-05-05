#include "Server.h"
#include <vector>
#include "Message.hpp"
#include "../include/Matrice100.hpp"
#include "../include/Constants.hpp"

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

/** Send a error message to the client
    for backend errors. */
void SendError(std::string text) {
    ErrorMessage msg;
    msg.text = text;
    send(msg);
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
        case ERROR_MSG {
            ErrorMessage msg;
            msg.decode(decoder);
            std::cerr << "[CLIENT_ERROR] " << msg.text << std::endl;
        }
        case SET_AUTH_MSG: {
            int result = drone->request_permission();
            if (result == 0) {
                SendError("<drone::request_permission>: Failed to call sdk authority service");
            }
            break;
        }
        case ARM_MSG: {
            ArmMessage msg;
            msg.decode(decoder);

            int result = _drone->arm();
            if (result == 0) {
                SendError("<drone::arm>: Failed to call service arm");
            }
            break;
        }
        case TAKEOFF_MSG: {
            int result = _drone->takeOff();
            if (result == 0) {
                SendError("<drone::takeOff>: Failed to call flight control");
            }
            else {
                *state = HOVER_STATE;
            }
            break;
        }
        case LAND_MSG: {
            int result = _drone->land();
            if (result == 0) {
                SendError("<drone::land>: Failed to call flight control");
            }
            else {
                *state = GROUNDED_STATE;
            }
            break;
        }
        case SET_PID_MSG: {
            SetPIDMessage msg;
            msg.decode(decoder);

            _drone->setPIDValues(msg.kp, msg.ki, msg.kd, msg.flag);
            // TODO: Should'nt we change state here in order to
            // prevent continously setting PID values?
            break;
        }
        case SET_RPYTFF_MSG: {
            SetRPYTFFMessage msg;
            msg.decode(decoder);

            // Open or create file at ~/. (linux)
            _csvFile->open(msg.fileName);

            // Set target values
            _drone->setTargetValues(msg.roll, msg.pitch, msg.thrust, msg.yaw, msg.flag);

            // Update state
            *state = START_TEST_STATE;
            break;
        }
        case STOP_TEST_MSG: {
            *state = STOP_TEST_STATE;
            break;
        }
		default: {
			std::cerr << "Unrecognized message id: " << (int)messageId << std::endl;
            
            // Send error to frontend.
            std::string errText = "Drone received unrecognized message with id: ";
            errText += std::to_string((int)messageId);
            SendError(errText);

            // TODO: Hover state?
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
