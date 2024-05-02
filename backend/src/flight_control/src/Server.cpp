#include "Server.h"
#include <vector>
#include "Message.hpp"
#include "../include/Matrice100.hpp"
#include "../include/Main.hpp"


Server::Server(int port, Matrice100* drone, std::ofstream *csvFile)
    : _socket(0), _connected(false), _currentSize(0), _drone(drone), _csvFile(csvFile)
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

void Server::HandleConnection(int * state) 
{
    if (_currentSize == 0)
    {
        int bytesReadable = 0;
        int result = recv(_socket, reinterpret_cast<char*>(&bytesReadable), sizeof(int), MSG_PEEK);

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

    std::cout << "Message: " << (int)messageId << '\n';

    
    switch ((int)messageId) 
    {
    case TEST_MESSAGE_ID: 
    {
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
    case RPY_MESSAGE_ID:
    {
        RPYMessage m;
        m.decode(decoder);
         //pubAngle(m.roll,m.pitch, 20, m.yaw, 35);

        std::string message = "RPYMessage: \n";
        message += "Roll: " + std::to_string(m.roll) + "\n";
        message += "Pitch: " + std::to_string(m.pitch) + "\n";
        message += "Yaw: " + std::to_string(m.yaw);

        std::cout << message << std::endl;
        break;
    }
    case RPYT_MESSAGE_ID:
    {
        RPYTMessage m;
        m.decode(decoder);

        //update target angles and target thrust so main loop can publish given values
        _drone->setTargetValues(m.roll,m.pitch, m.thrust, m.yaw, 35);

        //Update current state
        *state = PUBLISH_ANGLE_STATE;

        std::string message = "RPYMessage: \n";
        message += "Roll: " + std::to_string(m.roll) + "\n";
        message += "Pitch: " + std::to_string(m.pitch) + "\n";
        message += "Yaw: " + std::to_string(m.yaw) + "\n";
        message += "Thrust: " + std::to_string(m.thrust);

        std::cout << message << std::endl;
        break;
    }
    case ARM_MESSAGE_ID:
    {
        ArmMessage m;
        m.decode(decoder);

        //Send request to arm client
        _drone->arm(m.arm);

        //Update current state
        *state = ARMED_STATE;

        break;
    }
    case START_TEST_MESSAGE_ID:
    {
        StartTestMessage m;
        m.decode(decoder);

        //Open or create file at given location
        _csvFile->open(m.fileName);

        //Set target values
        _drone->setTargetValues(m.roll,m.pitch, m.thrust, m.yaw, m.flag);

        //Update current state
        *state = START_TEST_STATE;

        break;
    }
    case STOP_TEST_MESSAGE_ID:
    {
         //Update current state
        *state = STOP_TEST_STATE;

        break;
    }
    case PID_MESSAGE_ID:
    {
        PIDMessage m;
        m.decode(decoder);

        _drone->setPIDValues(m.kp, m.ki, m.kd, m.type);

        break;
    }
    case AUTHORITY_MESSAGE_ID:
    {
        _drone->request_permission();

        break;
    }
    case TAKEOFF_MESSAGE_ID:
    {
        _drone->takeOff();

        *state = HOVER_STATE;
        break;
    }
    case LAND_MESSAGE_ID:
    {
        _drone->land();

        *state = GROUNDED_STATE;
        break;
    }
    case FLIGHT_PATH_MESSAGE_ID:
    {
        // Initialise path following
        SetFlightPathMessage m;
        m.decode(decoder);

        //_drone->loadPathFromString(m.xmlContent)
        //_drone->startMission();

        *state = ENROUTE_STATE;

        break;
    }
    case STOP_FLIGHT_PATH_MESSAGE_ID:
    {
        // Stop drone from following path
        //_drone->stopMission();

        *state = ENROUTE_STOPPED_STATE;

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
