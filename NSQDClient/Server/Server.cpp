#include "Server.h"
#include <vector>
#include "Message.hpp"

Server::Server(int port)
    : _socket(0), _connected(false), _currentSize(0)
{
    // os specific socket initialize
#ifdef _WIN32
    // initialize windows socket API
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Failed to initialize WSAStartup" << std::endl;
        exit(-1);
    }

    // define TCP socket
    _serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (_serverSocket == INVALID_SOCKET) {
        std::cerr << "Failed to create socket: " << WSAGetLastError() << std::endl;
        WSACleanup();
        exit(-1);
    }

    // set socket to reusable
    int yes = 1;
    if (setsockopt(_serverSocket, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(int)) == SOCKET_ERROR) {
        std::cerr << "Failed to set reuse address: " << WSAGetLastError() << std::endl;
        closesocket(_serverSocket);
        WSACleanup();
        exit(-1);
    }

    // set the socket to non-blocking
    u_long mode = 1;  // 1 enables non-blocking mode
    if (ioctlsocket(_serverSocket, FIONBIO, &mode) == SOCKET_ERROR) {
        std::cerr << "Failed to set non-blocking: " << WSAGetLastError() << std::endl;
        closesocket(_serverSocket);
        WSACleanup();
        exit(-1);
    }

    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    // bind to address and port
    if (bind(_serverSocket, reinterpret_cast<sockaddr*>(&serverAddress), sizeof(serverAddress)) == SOCKET_ERROR) {
        std::cerr << "Failed to bind: " << WSAGetLastError() << std::endl;
        closesocket(_serverSocket);
        WSACleanup();
        exit(-1);
    }

    // backlog of 5
    if (listen(_serverSocket, 5) == SOCKET_ERROR) {
        std::cerr << "Failed to listen: " << WSAGetLastError() << std::endl;
        closesocket(_serverSocket);
        WSACleanup();
        exit(-1);
    }

#elif __linux__
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
    flags |= O_NONBLOCK;
    if (fcntl(_serverSocket, F_SETFL, flags) == -1) {
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
#endif

    std::cout << "Server Running on port: " << port << std::endl;
}

Server::~Server()
{
}

void Server::AcceptConnection()
{
    // os specific socket accept
#ifdef _WIN32
    struct sockaddr addr;
    int addrLen = sizeof(addr);

    int socket = accept(_serverSocket, &addr, &addrLen);
    if (socket == SOCKET_ERROR)
    {
        int errorCode = WSAGetLastError();
        if (errorCode != WSAEWOULDBLOCK) {
            std::string message = "FATAL ERROR: Failed to accept: " + errorCode;
            Disconnect(message);
            exit(-1);
        }
        return;
    }

#elif __linux__
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
#endif

    std::cout << "Client Connection Established" << std::endl;
 
    _connected = true;
    _socket = socket;
}

void Server::HandleConnection() 
{
    if (_currentSize == 0)
    {
        int bytesReadable = 0;
        int result = recv(_socket, reinterpret_cast<char*>(&bytesReadable), sizeof(int), MSG_PEEK);
#ifdef _WIN32
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
#elif __linux
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
#endif
        
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
#ifdef _WIN32
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
#elif __linux
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
#endif

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

        std::string message = "RPYMessage: \n";
        message += "Roll: " + std::to_string(m.roll) + "\n";
        message += "Pitch: " + std::to_string(m.pitch) + "\n";
        message += "Yaw: " + std::to_string(m.yaw);

        std::cout << message << std::endl;
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
#ifdef _WIN32
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
#elif __linux
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
#endif
    std::cout << "Sent: " << result << " bytes";
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

#ifdef _WIN32
    closesocket(_socket);
#elif __linux__
    close(_socket);
#endif
}