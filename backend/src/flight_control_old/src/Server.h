#ifndef SERVER_H
#define SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <iostream>
#include <errno.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <string>

class Message;
class Server
{
public:
    Server(int port);
    ~Server();
    
    bool connected() const { return _connected; }

    void AcceptConnection();
    void HandleConnection();
    void Send(Message& message);
    void Disconnect(const std::string& reason);

private:
    int _socket;
    bool _connected;
    int _currentSize;

    int _serverSocket;
};

#endif
