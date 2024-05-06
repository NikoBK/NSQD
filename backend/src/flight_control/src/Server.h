#ifndef SERVER_H
#define SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <errno.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <string>
#include "../include/Matrice100.hpp"


class Message;

class Server
{
public:
    Server(int port, Matrice100* drone, std::ofstream* csvFile);
    ~Server();
    
    bool connected() const { return _connected; }

    void AcceptConnection();
    void HandleConnection(int *state);
    void Send(Message& message);
    void Disconnect(const std::string& reason);
    void SendError(std::string text);

private:
    int _socket;
    bool _connected;
    int _currentSize;

    int _serverSocket;
    Matrice100* _drone;
    std::ofstream* _csvFile;
};

#endif
