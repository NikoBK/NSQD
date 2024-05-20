#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H

#include <string>
#include <WS2tcpip.h>
#include <thread>

//For data exchange between server and gui
#include "../include/gui.h"

class Message;
class TCPSocket
{
public:
	TCPSocket(UpdateVariables* updVars);
	~TCPSocket();

	bool Connect(const std::string& host, int port);
	void HandleReceive();
	void Send(Message& message);
	void Disconnect(const std::string& reason);

	bool connected() const;


private:
	bool _connected;
	std::thread _thread;
	int _currentSize;
	SOCKET _socket;

	//For data exchange between server and gui
	UpdateVariables * _updVars;
};

#endif // !TCP_SOCKET_H
