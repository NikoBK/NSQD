#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H

#include <string>
#include <WS2tcpip.h>
#include <thread>

class Message;
class TCPSocket
{
public:
	TCPSocket();
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
};

#endif // !TCP_SOCKET_H
