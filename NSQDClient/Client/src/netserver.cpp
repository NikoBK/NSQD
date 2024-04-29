// COMMENTED OUT - SERVES AS BACKUP
//#include <cstdint>
//#include <iostream>
//#include <cstring>
//#include <WinSock2.h>
//#include <string>

/// <summary>
/// Hello = 1
/// RPY = 2
/// ... = ?
/// </summary>
//class Message {
//protected:
//    char id;
//
//public:
//    Message(char id) : id(id) {}
//
//    char getID() const { return id; }
//};
//
//class IncomingMessage : public Message {
//public:
//    IncomingMessage(char id) : Message(id) {}
//
//    virtual void readData(const char* buffer) = 0;
//    virtual void handle() = 0;
//};
//
//class OutgoingMessage : public Message {
//public:
//    OutgoingMessage(char id) : Message(id) {}
//
//    virtual void writeData(char* buffer) const = 0;
//};
//
//class HelloMessage : public IncomingMessage {
//private:
//    int resCode;
//
//public:
//    HelloMessage(char id) : IncomingMessage(id) {}
//
//    void readData(const char* buffer) override {
//        id = buffer[0];
//        memcpy(&resCode, buffer + 1, sizeof(int));
//        handle();
//    }
//
//    void handle() override {
//        std::cout << "Handled Hello message, resCode: " << resCode << std::endl;
//    }
//};
//
//class RPYMessage : public IncomingMessage {
//private:
//    float roll;
//    float pitch;
//    float yaw;
//
//public:
//    RPYMessage(char id) : IncomingMessage(id) {}
//
//    void readData(const char* buffer) override {
//        id = buffer[0];
//        memcpy(&roll, buffer + 1, sizeof(float));
//        memcpy(&pitch, buffer + 5, sizeof(float));
//        memcpy(&yaw, buffer + 9, sizeof(float));
//        handle();
//    }
//
//    void handle() override {
//        std::cout << "Handled RPY message, roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
//    }
//};
//
//void onRPYMsg(char msgId, const char* buffer) {
//    RPYMessage msg = RPYMessage(msgId);
//    msg.readData(buffer);
//}
//
//void onHelloMsg(char msgId, const char* buffer) {
//    HelloMessage msg = HelloMessage(msgId);
//    msg.readData(buffer);
//}
//
//void sendMessage(const OutgoingMessage& msg, int skt) {
//    char buffer[sizeof(msg)];
//    msg.writeData(buffer);
//
//    send(skt, buffer, sizeof(msg), 0);
//}
//
//void receiveAndHandleMessages(int clientSocket) {
//    char buffer[1024];
//    while (true) {
//        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
//        std::cout << "receiving data. bytesReceived: " << bytesReceived << std::endl;
//        if (bytesReceived <= 0) {
//            std::cerr << "bytes received are less than or equal to 0. Breaking loop..." << std::endl;
//            break;
//        }
//
//        char messageId = buffer[0];
//        // std::cout << "Received message with ID: " << static_cast<int>(messageId) << std::endl;
//        switch (static_cast<int>(messageId))
//        {
//        case 1: // Hello
//            onHelloMsg(messageId, buffer);
//            break;
//        case 2:
//            onRPYMsg(messageId, buffer);
//            break;
//        default:
//            std::cerr << "Unhandled message id: " << static_cast<int>(messageId) << std::endl;
//            break;
//        }
//    }
//}
//
//int main() {
//    WSADATA wsaData;
//    WSAStartup(MAKEWORD(2, 2), &wsaData);
//    std::cout << "Starting server..." << std::endl;
//
//    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
//    struct sockaddr_in serverAddress;
//    serverAddress.sin_family = AF_INET;
//    serverAddress.sin_port = htons(8888);
//    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
//    bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
//    listen(serverSocket, SOMAXCONN);
//    std::cout << "Server started" << std::endl;
//
//    struct sockaddr_in clientAddress;
//    int clientAddressSize = sizeof(clientAddress);
//    int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddress, &clientAddressSize);
//    if (clientSocket == -1) {
//        std::cerr << "Error accepting client connection." << std::endl;
//        return 1;
//    }
//    else {
//        std::cout << "Client connected!" << std::endl;
//    }
//
//    receiveAndHandleMessages(clientSocket);
//
//    std::cout << "Stopping server..." << std::endl;
//    closesocket(clientSocket);
//    closesocket(serverSocket);
//    WSACleanup();
//    return 0;
//}