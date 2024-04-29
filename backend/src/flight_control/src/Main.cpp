#include "Server.h"

int main()
{
    Server server(8888);

    while (true) 
    {
        // if not connected
        if (!server.connected()) 
        {
            // see if theres a connection
            server.AcceptConnection();
        }
        else 
        {
            // handle the current connection
            server.HandleConnection();
        }
    }
}