#include <iostream>
#include "Client.h"


int main()
{
    struct ABC
    {
        int a;
        int b;
        int c;
    };
    
    ABC abc{1, 2, 3};
    
	Client client;
	client.Run();
//    Client::FileSpp();
	return 0;
}

