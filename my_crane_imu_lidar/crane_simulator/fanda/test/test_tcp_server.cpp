#include <iostream>
#include "../include/fanda/Tcp.hpp"

int main()
{
	TCP::Server server(50030);
	
	while(true){
		for (int i = 0; i < 100; i++) {
			std::string msgs = server.receive();
			std::cout << msgs << std::endl;
			msgs += "!";
			server.send(msgs);
		}
		server.reinitialize(50030);
	}

	
	return 0;
}
