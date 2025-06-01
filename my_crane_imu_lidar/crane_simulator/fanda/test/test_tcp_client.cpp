#include <iostream>
#include "../include/fanda/Tcp.hpp"

int main()
{
	TCP::Client client(50030);

	for (int i = 0; i < 100; i++) {
		std::string msgs("hello");
		std::cout << "[Send]\t" << msgs << std::endl;
		client.send(msgs);
		std::cout << client.receive() << std::endl;
	}
	
	return 0;
}
