#include <iostream>
#include "../include/fanda/Udp.hpp"

int main(int argc, char const* argv[])
{
	UDP::Client client;

	for (int i = 0; i < 100; i++) {
		std::cout << client.receive() << std::endl;
	}
	return 0;
}
