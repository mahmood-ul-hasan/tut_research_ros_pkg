#include <iostream>
#include <unistd.h>
#include "../include/fanda/Udp.hpp"

int main(int argc, char const* argv[])
{
	UDP::Server server;

	for (int i = 0; i < 100; i++) {
		std::string msgs = "hello\t" + std::to_string(i);
		server.send(msgs);
		sleep(1);
	}
	return 0;
}
