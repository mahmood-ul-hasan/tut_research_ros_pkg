#pragma once
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace UDP {
	/**
	 * @brief UDP server class
	 * A role of this class in UDP is sender.
	 * If you receive messages, use Udp::Client class.
	 *
	 *  [server] -- UDP --> [client (port, ip)]
	 */
	class Server {
		
		public:
			Server(const unsigned int port = 50030, const std::string ip = "127.0.0.1");
			~Server();
			int send(const std::string message);

		private:
			int sock;
			struct sockaddr_in address;

	};

	/**
	 * @brief Constructor of server(Sender)
	 *
	 * @param port Port of UDP. Default port number is 50030. Use same number which is set in Udp::Client class's constructor.
	 * @param ip IP address of UDP. Default IP address is 127.0.0.1(localhost)
	 */
	Server::Server(const unsigned int port, const std::string ip){

		//Make socket
		this->sock = socket(AF_INET, SOCK_DGRAM, 0);
		if (this->sock < 0) {
			perror("Socket");
		}

		//Set configure
		this->address.sin_family = AF_INET;
		this->address.sin_port   = htons(port);
		this->address.sin_addr.s_addr = inet_addr(ip.c_str());
	}

	/**
	 * @brief Destructor of server(sender)
	 *
	 * This method close the socket.
	 */
	Server::~Server(){
		close(this->sock);
	}

	/**
	 * @brief This method send to the message to the client(receiver)
	 *
	 * @param message Message that is sent to the client.
	 * Maximum message that you can send is 1024 byte.
	 *
	 * @return Number of sent message.
	 */
	int Server::send(const std::string message){
		int result = sendto(this->sock, message.c_str(), message.size(), 0, (struct sockaddr *)&this->address, sizeof(this->address));
		if (result <= 0) {
			perror("Send");
		}
		return result;
	}


	/**
	 * @brief UDP client classs
	 * A role of this class in UDP is receiver.
	 * If you want to send messages, use Udp::Server class.
	 *
	 *  [server] -- UDP --> [client (port, ip)]
	 */
	class Client {
		public:
			Client(const unsigned int port = 50030);
			~Client();
			std::string receive();

		private:
			int sock;
			struct sockaddr_in address;
	};

	/**
	 * @brief Constructor of client class.
	 *
	 * @param port A port of UDP. Use same port which you set in Udp::Server class's constructor. Default port number is 50030.
	 */
	Client::Client(const unsigned int port) {
		
		//Make socket
		this->sock = socket(AF_INET, SOCK_DGRAM, 0);
		if (this->sock < 0) {
			perror("Socke");
		}

		//Set configure
		this->address.sin_family = AF_INET;
		this->address.sin_port = htons(port);
		this->address.sin_addr.s_addr = INADDR_ANY;
		bind(this->sock, (struct sockaddr *)&this->address, sizeof(this->address));
	}

	/**
	 * @brief Destructor of client class.
	 * 
	 * This method closes socket.
	 */
	Client::~Client(){
		close(this->sock);
	}

	/**
	 * @brief This method receives messages from server.
	 *
	 * @return Received message.
	 *
	 * A capacity of receive message is 1024 byte.
	 */
	std::string Client::receive(){
		char buf[1024] = {};

		int result = recv(this->sock, buf, sizeof(buf), 0);

		if (result <= 0) {
			perror("Receive");
			close(this->sock);
		}

		return std::string(buf);
	}
} // namespace Udp

