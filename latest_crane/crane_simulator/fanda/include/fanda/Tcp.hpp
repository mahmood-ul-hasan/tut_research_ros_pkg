//This program is a simple TCP library which is implemented server and client.

#pragma once
#include <iostream>
#include <cstring>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <error.h>

namespace TCP {
class Server {
	public:
		Server(const unsigned int port = 50030);
		~Server();
		int send(const std::string message);
		std::string receive();
		void reinitialize(const unsigned int port = 50030);
		
	private:
		int server_sock;
		int client_sock;
		struct sockaddr_in server_address;
		struct sockaddr_in client_address;
};

/**
 * @brief A Constructor of TCP server
 *
 * @param port A port which is specified between 49152 and 65535. Default argument port number is 50030
 */
Server::Server(const unsigned int port) {

	this->reinitialize(port);

}

Server::~Server(){
}

/**
 * @brief This function reset the configuration of TCP server
 *
 * @param port set between 49152 and 65535. Default argument port number is 50030
 *
 * If you want the server to keep to stay alive after a client is closed, 
 * call this function after connection is closed.
 * See a sample program if you want to know more detail.
 */
void Server::reinitialize(const unsigned int port){

	//自身のソケットの設定
	bzero((char *)&this->server_address, sizeof(this->server_address));
	this->server_address.sin_port = htons(port);
	this->server_address.sin_family = AF_INET;
	this->server_address.sin_addr.s_addr = INADDR_ANY;

	//ソケットの生成
	this->server_sock = socket(AF_INET, SOCK_STREAM, 0);
	int yes = 1;
	setsockopt(this->server_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));

	//ソケットに設定を渡す
	bind(this->server_sock, (struct sockaddr *)&this->server_address, sizeof(this->server_address));

	//接続待ちに入る
	listen(this->server_sock, 5);

	//接続の受付
	std::cout << "接続を待っています．クライアントプログラムを動かしてください．" << std::endl;
	unsigned int client_address_size = sizeof(this->client_address);
	this->client_sock = accept(this->server_sock, (struct sockaddr *)&this->client_address, &client_address_size);
	std::cout << "[Accessed from " << inet_ntoa(client_address.sin_addr) << "\t" << ntohs(client_address.sin_port) << std::endl;
	close(this->server_sock);
}

/**
 * @brief This function send a message from server to client.
 *
 * @param message message that you want to send to client.
 *
 * @return A amount of byte which is sent.
 *
 * Call this function after connection between server and client is established
 */
int Server::send(const std::string message){

	int result_write = write(this->client_sock, message.c_str(), message.size());
	if (result_write <= 0) {
		perror("Send");
	}
	return result_write;
}

/**
 * @brief This function receive a message from client.
 *
 * @return message from client. Maximam receive byte is 1024 byte
 *
 * Call this function after connection between server and client is established
 */
std::string Server::receive(){

	char receive_buff[1024] = {};
	int result_read = read(this->client_sock, receive_buff, sizeof(receive_buff));

	if (result_read <= 0) {
		perror("Receive");
		close(this->client_sock);
	}

	return std::string(receive_buff);
}


class Client {
	public:
		Client(unsigned int port = 50030, std::string ip = "127.0.0.1");
		~Client();
		int send(const std::string message);
		std::string receive();

	private:
		int server_sock;
		int client_sock;
		struct sockaddr_in server_address;
		struct sockaddr_in client_address;


};

/**
 * @brief A constructor of TCP client
 *
 * @param port A port which is specified between 49152 and 65535. Default argument port number is 50030
 * @param ip A ip address which is spedified. Default argument ip is localhost(127.0.0.1)
 */
Client::Client(unsigned int port, std::string ip) {

	//サーバのアドレスを設定
	bzero((char *)&this->server_address, sizeof(this->server_address));
	this->server_address.sin_family = AF_INET;
	this->server_address.sin_port = htons(port);
	this->server_address.sin_addr.s_addr = inet_addr(ip.c_str());

	//ソケットの生成
	this->server_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (this->server_sock < 0) {
		perror("Socket");
	}

	//接続
	int result_connect = connect(this->server_sock, (struct sockaddr *)&this->server_address, sizeof(this->server_address));
	if (result_connect < 0) {
		perror("Connect");
	}
}

Client::~Client(){
	close(this->server_sock);
}

/**
 * @brief This function send a message from client to server.
 *
 * @param message message that you want to send to server.
 *
 * @return A amount of byte which is sent.
 *
 * Call this function after connection between server and client is established
 */
int Client::send(const std::string message){

	int result_write = write(this->server_sock, message.c_str(), message.size());
	if (result_write <= 0) {
		perror("Sending message failed");
	}
	return result_write;
}

/**
 * @brief This function receive a message from server.
 *
 * @return message from client. Maximam receive byte is 1024 byte
 *
 * Call this function after connection between server and client is established
 */
std::string Client::receive(){

	char receive_buff[1024] = {};
	int result_read = read(this->server_sock, receive_buff, sizeof(receive_buff));

	if (result_read <= 0) {
		perror("Receiving message failed");
		close(this->server_sock);
	}

	return std::string(receive_buff);
}

}
