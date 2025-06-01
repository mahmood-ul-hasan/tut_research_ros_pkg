#include <cstdlib>
#include <fstream>
#include "../include/fanda/File.hpp"
#include "../include/fanda/String.hpp"

bool File::copy(const std::string from, const std::string to){

	std::string command = "cp " + from + " " + to;

	int result = std::system(command.c_str());

	if (result == 0) {
		return false;
	}
	else {
		return true;
	}
}

bool File::move(const std::string from, const std::string to){

	std::string command = "mv " + from + " " + to;

	int result = std::system(command.c_str());

	if (result == 0) {
		return false;
	}
	else {
		return true;
	}
}

bool File::remove(const std::string path){

	std::string command = "rm -rf " + path;

	int result = std::system(command.c_str());

	if (result == 0) {
		return false;
	}
	else {
		return true;
	}
}

std::vector<std::string> File::list(const std::string path){

	std::string command = "ls " + path + " > .buf_pocketool";
	std::vector<std::string> result;
	std::string buffer;

	File::remove(".buf_pocketool");
	std::system(command.c_str());
	std::ifstream buffer_file(".buf_pocketool");

	do {
		buffer_file >> buffer;
		result.push_back(buffer);
	} while (!buffer_file.eof());

	File::remove(".buf_pocketool");

	return result;
}

std::vector<std::string> File::locate(const std::string file_name){

	std::string command = "locate " + file_name + " > .buf_pocketool";
	std::vector<std::string> result;
	std::string buffer;

	File::remove(".buf_pocketool");
	std::system(command.c_str());
	std::ifstream buffer_file(".buf_pocketool");

	if (buffer_file.is_open()) {

		do {
			buffer_file >> buffer;
			result.push_back(buffer);
			std::cout << buffer << std::endl;
		} while (!buffer_file.eof());

		File::remove(".buf_pocketool");
	}

	return result;
}

bool File::make_dir(const std::string path){

	std::string command = "mkdir -p " + path;

	int result = std::system(command.c_str());

	if (result == 0) {
		return false;
	}
	else {
		return true;
	}
}

std::string File::current_path(){
	
	std::string buffer;
	std::string command = "pwd > .buf_pocketool";

	File::remove(".buf_pocketool");
	std::system(command.c_str());
	std::ifstream buffer_file(".buf_pocketool");
	buffer_file >> buffer;

	File::remove(".buf_pocketool");

	return buffer;
}

unsigned int File::size(const std::string path){

	std::string buffer;
	std::string command = "du -b " + path + " > .buf_pocketool";

	File::remove(".buf_pocketool");
	std::system(command.c_str());
	std::ifstream buffer_file(".buf_pocketool");
	buffer_file >> buffer;
	File::remove(".buf_pocketool");

	return std::atoi(buffer.c_str());
}

std::string File::who_am_i(){

	std::string buffer;
	std::string command = "whoami > .buf_pocketool";

	File::remove(".buf_pocketool");
	std::system(command.c_str());
	std::ifstream buffer_file(".buf_pocketool");
	buffer_file >> buffer;

	File::remove(".buf_pocketool");

	return buffer;
}
