#include <iostream>
#include "../include/fanda/Zip.hpp"

void Zip::extract_tar(const std::string path){

	std::string command = "tar -xvf " + path;

	std::system(command.c_str());
}

void Zip::compress_tar(const std::string from_directory, const std::string to_compress){
	
	std::string command = "tar -cvf " + to_compress + ".tar " + from_directory;

	std::system(command.c_str());
}

void Zip::extract_zip(const std::string path){

	std::string command = "unzip " + path;

	std::system(command.c_str());
}

void Zip::compress_zip(const std::string from_directory, const std::string to_compress){

	std::string command = "zip -r " + to_compress + " " + from_directory;

	std::system(command.c_str());

}

void Zip::extract_tar_gz(const std::string path){

	std::string command = "tar -zxvf " + path;

	std::system(command.c_str());

}

void Zip::compress_tar_gz(const std::string from_directory, const std::string to_compress){

	std::string command = "tar -zcvf " + to_compress + ".tar.gz " + from_directory;

	std::system(command.c_str());
}
