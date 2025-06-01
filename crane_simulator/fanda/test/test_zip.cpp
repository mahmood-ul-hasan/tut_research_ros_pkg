#include <iostream>
#include "../include/fanda/Zip.hpp"

int main(int argc, char const* argv[])
{
	std::string file_name = "test.txt";
	std::string file_name_com = "test";
	Zip::compress_tar(file_name, file_name_com);
	Zip::compress_zip(file_name, file_name_com);
	Zip::compress_tar_gz(file_name, file_name_com);
	return 0;
}
