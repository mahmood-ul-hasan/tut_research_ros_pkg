#include <iostream>
#include "../include/fanda/File.hpp"

int main(int argc, char const* argv[])
{
	File::copy("test.txt", "../");
	File::move("test.txt", "test2.txt");
	File::remove("test2.txt");
	auto file_itr = File::list(".");
	for (auto&& e : file_itr){
		std::cout << e << std::endl;
	}

	file_itr = File::locate("moveit");
	for (auto&& e : file_itr){
		std::cout << e << std::endl;
	}

	File::make_dir("test_dir");

	std::cout << File::current_path() << std::endl;
	std::cout << File::who_am_i() << std::endl;
	std::cout << File::size("size.txt") << " byte" << std::endl;
	return 0;
}
