#include <iostream>
#include "../include/fanda/String.hpp"

int main(int argc, char const* argv[])
{
	std::string sentence = "This : is : good : news :";

	auto itr = String::split(sentence, ':');

	for (auto&& e : itr ){
		std::cout << e << std::endl;
	}

	std::cout << "-----------------------------------" << std::endl;
	std::cout << String::get_file_extionsion("test.exe") << std::endl;
	std::cout << String::get_file_extionsion("test.exe.so") << std::endl;
	return 0;
}
