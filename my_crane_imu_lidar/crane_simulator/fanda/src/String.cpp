#include "../include/fanda/String.hpp"

std::vector<std::string> String::split(const std::string sentence, const char delimiter){

	std::stringstream sentence_stream;
	sentence_stream << sentence;
	std::string buf;
	std::vector<std::string> result;

	while (std::getline(sentence_stream, buf, delimiter)) {
		result.push_back(buf);
	}

	return result;
}

std::string String::get_file_extionsion(const std::string file_name){

	auto result = String::split(file_name, '.');
	auto extension = result.back();
	return extension;
}

