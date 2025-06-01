#pragma once

#include <iostream>
#include <vector>
#include <sstream>

namespace String {
	/**
	 * @brief split give sentence with delimiter.
	 *
	 * @param sentence the sentence which you want to split.
	 * @param delimiter the sentence will be split every the delimiter.
	 *
	 * @return splited word.
	 *
	 * For example 
	 *
	 * sentence -> This is a good sentence
	 * delimiter -> :
	 *
	 * This sentence will be...
	 * ```
	 * This:is:a:good:sentence
	 * ```
	 *
	 * converted to 
	 * ```
	 * This
	 * is 
	 * a
	 * good
	 * sentence
	 * ```
	 *
	 */
	std::vector<std::string> split(const std::string sentence, const char delimiter);

	/**
	 * @brief get file extionsion (i.g. exe, so, tar, and so on)
	 *
	 * @param file_name file name which you want to know extension.
	 *
	 * @return extension name
	 *
	 * For example
	 *
	 * file name : good_file.exe
	 * extension : exe
	 *
	 */
	std::string get_file_extionsion(const std::string file_name);

} // namespace String
