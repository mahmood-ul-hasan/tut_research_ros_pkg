#pragma once

#include <iostream>
#include <cstdlib>
#include <vector>

namespace File {
	/**
	 * @brief copy a file/direcotry to another location.
	 *
	 * @param from file/directory which will be copied.
	 * @param to file/direcotory which is destination.
	 *
	 * @return if success, return true, otherwise, false.
	 */
	bool copy(const std::string from, const std::string to);

	/**
	 * @brief move a file/directory to another location.
	 *
	 * @param from	file/direcotory which will be moved.
	 * @param to file/direcotory which is destination.
	 *
	 * @return if success, return true, otherwise, false.
	 */
	bool move(const std::string from, const std::string to);

	/**
	 * @brief remove a file/directory.
	 *
	 * @param path file/directory path which will be deleted.
	 *
	 * @return if success, ture will be returned, otherwise, false.
	 */
	bool remove(const std::string path);

	/**
	 * @brief return file and directory list of given path.
	 *
	 * @param path the path which you want to get contents list.
	 *
	 * @return the file and directory list.
	 */
	std::vector<std::string> list(const std::string path);

	/**
	 * @brief return file path which you want to find. 
	 *
	 * @param file_name file name which you want to know the path.
	 *
	 * @return the path of given file name.
	 */
	std::vector<std::string> locate(const std::string file_name);

	/**
	 * @brief make a directory.
	 *
	 * @param path directory structure which you want to make.
	 *
	 * @return true will be returned if success, otherwise, false.
	 *
	 * It is possible to make multi directory in once.
	 * ```
	 * make_dir("dir1/dir2/dir3");
	 * ```
	 */
	bool make_dir(const std::string path);

	/**
	 * @brief get current working directory.
	 *
	 * @return current working directory
	 */
	std::string current_path();

	/**
	 * @brief calculate file or folder size[byte]
	 *
	 * @param path path which you want to know the size.
	 *
	 * @return size[byte] of the file or folder.
	 */
	unsigned int size(const std::string path);

	/**
	 * @brief get user name.
	 *
	 * @return current user name.
	 */
	std::string who_am_i();
} // namespace File

