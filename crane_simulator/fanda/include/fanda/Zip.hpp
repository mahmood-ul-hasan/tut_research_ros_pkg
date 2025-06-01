#pragma once

#include <iostream>
#include <cstdlib>

namespace Zip {

	/**
	 * @brief Extract ** .tar ** file.
	 *
	 * @param path the file path which you want to extract.
	 */
	void extract_tar(const std::string path);

	/**
	 * @brief Extract ** .zip ** file.
	 *
	 * @param path the file path which you want to extract.
	 */
	void extract_zip(const std::string path);

	/**
	 * @brief Extract ** .tar.gz ** file.
	 *
	 * @param path the file path which you want to extract.
	 */
	void extract_tar_gz(const std::string path);


	/**
	 * @brief Compress to ** .tar ** file.
	 *
	 * @param from_directory the directory which you want to compress as tar file.
	 * @param to_compress the name of the tar file.
	 *
	 * For example
	 *
	 * ```
	 * compless_tar("from", "to");
	 * ```
	 * 
	 * Before
	 * - from
	 *
	 * After
	 * - from
	 * - to.tar
	 */
	void compress_tar(   const std::string from_directory, const std::string to_compress);

	/**
	 * @brief Compress ** .zip ** file.
	 *
	 * @param from_directory the directory which you want to compress as zip file.
	 * @param to_compress the name of the zip file.
	 */
	void compress_zip(   const std::string from_directory, const std::string to_compress);

	/**
	 * @brief Compress ** .tar.gz ** file.
	 *
	 * @param from_directory the directory which you want to compress as tar.gz file.
	 * @param to_compress the neme of the tar.gz file.
	 */
	void compress_tar_gz(const std::string from_directory, const std::string to_compress);
} // namespace Zip
