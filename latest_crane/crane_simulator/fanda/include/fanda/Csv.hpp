#pragma once

#include <iostream>
#include <vector>
//#include <optional>

namespace CSV{
class Data {
	public:
		/**
		 * @brief Get the CSV data as double
		 *
		 * @return Get CSV data as double. If you try to convert string to double, you get error.
		 */
		double get_as_double();

		/**
		 * @brief Get the CSV data as std::string
		 *
		 * @return Get the CSV data as std::string. No error occured.
		 */
		std::string get_as_string();

		/**
		 * @brief Get the CSV data as int
		 *
		 * @return Get the CSV data as int. If you try to convert string to double, you get error.
		 */
		int get_as_int();

		/**
		 * @brief Change the CSV data.
		 *
		 * @param value You should use this function to change data.
		 */
		void set(const std::string value);

	private:
		std::string value = "";
};

class CsvFile {
	public:

		/**
		 * @brief Constructor. 
		 */
		CsvFile();

		/**
		 * @brief Constructor. If the CSV file can't open, the error occured.
		 *
		 * @param file_path The path of CSV file
		 */
		CsvFile(const std::string file_path);

		/**
		 * @brief Open CSV file If you use constructor without arguments, you should use this function to open CSV file.
		 * If the CSV file can open, this class load CSV file data at the same time.
		 *
		 * @param file_path The path of CSV file
		 *
		 * @return If the CSV file opened succssfuly, return true. Otherwize return false.
		 */
		bool open(const std::string file_path);

		/**
		 * @brief Re-read CSV file. If data in CSV file change dynamically, you should use this function to follow that change.
		 *
		 * @return If the CSV file can open, return true. Otherwise, return false.
		 */
		bool reload();

		/**
		 * @brief Get the Data which CsvFile has.
		 *
		 * @param collumn The number of Collumn
		 * @param row The number of row
		 *
		 * @return Data locate in (collumn, row)
		 */
		Data operator()(const unsigned int collumn, const unsigned int row);

		/**
		 * @brief Get the collumn size of CSV file
		 *
		 * @return The collumn size of CSV file
		 */
		int collumn_size();

		/**
		 * @brief Get the row size of CSV file
		 *
		 * @return The row size of CSV file
		 */
		int row_size();

		/**
		 * @brief Display all CSV data.
		 */
		void print();

		/**
		 * @brief If CSV file opend successflly, return true. Otherwize, false.
		 *
		 * @return The CSV file could open : true, Could not open : false
		 */
		bool is_open();

		/**
		 * @brief connect another csv file to this csv file.
		 *
		 * @param another_csv_file The csv file which will be concatenate to this csv file.
		 *
		 */
		void connect(const CsvFile another_csv_file);

		/**
		 * @brief add new line to this CsvFile. new line will be a part of this CsvFile.
		 *
		 * @param new_line new line will be added to this CsvFile.
		 */
		bool add(const std::vector<std::string> new_line);
		bool add(const std::vector<double> new_line);
		bool add(const std::vector<int> new_line);

		/**
		 * @brief do random sampling and get those data as new CsvFile.
		 *
		 * @param sample_size How many line should be return. Returned Csv is composed of continued line. Default size is 1.
		 *
		 * @return new CsvFile which is the result of random sampling.
		 */
		CsvFile get_random_sampling(const unsigned int sampling_size = 1);


	private:

		/**
		 * @brief A table containing all CSV data
		 */
		std::vector<std::vector<Data>> table;

		/**
		 * @brief A path to the CSV file
		 */
		std::string file_path;

		/**
		 * @brief Flag whether the file could open or not
		 */
		bool did_open = false;
};
}

