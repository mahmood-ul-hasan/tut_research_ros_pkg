#define BOOST_TEST_MODULE CSV MODULE
#include <boost/test/included/unit_test.hpp>

#include "../include/fanda/Csv.hpp"

BOOST_AUTO_TEST_CASE(open_csv)
{
	CSV::CsvFile csv("./test.csv");
	BOOST_TEST(csv.is_open());
}

BOOST_AUTO_TEST_CASE(size)
{
	CSV::CsvFile csv("./test.csv");
	BOOST_TEST(csv.row_size() == 3); 
	BOOST_TEST(csv.collumn_size() == 10);
	BOOST_TEST(csv.row_size() != 5); 
	BOOST_TEST(csv.collumn_size() != 5);
}

BOOST_AUTO_TEST_CASE(get_value)
{
	CSV::CsvFile csv("./test.csv");
	BOOST_TEST(csv(0,0).get_as_string() == "1");
	BOOST_TEST(csv(0,0).get_as_int() == 1);
	BOOST_TEST(csv(0,0).get_as_double() == 1.0);
}

BOOST_AUTO_TEST_CASE(connect)
{
	CSV::CsvFile csv1("./test.csv");
	CSV::CsvFile csv2("./test.csv");
	csv1.connect(csv2);
	csv1.print();
}

BOOST_AUTO_TEST_CASE(add0)
{
	CSV::CsvFile csv("./test.csv");
	std::vector<std::string> vec = {"a", "b", "c"};
	csv.add(vec);
	csv.print();
}

BOOST_AUTO_TEST_CASE(add1)
{
	CSV::CsvFile csv;
	std::vector<std::string> vec = {"a", "b", "c"};
	BOOST_TEST(csv.add(vec) == true);
	csv.print();
}

BOOST_AUTO_TEST_CASE(add2)
{
	CSV::CsvFile csv;
	std::vector<int> vec = {1, 2, 3};
	BOOST_TEST(csv.add(vec) == true);
	csv.print();
}

BOOST_AUTO_TEST_CASE(add3)
{
	CSV::CsvFile csv;
	std::vector<double> vec = {1.2, 2.2, 3.2};
	BOOST_TEST(csv.add(vec) == true);
	csv.print();
}

BOOST_AUTO_TEST_CASE(add4)
{
	CSV::CsvFile csv;
	std::vector<double> v1 = {1.1, 2.1, 3.1, 4.1};
	BOOST_TEST(csv.add(v1) == true);
	std::vector<double> v2 = {1.2, 2.2, 3.2};
	BOOST_TEST(csv.add(v2) == false);
	csv.print();
}

BOOST_AUTO_TEST_CASE(random_sampling)
{
	CSV::CsvFile csv("./test.csv");
	auto new_csv = csv.get_random_sampling(2);
	new_csv.print();
}

BOOST_AUTO_TEST_CASE(set_data1)
{
	CSV::CsvFile csv;
	std::vector<double> v1 = {1.1, 2.1, 3.1, 4.1};
	csv.add(v1);
	csv(0,0).set("2.2");
	BOOST_TEST(csv(0,0).get_as_double() = 2.2);
}

BOOST_AUTO_TEST_CASE(set_data2)
{
	CSV::CsvFile csv;
	std::vector<double> v1 = {1.1, 2.1, 3.1, 4.1};
	csv.add(v1);
	BOOST_TEST(csv(0,0).get_as_int() == 1);
}

BOOST_AUTO_TEST_CASE(display)
{
	CSV::CsvFile csv;
	for (int i = 0; i < 1000; i++) {
		std::vector<int> vec = {1,2,3,4,5,6};
		csv.add(vec);
	}
	csv.print();
}
