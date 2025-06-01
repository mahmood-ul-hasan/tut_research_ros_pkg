#include <iostream>
#include <sstream>

#include <Gpop/Series.hpp>

int main(int argc, char const* argv[])
{
	std::vector<std::vector<double>> vec_table;
	Gpop::Series plot;
	for (int coeff = 1; coeff < 5; coeff++) {
		std::vector<double> v;
		for (int i = -100; i < 100; i++) {
			v.push_back(coeff*i*i);
		}
		vec_table.push_back(v);
	}

	int i = 0;
	for (auto&& vec : vec_table){
		std::stringstream option;
		option << " title \"num" << i++ << " \" ";
		plot.plot(vec, option.str());
	}
	plot.show();

	std::cout << "Press Enter Key" << std::endl;
	std::cin.get();
	return 0;
}
