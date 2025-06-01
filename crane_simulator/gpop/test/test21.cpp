#include <iostream>

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

	for (auto&& vec : vec_table){
		plot.plot(vec);
	}
	plot.show();

	std::cout << "Press Enter Key" << std::endl;
	std::cin.get();
	return 0;
}
