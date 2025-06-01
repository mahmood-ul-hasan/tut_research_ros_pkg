#include <iostream>
#include <random>
#include <vector>

#include <Gpop/Series.hpp>

int main(int argc, char const* argv[])
{
	std::random_device rnd;
	std::vector<double>  vec;
	for (int i = 0; i < 100; i++) {
		vec.push_back(rnd());
	}

	Gpop::Series plot;
	plot.plot(vec, "t\"Main\"");
	plot.set_title("Main", 12);
	plot.set_x_label("x label", 11);
	plot.set_y_label("y label", 11);
	plot.show();

	std::cout << "Press Enter Key" << std::endl;
	std::cin.get();
	return 0;
}
