#include <iostream>
#include <random>

#include <Gpop/Hist.hpp>

int main(int argc, char const* argv[])
{
	std::random_device rnd;
	std::normal_distribution<> dist;

	std::vector<double> data;
	for (int i = 0; i < 1000; i++) {
		data.push_back(dist(rnd));
	}

	Gpop::Hist plot;
	plot.set_x_range(-5,5);
	plot.plot(data);
	plot.show();

	std::cout << "Press Enter Key" << std::endl;
	std::cin.get();
	
	return 0;
}
