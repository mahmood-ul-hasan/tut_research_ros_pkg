#include <iostream>
#include <vector>
#include <cmath>
#include <unistd.h>

#include <Gpop/Series.hpp>

int main(int argc, char const* argv[])
{
	Gpop::Series plot;
	double data1 = 0;
	double data2 = 0;


	//make data
	std::vector<double> v;


	for (int i = 0; i < 100; i++) {
		data1 = std::sin(i * M_PI / 180);
		data2 = std::cos(i * M_PI / 180);

		plot.plot(data1);
		plot.plot(data2);
		plot.pause(100000);
	}
	return 0;
}
