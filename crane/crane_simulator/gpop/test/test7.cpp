#include <iostream>
#include <vector>
#include <cmath>
#include <unistd.h>

#include <Gpop/Series.hpp>

int main(int argc, char const* argv[])
{
	Gpop::Series plot1("1", 300, 300);
	Gpop::Series plot2("2", 300, 300);

	double data1 = 0;
	double data2 = 0;

	for (int i = 0; i < 100; i++) {
		data1 = std::sin(i * M_PI / 180);
		data2 = std::cos(i * M_PI / 180);

		plot1.plot(data1);
		plot2.plot(data2);
		plot1.pause(100000);
		plot2.pause(100000);
	}
	return 0;
}
