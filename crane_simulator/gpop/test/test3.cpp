#include <unistd.h>
#include <iostream>
#include <vector>
#include <cmath>

#include <Gpop/Series.hpp>

int main(int argc, char const* argv[])
{
	
	Gpop::Series plot;
	double data = 0;

	plot.limit_max_number(50);
	plot.set_y_range(-1.2,1.2);

	////make data
	for (int i = 0; i < 1000; i++) {
		data = sin(i * M_PI / 180);

		plot.plot(data);
		plot.pause(50000);
	}

	return 0;
}


