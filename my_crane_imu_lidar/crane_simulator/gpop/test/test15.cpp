#include <iostream>
#include <cmath>
#include <vector>
#include <unistd.h>

#include <Gpop/Bar.hpp>

int main(int argc, char const* argv[])
{
	Gpop::Bar plot;
	plot.set_y_range(0, 1.0);
	for (double i = 0; i < M_PI/2; i+=0.03) {
		double good_data = std::cos(i);
		double bad_data  = 1-std::sin(i);
		plot.plot(good_data, "good");
		plot.plot(bad_data,  "bad");
		plot.pause(100000);
	}
	return 0;
}
