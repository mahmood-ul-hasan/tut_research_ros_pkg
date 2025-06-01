#include <iostream>
#include <vector>
#include <random>

#include <Gpop/Bar.hpp>

int main(int argc, char const* argv[])
{
	Gpop::Bar plot("Bar graph");
	plot.rotate_label(-90);
	plot.set_box_width(0.1);
	plot.set_autoscale();
	plot.plot(10, "good");
	plot.plot(3, "bad");
	plot.show();

	std::cout << "Press Enter Key" << std::endl;
	std::cin.get();
	return 0;
}
