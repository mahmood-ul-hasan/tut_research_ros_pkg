#include <iostream>

#include <Gpop/Pie.hpp>

int main(int argc, char const* argv[])
{
	Gpop::Pie plot;

	plot.plot(30, "a");
	plot.plot(70, "b");

	plot.show();

	plot.save_as_png("This_is_pie_tutorial");
	std::cout << "png file was saved" << std::endl;

	std::cout << "Eress Enter Key" << std::endl;
	std::cin.get();
	return 0;
}
