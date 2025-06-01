#include <iostream>
#include <string>
#include <unistd.h>

#include <Gpop/Pie.hpp>

int main(int argc, char const* argv[])
{
	Gpop::Pie plot;
	
	plot.plot(30, "a");
	plot.plot(70, "b");
	plot.plot(70, "c");
	plot.show();

	std::cout << "Press Enter Key" << std::endl;
	std::cin.get();

	return 0;
}
