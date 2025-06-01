#include <iostream>
#include <vector>

#include <Gpop/Bar.hpp>

int main(int argc, char const* argv[])
{
	Gpop::Bar plot;
	plot.plot(10, "good");
	plot.plot(24, "soso");
	plot.plot(6,  "nomal");
	plot.plot(4,  "notgood");
	plot.plot(1,  "bad");
	plot.show();

	std::cout << "Press Enter Key" << std::endl;
	std::cin.get();

	return 0;
}
