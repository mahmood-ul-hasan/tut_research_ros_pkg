#include <Gpop/Series.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

int main(int argc, char const* argv[])
{
	
	Gpop::Series plot;

	std::vector<double> v;
	for (int i = 0; i < 180; i++) {
		v.push_back(std::cos(i*M_PI/180));
	}
	plot.plot(v);
	plot.show();

	std::cout << "Press Enter Key" << std::endl;
	std::cin.get(); 

	return 0;
}


