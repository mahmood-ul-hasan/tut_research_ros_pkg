#include <iostream>
#include <cmath>

#include <Gpop/Hist.hpp>

using namespace std;

// Nomal Distribution 
double nd(double x) {
	return 1/(sqrt(2*M_PI))*exp(-(x*x)/2);
}

int main(int argc, char const* argv[])
{
	double res = 50;

	Gpop::Hist plot;

	std::vector<double> vec;
	for (int i = -50; i < res; i++) {
		double x = (double)i/res;
		vec.push_back(nd(x));
	}
	plot.plot(vec);
	plot.show();

	std::cout << "Press Enter Key" << std::endl;
	cin.get();

	return 0;
}
