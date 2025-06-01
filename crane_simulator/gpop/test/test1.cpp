#include <iostream>
#include <unistd.h>

#include <Gpop/Gnuplot.hpp>

int main(int argc, char const* argv[])
{
	
	Gnuplot gp;
	gp.write("plot sin(x)\n");
	gp.flush();
	sleep(5);

	return 0;
}
