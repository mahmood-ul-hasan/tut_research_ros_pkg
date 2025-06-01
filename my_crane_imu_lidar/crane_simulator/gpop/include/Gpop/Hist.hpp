#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include "Gnuplot.hpp"

namespace Gpop {
	
class Bin {
	public:
		double x;
		double y;
};

class Hist {
	public:
		Hist();
		Hist(const unsigned int bin_number);
		~Hist();
		void plot(std::vector<double> &data);
		void show();
		void set_line(bool should_use_line);
		void set_x_label(std::string label);
		void set_y_label(std::string label);
		void set_window_size(unsigned int width, unsigned int height);
		void set_x_range(double min, double max);
		void set_y_range(double min, double max);
		void set_autoscale(bool should_autoscale = true);


	private:
		std::vector<std::vector<Bin>> bin_container;
		Gnuplot pipe;
		bool is_line  = false;
		double trasnparency = 0;
		double relative_width;
		double bin_width;
		unsigned int bin_number = 10;
		std::vector<Bin> make_hist(std::vector<double> data);
		unsigned int count_elements(std::vector<double> data, double first_number, double end_number);
		std::string make_command();
};

} // namespace Gpop
