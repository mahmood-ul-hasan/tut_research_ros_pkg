#pragma once

#include <string>
#include <iostream>
#include <list>
#include <vector>
#include <fstream>
#include <utility>
#include <unistd.h>
#include "Gnuplot.hpp"

namespace Gpop {
	
	
/**
 * @brief ２次元座標を表すクラス
 */
class Data {
	public :
		double data = 0;
		std::string label;
};

/**
 * @brief ２次元のプロットのクラス
 */
class Bar {
	public:
		Bar();
		Bar(std::string title);
		Bar(std::string title, unsigned int width, unsigned int height);
		~Bar();

		void plot(double data, const std::string label);
		void show();
		void pause(int usec = 0);
		void set_box_width(double relative = 0.5);
		void rotate_label(int angle = -45);
		void set_x_range(double min, double max);
		void set_y_range(double min, double max);
		void set_title(std::string title);
		void set_x_label(std::string label);
		void set_y_label(std::string label);
		void save_as_png(std::string title);
		void set_window_size(unsigned int width, unsigned int height);
		void set_autoscale(bool should_autoscale = true);

	private:
		std::string make_command();
		Gnuplot pipe;
		std::vector<Data> data_container;
		unsigned int window_height = 640;
		unsigned int window_width  = 480;
};

} // namespace Gpop
