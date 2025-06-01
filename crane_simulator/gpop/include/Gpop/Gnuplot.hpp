#pragma once

#include <cstdio>
#include <memory>
#include <iostream>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

class Gnuplot {
	public:


		Gnuplot();
		~Gnuplot();
		void write(std::string command);
		void write_command(std::string command);
		bool is_open();
		void flush();

		//設定関数．各クラスで適宜再実装される
		void util_set_title(std::string title);
		void util_set_title(std::string title, unsigned int font_size);
		void util_set_x_label(std::string label);
		void util_set_x_label(std::string label, unsigned int font_size);
		void util_set_y_label(std::string label);
		void util_set_y_label(std::string label, unsigned int font_size);
		void util_set_window_size(unsigned int width, unsigned int height);
		void util_set_x_range(double min, double max);
		void util_set_y_range(double min, double max);
		void util_set_grid(bool should_set);
		void util_set_autoscale(bool should_autoscale);	


	private:
		FILE* file_discripter;
		bool pipe_state = false;

		bool open_gnuplot();
		void close_gnuplot();
};
