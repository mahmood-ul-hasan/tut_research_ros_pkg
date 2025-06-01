#pragma once

#include <iostream>
#include <string>
#include <list>
#include <vector>
#include <utility>
#include "Gnuplot.hpp"

namespace Gpop {
	
/**
 * @brief 2次元ベクターを表すクラス
 */
class VectorElement {
	public:
		double x = 0;	///x座標
		double y = 0;   ///y座標
		double dx = 0;  ///x成分
		double dy = 0;  ///y成分
};

class Vector {
	public:
		Vector();
		~Vector();
		void plot(double x, double y, double dx, double dy);
		void show();
		void set_x_label(std::string label);
		void set_y_label(std::string label);
		void set_window_size(unsigned int width, unsigned int height);
		void set_x_range(double min, double max);
		void set_y_range(double min, double max);

	private:
		Gnuplot pipe;
		std::vector<VectorElement> data_container;
};

} // namespace Gpop
