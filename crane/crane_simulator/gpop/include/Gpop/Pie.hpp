#pragma once

#include <iostream>
#include <string>
#include <vector>
#include "Gnuplot.hpp"

namespace Gpop {
	

class Item {
	public:
		double data = 0;
		std::string name;
};

class Pie {
	public:
		Pie();
		Pie(std::string title);
		~Pie();

		void plot(double data, std::string name);
		void show();
		void pause(int usec = 0);
		void save_as_png(std::string title);
		void set_window_size(unsigned int width, unsigned int height);

	private:
		double sum();
		void set_x_range(double min, double max);
		void set_y_range(double min, double max);
		void normalize_data();


		/**
		 * @brief gnuplotと接続するパイプ
		 */
		Gnuplot pipe;


		/**
		 * @brief 表示データをためておくコンテナ
		 */
		std::vector<Item> data_container;


		/**
		 * @brief ％で計算する場合のフラグ
		 *
		 * ％で入力されていると判断した場合 true
		 * 実際の値で入力されていると判断した場合 false
		 */
		bool percent_calc = true;


		/**
		 * @brief 最後に表示したデータの角度,今回描画するデータの始まる角度
		 */
		double last_angle = 0;
};

} // namespace Gpop
