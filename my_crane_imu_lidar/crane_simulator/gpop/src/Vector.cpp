#include "../include/Gpop/Vector.hpp"

namespace Gpop {
	
/**
 * @brief コンストラクタ
 */
Vector::Vector() : pipe() {

}

/**
 * @brief デストラクタ
 *
 * gnuplotを終了する
 */
Vector::~Vector() {
	this->pipe.write_command("quit");
}

/**
 * @brief 描画するベクターデータを追加する
 *
 * @param x ベクターのｘ座標
 * @param y ベクターのｙ座標
 * @param dx ベクターのｘ要素
 * @param dy ベクターのｙ要素
 */
void Vector::plot(double x, double y, double dx, double dy){
	VectorElement elem;
	
	elem.x = x;
	elem.y = y;
	elem.dx = dx;
	elem.dy = dy;

	this->data_container.push_back(elem);
}

/**
 * @brief Vector::plot()で追加したベクターを描画する
 */
void Vector::show(){

	//これからインラインモードでデータ入力することをgnuplotに伝える
	this->pipe.write_command("plot '-' with vector");

	for (auto&& e : this->data_container){
		std::string command = std::to_string(e.x) + "\t" 
			                + std::to_string(e.y) + "\t"
							+ std::to_string(e.dx) + "\t"
							+ std::to_string(e.dy);

		this->pipe.write_command(command);
	}
	
	//全てのデータの送信が終了したことをeを送ることで伝える
	this->pipe.write_command("e");
	//強制的にpipeの内容データを書き出し
	this->pipe.flush();
}

void Vector::set_x_label(std::string label){

	this->pipe.util_set_x_label(label);
}

void Vector::set_y_label(std::string label){

	this->pipe.util_set_y_label(label);
}

void Vector::set_x_range(double min, double max){

	this->pipe.util_set_x_range(min, max);
}

void Vector::set_y_range(double min, double max){

	this->pipe.util_set_y_range(min, max);
}

void Vector::set_window_size(unsigned int width, unsigned int height){
	
	this->pipe.util_set_window_size(width, height);
}

} // namespace Gpop
