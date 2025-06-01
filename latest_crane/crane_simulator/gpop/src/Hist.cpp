#include <algorithm>
#include <cmath>
#include "../include/Gpop/Hist.hpp"

namespace Gpop {
	
Hist::Hist() : pipe() {

}

/**
 * @brief コンストラクタ
 *
 * @param bin_number ビンの数
 *
 * ビンとは各階級の数．データをどのくらいの頻度に分割するか
 * を示す．
 *
 * \note デフォルトはbin_number = 10
 */
Hist::Hist(const unsigned int bin_number){

	this->bin_number = bin_number;

}

/**
 * @brief デストラクタ
 */
Hist::~Hist() {

	this->pipe.write_command("quit");
}


/**
 * @brief ヒストグラムを作成するための生データを入力する
 * ための関数．
 *
 * @param data
 */
void Hist::plot(std::vector<double> &data){

	this->bin_container.push_back(this->make_hist(data));

}

/**
 * @brief 指定範囲内にある要素の数を計算する
 *
 * @param data 元のデータ
 * @param first_number 調べたい範囲の最小値
 * @param end_number 調べたい範囲の最大値
 *
 * first_number <= x < end_number
 * の値をカウントする
 *
 * @return 範囲内に存在するデータの数
 */
unsigned int Hist::count_elements(std::vector<double> data, double first_number, double end_number){

	unsigned int counted_number = 0;

	for (auto&& e : data){
		if ((first_number <= e) && (e < end_number)) {
			counted_number++;
		}
	}

	return counted_number;
}


/**
 * @brief 与えられたデータからヒストグラムを計算する関数
 *
 * @param data ヒストグラムを計算する元となるデータ
 *
 * @return 計算したヒストグラムのデータ
 * 各要素の階級と度数が入ったデータ
 */
std::vector<Bin> Hist::make_hist(std::vector<double> data){

	std::vector<Bin> bin_vec;

	//dataの最大値を抽出
	double max = *std::max_element(data.begin(), data.end());
	double min = *std::min_element(data.begin(), data.end());

	//bin_widthを決めるために最大値をbin_numberで割る
	this->bin_width = std::abs(max - min) / this->bin_number;

	//範囲内の要素の数を数える
	for (unsigned int i = 0; i < this->bin_number; i++) {
		Bin bin;
		//binの中央値を計算
		double first_number = min + i * this->bin_width;
		double end_number = first_number + this->bin_width;
		bin.x = first_number + this->bin_width / 2.0;
		bin.y = this->count_elements(data, first_number, end_number); 
		bin_vec.push_back(bin);
	}

	return bin_vec;
}

/**
 * @brief グラフを表示する関数
 *
 */
void Hist::show() {

	//棒グラフを色で埋めることをgnuplotに伝える
	if (this->trasnparency > 0) {
		this->pipe.write_command("set style fill transparent solid 0.7");
	}
	else {
		this->pipe.write_command("set style fill solid");
	}

	//棒グラフに枠を付ける
	if (this->is_line) {
		this->pipe.write_command("set style fill solid border lc rgb \"black\"");
	}

	//これからインラインモードでデータを入力することをgnuplotに伝える
	//this->pipe.write_command("plot '-'with boxes lw 1 notitle");
	this->pipe.write_command(this->make_command());

	for (auto&& vec : this->bin_container){
		for (auto&& e : vec){
			std::string com = std::to_string(e.x) + "\t" + std::to_string(e.y);
			this->pipe.write_command(com);
		}
		//ひとつの行が終わったことをeを送ることで伝える．
		this->pipe.write_command("e");
	}

	//全てのデータの送信が終了したことをeを送ることで伝える
	this->pipe.flush();
}

/**
 * @brief ヒストグラムの棒グラフをラインで囲むかどうかを
 * 設定する関数
 *
 * @param shoule_use_line  true -> use line, false -> unuse line
 */
void Hist::set_line(bool shoule_use_line){

	if (shoule_use_line) {
		this->is_line = true;
	}
	else {
		this->is_line = false;
	}
}

/**
 * @brief 追加されたデータの数に応じてgnuplotのコマンドを作成する関数
 *
 * @return 作成されたgnuplotの関数
 */
std::string Hist::make_command(){
	
	std::string command = "plot ";

	//bin_containerのデータの種類だけ<plot '-'with boxes lw 1 notitle"
	for (int i = 0; i < (int)this->bin_container.size(); i++) {
		command += "'-' w boxes lw 1 notitle , ";
	}

	return command;
}

/**
 * @brief x軸のラベルを設定する関数
 *
 * @param label 設定したい軸ラベル
 */
void Hist::set_x_label(std::string label){
	this->pipe.util_set_x_label(label);
}

/**
 * @brief y軸のラベルを設定する関数
 *
 * @param label 設定したい軸ラベル
 */
void Hist::set_y_label(std::string label){
	this->pipe.util_set_y_label(label);
}

/**
 * @brief x軸の値域を設定する関数
 *
 * @param min 値域の下限
 * @param max 値域の上限
 */
void Hist::set_x_range(double min, double max){
	this->pipe.util_set_x_range(min, max);
}

/**
 * @brief y軸の値域を設定する関数
 *
 * @param min 値域の下限
 * @param max 値域の上限
 */
void Hist::set_y_range(double min, double max){
	this->pipe.util_set_y_range(min, max);
}

/**
 * @brief ウィンドウの大きさを設定する関数
 *
 * @param width 横幅
 * @param height 縦幅
 */
void Hist::set_window_size(unsigned int width, unsigned int height){
	this->pipe.util_set_window_size(width, height);
}

/**
 * @brief x,yの値域を描画データの下限と上限に設定する
 *
 * @param should_autoscale true -> use auto scale, false -> unuse auto scale
 */
void Hist::set_autoscale(bool should_autoscale){
		this->pipe.util_set_autoscale(should_autoscale);
}

} // namespace Gpop
