#include <string>
#include <cmath>
#include "../include/Gpop/Pie.hpp"

namespace Gpop {
	
/**
 * @brief 円グラフのコンストラクタ 1 of 2
 */
Pie::Pie() : pipe() {

}

/**
 * @brief 円グラフのコンストラクタ 2 of 2
 *
 * @param title グラフのタイトル
 */
Pie::Pie(std::string title) : pipe() {

	std::string com = "\"" + title + "\"";
	this->pipe.write_command("set title " + com);

}

/**
 * @brief 円グラフのデストラクタ
 */
Pie::~Pie(){

	this->pipe.write_command("quit");
}

/**
 * @brief 描画するデータを追加する関数
 *
 * @param data 円グラフのデータ
 * @param name dataで入力したデータの要素の名前
 */
void Pie::plot(double data, std::string name){
	
	//data_containerに格納するようのItemを作成する
	Item item;
	item.data = data;
	item.name = name;

	//data_containerに格納する
	this->data_container.push_back(item);
}


/**
 * @brief 表示するx座標のデータの範囲を設定する
 *
 * @param min x座標の最小値
 * @param max x座標の最大値
 */
void Pie::set_x_range(double min, double max){

	this->pipe.write_command("set xrange["
							 + std::to_string(min)
							 + ":"
							 + std::to_string(max)
							 + "]");
}


/**
 * @brief 表示するy座標のデータの範囲を設定する
 *
 * @param min y座標の最小値
 * @param max y座標の最大値
 */
void Pie::set_y_range(double min, double max){

	this->pipe.write_command("set yrange["
							 + std::to_string(min)
							 + ":"
							 + std::to_string(max)
							 + "]");
}


/**
 * @brief 円グラフの描画を行う関数
 */
void Pie::show(){

	//設定を行う
	//アスペクト比を１：１に
	this->pipe.write_command("set size ratio -1");
	//x,yの範囲を設定
	this->set_x_range(-1, 1);
	this->set_y_range(-1, 1);
	//塗りつぶし，透過などの設定
	this->pipe.write_command("set style fill transparent solid 0.6 noborder");
	//判例を消す
	this->pipe.write_command("unset key");
	//軸を消す
	this->pipe.write_command("unset xtics");
	this->pipe.write_command("unset ytics");
	//枠を消す
	this->pipe.write_command("unset border");

	//色設定の変数を宣言する
	this->last_angle = 0;
	int color_number = 1;


	//入力されたdata_containerの中の値を100分率に直す
	this->normalize_data();

	//角度の単位ををdegreeに変更
	this->pipe.write_command("set angles degrees");
	//ラベルを描画
	for(int i = 0; i < (int)this->data_container.size(); i++){
		//％から角度に変更
		this->data_container[i].data = this->data_container[i].data * 360 / 100;
		//文字を表示する座標を作成
		double angle = this->last_angle + this->data_container[i].data / 2;
		angle = angle * M_PI / 180;
		double x_coord = 0.5 + 0.5/2 * std::cos(angle);
		double y_coord = 0.5 + 0.5/2 * std::sin(angle);
		this->last_angle += this->data_container[i].data;
		//コマンドを作成
		//フォント２０，Arialで描画
		std::string com = "set label " + std::to_string(i+1) + "center at graph ";
		com += std::to_string(x_coord) + ", " + std::to_string(y_coord)
	    		+ " \"{" + "/Arial:Normal=20 "  + this->data_container[i].name + "}\"";
			//TODO 改行して数値を表示するようにする
			//+ "\n" + std::to_string(this->data_container[i].data) + "\"";
		//コマンドを送信
		this->pipe.write_command(com);
	}

	//円グラフを描画
	this->last_angle = 0;
	//円グラフを書くことを通達
	this->pipe.write_command("plot '-'using 1:2:3:4:5:6 with circles lc var");
	for(auto&& elem : this->data_container){
		//コマンドを作成
		std::string com = "0 0 1 ";
		//コマンドに開始角度を追加
		com += std::to_string(this->last_angle) + " ";
		//コマンドに終了角度を追加
		this->last_angle += elem.data;
		com += std::to_string(this->last_angle) + " ";
		//コマンドに塗りつぶし用の色を設定
		com += std::to_string(color_number++);
		//コマンドを送信
		this->pipe.write_command(com);
	}

	//１つ分のデータの入力が終了したことを通達する
	this->pipe.write_command("e");
	this->pipe.flush();
}

void Pie::pause(int usec){

	//設定を行う
	//アスペクト比を１：１に
	this->pipe.write_command("set size ratio -1");
	//x,yの範囲を設定
	this->set_x_range(-1, 1);
	this->set_y_range(-1, 1);
	//塗りつぶし，透過などの設定
	this->pipe.write_command("set style fill transparent solid 0.6 noborder");
	//判例を消す
	this->pipe.write_command("unset key");
	//軸を消す
	this->pipe.write_command("unset xtics");
	this->pipe.write_command("unset ytics");
	//枠を消す
	this->pipe.write_command("unset border");

	//色設定の変数を宣言する
	this->last_angle = 0;
	int color_number = 1;


	//入力されたdata_containerの中の値を100分率に直す
	this->normalize_data();

	//角度の単位ををdegreeに変更
	this->pipe.write_command("set angles degrees");
	//ラベルを描画
	for(int i = 0; i < (int)this->data_container.size(); i++){
		//％から角度に変更
		this->data_container[i].data = this->data_container[i].data * 360 / 100;
		//文字を表示する座標を作成
		double angle = this->last_angle + this->data_container[i].data / 2;
		angle = angle * M_PI / 180;
		double x_coord = 0.5 + 0.5/2 * std::cos(angle);
		double y_coord = 0.5 + 0.5/2 * std::sin(angle);
		this->last_angle += this->data_container[i].data;
		//コマンドを作成
		//フォント２０，Arialで描画
		std::string com = "set label " + std::to_string(i+1) + "center at graph ";
		com += std::to_string(x_coord) + ", " + std::to_string(y_coord)
	    		+ " \"{" + "/Arial:Normal=20 "  + this->data_container[i].name + "}\"";
			//TODO 改行して数値を表示するようにする
			//+ "\n" + std::to_string(this->data_container[i].data) + "\"";
		//コマンドを送信
		this->pipe.write_command(com);
	}

	//円グラフを描画
	this->last_angle = 0;
	//円グラフを書くことを通達
	this->pipe.write_command("plot '-'using 1:2:3:4:5:6 with circles lc var");
	for(auto&& elem : this->data_container){
		//コマンドを作成
		std::string com = "0 0 1 ";
		//コマンドに開始角度を追加
		com += std::to_string(this->last_angle) + " ";
		//コマンドに終了角度を追加
		this->last_angle += elem.data;
		com += std::to_string(this->last_angle) + " ";
		//コマンドに塗りつぶし用の色を設定
		com += std::to_string(color_number++);
		//コマンドを送信
		this->pipe.write_command(com);
	}

	//１つ分のデータの入力が終了したことを通達する
	this->pipe.write_command("e");
	this->pipe.flush();

	//次に書くためにdata_containerの値をカラにする
	this->data_container.clear();

	//sleep
	usleep(usec);

}

/**
 * @brief 入力されたdata_containerの現在の合計値を返す
 *
 * @return data_containerの合計値
 */
double Pie::sum(){

	double sum = 0; 

	for(auto&& elem : this->data_container){
		sum += elem.data;
	}
	return sum;
}


/**
 * @brief 入力されたdata_containerを100分率に直す
 */
void Pie::normalize_data(){

	//合計値を求める
	double sum = this->sum();
	for(auto&& elem : this->data_container){
		elem.data = (elem.data / sum) * 100;
	}
}

/**
 * @brief ウィンドウのサイズを変更する関数
 *
 * @param width ウィンドウの横幅
 * @param height ウィンドウの高さ
 */
void Pie::set_window_size(unsigned width, unsigned height){
	
	std::string com = "set term qt size ";
	com += std::to_string(width);
	com += ", ";
	com += std::to_string(height);

	this->pipe.write_command(com);
}

/**
 * @brief プロットをpngファイルとして保存する
 *
 * @param title 保存するpngファイルの名前.拡張子は不要
 */
void Pie::save_as_png(std::string title){
	title += ".png";
	this->pipe.write_command("set terminal pngcairo");
	std::string com = "set output \'" + title + "\'";
	this->pipe.write_command(com);
	this->pipe.write_command("replot");
	this->pipe.write_command("set terminal x11");
	this->pipe.write_command("set output");
}

} // namespace Gpop
