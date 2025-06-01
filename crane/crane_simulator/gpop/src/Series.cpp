#include "../include/Gpop/Series.hpp"

namespace Gpop {

/**
 * @brief コンストラクタ 1 of 2
 */
Series::Series() : pipe()
{
	this->pipe.util_set_grid(true);
}


/**
 * @brief コンストラクタ 2 of 2
 *
 * @param title プロットのタイトル
 */
Series::Series(std::string title) : pipe()
{
	this->pipe.util_set_grid(true);
	this->set_title(title);
}


Series::Series(std::string title, unsigned int width, unsigned int height) : pipe()
{
	this->pipe.util_set_grid(true);
	this->set_title(title);
	this->set_window_size(width, height);
}


/**
 * @brief デストラクタ
 */
Series::~Series()
{
	this->pipe.write_command("quit");
}

/**
 * @brief windowのサイズを変更する関数.
 * コンストラクタでもウィンドウのサイズは設定可能
 * デフォルトのサイズは640x480
 *
 * @param width ウィンドウの横幅
 * @param height ウィンドウの高さ
 */
void Series::set_window_size(unsigned int width, unsigned int height)
{
	this->window_height = height;
	this->window_width  = width;

	std::string com = "set terminal qt size ";
	com += std::to_string(this->window_width);
	com += ", ";
	com += std::to_string(this->window_height);

	this->pipe.write_command(com);
}


/**
 * @brief プロットするベクターデータを加える 1 of 2
 *
 * 一度に全部のデータを入力する場合は，この関数を使用して一気に追加する
 * x座標のデータを使用しないため，x座標は連続した整数が用いられる
 *
 * @param data プロットしたいデータ
 *
 * \note 
 * リアルタイムプロットを行いたい場合はこの関数は使用できません．
 */
void Series::plot(std::vector<double> &data)
{
	std::list<XY> coordinate_list;
	XY coorinate;

	//x軸のデータが与えられないので，
	//0からdata.size()個だけx軸のデータを作成する．
	//同時に与えられたy軸のデータを格納する
	for (int i = 0; i < (int)data.size() ; i++) {

		//座標を作成
		coorinate.x = i;
		coorinate.y = data[i];

		//作成した座標をベクターに格納
		coordinate_list.push_back(coorinate);
	}

	//作成したxy座標のベクターを保存
	this->data_container.push_back(coordinate_list);

	//this->propertyを適当に設定
	this->data_property_buffer.push_back("with lines");
}

/**
 * @brief 線の種類込みで，プロットしたいベクターデータを加える
 *
 * @param x_data プロットしたいxデータ
 * @param y_data プロットしたいyデータ
 *
 * (例) with lp lt 7 lw 2
 *
 * \note 
 * リアルタイムプロットを行いたい場合はこの関数は使用できません．
 */
void Series::plot(std::vector<double> &x_data, std::vector<double> &y_data){
	std::list<XY> coordinate_list;
	XY coorinate;

	//x_dataとy_dataのサイズが等しくなければエラーを起こす
	if (x_data.size() != y_data.size()) {
		std::cout << "[error] in Series::plot. x_data's size and y_data' size is not same" << std::endl;
		std::exit(1);
	}

	//data_containerに格納するためのデータを作成する
	for (int i = 0; i < (int)x_data.size() ; i++) {

		//座標を作成
		coorinate.x = x_data[i];
		coorinate.y = y_data[i];

		//作成した座標をベクターに格納
		coordinate_list.push_back(coorinate);
	}

	//作成したxy座標のベクターを保存
	this->data_container.push_back(coordinate_list);

	//this->propertyを適当に設定
	this->data_property_buffer.push_back("with lines");
}

/**
 * @brief 線の種類込みで，プロットしたいベクターデータを加える
 *
 * @param x_data プロットしたいxデータ
 * @param y_data プロットしたいyデータ
 * @param property 線の種類などの設定．設定方法はgnuplotに従う
 *
 * (例) with lp lt 7 lw 2
 *
 * \note 
 * リアルタイムプロットを行いたい場合はこの関数は使用できません．
 */
void Series::plot(std::vector<double> &x_data, std::vector<double> &y_data, const std::string property){
	std::list<XY> coordinate_list;
	XY coorinate;

	//x_dataとy_dataのサイズが等しくなければエラーを起こす
	if (x_data.size() != y_data.size()) {
		std::cout << "[error] in Series::plot. x_data's size and y_data' size is not same" << std::endl;
		std::exit(1);
	}

	//data_containerに格納するためのデータを作成する
	for (int i = 0; i < (int)x_data.size() ; i++) {

		//座標を作成
		coorinate.x = x_data[i];
		coorinate.y = y_data[i];

		//作成した座標をベクターに格納
		coordinate_list.push_back(coorinate);
	}

	//作成したxy座標のベクターを保存
	this->data_container.push_back(coordinate_list);

	//this->propertyを適当に設定
	this->data_property_buffer.push_back(property);
}

/**
 * @brief 線の種類込みで，プロットしたいベクターデータを加える
 *
 * @param data プロットしたいデータ
 * @param property 線の種類などの設定．設定方法はgnuplotに従う
 *
 * (例) with lp lt 7 lw 2
 *
 * \note 
 * リアルタイムプロットを行いたい場合はこの関数は使用できません．
 */
void Series::plot(std::vector<double> &data, const std::string property)
{
	std::list<XY> coordinate_list;
	XY coorinate;

	//x軸のデータが与えられないので，
	//0からdata.size()個だけx軸のデータを作成する．
	//同時に与えられたy軸のデータを格納する
	for (int i = 0; i < (int)data.size() ; i++) {

		//座標を作成
		coorinate.x = i;
		coorinate.y = data[i];

		//作成した座標をベクターに格納
		coordinate_list.push_back(coorinate);
	}

	//作成したxy座標のベクターを保存
	this->data_container.push_back(coordinate_list);

	//this->propertyを適当に設定
	this->data_property_buffer.push_back(property);
}

/**
 * @brief プロットしたいデータを加える 2 of 2
 *
 * プロットとのためのデータをこの関数を使用して1つずつ追加する
 *
 * @param data 追加するデータ
 */
void Series::plot(double data){

	//data_bufferに新しいdataを追加する
	this->data_buffer.push_back(data);

	//this->propertyを適当に設定
	this->data_property_buffer.push_back("with lines");
}


/**
 * @brief 線の種類込みでプロットしたいデータを加える
 *
 * プロットとのためのデータをこの関数を使用して1つずつ追加する
 * リアルタイムプロットの場合，最初だけこの関数を使用して，線の種類を指定すれば
 * 2回目以降は線の種類などを指定しないplot関数を使用した場合でも，1回目の設定を再利用します．
 *
 * @param data 追加するデータ
 * @param property 線の種類などの設定．設定方法はgnuplotに従う
 *
 * (例) with lp lt 7 lw 2
 */
void Series::plot(double data, const std::string property){

	//data_bufferに新しいdataを追加する
	this->data_buffer.push_back(data);

	//this->propertyを適当に設定
	this->data_property.push_back(property);
}

/**
 * @brief 線の種類込みでプロットしたいデータを加える
 *
 * プロットとのためのデータをこの関数を使用して1つずつ追加する
 *
 * @param data 追加するxデータ
 * @param data 追加するyデータ
 *
 */
void Series::plot(double x_data, double y_data){

	//最初の一回だけ，segmentation faultを防ぐために空データを
	//data_containerに追加する．
	if (this->data_container.empty()) {
		this->data_container.push_back({});
	}

	//data_containerに追加用のデータ作成
	XY coordinate;
	coordinate.x = x_data;
	coordinate.y = y_data;

	//data_container_index行の一番後ろに作成したデータ追加
	this->data_container[this->data_container_index].push_back(coordinate);
	this->data_container_index++;

	//this->propertyを適当に設定
	this->data_property_buffer.push_back("with lines");
}

/**
 * @brief 線の種類込みで，プロットしたいベクターデータを加える
 *
 * @param x_data プロットしたいxデータ
 * @param y_data プロットしたいyデータ
 * @param property 線の種類などの設定．設定方法はgnuplotに従う
 *
 * (例) with lp lt 7 lw 2
 */
void Series::plot(double x_data, double y_data, const std::string property){

	//最初の一回だけ，segmentation faultを防ぐために空データを
	//data_containerに追加する．
	if (this->data_container.empty()) {
		this->data_container.push_back({});
	}

	//data_containerに追加用のデータ作成
	XY coordinate;
	coordinate.x = x_data;
	coordinate.y = y_data;

	//data_container_index行の一番後ろに作成したデータ追加
	this->data_container[this->data_container_index].push_back(coordinate);
	this->data_container_index++;

	//this->propertyを適当に設定
	this->data_property_buffer.push_back(property);
}

/**
 * @brief gnuplotに送るようのコマンドを作成する関数
 *
 * @return コマンド
 */
std::string Series::make_command(){

	std::string command = "plot ";

	//data_containerのデータの種類だけ<"-" w lp lt 7 lw 1.5,>を作成する．
	for (int i = 0; i < (int)this->data_container.size(); i++) {
		//command += "'-' w p pt 7 lw 1.5,";
		command += "'-'" + this->data_property[i] + ',';
	}
	//std::cout << "com " << command << std::endl;
	
	return command;
}


/**
 * @brief plotのタイトルを設定する関数
 *
 * @param title グラフのタイトル
 */
void Series::set_title(std::string title)
{
	std::string com = "\"" + title + "\"";
	this->pipe.write_command("set title " + com);
}

void Series::set_title(std::string title, unsigned font_size){
	this->pipe.util_set_title(title, font_size);
}


/**
 * @brief プロットを描画する
 * 
 * 全てのデータを一気に表示するときに使用する 
 */
void Series::show()
{
	//data_container_indexを0に初期化する
	this->data_container_index = 0;

	//最初の一回だけdata_propertyにdata_property_bufferの中身をそのまま移す
	if (this->data_property.empty()) {
		this->data_property = this->data_property_buffer;
		this->data_property_buffer.clear();
	}
	else {
		//2回目以降は追加された分をdata_propertyに上書きする
		for (int i = 0; i < (int)this->data_property_buffer.size(); i++) {
			this->data_property[i] = this->data_property_buffer[i];
		}
		this->data_property_buffer.clear();
	}

	//これからインラインモードでデータを入力することをgnuplotに伝える．
	this->pipe.write_command(this->make_command());

	//pipeでgnuplotに描画する．
	for (auto&& vec : this->data_container){
		for (auto&& elem : vec){

			//座標を基にコマンドを作成
			std::string command = std::to_string(elem.x) + "\t" + std::to_string(elem.y); 

			//コマンドをgnuplotに送る
			this->pipe.write_command(command);
		}

		//ひとつの行が終わったことをeを送ることで伝える．
		this->pipe.write_command("e");
	}
	//強制的にpipeの内容データを書き出し
	this->pipe.flush();
}


/**
 * @brief x軸の範囲を設定する
 *
 * @param min x軸の最小値
 * @param max x軸の最大値
 *
 * \note
 * この関数を使用するとデフォルトで有効なautoscaleはすべて解除されます．
 */
void Series::set_x_range(double min, double max)
{
	this->is_autoscale = false;
	this->pipe.write_command("set xrange["
			                 + std::to_string(min)
							 + ":"
							 + std::to_string(max)
							 + "]");
}

/**
 *  @brief autoscaleを使用する
 *
 *  @param should_autoscale  true -> set, false -> unset.
 *
 */
void Series::set_autoscale(bool should_autoscale){
	if (should_autoscale == true) {
		this->pipe.write_command("set autoscale");
		this->is_autoscale = true;
	}
	else {
		this->pipe.write_command("unset autoscale");
		this->is_autoscale = false;
	}
}


/**
 * @brief y軸の範囲を設定する
 *
 * @param min y軸の最小値
 * @param max y軸の最小値
 *
 * \note
 * この関数を使用するとデフォルトで有効なautoscaleはすべて解除されます．
 */
void Series::set_y_range(double min, double max)
{
	this->is_autoscale = false;
	this->pipe.write_command("set yrange["
			                 + std::to_string(min)
							 + ":"
							 + std::to_string(max)
							 + "]");
}


void Series::resize_data_container()
{
	for(auto&& vec : this->data_container){
		if ((int)vec.size() > this->max_number) {
			while ((int)vec.size() != this->max_number) {
				vec.pop_front();
			}
		}
	}
}


/**
 * @brief リアルタイムで追加されたデータのプロットを描画する
 *
 * @param msec ポーズする時間の指定，0以上の値を設定する．
 * 0を指定すると最速で描画する
 *
 */
void Series::pause(int usec){

	//data_container_indexを0に初期化する
	this->data_container_index = 0;

	//もし最大表示個数が設定されていたら，その分だけ，data_containerを整理する．
	if (this->max_number != 0) {
		this->resize_data_container();
	}

	//最初の一回だけx=0,y=data_bufferの座標をdata_containerに代入する
	if (this->data_container.empty()) {
		//for (int i = 0; i < (int)this->data_buffer.size(); i++) {
		for(auto&& elem_buffer : this->data_buffer){
			//代入する座標作成
			XY coorinate;
			coorinate.x = 0;
			//coorinate.y = this->data_buffer[i];
			coorinate.y = elem_buffer;

			//data_containerに代入
			this->data_container.push_back({coorinate});
		}
	}

	//最初の一回だけdata_propertyにdata_property_bufferの中身をそのまま移す
	if (this->data_property.empty()) {
		this->data_property = this->data_property_buffer;
		this->data_property_buffer.clear();
	}
	else {
		//2回目以降は追加された分をdata_propertyに上書きする
		for (int i = 0; i < (int)this->data_property_buffer.size(); i++) {
			this->data_property[i] = this->data_property_buffer[i];
		}
		this->data_property_buffer.clear();
	}

	//data_containerにdata_bufferを追加する
	int i = 0;
	for(auto&& elem_buffer : this->data_buffer){

		//追加する座標を作成する
		XY coorinate;
		coorinate.y = elem_buffer;
		coorinate.x = this->data_container[i].back().x + 1;

		//data_containerに作成した座標を追加する
		this->data_container[i].push_back(coorinate);

		//iteretorを更新
		i++;
	}


	//gnuplotにdata_containerのデータを転送する
	this->pipe.write_command(this->make_command());

	//pipeでgnuplotに描画する．
	for (auto&& vec : this->data_container){
		for (auto&& elem : vec){

			//座標を基にコマンドを作成
			std::string command = std::to_string(elem.x) + "\t" + std::to_string(elem.y); 

			//コマンドをgnuplotに送る
			this->pipe.write_command(command);
		}

		//ひとつの行が終わったことをeを送ることで伝える．
		this->pipe.write_command("e");
	}

	//data_bufferを空にする
	this->data_buffer.clear();

	//もし，set_autoscale()が使用されていたらautoscaleを設定
	if (this->is_autoscale) {
		this->set_autoscale();
	}

	//pipeを強制的に書き出す．
	this->pipe.flush();

	//sleep
	usleep(usec);
}

/**
 * @brief 表示する最大のデータ数を設定する．
 *
 * @param number 表示する最大のデータ数
 *
 *  デフォルトではnumber = 10000となっている．
 *  リアルタム描画の時に最新の１０個のデータだけを表示したい場合
 *  limit_max_number(10)と設定することにより最新の１０個のデータのみを
 *  表示することができる．
 */
void Series::limit_max_number(int number){

	this->max_number = number;
}


/**
 * @brief x軸の名称を設定する
 *
 * @param label x軸の名称
 */
void Series::set_x_label(std::string label)
{
	std::string com = "\"" + label + "\"";

	this->pipe.write_command("set xl " + com);
}

void Series::set_x_label(std::string label, unsigned int font_size){
	this->pipe.util_set_x_label(label, font_size);
}


/**
 * @brief y軸の名称を設定する
 *
 * @param label y軸の名称
 */
void Series::set_y_label(std::string label)
{
	std::string com = "\"" + label + "\"";

	this->pipe.write_command("set yl " + com);
}

void Series::set_y_label(std::string label, unsigned int font_size){
	this->pipe.util_set_y_label(label, font_size);
}

/**
 *  @brief pngとしてグラフを保存する．
 *
 *  @param title 保存する画像の名前
 *  
 *  \pre この関数を呼び出す前にSeries::show()を呼び出してグラフを描画してください.
 */
void Series::save_as_png(std::string title)
{
	title += ".png";
	this->pipe.write_command("set terminal pngcairo");
	std::string com = "set output \'" + title + '\'';
	this->pipe.write_command(com);
	this->pipe.write_command("replot");
	this->pipe.write_command("set terminal x11");
	this->pipe.write_command("set output");
}

} // namespace Gpop
