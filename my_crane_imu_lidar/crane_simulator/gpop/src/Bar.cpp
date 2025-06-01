#include "../include/Gpop/Bar.hpp"

namespace Gpop {
	

/**
 * @brief コンストラクタ 1 of 3
 */
Bar::Bar() : pipe()
{
	this->set_box_width();
	this->rotate_label();
	//棒グラフを色で埋めることをgnuplotに伝える
	this->pipe.write_command("set style fill solid");
	//棒グラフに枠を付ける
	this->pipe.write_command("set style fill solid border lc rgb \"black\"");
}


/**
 * @brief コンストラクタ 2 of 3
 *
 * @param title プロットのタイトル
 */
Bar::Bar(std::string title) : pipe()
{
	this->set_title(title);
	this->set_box_width();
	this->rotate_label();
	//棒グラフを色で埋めることをgnuplotに伝える
	this->pipe.write_command("set style fill solid");
	//棒グラフに枠を付ける
	this->pipe.write_command("set style fill solid border lc rgb \"black\"");
}

void Bar::set_box_width(double relative){

	std::string com = "set boxwidth " + std::to_string(relative) + " relative";
	this->pipe.write_command(com);
}

void Bar::rotate_label(int angle){
	std::string com = "set xtics rotate by " + std::to_string(angle);
	this->pipe.write_command(com);
}

/**
 * @brief コンストラクタ 3 of 3
 *
 * @param title プロットのタイトル
 * @param width ウィンドウの横幅
 * @param height ウィンドウの縦幅
 */
Bar::Bar(std::string title, unsigned int width, unsigned int height) : pipe()
{
	this->set_title(title);
	this->set_box_width();
	this->rotate_label();
	this->set_window_size(width, height);
}


/**
 * @brief デストラクタ
 */
Bar::~Bar()
{
	this->pipe.write_command("quit");
}

void Bar::plot(double data, const std::string label){

	Data data_element;
	data_element.data = data;
	data_element.label = label;

	this->data_container.push_back(data_element);
}

/**
 * @brief windowのサイズを変更する関数.
 * コンストラクタでもウィンドウのサイズは設定可能
 * デフォルトのサイズは640x480
 *
 * @param width ウィンドウの横幅
 * @param height ウィンドウの高さ
 */
void Bar::set_window_size(unsigned int width, unsigned int height)
{
	this->window_height = height;
	this->window_width  = width;

	this->pipe.util_set_window_size(width, height);
}



/**
 * @brief gnuplotに送るようのコマンドを作成する関数
 *
 * @return コマンド
 */
std::string Bar::make_command(){


	//data_containerのデータの種類だけ<"-" w lp,>を作成する．
	//for (int i = 0; i < (int)this->data_container.size(); i++) {
	//	command += "'-' using 0:2:xtic(1) with boxes lw 1 notitle,";
	//}
	
	std::string command = "plot '-' using 0:2:xtic(1) with boxes lw 1 notitle,";
	this->pipe.write_command(command);
	
	return command;
}


/**
 * @brief プロットのタイトルを設定する関数
 *
 * @param title グラフのタイトル
 */
void Bar::set_title(std::string title)
{
	std::string com = "\"" + title + "\"";
	this->pipe.write_command("set title " + com);
}


/**
 * @brief プロットを描画する
 * 
 * 全てのデータを一気に表示するときに使用する 
 */
void Bar::show()
{
	//これからインラインモードでデータを入力することをgnuplotに伝える．
	this->pipe.write_command(this->make_command());

	//pipeでgnuplotに描画する．
	for (auto&& elem : this->data_container){
		//座標を基にコマンドを作成
		std::string command = elem.label + "\t" + std::to_string(elem.data); 

		//コマンドをgnuplotに送る
		this->pipe.write_command(command);
	}
	//ひとつの行が終わったことをeを送ることで伝える．
	this->pipe.write_command("e");

	//強制的にpipeの内容データを書き出し
	this->pipe.flush();

	//data_containerの中身を次の描画に備えて消す
	this->data_container.clear();
}

/**
 * @brief リアルタイムで追加されたデータのグラフを描画する
 *
 * @param usec ポーズする時間の指定，０以上の値を設定する．
 * ０を指定すると最速で描画する．
 */
void Bar::pause(int usec){
	this->show();

	if (usec != 0) {
		usleep(usec);
	}
}


/**
 * @brief x軸の範囲を設定する
 *
 * @param min x軸の最小値
 * @param max x軸の最大値
 */
void Bar::set_x_range(double min, double max)
{
	this->pipe.write_command("set xrange["
			                 + std::to_string(min)
							 + ":"
							 + std::to_string(max)
							 + "]");
}


/**
 * @brief y軸の範囲を設定する
 *
 * @param min y軸の最小値
 * @param max y軸の最小値
 */
void Bar::set_y_range(double min, double max)
{
	this->pipe.write_command("set yrange["
			                 + std::to_string(min)
							 + ":"
							 + std::to_string(max)
							 + "]");
}


/**
 * @brief x軸の名称を設定する
 *
 * @param label x軸の名称
 */
void Bar::set_x_label(std::string label)
{
	std::string com = "\"" + label + "\"";

	this->pipe.write_command("set xl " + com);
}


/**
 * @brief y軸の名称を設定する
 *
 * @param label y軸の名称
 */
void Bar::set_y_label(std::string label)
{
	std::string com = "\"" + label + "\"";

	this->pipe.write_command("set yl " + com);
}


void Bar::save_as_png(std::string title)
{
	title += ".png";
	this->pipe.write_command("set terminal pngcairo");
	std::string com = "set output \'" + title + '\'';
	this->pipe.write_command(com);
	this->pipe.write_command("replot");
	this->pipe.write_command("set terminal x11");
	this->pipe.write_command("set output");
}


/**
 * @brief autoscaleを設定する関数
 *	ただし，棒グラフの場合は自動でgnuplot内で自動的に設定されるので
 *	手動で設定する必要なし．そのため，将来的に必要になった時の
 *	ためにprivate関数にしておく．
 *
 * @param should_autoscale 
 * 自動でスケール調整を行うか否かを設定する．
 */
void Bar::set_autoscale(bool should_autoscale){
	this->pipe.util_set_autoscale(should_autoscale);
}

} // namespace Gpop
