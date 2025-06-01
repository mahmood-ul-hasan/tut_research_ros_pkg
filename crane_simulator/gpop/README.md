This repository was cloned from [GitHub](https://github.com/harumo11/Gpop).

---

# Gpop (Gnuplot on pipe)

簡単にデータをC++11プロットするライブラリです．
できるだけ簡単にプロットすることに注力しています．

現在，使用可能なプロットの種類は以下のとおりです．

- Series (折れ線グラフ)
- Pie (円グラフ)
- Vector (矢印で構成されるグラフ)
- Histogram (ヒストグラム)
- Bar (棒グラフ)

また，プロットする機能以外にPNGファイルとして保存したり，逐次データを更新してリアルタイムプロットを行うこともできます．下の表で表記がNoとなっていてもPNGの保存はGUIから行うことが可能であり，またEPSもGUIから保存可能です．あくまで，プログラム上から保存の命令が出せないことを意味します．

|Name|Save as PNG|Real time plot|
|:----:|:-----------:|:--------------:|
|Serise|OK       |OK            |
|Pie|  OK        |OK            |
|Vector|NO       |NO            |
|Histogram|  OK  |OK      |
|Bar| OK        |OK             |

## API 

APIに関するドキュメントは[ここ](
https://harumo11.github.io/Gpop/
)にあります．

## Dependency

- gnuplot
- Linux (Tested on Ubuntu18.04)
- C++11

## Instalation

Install gnuplot
```sh
sudo apt-get install gnuplot gnuplot-qt
```

Build gpop
```sh
cd gpop
mkdir build
cd build
cmake ../
make -j
sudo make install
```

## Sample
わずか数行でプロットすることが可能なことが以下のプログラムを見るとわかるでしょう．

```cpp
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "Gpop/Series.hpp"

int main(void){
	//描画データ作成
	std::vector<double> v;
	for(int i = 0; i < 180; i++){
		v.push_back(std::cos(i*M_PI/180));
	}

	//描画
	Gpop::Series plot;
	plot.plot(v);
	plot.show();

	//終了処理．エンターを押すと，プロットウィンドウが閉じます
	std::cout << "Press Enter Key" << std::endl;
	std::string end;
	std::cin >> end;
	return 0;
}
```

他にもサンプルがあります．

|Name|Discription|Used class name|
|:----:|:---------:|:-------------:|
|test0     |cos関数をプロット|Series|
|test1     |pipeを使ってsin関数をプロット|Gnuplot|
|test2     |複数のデータのプロット＆ラベルとタイトル設定|Series|
|test3     |リアルタイムプロット（描画範囲変更）|Series|
|test4     |複数のデータのリアルタイムプロット（描画範囲固定）|Series|
|test5     |タイトル＆ラベル設定|Series|
|test6     |円グラフ|Pie|
|test7     |複数ウィンドウのリアルタイムプロット|Series|
|test8     |ウィンドウサイズの変更|Series|
|test9     |ベクター図の作成|Vector|
|test10    |リアルタイム円グラフ|Pie|
|test11    |正規分布の棒グラフ|Hist|
|test12    |ランダムな値のヒストグラム|Bar|
|test13    |合計が100となる入力の円グラフのプロット|Pie|
|test14    |test13の内容をpngとして保存するもの|Pie|
|test15    |棒グラフのリアルタイムプロット|Bar|
|test16    |棒グラフの設定|Bar|
|test17    |描画設定込みでのプロット1|Series|
|test18    |描画設定込みでのプロット2|Series|
|test19    |凡例の書き方|Series|
|test20    | フォントの大きさをなどのラベルと凡例の設定|
|test21	   |色を設定しなかったときの自動着色の例	|
|test22		|プロットの点の設定方法|
|test23		|ヒストグラムの描画方法|
|test24		|2つのヒストグラムのプロット|

下記のコマンドで，全てのサンプルをビルドできます．

```sh
cd Gpop
cd test
mkdir build
cd build
cmake ..
make
```

