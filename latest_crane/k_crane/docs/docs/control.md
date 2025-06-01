# Control

このパッケージは主にgazeboをプログラム内から操作するためのプログラムです．
また，各関節のPIDパラメータの設定ファイルを含んでいます．

```
├── ds4toangle.cpp : PS4コントローラからクレーンを動かすプログラム
├── go_initial_pose.cpp : 所望の初期位置へ移動するプログラム
├── initial_joint12.cpp : ロープの関節joint12を０度にするプログラム
├── initial_joint23.cpp : ロープの関節joint23を０度にするプログラム
├── initial_joint4.cpp : ロープの関節joint4を０度にするプログラム
├── initial_joint_boom.cpp : テスト用の初期位置移動プログラム
├── initial_joint_boom_pi.cpp : テスト用の初期位置移動プログラム
├── initial_state.cpp : テスト用の初期位置移動プログラム
├── integral.cpp : 速度軌道を積分して位置軌道に変換するプログラム
├── joint_trajectory_action.cpp : 予め与えられた軌道（速度，位置，時刻）を実行するプログラム
├── logger.cpp : ロギング用プログラム
├── move_trapezoid.cpp : テスト用，台形加速プログラム
├── pause_gazebo.cpp : gazeboを一時停止させるプログラム
├── sano_joint_trajectory_action.cpp : 佐野先生の制振軌道を送るプログラム
└── unpause_gazebo.cpp : gaeboを一時停止から復帰させるプログラム
```

クレーンを時々刻々移動させたいときは，`/k_crane/k_crane_joint_controller/command`トピックに
パブリッシュすると移動させることができます．具体的には`k_crane_control/src/ds4toangle.cpp`を見てください．
