## Task
Develop a control program using ROS that connects to the TurtleBot3 Gazebo simulation environment and navigates the TurtleBot3 continuously in the standard world without colliding with obstacles using randomly generated goals.

## Approach
### Random Goal Navigation
- The TurtleBot3 navigates to randomly generated pose goals.
- After reaching a goal, a new random goal is generated, and the TurtleBot3 starts moving towards it.
- This process repeats continuously, ensuring the robot keeps moving around the environment without stopping.

## Prerequisites
- **Operating System**: Ubuntu 20.04 (recommended)
- **ROS Distribution**: ROS Noetic
- **TurtleBot3 Packages**: Ensure the following packages are installed:
  - `turtlebot3_navigation`
  - `turtlebot3_simulations`
  - `turtlebot3_slam`
  - `turtlebot3_teleop`

## Installation Instructions
1. **Install Required Packages**:
   - Clone the TurtleBot3 packages from the official GitHub repository:
     ```bash
     git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
     ```
   - Or use the following commands to install the required packages via APT:
     ```bash
     sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
     sudo apt-get install ros-noetic-turtlebot3-gazebo
     ```

2. **Copy and Build the Project**:
   - Copy the `keep_turtlebot3_moving` folder into your catkin workspace `src` directory:
     ```bash
     cd ~/catkin_ws/src/
     cp -r /path/to/keep_turtlebot3_moving .
     ```
   - Build the catkin workspace:
     ```bash
     cd ~/catkin_ws/
     catkin_make
     source devel/setup.bash
     ```

## How To Run
1. **Set TurtleBot3 Model**:
   - To avoid model errors during the launch of TurtleBot3 packages, set the TurtleBot3 model to `burger`:
     ```bash
     echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
     source ~/.bashrc
     ```

2. **Launch TurtleBot3 in Gazebo and RViz**:
   - Start the TurtleBot3 simulation in Gazebo along with RViz for visualization:
     ```bash
     roslaunch keep_turtlebot3_moving turtlebot3_navigation_gazebo_rviz.launch
     ```

3. **Set Initial Pose in RViz**:
   - In RViz, press the `2D Pose Estimate` button.
   - Click on the approximate location of TurtleBot3 as visible in the Gazebo view and drag to set the initial orientation before releasing the mouse.

4. **Start TurtleBot3 Movement**:
   - To move TurtleBot3 using randomly generated goals, run:
     ```bash
     roslaunch keep_turtlebot3_moving move_bot_by_random_goal.launch
     ```

## Additional Notes
- Ensure that you have correctly set the initial pose of the TurtleBot3 in RViz for proper localization and navigation.
- You can adjust the parameters for the navigation stack in the configuration files located in the `keep_turtlebot3_moving` package to optimize the robot's behavior for different environments.




----

----

----

# TurtleBot3 Random Goal Navigation Tutorial

###  Task
Develop a control program using ROS that connects to the TurtleBot3 Gazebo simulation environment and navigates the TurtleBot3 continuously in the standard world without colliding with obstacles using randomly generated goals.

### Introduction
This tutorial demonstrates how to use ROS to control a TurtleBot3 robot in a simulated Gazebo environment. The objective is to implement a program that autonomously navigates the robot by generating random goals within the simulation boundaries. The robot will avoid obstacles and continuously explore the environment by following these goals.

# Approach

#### Random Goal Navigation
In this approach, the TurtleBot3 robot navigates to randomly generated goals within the Gazebo environment. The process involves the following steps:

1. **Generate Random Goals**:
   - A random `(x, y)` position is generated within predefined boundaries of the simulated environment.
   - The `z`-coordinate is kept at 0, as the movement is confined to a 2D plane.
   - Each goal has a fixed orientation, focusing on position-based navigation without rotation.

2. **Send Goals to `move_base`**:
   - The `move_base` action server is used to send the randomly generated goals to the robot.
   - The robot leverages the ROS navigation stack, which includes path planning, localization, and obstacle avoidance, to navigate to the goal.

3. **Continuous Movement**:
   - Once the robot reaches the current goal or fails due to an obstacle or other issue, a new random goal is generated.
   - This cycle repeats continuously, ensuring the TurtleBot3 keeps moving and exploring the environment.

4. **Obstacle Avoidance**:
   - TurtleBot3 uses its built-in sensors (e.g., LiDAR) and the navigation stack to detect and avoid obstacles.
   - The navigation stack dynamically recalculates the path if an obstacle is detected, ensuring safe navigation to the goal.
---

# Implementation Details

The implementation can be broken down into key components and functions:

#### 1. Initialization
- The ROS node is initialized, and a `move_base` action client is set up to communicate with the navigation stack.
- A random goal is generated using the `random` library, ensuring that the goals are within the allowed boundaries.

#### 2. Goal Generation
- A random `(x, y)` coordinate is generated within the specified limits of the environment.
- The coordinates are adjusted to avoid setting goals too close to obstacles or in restricted areas.
- A `MoveBaseGoal` message is constructed with the generated position and a fixed orientation.

#### 3. Goal Sending and Feedback Handling
- The generated goal is sent to the `move_base` action server.
- The action client waits for the robot to reach the goal, with a timeout set to handle cases where the goal is unreachable.
- If the robot successfully reaches the goal, a new goal is generated.
- If the goal is not reached (due to an obstacle or timeout), the current goal is canceled, and a new one is generated.

#### 4. Continuous Execution
- The process of generating, sending, and monitoring goals is repeated in a loop, creating continuous movement and exploration of the environment.
- This loop ensures that the robot remains active and navigates continuously in the simulated world.
---


# Code Structure

The main components of the implementation are as follows:

#### 1. Class Definition (`keep_MoveBase_moving`)
- This class handles the initialization of the ROS node and the `move_base` action client.
- It includes methods for generating and sending random goals (`go_to_target()`).

#### 2. Random Goal Generation
- Random `(x, y)` coordinates are generated within a safe range.
- The coordinates are adjusted to prevent setting goals too close to the robot or within restricted areas.

#### 3. Sending Goals to `move_base`
- The `send_goal()` method constructs and sends the goal to the `move_base` action server.
- The method waits for the robot to reach the goal or cancels the goal if the robot fails to reach it within the timeout.

#### 4. Continuous Execution
- The main loop continuously generates and sends new goals, ensuring the robot remains active in the environment.
- This ensures that the robot is always exploring, adapting to the environment, and navigating towards new goals.

### Summary
This tutorial provides a structured approach to implementing random goal navigation for TurtleBot3 in the Gazebo simulation environment using ROS. By generating random goals and using the `move_base` action server, the robot can autonomously explore the environment, avoiding obstacles and continuously moving to new destinations.

Feel free to experiment with the code, adjust parameters, and expand the functionality to include more complex behaviors such as adaptive goal generation, mapping, or dynamic obstacle avoidance!




# 
# Translated using DeepL.com

# 課題
ROS を用いて TurtleBot3 Gazebo シミュレーション環境に接続し、ランダムに生成されるゴールを用いて障害物に衝突することなく TurtleBot3 を標準ワールド内で連続的にナビゲートする制御プログラムを開発する。

## アプローチ
### ランダムゴールナビゲーション
- TurtleBot3 はランダムに生成されたポーズゴールにナビゲートします。
- ゴールに到達すると、新しいランダムなゴールが生成され、TurtleBot3 はそれに向かって移動を開始します。
- このプロセスは連続的に繰り返され、ロボットは止まることなく環境内を移動し続けます。

## 前提条件
- オペレーティングシステム**： Ubuntu 20.04（推奨）
- ROSディストリビューション**： ROS Noetic
- **TurtleBot3 パッケージ**： 以下のパッケージがインストールされていることを確認してください：
  - turtlebot3_navigation` 以下のパッケージがインストールされていることを確認してください。
  - turtlebot3_simulations`（タートルボット3シミュレーション
  - `turtlebot3_slam`
  - turtlebot3_teleop` `turtlebot3_teleop`

### インストール手順
1. **必要なパッケージのインストール
   - 公式GitHubリポジトリからTurtleBot3パッケージをクローンする：
     ```
     git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
     ```
   - または、以下のコマンドを使用してAPT経由で必要なパッケージをインストールします：
     バッシュ
     ```
     sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
     sudo apt-get install ros-noetic-turtlebot3-gazebo
     ```

2. **プロジェクトのコピーとビルド**：
   - keep_turtlebot3_moving`フォルダをcatkinワークスペースの `src` ディレクトリにコピーします：

    ```bash
     cd ~/catkin_ws/src/
     cp -r /path/to/keep_turtlebot3_moving .
     ```
   - catkinワークスペースをビルドします：
     
    ```bash
     cd ~/catkin_ws/
     catkin_make
     source devel/setup.bash
    ```

## 実行方法
1. **TurtleBot3モデルの設定**：
   - TurtleBot3 パッケージの起動時のモデルエラーを回避するために、TurtleBot3 のモデルを `burger` に設定します：
     
     ```bash
     echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
     source ~/.bashrc
     ```

2. **Gazebo と RViz で TurtleBot3 を起動します：
   - Gazebo と RViz で TurtleBot3 を起動します：
     
     ```bash
     roslaunch keep_turtlebot3_moving turtlebot3_navigation_gazebo_rviz.launch
     ```

3. **RVizで初期ポーズを設定します：
   - RVizで`2D Pose Estimate`ボタンを押します。
   - ガゼボビューに表示されているTurtleBot3のおおよその位置をクリックし、マウスを離す前にドラッグして初期姿勢を設定します。

4. **TurtleBot3 の移動を開始します：
   - ランダムに生成されたゴールを使ってTurtleBot3を移動させるには、以下のコマンドを実行します：

     ```bash
     roslaunch keep_turtlebot3_moving move_bot_by_random_goal.launch
     ```

## その他の注意事項
- ローカライズとナビゲーションを正しく行うために、RVizでTurtleBot3の初期ポーズを正しく設定してください。
- keep_turtlebot3_moving`パッケージにある設定ファイルでナビゲーションスタックのパラメータを調整し、ロボットの動作を様々な環境に最適化することができます。

