source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --merge-devel # this is important, otherwise you may get weird linking errors
cd src
git clone https://github.com/ethz-asl/rovio.git
git clone https://github.com/ethz-asl/kindr.git #needed dep
cd rovio
git submodule update --init --recursive
catkin build rovio



# Method 02
modify CMakeListtxt of rovio from ${GLUT_LIBRARY} ${GLEW_LIBRARY} to ${GLUT_LIBRARIES} ${GLEW_LIBRARIES}
cd catkin_ws/src
catkin_init_workspace
cd src/vins/rovio
git submodule update --init --recursive
cd catkin_ws
catkin_make rovio --cmake-args -DCMAKE_BUILD_TYPE=release
catkin_make rovio --cmake-args -DCMAKE_BUILD_TYPE=release -DMAKE_SCENE=ON
catkin_make


# eRROR
if you get:
what(): Error opening file: /some/path/rovio/my.bag
do:
mkdir /some/path/rovio