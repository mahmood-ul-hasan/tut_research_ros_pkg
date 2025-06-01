# pointcloud_integrator_

This package integrates input point clouds based on thier relationship described as a graph.
The entire point clouds are also rendered using OpenGL.

To build this package, you first have to install the following ROS package.
```bash
$ sudo apt install libsuitesparse-dev libqglviewer-dev-qt4 ros-kinetic-libg2o
$ sudo ln -s /usr/lib/x86_64-linux-gnu/libQGLViewer-qt4.so /usr/lib/x86_64-linux-gnu/libQGLViewer.so
```

This package is originally from lsd_slam_viewer.
Please visit the following URL for more information.
[lsd_slam on GitHub_](https://github.com/tum-vision/lsd_slam)

