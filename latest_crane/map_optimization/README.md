
## Instalation and Configuration
### -------------------------------------------------------------------

### 3D Mappinig depends on
1. G20 library
2. eign3 library


### Install G2o
You can either install using g2o binaries using command: sudo apt install ros-noetic-libg2o or build g2o from source.
1. install
    $ sudo apt-get install ros-noetic-libg2o
2. Copy file "FindG2O.cmake" into /usr/share/cmake-3.5/Modules/
    sudo cp FindG2O.cmake /usr/share/cmake-3.5/Modules/

### Install pcl lib
1. $ sudo apt-get install ros-noetic-pcl-ros
2. $ sudo apt install libpcl-dev
3. $ sudo apt install libeigen3-dev

### G2O Configuration
1. Download g20
     https://github.com/RainerKuemmerle/g2o

2. Compilation in g20 folder

        mkdir build
        cd build
        cmake ../
        make
        sudo make install
        checkinstall 

3. Copy file "FindG2O.cmake" into /usr/share/cmake-3.5/Modules/
    sudo cp FindG2O.cmake /usr/share/cmake-3.5/Modules/

4. link the project with with g2o by adding following line to g20
        LIST(APPEND CMAKE_MODULE_PATH /home/aisl/optimization/g2o/cmake_modules)
        set (G2O_ROOT  /home/aisl/optimization/g2o)

### -------------------------------------------------------------------
## How to run Plane_extraction package
These steps guide you through the process of utilizing the Plane Extraction Package to analyze point cloud data and extract relevant planes
### -------------------------------------------------------------------
#### Overview of Related Folders
1. Folder: "config/plane_extraction_parameters"
Each .yaml file within this folder contains customized parameters for processing PCD data to extract planes within the environment.

2. Folder: "input_pointcloud_data"
This folder holds the input point cloud data for various environments, with each environment's data stored in a separate subfolder. Extracted wall and ground planes, along with their indices, are saved within the same subfolders.

#### Step to run Plane_extraction packages
1. Begin by creating a new folder within the "input_pointcloud_data" directory. Place the input point cloud (.pcd) file of the specific environment within this newly created folder.

2. Adjust the parameters within the .yaml file located in the "config/plane_extraction_parameters" folder. Follow the provided template to fine-tune the extraction process.

3. Specify the directory of the parameter.yaml file within the launch file.

4. Launch the plane extraction process by executing the following command:
                        
        roslaunch map_optimization plane_extraction.launch





### -------------------------------------------------------------------
# TroubleShoot
## Delete the previously compiled and installed version
### -------------------------------------------------------------------

1. Delete /usr/local/include/g2o, the instruction is sudo rm -rf /usr/local/include/g2o
sudo make clean
2. Delete the library files related to libg2o_*.so under /usr/local/lib, first enter the directory cd /usr/local/lib, and then delete one by one (multiple at the same time) sudo rm -rf libg2o_*
3. If g2o version issue remove G2O older version. First find location

        sudo pkg-config --cflags eigen3
then remove

        sudo rm -r /usr/local/include/g2o
        sudo rm -r /usr/local/lib/libg2o*
        sudo rm -r /usr/local/bin/g2o*

        sudo rm -r /usr/local/include/pcl-1.8
        sudo rm -r /usr/local/share/pcl-1.8
        sudo rm -r /usr/local/lib/pcl-1.8*
        sudo rm -r /usr/local/bin/pcl-1.8*

4. Remove pcl 

        sudo apt-get purge libpcl*
        sudo apt-get remove ros-noetic-pcl-ros
        sudo apt remove libeigen3-dev
        sudo apt-get purge --auto-remove libeigen3-dev
        sudo apt-get purge --auto-remove ros-noetic-pcl-ros
        sudo apt-get purge --auto-remove libpcl*

5. Uninstalling the pcl-tools 

For uninstalling this package you can easily use the apt command and remove the package from Linux Operating System. Following command is used to remove the pcl-tools package along with its dependencies:

    sudo apt-get remove purge --auto-remove pcl-tools

Completely removing pcl-tools with all configuration files:
Following command should be used with care as it deletes all the configuration files and data:
        
        sudo apt-get purge pcl-tools
        or you can use following command also:
        sudo apt-get purge --auto-remove pcl-tools

6. Delete all files in current directory and its sub-directories where the file name starts with "foo":

        $ find . -type f -name foo\* -exec rm {} \;
        # to delete a dir
        sudo rm -rf /usr/share/doc/foo*/

NB: use with caution - back up first - also do a dry run first, e.g.

        $ find . -type f -name foo\*



