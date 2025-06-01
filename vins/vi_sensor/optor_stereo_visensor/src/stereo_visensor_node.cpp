#include "optor_stereo_visensor_ros/stereo_visensor_cam.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "stereo_visensor_node");
    ros::NodeHandle nh;
    char *path = "/home/aisl/catkin_ws/src/optor_stereo_visensor/optor_VISensor_Setups.txt";
    std::cout<< "file path: " << path << std::endl;

    // if(argv[1]) {
    //     path = argv[1];
    // }

    std::cout << "stereo_visensor_node starting------- " << std::endl;
    

    StereoVisensorCam StereoVisensorStart(nh, path);

    std::cout << "stereo_visensor_node started " << std::endl;

    int c_strt = StereoVisensorStart.startCam();
    if(!c_strt)
        std::cout << "ERROR: camera open failed" << std::endl;
    if(c_strt)
        std::cout << "camera open success........" << std::endl;

    if(!StereoVisensorStart.startImu())
        std::cout << "ERROR: imu open failed" << std::endl;

    std::cout << "stereo_visensor_node:---------- " << std::endl;

    StereoVisensorStart.exectu();

    return 0;
}