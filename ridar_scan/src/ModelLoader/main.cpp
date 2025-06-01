#include "ModelLoader/ModelLoader.h"

int main(int argc, char **argv) {
    //#### ROS-Initialisation ####
    ros::init(argc, argv, "model_loader");
    ROS_INFO("Starting ModelLoader-node with node name %s", ros::this_node::getName().c_str());
    ros::NodeHandle node;
    ros::NodeHandle nodeLocal("~");

    //#### Check whether a scan list is provided or not #####
    if(argc != 2) {
        ROS_ERROR("Insufficient arg. A list of scan files must be given.");
        exit(-1);
    }

    // Create a ModelLoader instance
    ModelLoader modelLoader(&node, &nodeLocal, argv[1]);

    // Publish models in the scan list. You may save the pts as ply.
    bool _refNormalization, _savePly;
    double _refMin, _refMax, _meshThresh;
    if(ros::param::get("ref_normalization", _refNormalization)
    && ros::param::get("ref_min", _refMin) && ros::param::get("ref_max", _refMax)
    && ros::param::get("save_ply", _savePly) && ros::param::get("mesh_thresh", _meshThresh)){
        modelLoader.publishScanData(_refNormalization, _refMin, _refMax, _savePly, _meshThresh, ply::BINARY);
    }
    else{
        ROS_WARN("PTS loading properties are not given...\n load a set of PTS using default setting.");
        modelLoader.publishScanData();
    }

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
