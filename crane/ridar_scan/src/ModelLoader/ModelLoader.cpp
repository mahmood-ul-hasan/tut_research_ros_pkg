#include "ModelLoader/ModelLoader.h"



ModelLoader::ModelLoader()
{

}



ModelLoader::ModelLoader(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr, const std::string& fileName)
{
    //Set node handler ptrs
    nodeHandlePtr_      = nodeHandlePtr;
    localNodeHandlePtr_ = localNodeHandlePtr;

    //Initialize Subscribers and Publishers
    scanDataWithPose_ = nodeHandlePtr_->advertise<pointcloud_integrator::keyScanMsg>("ridar_scan/logScan", 1);

    //Load the scan model list
    if(!loadModelList(fileName)){
        ROS_ERROR_STREAM("Could't load the scan list.");
        exit(-1);
    }
}



ModelLoader::~ModelLoader()
{

}



//***********************************************************************
bool ModelLoader::loadModelList(const std::string& fileName)
{
    //std::string         _filePath = ros::package::getPath("ridar_scan") + "/" + fileName;
    //std::ifstream       _list(_filePath.c_str());
    //if(_list.fail())    return false;

    std::ifstream       _list(fileName);
    if(_list.fail())    return false;

    std::string _scanDirectory;
    for (const std::string& _el : split(fileName, '/')){
        if(_el.find("txt") == std::string::npos) _scanDirectory += (_el + std::string("/"));
    }

    std::string _el;
    while(getline(_list, _el)) scanPathList_.push_back(_scanDirectory + _el);
    scanNum_ = scanPathList_.size();

    return true;
}


void ModelLoader::publishScanData(const bool normalizeReflectance, const double refMin, const double refMax, const bool exportPLY, const double meshThreshPLY, const ply::PLYTYPE type) const
{
    //TODO:check pts or ptx

    //load the scan files and publish them as keyScanMsg
    int _modelID(0);
    for(const std::string& _el : scanPathList_){
        Pts _elScan;
        Ply _elPly;
        _elScan.loadPtsFile(_el, 1, 0, false, normalizeReflectance, refMin, refMax);
        _elScan.save8bitImages(_el.substr(0, _el.find(".pts")));

        if(exportPLY){
            _elScan.convertToPly(_elPly, meshThreshPLY);
            _elPly.savePlyFile(_el.substr(0, _el.find(".pts")) + ".ply", type);
        }

        //Convert the data to keyScanMsg
        pointcloud_integrator::keyScanMsg _elScanMsg;
        pts2keyScanMsg(_elScan, _elPly, _elScanMsg, _modelID, normalizeReflectance);

        //Publish
        scanDataWithPose_.publish(_elScanMsg);
        ROS_INFO_STREAM("Publish " << _modelID << "-th scan data");

        _modelID++;
        //TODO:Sleep?
        // http://qiita.com/hashimotoryoh/items/609ed5bf4dd4cd90178a#comment-bfc464f7263aac1d76c3
        //usleep(std::chrono::duration_cast<std::chrono::microseconds>(1std::literals::ms).count());
    }

    ROS_INFO_STREAM(_modelID << " out of " << scanNum_ << " have been published!");
}


void ModelLoader::pts2keyScanMsg(const Pts& pts, const Ply& ply, pointcloud_integrator::keyScanMsg& ptsMsg, const int& ID, bool normalizeReflectance) const
{
    //TODO:Convert Pts to PCD to apply outlier removal and voxel down sampling

    ptsMsg.id                   = ID;
    ptsMsg.time                 = ros::Time::now();
    ptsMsg.isAdditionalFrame    = false;
    ptsMsg.is8bitNormalized     = normalizeReflectance;

    ptsMsg.height   =   pts.row();
    ptsMsg.width    =   pts.column();

    Eigen::Matrix4f _poseMat;
    _poseMat << pts.R()(0), pts.R()(1), pts.R()(2), pts.T()(0),
                pts.R()(3), pts.R()(4), pts.R()(5), pts.T()(1),
                pts.R()(6), pts.R()(7), pts.R()(8), pts.T()(2),
                0         , 0         , 0         , 1;
    Sophus::Sim3f _poseSim3(_poseMat); //scaled rotation, translation
    memcpy(ptsMsg.camToWorld.data(), _poseSim3.data(), 7*sizeof(float));


    ptsMsg.pointcloud.reserve(ptsMsg.height * ptsMsg.width);
    for (long idx = 0; idx < ptsMsg.width * ptsMsg.height; idx++) {
        pointcloud_integrator::scanMsg _tmp;
        _tmp.point[0] = pts.vertex()[idx].x();
        _tmp.point[1] = pts.vertex()[idx].y();
        _tmp.point[2] = pts.vertex()[idx].z();

        //float intensity = pts.refImage_.at<float>(y,x);
        float intensity = pts.reflectance()[idx];
        _tmp.color[0] = _tmp.color[1] = _tmp.color[2] = _tmp.color[3] = intensity;

        ptsMsg.pointcloud.emplace_back(_tmp);
        //ptsMsg.pointcloud.push_back(_tmp);
    }
    
    if(ply.nvertex()>0){
        ptsMsg.nmeshvertex = ply.nvertex();
        
        ptsMsg.meshPointcloud.reserve(ply.nvertex());
        for (long idx = 0; idx < ply.nvertex(); idx++) {
            pointcloud_integrator::scanMsg _tmp;
            _tmp.point[0] = ply.vertex()[idx].x();
            _tmp.point[1] = ply.vertex()[idx].y();
            _tmp.point[2] = ply.vertex()[idx].z();

            float intensity = ply.attribute()[idx].intensity_;
            _tmp.color[0] = _tmp.color[1] = _tmp.color[2] = _tmp.color[3] = intensity;

            ptsMsg.meshPointcloud.emplace_back(_tmp);
        }
        //for(int i=0;i<100;i++) ROS_INFO_STREAM("RDSpointcloud:" << ptsMsg.meshPointcloud[i].point[0] << "," << ptsMsg.meshPointcloud[i].point[1] << "," << ptsMsg.meshPointcloud[i].point[2]);

        ptsMsg.nindex = ply.nmesh()*3;
        ptsMsg.meshIndex.reserve(ptsMsg.nindex);
        for (long idx = 0; idx < ply.nmesh(); idx++) {
            ptsMsg.meshIndex.emplace_back(ply.mesh()[idx].index0_);
            ptsMsg.meshIndex.emplace_back(ply.mesh()[idx].index1_);
            ptsMsg.meshIndex.emplace_back(ply.mesh()[idx].index2_);
        }
        //for(int i=0;i<100;i++) ROS_INFO_STREAM("RDSindex:" << ptsMsg.meshIndex[i]);
    }   
    ROS_INFO_STREAM("ScanMsg:" << ptsMsg.width << "x" << ptsMsg.height << "=" << ptsMsg.pointcloud.size() << " points.");
}


std::vector<std::string> ModelLoader::split(const std::string& input, char delimiter)
{
    std::istringstream stream(input);

    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) result.push_back(field);

    return result;
}

