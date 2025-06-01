#pragma once

#include <list>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include <chrono>
#include <unistd.h>


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>

#include "pointcloud_integrator/scanMsg.h"
#include "pointcloud_integrator/keyScanMsg.h"


#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>


#include "sophus/sim3.hpp"
#include "sophus/se3.hpp"

#include "Common/Pts.h"
//#include "Common/Ptx.h"
#include "Common/Ply.h"


class ModelLoader {
    public:
        // Constructor and Destructor
        ModelLoader();
        ModelLoader(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr, const std::string& fileName);
        ~ModelLoader();

        // Public functions
        void publishScanData(const bool normalizeReflectance = false, const double refMin = 0.0, const double refMax = 0.0, const bool exportPLY = false, const double meshThreshPLY = 1.0, const ply::PLYTYPE type = ply::BINARY) const;

        // Accessors
        const int&                      scanNum()       const   { return this->scanNum_; }
        const std::list<std::string>&   scanPathList()  const   { return this->scanPathList_; }


    private:
        // Private functions
        bool                        loadModelList(const std::string& fileName);
        void                        pts2keyScanMsg(const Pts& pts, const Ply& ply, pointcloud_integrator::keyScanMsg& ptsMsg, const int& ID, bool normalizeReflectance) const;
        std::vector<std::string>    split(const std::string& input, char delimiter);


        // Member variables
        int                         scanNum_;
        std::list<std::string>      scanPathList_;

        ros::NodeHandle*            nodeHandlePtr_;
        ros::NodeHandle*            localNodeHandlePtr_;
        ros::Publisher              scanDataWithPose_;
};
