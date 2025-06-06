/**
 * This file is part of LSD-SLAM.
 *
 * Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
 * For more information see <http://vision.in.tum.de/lsdslam> 
 *
 * LSD-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LSD-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with dvo. If not, see <http://www.gnu.org/licenses/>.
 */


#include "ros/ros.h"
#include "boost/thread.hpp"
#include "settings.h"
#include "PointCloudViewer.h"

#include <dynamic_reconfigure/server.h>
#include "pointcloud_integrator/PointCloudIntegratorParamsConfig.h"
#include <qapplication.h>

#include "pointcloud_integrator/keyScanGraphMsg.h"
#include "pointcloud_integrator/keyScanMsg.h"
#include "pointcloud_integrator/snapshot.h"

#include "boost/foreach.hpp"
#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

//#include "KeyFrameDisplay.h"
//#include "sophus/sim3.hpp"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>

PointCloudViewer* viewer = 0;


void dynConfCb(pointcloud_integrator::PointCloudIntegratorParamsConfig &config, uint32_t level)
{

    pointTesselation = config.pointTesselation;
    lineTesselation = config.lineTesselation;

    keepInMemory = config.keepInMemory;
    showKFCameras = config.showKFCameras;
    showKFPointclouds = config.showKFPointclouds;
    showConstraints = config.showConstraints;
    showCurrentCamera = config.showCurrentCamera;
    showCurrentPointcloud = config.showCurrentPointcloud;


    scaledDepthVarTH = exp10( config.scaledDepthVarTH );
    absDepthVarTH = exp10( config.absDepthVarTH );
    minNearSupport = config.minNearSupport;
    sparsifyFactor = config.sparsifyFactor;
    cutFirstNKf = config.cutFirstNKf;

    saveAllVideo = config.saveAllVideo;

}

void frameCb(pointcloud_integrator::keyScanMsgConstPtr msg)
{
    //TODO: This sentence is essential?
    if(msg->time.toSec() > lastFrameTime) return;

    if(viewer != 0)
        viewer->addFrameMsg(msg);
}

void graphCb(pointcloud_integrator::keyScanGraphMsgConstPtr msg)
{
    if(viewer != 0)
        viewer->addGraphMsg(msg);
}

//void cameraParamsCb(sensor_msgs::CameraInfoConstPtr msg)
//{
//    if(!viewer->hasIntrinsicParams())
//        viewer->setIntrinsicParameters(msg);
//}

bool snapshotSrv(pointcloud_integrator::snapshot::Request &req,
              pointcloud_integrator::snapshot::Response &res)
{
  // Set extrinsic camera paramters
    viewer->setExtrinsicParameters(req.camPose);

    // Set image pr
    //res.snapshot.header;
    //res.snapshot.header.seq = viewer->bufferTextureId(); //error viewer->bufferTextureid();
    res.snapshot.header.stamp = ros::Time::now();
    res.snapshot.header.frame_id = "/map";
    res.snapshot.encoding = sensor_msgs::image_encodings::RGBA8;
    res.snapshot.height = viewer->height();
    res.snapshot.width = viewer->width();
    
    usleep(52500);
    viewer->snapshot_.toImageMsg(res.snapshot);

    return true;
}

bool changeCamPoseSrv(pointcloud_integrator::snapshot::Request &req,
        pointcloud_integrator::snapshot::Response &res)
{
    // Set extrinsic camera paramters
    viewer->setExtrinsicParameters(req.camPose);

    return true;
}


bool snapshotWithoutCamPoseSrv(pointcloud_integrator::snapshot::Request &req,
        pointcloud_integrator::snapshot::Response &res)
{
    viewer->snapshot_.toImageMsg(res.snapshot);
 
    return true;
}

void rosThreadLoop( int argc, char** argv )
{
    printf("Started ROS thread\n");

    //glutInit(&argc, argv);

    ros::init(argc, argv, "pointcloud_integrator");
    ROS_INFO("pointcloud_integrator started");

    dynamic_reconfigure::Server<pointcloud_integrator::PointCloudIntegratorParamsConfig> srv;
    srv.setCallback(dynConfCb);


    ros::NodeHandle nh;

    //ros::Subscriber liveFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/liveframes"),1, frameCb);
    //ros::Subscriber keyFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/keyframes"),20, frameCb);
    //ros::Subscriber graph_sub       = nh.subscribe(nh.resolveName("lsd_slam/graph"),10, graphCb);

    ros::Subscriber logScans_sub = nh.subscribe(nh.resolveName("ridar_scan/logScan"), 5, frameCb);              //log scan data (PTS, PTX, etc...)
    ros::Subscriber liveScans_sub = nh.subscribe(nh.resolveName("ridar_scan/liveScan"), 5, frameCb);            //live scan data (from 3D Lidar)
    //ros::Subscriber cameraParams_sub = nh.subscribe(nh.resolveName("usb_cam/camera_info"), 1, cameraParamsCb);    //Intrinsic camera parameters

    ros::ServiceServer snapshot_server = nh.advertiseService("pointcloud_integrator/snapshot", snapshotSrv);       //Provide renderd images
    ros::ServiceServer changeCamPose_server = nh.advertiseService("pointcloud_integrator/changeCamPose", changeCamPoseSrv);       //Provide renderd images
    ros::ServiceServer snapshotWithoutCamPose_server = nh.advertiseService("pointcloud_integrator/snapshotWithoutCamPose", snapshotWithoutCamPoseSrv);       //Provide renderd images

    //if(viewer->hasIntrinsicParams()) cameraParams_sub.shutdown();

    ros::spin();
    ros::shutdown();
    printf("Exiting ROS thread\n");

    exit(1);
}


void rosFileLoop( int argc, char** argv )
{
    ros::init(argc, argv, "pointcloud_integrator");
    dynamic_reconfigure::Server<pointcloud_integrator::PointCloudIntegratorParamsConfig> srv;
    srv.setCallback(dynConfCb);

    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/lsd_slam/liveframes"));
    topics.push_back(std::string("/lsd_slam/keyframes"));
    topics.push_back(std::string("/lsd_slam/graph"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    //for(rosbag::MessageInstance const m = view.begin(); m < view.end(); ++m)
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {

        if(m.getTopic() == "/lsd_slam/liveframes" || m.getTopic() == "/lsd_slam/keyframes")
            frameCb(m.instantiate<pointcloud_integrator::keyScanMsg>());


        if(m.getTopic() == "/lsd_slam/graph")
            graphCb(m.instantiate<pointcloud_integrator::keyScanGraphMsg>());
    }

    ros::spin();

    ros::shutdown();

    printf("Exiting ROS thread\n");

    exit(1);
}




int main( int argc, char** argv )
{


    printf("Started QApplication thread\n");
    // Read command lines arguments.
    QApplication application(argc,argv);

    // Instantiate the viewer.
    viewer = new PointCloudViewer();

#if QT_VERSION < 0x040000
    // Set the viewer as the application main widget.
    application.setMainWidget(viewer);
#else
    viewer->setWindowTitle("PointCloud Integrator");
#endif

    // Make the viewer window visible on screen.
    viewer->show();

    boost::thread rosThread;

    if(argc > 1)
    {
        rosThread = boost::thread(rosFileLoop, argc, argv);
    }
    else
    {
        // start ROS thread
        rosThread = boost::thread(rosThreadLoop, argc, argv);
    }


    application.exec();

    printf("Shutting down... \n");
    ros::shutdown();
    rosThread.join();
    printf("Done. \n");

}
