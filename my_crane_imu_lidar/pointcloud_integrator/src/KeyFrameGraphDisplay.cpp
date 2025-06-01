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


#include "KeyFrameGraphDisplay.h"
#include "KeyFrameDisplay.h"
#include "settings.h"

#include <sstream>
#include <fstream>

#include "ros/ros.h"
#include "ros/package.h"

#include "sophus/sim3.hpp"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>

KeyFrameGraphDisplay::KeyFrameGraphDisplay()
{
	flushPointcloud = false;
	printNumbers = false;
}

KeyFrameGraphDisplay::~KeyFrameGraphDisplay()
{
	for(unsigned int i=0;i<keyframes.size();i++)
		delete keyframes[i];
}


void KeyFrameGraphDisplay::draw()
{
    dataMutex.lock();
    numRefreshedAlready = 0;

    // draw keyframes
    // TODO:set a viewpoint
    float color[3] = {0,0,1};
    for(unsigned int i=0;i<keyframes.size();i++)
    {
        if(showKFCameras)
            keyframes[i]->drawCam(lineTesselation, color);

        if((showKFPointclouds && (int)i > cutFirstNKf) || ((int)i == 0 && keyframes.size() > 0))
            keyframes[i]->drawPC(pointTesselation, 1);
    }

    // TODO:capture command if needed
    if(flushPointcloud)
    {

        printf("Flushing Pointcloud to %s!\n", (ros::package::getPath("pointcloud_integrator")+"/pc_tmp.ply").c_str());
        std::ofstream f((ros::package::getPath("pointcloud_integrator")+"/pc_tmp.ply").c_str());
        int numpts = 0;
        for(unsigned int i=0;i<keyframes.size();i++)
        {
            if((int)i > cutFirstNKf)
                numpts += keyframes[i]->flushPC(&f);
        }
        f.flush();
        f.close();

        std::ofstream f2((ros::package::getPath("pointcloud_integrator")+"/pc.ply").c_str());
        f2 << std::string("ply\n");
        f2 << std::string("format binary_little_endian 1.0\n");
        f2 << std::string("element vertex ") << numpts << std::string("\n");
        f2 << std::string("property float x\n");
        f2 << std::string("property float y\n");
        f2 << std::string("property float z\n");
        f2 << std::string("property float intensity\n");
        f2 << std::string("end_header\n");

        std::ifstream f3((ros::package::getPath("pointcloud_integrator")+"/pc_tmp.ply").c_str());
        while(!f3.eof()) f2.put(f3.get());

        f2.close();
        f3.close();

        system(("rm "+ros::package::getPath("pointcloud_integrator")+"/pc_tmp.ply").c_str());
        flushPointcloud = false;
        printf("Done Flushing Pointcloud with %d points!\n", numpts);

    }


    if(printNumbers)
    {
        int totalPoint = 0;
        int visPoints = 0;
        for(unsigned int i=0;i<keyframes.size();i++)
        {
            totalPoint += keyframes[i]->totalPoints;
            visPoints += keyframes[i]->displayedPoints;
        }

        printf("Have %d points, %d keyframes, %d constraints. Displaying %d points.\n",
                totalPoint, (int)keyframes.size(), (int)constraints.size(), visPoints);
        printNumbers = false;
    }




    if(showConstraints)
    {
        // draw constraints
        glLineWidth(lineTesselation);
        glBegin(GL_LINES);
        for(unsigned int i=0;i<constraints.size();i++)
        {
            if(constraints[i].from == 0 || constraints[i].to == 0)
                continue;

            double colorScalar = std::max(0.0, std::min(1.0, constraints[i].err / 0.05));
            glColor3f(colorScalar, 1 - colorScalar, 0);


            Sophus::Vector3f t = constraints[i].from->camToWorld.translation();
            glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);

            t = constraints[i].to->camToWorld.translation();
            glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);

        }
        glEnd();
    }

    dataMutex.unlock();
}

void KeyFrameGraphDisplay::addMsg(pointcloud_integrator::keyScanMsgConstPtr msg)
{
    dataMutex.lock();

    if(keyframesByID.count(msg->id) == 0)
    {
        KeyFrameDisplay* disp = new KeyFrameDisplay();
        keyframesByID[msg->id] = disp;
        keyframes.push_back(disp);

        ROS_INFO_STREAM("added new KF, now there are " << (int)keyframes.size() << ".\n");
    }

    if (msg->id == 0)
    {
        keyframesByID[msg->id]->setFrom(msg);
    }
    else
    {   
        std::cout << "Do you align the point clouds using NDT algorithm? (y/n): ";

        // Select whether or not you'll align the point cloud data using NDT algorithm
        while(1)
        {
            std::string selectNDT;
            std::cin >> selectNDT;
            std::transform(selectNDT.begin(), selectNDT.end(), selectNDT.begin(), ::tolower);
    
            if (selectNDT == "y")
            {
                pointcloud_integrator::keyScanMsg msgAligned = alignPointCloud(keyframesByID[msg->id - 1], msg);
                keyframesByID[msg->id]->setFrom(msgAligned);
                break;
            }
            else if (selectNDT == "n")
            {
                keyframesByID[msg->id]->setFrom(msg);
                break;
            }
            else
            {
                std::cout << "`PLease enter 'y' or 'n': ";
            }
        }
    }

    dataMutex.unlock();
}

void KeyFrameGraphDisplay::addGraphMsg(pointcloud_integrator::keyScanGraphMsgConstPtr msg)
{
	dataMutex.lock();

	constraints.resize(msg->numConstraints);
	assert(msg->constraintsData.size() == sizeof(GraphConstraint)*msg->numConstraints);
	GraphConstraint* constraintsIn = (GraphConstraint*)msg->constraintsData.data();
	for(int i=0;i<msg->numConstraints;i++)
	{
		constraints[i].err = constraintsIn[i].err;
		constraints[i].from = 0;
		constraints[i].to = 0;

		if(keyframesByID.count(constraintsIn[i].from) != 0)
			constraints[i].from = keyframesByID[constraintsIn[i].from];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].from);


		if(keyframesByID.count(constraintsIn[i].to) != 0)
			constraints[i].to = keyframesByID[constraintsIn[i].to];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].to);
	}



	GraphFramePose* graphPoses = (GraphFramePose*)msg->scanData.data();
	int numGraphPoses = msg->numScans;
	assert(msg->scanData.size() == sizeof(GraphFramePose)*msg->numScans);

	for(int i=0;i<numGraphPoses;i++)
	{
		if(keyframesByID.count(graphPoses[i].id) == 0)
		{
		//	printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[i].id, graphPoses[i].id);
		}
		else
			memcpy(keyframesByID[graphPoses[i].id]->camToWorld.data(), graphPoses[i].camToWorld, 7*sizeof(float));
	}

	dataMutex.unlock();

//	printf("graph update: %d constraints, %d poses\n", msg->numConstraints, msg->numScans);
}

pointcloud_integrator::keyScanMsg KeyFrameGraphDisplay::alignPointCloud(const KeyFrameDisplay* target, pointcloud_integrator::keyScanMsgConstPtr input)
{
    double x, y, z, yaw;
    std::cout << "Please input initial values (x, y, z, yaw) for alignment." << std::endl;
    std::cin >> x >> y >> z >> yaw;
    std::cout << "Start to align with initial values set to " << "(" << x << ", " << y << ", " << z << ", " << yaw << ")" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr targetCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZI>);

    // Input taeget_cloud
    for (size_t index = 0; index < target->getOriginalInput().size(); ++index)
    {
        pcl::PointXYZI _tmp;
        InputPoint _ip;
        _ip = target->getOriginalInput()[index];

        _tmp.x = _ip.point[0];
        _tmp.y = _ip.point[1];
        _tmp.z = _ip.point[2];
        _tmp.intensity = _ip.color[0];

        targetCloud->push_back(_tmp);
    }

    // Input input_cloud
    for(auto _el : input->pointcloud)
    {
        pcl::PointXYZI _tmp;

        _tmp.x = _el.point[0];
        _tmp.y = _el.point[1];
        _tmp.z = _el.point[2];
        _tmp.intensity = _el.color[0];
        
        inputCloud->push_back(_tmp);
    }

    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximateVoxelFilter;
    approximateVoxelFilter.setLeafSize (0.24, 0.24, 0.24);
    approximateVoxelFilter.setInputCloud (inputCloud);
    approximateVoxelFilter.filter (*filteredCloud);
    std::cout << "Filtered cloud contains " << filteredCloud->size ()
                << " data points from inputCloud" << std::endl;

    // Initializing Normal Distributions Transform (NDT OpenMP).
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.c_building_left.pts
    ndt.setTransformationEpsilon (1e-6);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (2.0);
    // Setting max number of registration iterations.
    ndt.setMaximumIterations (100);
    ndt.setNeighborhoodSearchMethod (pclomp::KDTREE);

    // Setting point cloud to be aligned.
    ndt.setInputSource (filteredCloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (targetCloud);

    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf initRotation (yaw, Eigen::Vector3f::UnitZ ());
    Eigen::Matrix4f initGuess;
    Eigen::Translation3f initTranslation (x, y, z);
    initGuess = (initTranslation * initRotation).matrix ();

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZI> outputCloud;// (new pcl::PointCloud<pcl::PointXYZI>);

    ndt.align (outputCloud, initGuess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
                << " score: " << ndt.getFitnessScore () << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*inputCloud, outputCloud, ndt.getFinalTransformation ());

    pointcloud_integrator::keyScanMsg msgAligned;
    msgAligned = *input;  
    msgAligned.pointcloud.clear();

    for (size_t index = 0; index < outputCloud.size(); ++index)
    {
        pointcloud_integrator::scanMsg _tmp;
        pcl::PointXYZI _point;
        _point = outputCloud[index];

        _tmp.point[0] = _point.x;
        _tmp.point[1] = _point.y;
        _tmp.point[2] = _point.z;

        double intensity = _point.intensity;
        _tmp.color[0] = intensity;
        _tmp.color[1] = intensity;
        _tmp.color[2] = intensity;
        _tmp.color[3] = intensity;

        msgAligned.pointcloud.emplace_back(_tmp);
    }

    return msgAligned;
}