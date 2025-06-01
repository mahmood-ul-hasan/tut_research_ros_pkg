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



#ifndef KEYFRAMEGRAPHDISPLAY_H_
#define KEYFRAMEGRAPHDISPLAY_H_


#include "pointcloud_integrator/keyScanGraphMsg.h"
#include "pointcloud_integrator/keyScanMsg.h"
#include "boost/thread.hpp"

#include "sophus/sim3.hpp"

#include <cstdlib>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "ndt_omp.h"

class KeyFrameDisplay;


struct GraphConstraint
{
	int from;
	int to;
	float err;
};


struct GraphConstraintPt
{
	KeyFrameDisplay* from;
	KeyFrameDisplay* to;
	float err;
};

struct GraphFramePose
{
	int id;
	float camToWorld[7];
};


class KeyFrameGraphDisplay {
public:
	KeyFrameGraphDisplay();
	virtual ~KeyFrameGraphDisplay();

	void draw();

	void addMsg(pointcloud_integrator::keyScanMsgConstPtr msg);
	void addGraphMsg(pointcloud_integrator::keyScanGraphMsgConstPtr msg);

	pointcloud_integrator::keyScanMsg alignPointCloud(const KeyFrameDisplay* target, pointcloud_integrator::keyScanMsgConstPtr input);

	bool flushPointcloud;
	bool printNumbers;

private:
	std::map<int, KeyFrameDisplay*> keyframesByID;
	std::vector<KeyFrameDisplay*> keyframes;
	std::vector<GraphConstraintPt> constraints;

	boost::mutex dataMutex;
	
};

#endif /* KEYFRAMEGRAPHDISPLAY_H_ */
