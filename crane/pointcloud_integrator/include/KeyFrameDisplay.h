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

#pragma once

#undef Success
#include <Eigen/Core>

#include "QGLViewer/qglviewer.h"
#include "pointcloud_integrator/scanMsg.h"
#include "pointcloud_integrator/meshMsg.h"
#include "pointcloud_integrator/keyScanMsg.h"
#include "sophus/sim3.hpp"

#include <sstream>
#include <fstream>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>

struct MyVertex
{
	float point[3];
	uchar color[4];
};

//struct InputPointDense
//{
//	float idepth;
//	float idepth_var;
//	uchar color[4];
//};

struct InputPoint
{
    //float getDistance(void) {return ;}

    float point[3];
    float color[4];
};

struct meshIndex
{
  int nindex;
  int index[3];
};

// stores a pointcloud associated to a Keyframe.
class KeyFrameDisplay
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	KeyFrameDisplay();
	~KeyFrameDisplay();


	void setFrom(pointcloud_integrator::keyScanMsg msg);
	void setFrom(pointcloud_integrator::keyScanMsgConstPtr msg);
	void drawCam(float lineWidth = 1, float* color = 0);
	//void drawPC(float pointSize = 1, float alpha = 1, const std::vector<float>* viewPoint);
	void drawPC(const float pointSize, const float alpha);
	void refreshPC();
	
	int flushPC(std::ofstream* f);
	const std::vector<InputPoint>& getOriginalInput() const { return this->originalInput; }

	int id;
	double time;
	bool isAdditionalFrame, is8bitNormalized;

	int totalPoints, displayedPoints;


	// camera pose
	// may be updated by kf-graph.
	Sophus::Sim3f camToWorld;

private:
	int width, height;
    int nmeshvertex;
    int nindex;

    float my_scaledTH, my_absTH, my_scale;
	int my_minNearSupport;
	int my_sparsifyFactor;


	// pointcloud data & respective buffer
	//InputPointDense* originalInput;
	std::vector<InputPoint> originalInput;
	std::vector<InputPoint> originalMeshInput;
	GLuint* originalMeshIndex;

	// buffer & how many
	GLuint vertexBufferId;
	GLuint vertexMeshBufferId;
	int vertexBufferNumPoints;

	bool vertexBufferIdValid;	// true if the vertixBufferID is valid (doesnt mean the data in there is still valid)
	bool glBuffersValid;		// true if the vertexBufferID contains valid data

};



