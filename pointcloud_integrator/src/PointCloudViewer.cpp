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

#define GL_GLEXT_PROTOTYPES 1
#include "PointCloudViewer.h"
#include "qfiledialog.h"
#include "qcoreapplication.h"
#include <stdio.h>
#include "settings.h"
#include "ros/package.h"
#include <ros/init.h>

#include <zlib.h>
#include <iostream>


#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "QGLViewer/manipulatedCameraFrame.h"

#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"

#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#define _USE_MATH_DEFINES
#include <math.h>

PointCloudViewer::PointCloudViewer()
{
    setPathKey(Qt::Key_0,0);
    setPathKey(Qt::Key_1,1);
    setPathKey(Qt::Key_2,2);
    setPathKey(Qt::Key_3,3);
    setPathKey(Qt::Key_4,4);
    setPathKey(Qt::Key_5,5);
    setPathKey(Qt::Key_6,6);
    setPathKey(Qt::Key_7,7);
    setPathKey(Qt::Key_8,8);
    setPathKey(Qt::Key_9,9);


    currentCamDisplay = 0;
    graphDisplay = 0;


    for(int i=0;i<10;i++)
    {
        KFexists[i] = 0;
        KFautoPlayIdx[i] = -1;
    }

    kfInt = new qglviewer::KeyFrameInterpolator(new qglviewer::Frame());
    customAnimationEnabled = false;

    setSnapshotFormat(QString("PNG"));
    setBackgroundColor(QColor(0,0,0));
    
    

    reset();
}


PointCloudViewer::~PointCloudViewer()
{
    delete currentCamDisplay;
    delete graphDisplay;
}


void PointCloudViewer::reset()
{
    if(currentCamDisplay != 0)
        delete currentCamDisplay;
    if(graphDisplay != 0)
        delete graphDisplay;

    currentCamDisplay = new KeyFrameDisplay();
    graphDisplay = new KeyFrameGraphDisplay();

    KFcurrent = 0;
    KFLastPCSeq = -1;

    resetRequested=false;

    save_folder = ros::package::getPath("pointcloud_integrator")+"/save/";
    localMsBetweenSaves = 1;
    simMsBetweenSaves = 1;
    lastCamID = -1;
    lastAnimTime = lastCamTime = lastSaveTime = 0;
    char buf[500];
    snprintf(buf,500,"rm -rf %s",save_folder.c_str());
    int k = system(buf);
    snprintf(buf,500,"mkdir %s",save_folder.c_str());
    k += system(buf);


    assert(k != -42);

    setSceneRadius(80);
    setTextIsEnabled(false);
    lastAutoplayCheckedSaveTime = -1;

    animationPlaybackEnabled = false;
}


//void PointCloudViewer::setIntrinsicParameters(sensor_msgs::CameraInfoConstPtr msg)
void PointCloudViewer::setIntrinsicParameters()
{
    meddleMutex.lock();

    int _width, _height;
    float _K[9];
    
    if(!getIntrinsicParams(_height,_width,_K))
    {
      printf("can't read camera parameters");
    }else{
    
      //printf("_width = %d\n",_width);
      //printf("_height = %d\n",_height);
      //for(int i=0;i<9;i++){
      // printf("_K[%d] = %f\n", i, _K[i]); 
      //}
    
    
      // QGLViewer camera papameter adjustment references:
      // http://libqglviewer.com/refManual/classqglviewer_1_1Camera.html
      // http://users.monash.edu/~cema/Software/glitch/developer-doc/classqglviewer_1_1Camera.html

      // Set window size
      this->setFixedSize(_width,_height);
      camera()->setScreenWidthAndHeight(_width,_height);
    
      // Set FoV
      camera()->setFieldOfView(2 * atan( _K[5] / _K[4]));
    
      camera()->computeProjectionMatrix();
      camera()->loadProjectionMatrix();
    
      hasIntrinsicParams_ = true;
    }
    
    meddleMutex.unlock();
}


//void PointCloudViewer::setExtrinsicParameters(geometry_msgs::PoseConstPtr msg)
void PointCloudViewer::setExtrinsicParameters(const geometry_msgs::Pose& msg)
{
    meddleMutex.lock();

    camera()->setPosition(qglviewer::Vec(msg.position.x, msg.position.y, msg.position.z));
    camera()->setOrientation(qglviewer::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
    
    meddleMutex.unlock();
}

bool PointCloudViewer::hasIntrinsicParams(void)
{
    return hasIntrinsicParams_;
}

bool PointCloudViewer::getIntrinsicParams(int& height, int& width, float K[9])
{
    std::string _filename = "/home/aisl/catkin_ws/src/pointcloud_integrator/camparam/camparam.yaml" ;
    
    std::ifstream _list(_filename);
    if(_list.fail()){
     return false;
    }
    
    std::string _el;
    while(getline(_list, _el)){
      if(_el.find("image_height") != std::string::npos){
	std::vector<std::string> _height = split(_el, ':');
	height = std::stoi(_height[1]); 
      }else if(_el.find("image_width") != std::string::npos){
	std::vector<std::string> _width = split(_el, ':');
	width = std::stoi(_width[1]);
      }else if(_el.find("data") != std::string::npos){
	std::vector<std::string> _a = split(_el, ':');
	std::vector<std::string> _b = split(_a[1], '[');
	std::vector<std::string> _c = split(_b[1], ',');
	for(int i = 0;i<9;i++){
	 K[i] = std::stof(_c[i]); 
	}
	break;
      }
    }
    
    return true;
}


void PointCloudViewer::addFrameMsg(pointcloud_integrator::keyScanMsgConstPtr msg)
{
    meddleMutex.lock();

    graphDisplay->addMsg(msg);

    meddleMutex.unlock();
}


//void PointCloudViewer::addFrameMsg(pointcloud_integrator::keyScanMsgConstPtr msg)
//{
//    meddleMutex.lock();
//
//    if(!msg->isKeyframe)
//    {
//        if(currentCamDisplay->id > msg->id)
//        {
//            printf("detected backward-jump in id (%d to %d), resetting!\n", currentCamDisplay->id, msg->id);
//            resetRequested = true;
//        }
//        currentCamDisplay->setFrom(msg);
//        lastAnimTime = lastCamTime = msg->time;
//        lastCamID = msg->id;
//    }
//    else
//        graphDisplay->addMsg(msg);
//
//    meddleMutex.unlock();
//}

void PointCloudViewer::addGraphMsg(pointcloud_integrator::keyScanGraphMsgConstPtr msg)
{
    meddleMutex.lock();

    graphDisplay->addGraphMsg(msg);

    meddleMutex.unlock();
}


void PointCloudViewer::init()
{
    setAnimationPeriod(30);
    startAnimation();
    setIntrinsicParameters();
}

QString PointCloudViewer::helpString() const
{
    return QString("");
}

void PointCloudViewer::draw()
{
    meddleMutex.lock();


    if(resetRequested)
    {
        reset();
        resetRequested = false;
    }


    glPushMatrix();


    if(animationPlaybackEnabled)
    {
        double tm = ros::Time::now().toSec() - animationPlaybackTime;

        if(tm > kfInt->lastTime())
        {
            animationPlaybackEnabled = false;
            tm = kfInt->lastTime();
        }

        if(tm < kfInt->firstTime())
            tm = kfInt->firstTime();

        printf("anim at %.2f (%.2f to %.2f)\n", tm, kfInt->firstTime(), kfInt->lastTime());


        kfInt->interpolateAtTime(tm);
        camera()->frame()->setFromMatrix(kfInt->frame()-> matrix());



        double accTime = 0;
        for(unsigned int i=0;i<animationList.size();i++)
        {
            if(tm >= accTime && tm < accTime+animationList[i].duration && animationList[i].isFix)
            {
                camera()->frame()->setFromMatrix(animationList[i].frame.matrix());

                printf("fixFrameto %d at %.2f (%.2f to %.2f)\n", i, tm, kfInt->firstTime(), kfInt->lastTime());
            }

            accTime += animationList[i].duration;
        }


        accTime = 0;
        AnimationObject* lastAnimObj = 0;
        for(unsigned int i=0;i<animationList.size();i++)
        {
            accTime += animationList[i].duration;
            if(animationList[i].isSettings && accTime <= tm)
                lastAnimObj = &(animationList[i]);
        }
        if(lastAnimObj != 0)
        {
            absDepthVarTH = lastAnimObj->absTH;
            scaledDepthVarTH = lastAnimObj->scaledTH;
    
            minNearSupport = lastAnimObj->neighb;
            sparsifyFactor = lastAnimObj->sparsity;
            showKFCameras = lastAnimObj->showKeyframes;
            showConstraints = lastAnimObj->showLoopClosures;
        }
    }


    // TODO: set a viewpoint
    // cam_;

    if(showCurrentCamera)
        currentCamDisplay->drawCam(2*lineTesselation, 0);

    if(showCurrentPointcloud)
        currentCamDisplay->drawPC(pointTesselation, 1);

    graphDisplay->draw();

    // TODO: capture the rendered image
    // 1. Save the rendered image as a file, but useless for our purpose.
    // this->saveSnapshot();

    // 2. Store the rendered image in memory by QGLViewer::copyBufferToTexture + glCopyTexSubImage2D()?
    // this->copyBufferToTexture();

    // 3. Use OpenGL functions for storing the rendered image
    //int imageWidth = 600,imageHeight = 400;
    //cv::Mat bmp(imageHeight, imageWidth, CV_8UC4);

    //glFlush();
    //glReadBuffer(GL_BACK);
    //glPixelStorei(GL_PACK_ALIGNMENT, 4);
    //glReadPixels(0, 0, imageWidth, imageHeight, GL_BGRA, GL_UNSIGNED_BYTE, (void*)bmp.data);
    //cv::flip(bmp, bmp, 0);
    //cv::imshow("../test.bmp", bmp);
    //cv::waitKey(1000);

    snapshot();

    glPopMatrix();

    meddleMutex.unlock();




    if(saveAllVideo)
    {
        double span = ros::Time::now().toSec() - lastRealSaveTime;
        if(span > 0.4)
        {
            setSnapshotQuality(100);

            printf("saved (img %d @ time %lf, saveHZ %f)!\n", lastCamID, lastAnimTime, 1.0/localMsBetweenSaves);

            char buf[500];
            snprintf(buf,500,"%s%lf.png",save_folder.c_str(),  ros::Time::now().toSec());
            saveSnapshot(QString(buf));
            lastRealSaveTime = ros::Time::now().toSec();
        }


    }
}

void PointCloudViewer::snapshot()
{
    int _imageWidth = camera()->screenWidth();
    int _imageHeight = camera()->screenHeight();

    snapshot_.image = cv::Mat::zeros(_imageHeight,_imageWidth,CV_8UC3);
    
    glReadBuffer(GL_FRONT);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadPixels(0, 0, _imageWidth, _imageHeight, GL_BGR, GL_UNSIGNED_BYTE, (void*)snapshot_.image.data);
}

void PointCloudViewer::keyReleaseEvent(QKeyEvent *e)
{

}


void PointCloudViewer::setToVideoSize()
{
    this->setFixedSize(1600,900);
}


void PointCloudViewer::remakeAnimation()
{
    delete kfInt;
    kfInt = new qglviewer::KeyFrameInterpolator(new qglviewer::Frame());
    std::sort(animationList.begin(), animationList.end());

    float tm=0;
    for(unsigned int i=0;i<animationList.size();i++)
    {
        if(!animationList[i].isSettings)
        {
            kfInt->addKeyFrame(&animationList[i].frame, tm);
            tm += animationList[i].duration;
        }
    }

    printf("made animation with %d keyframes, spanning %f s!\n", kfInt->numberOfKeyFrames(), tm);
}

void PointCloudViewer::keyPressEvent(QKeyEvent *e)
{
    switch (e->key())
    {
        case Qt::Key_S :
            setToVideoSize();
            break;

        case Qt::Key_R :
            resetRequested = true;

            break;

        case Qt::Key_T:	// add settings item
            meddleMutex.lock();
            animationList.push_back(AnimationObject(true, lastAnimTime, 0));
            meddleMutex.unlock();
            printf("added St: %s\n", animationList.back().toString().c_str());

            break;

        case Qt::Key_K:	// add keyframe item
            meddleMutex.lock();


            double x,y,z;
            camera()->frame()->getPosition(x,y,z);
            animationList.push_back(AnimationObject(false, lastAnimTime, 2, qglviewer::Frame(qglviewer::Vec(0,0,0), camera()->frame()->orientation())));
            animationList.back().frame.setPosition(x,y,z);
            meddleMutex.unlock();
            printf("added KF: %s\n", animationList.back().toString().c_str());



            remakeAnimation();

            break;

        case Qt::Key_I :	// reset animation list
            meddleMutex.lock();
            animationList.clear();
            meddleMutex.unlock();
            printf("resetted animation list!\n");

            remakeAnimation();

            break;


        case Qt::Key_F :	// save list
            {
                meddleMutex.lock();
                std::ofstream myfile;
                myfile.open ("animationPath.txt");
                for(unsigned int i=0;i<animationList.size();i++)
                {
                    myfile << animationList[i].toString() << "\n";
                }
                myfile.close();
                meddleMutex.unlock();

                printf("saved animation list (%d items)!\n", (int)animationList.size());
            }
            break;


        case Qt::Key_L :	// load list
            {
                meddleMutex.lock();
                animationList.clear();

                std::ifstream myfile;
                std::string line;
                myfile.open ("animationPath.txt");

                if (myfile.is_open())
                {
                    while ( getline (myfile,line) )
                    {
                        if(!(line[0] == '#'))
                            animationList.push_back(AnimationObject(line));
                    }
                    myfile.close();
                }
                else
                    std::cout << "Unable to open file";
                myfile.close();
                meddleMutex.unlock();

                printf("loaded animation list! (%d items)!\n", (int)animationList.size());
                remakeAnimation();
            }
            break;


        case Qt::Key_A:
            if(customAnimationEnabled)
                printf("DISABLE custom animation!\n)");
            else
                printf("ENABLE custom animation!\n");
            customAnimationEnabled = !customAnimationEnabled;
            break;

        case Qt::Key_O:
            if(animationPlaybackEnabled)
            {
                animationPlaybackEnabled=false;
            }
            else
            {
                animationPlaybackEnabled = true;
                animationPlaybackTime = ros::Time::now().toSec();
            }
            break;


        case Qt::Key_P:
            graphDisplay->flushPointcloud = true;
            break;

        case Qt::Key_W:
            graphDisplay->printNumbers = true;
            break;
	    
	      case Qt::Key_H:
	          if(drawPointcloud==true)
	          {
	              drawPointcloud = false;
	              printf("draw PointCloud\n");
	          }else{
	              drawPointcloud = true;
	              printf("draw mesh\n");
	          }
	          this->draw();
	          break;

        default:
            QGLViewer::keyPressEvent(e);
            break;
    }
}

std::vector< std::string > PointCloudViewer::split(const std::string& input, char delimiter)
{
    std::istringstream stream(input);

    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) result.push_back(field);

    return result;
}
