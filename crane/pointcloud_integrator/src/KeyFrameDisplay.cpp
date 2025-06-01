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

#include "KeyFrameDisplay.h"
#include <stdio.h>
#include "settings.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "ros/ros.h"
#include "ros/package.h"

#include <GL/freeglut.h>

KeyFrameDisplay::KeyFrameDisplay()
{
    //originalInput = 0;
    originalInput.clear();
    originalInput.shrink_to_fit();
    id = 0;
    vertexBufferIdValid = false;
    glBuffersValid = false;

    camToWorld = Sophus::Sim3f();
    width=height=0;
    nmeshvertex=nindex=0;

    my_scaledTH = my_absTH = 0;

    totalPoints = displayedPoints = 0;

}


KeyFrameDisplay::~KeyFrameDisplay()
{
    if(vertexBufferIdValid)
    {
        glDeleteBuffers(1, &vertexBufferId);
        vertexBufferIdValid = false;
    }

    if(!originalInput.empty()){
        originalInput.clear();
        originalInput.shrink_to_fit();
    }
}


void KeyFrameDisplay::setFrom(pointcloud_integrator::keyScanMsg msg)
{
    // copy over campose.
    memcpy(camToWorld.data(), msg.camToWorld.data(), 7*sizeof(float));

    width = msg.width;
    height = msg.height;
    id = msg.id;
    time = msg.time.toSec();

    isAdditionalFrame = msg.isAdditionalFrame;
    is8bitNormalized = msg.is8bitNormalized;

    if(!originalInput.empty()){
        originalInput.clear();
        originalInput.shrink_to_fit();
    }

    if(msg.pointcloud.size() != width*height)
    {
        if(msg.pointcloud.size() != 0)
        {
            ROS_WARN_STREAM("PC with points, but number of points not right! (is " << msg.pointcloud.size() << " should be " << width << " x " << height << " = " << width*height << "\n");
        }
    }
    else
    {
        originalInput.reserve(width*height);
        for(auto _el : msg.pointcloud){
            InputPoint _tmp;
            _tmp.point[0] = _el.point[0];
            _tmp.point[1] = _el.point[1];
            _tmp.point[2] = _el.point[2];

            _tmp.color[0] = _el.color[0];
            _tmp.color[1] = _el.color[1];
            _tmp.color[2] = _el.color[2];
            _tmp.color[3] = _el.color[3];

            originalInput.emplace_back(_tmp);
        }
    }

    glBuffersValid = false;
}



void KeyFrameDisplay::setFrom(pointcloud_integrator::keyScanMsgConstPtr msg)
{
    // copy over campose.
    memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));

    width = msg->width;
    height = msg->height;
    nmeshvertex = msg->nmeshvertex;
    nindex = msg->nindex;
    id = msg->id;
    time = msg->time.toSec();

    isAdditionalFrame = msg->isAdditionalFrame;
    is8bitNormalized = msg->is8bitNormalized;



    if(!originalInput.empty()){
        originalInput.clear();
        originalInput.shrink_to_fit();
    }

    if(msg->pointcloud.size() != width*height)
    {
        if(msg->pointcloud.size() != 0)
        {
            ROS_WARN_STREAM("PC with points, but number of points not right! (is " << msg->pointcloud.size() << " should be " << width << " x " << height << " = " << width*height << "\n");
        }
    }
    else
    {
        // TODO: Should implement faster storing method...
        //originalInput = new InputPointDense[width*height];
        //memcpy(originalInput, msg->pointcloud.data(), width*height*sizeof(pointcloud_integrator::scanMsg));

        //originalInput.resize(width*height);
        //std::copy(originalInput.begin(), originalInput.end(), back_inserter(msg->pointcloud));

        originalInput.reserve(width*height);
        for(auto _el : msg->pointcloud){
            InputPoint _tmp;
            _tmp.point[0] = _el.point[0];
            _tmp.point[1] = _el.point[1];
            _tmp.point[2] = _el.point[2];

            _tmp.color[0] = _el.color[0];
            _tmp.color[1] = _el.color[1];
            _tmp.color[2] = _el.color[2];
            _tmp.color[3] = _el.color[3];

            originalInput.emplace_back(_tmp);
        }
        
        originalMeshInput.reserve(nmeshvertex);
        for(auto _el : msg->meshPointcloud){
            InputPoint _tmp;
            _tmp.point[0] = _el.point[0];
            _tmp.point[1] = _el.point[1];
            _tmp.point[2] = _el.point[2];

            _tmp.color[0] = _el.color[0];
            _tmp.color[1] = _el.color[1];
            _tmp.color[2] = _el.color[2];
            _tmp.color[3] = _el.color[3];

            originalMeshInput.emplace_back(_tmp);
        }
        //for(int i=0;i<100;i++) ROS_INFO_STREAM("PCIpointcloud:" << originalMeshInput[i].point[0] << "," << originalMeshInput[i].point[1] << "," << originalMeshInput[i].point[2]);
       
        //int i=0;
        //for(auto _el : msg->mesh){
        //    index[i] = _el.index[0];
        //    index[i+1] = _el.index[1];
        //    index[i+2] = _el.index[2];
        //    i += 3;
        //}
        //indexNum = msg->mesh.size()*3;
        //
        originalMeshIndex = new GLuint[nindex];
        for(int i=0;i<nindex;i++){
            originalMeshIndex[i] = msg->meshIndex[i];
        }
        //for(int i=0;i<100;i++) ROS_INFO("PCIindex:%d",originalMeshIndex[i]);
        //for(auto _el : msg->pointcloud){
        //    originalInput.push_back(_el);
        //}

    }

    glBuffersValid = false;
}


void KeyFrameDisplay::refreshPC()
{
    //	minNearSupport = 9;
    bool paramsStillGood = my_scaledTH == scaledDepthVarTH &&
        my_absTH == absDepthVarTH &&
        my_scale*1.2 > camToWorld.scale() &&
        my_scale < camToWorld.scale()*1.2 &&
        my_minNearSupport == minNearSupport &&
        my_sparsifyFactor == sparsifyFactor;



    if(glBuffersValid && (paramsStillGood || numRefreshedAlready > 10)) return;
    numRefreshedAlready++;

    glBuffersValid = true;


    // delete old vertex buffer
    if(vertexBufferIdValid)
    {
        glDeleteBuffers(1, &vertexBufferId);
        vertexBufferIdValid = false;
    }



    // if there are no vertices, done!
    if(originalInput.empty()) return;

    // make data
    MyVertex* tmpBuffer = new MyVertex[width*height];
    MyVertex* tmpMeshBuffer = new MyVertex[nmeshvertex];

    my_scaledTH =scaledDepthVarTH;
    my_absTH = absDepthVarTH;
    my_scale = camToWorld.scale();
    my_minNearSupport = minNearSupport;
    my_sparsifyFactor = sparsifyFactor;
    // data is directly in ros message, in correct format.
    vertexBufferNumPoints = 0;

    int total = 0, displayed = 0;
    for(int y=1;y<height-1;y++)
        for(int x=1;x<width-1;x++)
        {
            InputPoint _tmp = originalInput[x+y*width];
            float _len = sqrt(pow(_tmp.point[0], 2.0) + pow(_tmp.point[1], 2.0) + pow(_tmp.point[2], 2.0));
            if(_len < 1e-5) continue;
            total++;

            if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;

            //float depth = 1 / originalInput[x+y*width].idepth;
            //float depth4 = depth*depth; depth4*= depth4;


            //if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
            //    continue;

            //if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
            //    continue;

            if(my_minNearSupport > 1)
            {
                int nearSupport = 0;
                for(int dx=-1;dx<2;dx++)
                    for(int dy=-1;dy<2;dy++)
                    {
                        InputPoint _nibr    = originalInput[x+dx+(y+dy)*width];
                        float _nibrLen      = sqrt(pow(_nibr.point[0], 2.0) + pow(_nibr.point[1], 2.0) + pow(_nibr.point[2], 2.0));
                        if(_nibrLen > 1e-5)
                        {
                            float diff = _nibrLen - _len;
                            //if(diff*diff < 2*originalInput[x+y*width].idepth_var)
                            if(diff*diff < 2*0.1)
                                nearSupport++;
                        }
                    }

                if(nearSupport < my_minNearSupport)
                    continue;

            }

            tmpBuffer[vertexBufferNumPoints].point[0] = _tmp.point[0];
            tmpBuffer[vertexBufferNumPoints].point[1] = _tmp.point[1];
            tmpBuffer[vertexBufferNumPoints].point[2] = _tmp.point[2];

            if(is8bitNormalized){
                tmpBuffer[vertexBufferNumPoints].color[0] = _tmp.color[0]*255;
                tmpBuffer[vertexBufferNumPoints].color[1] = _tmp.color[1]*255;
                tmpBuffer[vertexBufferNumPoints].color[2] = _tmp.color[2]*255;
                tmpBuffer[vertexBufferNumPoints].color[3] = _tmp.color[3]*255;
            }else{
                tmpBuffer[vertexBufferNumPoints].color[0] = _tmp.color[0];
                tmpBuffer[vertexBufferNumPoints].color[1] = _tmp.color[1];
                tmpBuffer[vertexBufferNumPoints].color[2] = _tmp.color[2];
                tmpBuffer[vertexBufferNumPoints].color[3] = _tmp.color[3];
            }

            vertexBufferNumPoints++;
            displayed++;
        }
    
    for(int idx=0;idx<nmeshvertex;idx++)
    {
        InputPoint _tmp = originalMeshInput[idx];

        tmpMeshBuffer[idx].point[0] = _tmp.point[0];
        tmpMeshBuffer[idx].point[1] = _tmp.point[1];
        tmpMeshBuffer[idx].point[2] = _tmp.point[2];

        if(is8bitNormalized){
            tmpMeshBuffer[idx].color[0] = _tmp.color[0]*255;
            tmpMeshBuffer[idx].color[1] = _tmp.color[1]*255;
            tmpMeshBuffer[idx].color[2] = _tmp.color[2]*255;
            tmpMeshBuffer[idx].color[3] = _tmp.color[3]*255;
        }else{
            tmpMeshBuffer[idx].color[0] = _tmp.color[0];
            tmpMeshBuffer[idx].color[1] = _tmp.color[1];
            tmpMeshBuffer[idx].color[2] = _tmp.color[2];
            tmpMeshBuffer[idx].color[3] = _tmp.color[3];
        }
    } 

    totalPoints = total;
	displayedPoints = displayed;

    
    // create new ones, static
    vertexBufferId=0;
    glGenBuffers(1, &vertexBufferId);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);         // for vertex coordinates
    glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * vertexBufferNumPoints, tmpBuffer, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    //for mesh 
    vertexMeshBufferId=0;
    glGenBuffers(1, &vertexMeshBufferId);
    glBindBuffer(GL_ARRAY_BUFFER, vertexMeshBufferId);         // for vertex coordinates
    glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * nmeshvertex, tmpMeshBuffer, GL_STATIC_DRAW);

    vertexBufferIdValid = true;

    if(!keepInMemory)
    {
        originalInput.clear();
        originalInput.shrink_to_fit();
    }




    delete[] tmpBuffer;
    delete[] tmpMeshBuffer;
}



void KeyFrameDisplay::drawCam(float lineWidth, float* color)
{
    if(width == 0)
        return;


    glPushMatrix();

    Sophus::Matrix4f m = camToWorld.matrix();
    glMultMatrixf((GLfloat*)m.data());

    if(color == 0)
        glColor3f(1,0,0);
    else
        glColor3f(color[0],color[1],color[2]);

    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0.5,0,0);
    glVertex3f(0,0,0);
    glVertex3f(0,0.5,0);
    glVertex3f(0,0,0);
    glVertex3f(0,0,0.5);
    glEnd();
    glPopMatrix();

}

int KeyFrameDisplay::flushPC(std::ofstream* f)
{

    MyVertex* tmpBuffer = new MyVertex[width*height];
    int num = 0;
    for(int y=1;y<height-1;y++)
        for(int x=1;x<width-1;x++)
        {
            InputPoint  _tmp = originalInput[x+y*width];
            float       _len = sqrt(pow(_tmp.point[0], 2.0) + pow(_tmp.point[1], 2.0) + pow(_tmp.point[2], 2.0));

            if(_len <= 1e-5) continue;

            if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;

            //float depth = 1 / originalInput[x+y*width].idepth;
            //float depth4 = depth*depth; depth4*= depth4;

            //if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
            //    continue;

            //if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
            //    continue;


            if(my_minNearSupport > 1)
            {
                int nearSupport = 0;
                for(int dx=-1;dx<2;dx++)
                    for(int dy=-1;dy<2;dy++)
                    {
                        InputPoint _nibr    = originalInput[x+dx+(y+dy)*width];
                        float _nibrLen      = sqrt(pow(_nibr.point[0], 2.0) + pow(_nibr.point[1], 2.0) + pow(_nibr.point[2], 2.0));
                        if(_nibrLen > 1e-5)
                        {
                            float diff = _nibrLen - _len;
                            //if(diff*diff < 2*originalInput[x+y*width].idepth_var)
                            if(diff*diff < 2*0.1)
                                nearSupport++;
                        }
                    }

                if(nearSupport < my_minNearSupport)
                    continue;
            }


            Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f(_tmp.point[0], _tmp.point[1], _tmp.point[2]));
            tmpBuffer[vertexBufferNumPoints].point[0] = pt[0];
            tmpBuffer[vertexBufferNumPoints].point[1] = pt[1];
            tmpBuffer[vertexBufferNumPoints].point[2] = pt[2];

            tmpBuffer[num].color[0] = _tmp.color[0];
            tmpBuffer[num].color[1] = _tmp.color[1];
            tmpBuffer[num].color[2] = _tmp.color[2];
            tmpBuffer[num].color[3] = _tmp.color[3];

            num++;


        }




    for(int i=0;i<num;i++)
    {
        f->write((const char *)tmpBuffer[i].point,3*sizeof(float));
        float color = tmpBuffer[i].color[0] / 255.0;
        f->write((const char *)&color,sizeof(float));
    }
    //	*f << tmpBuffer[i].point[0] << " " << tmpBuffer[i].point[1] << " " << tmpBuffer[i].point[2] << " " << (tmpBuffer[i].color[0] / 255.0) << "\n";

    delete tmpBuffer;

    printf("Done flushing frame %d (%d points)!\n", this->id, num);
    return num;
}



void KeyFrameDisplay::drawPC(const float pointSize, const float alpha)
{
    refreshPC();


    if(!vertexBufferIdValid)
    {
    return;
    }

    GLfloat LightColor[] = {1, 1, 1, 1};
    if(alpha < 1)
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        LightColor[0] = LightColor[1] = 0;
        glEnable(GL_LIGHTING);
        glDisable(GL_LIGHT1);

        glLightfv (GL_LIGHT0, GL_AMBIENT, LightColor);
    }
    else
    {
        glDisable(GL_LIGHTING);
    }


    glPushMatrix();

    Sophus::Matrix4f m = camToWorld.matrix();
    glMultMatrixf((GLfloat*)m.data());

    glPointSize(pointSize);

    if(drawPointcloud)
        glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);
    else
        glBindBuffer(GL_ARRAY_BUFFER, vertexMeshBufferId);

    glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), 0);
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MyVertex), (const void*) (3*sizeof(float)));

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    if(drawPointcloud) //draw pointcloud
        glDrawArrays(GL_POINTS, 0, vertexBufferNumPoints);
    else //draw mesh
        glDrawElements(GL_TRIANGLES, nindex, GL_UNSIGNED_INT,originalMeshIndex);
    
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glPopMatrix();

    if(alpha < 1)
    {
        glDisable(GL_BLEND);
        glDisable(GL_LIGHTING);
        LightColor[2] = LightColor[1] = LightColor[0] = 1;
        glLightfv (GL_LIGHT0, GL_AMBIENT_AND_DIFFUSE, LightColor);
    }
}

