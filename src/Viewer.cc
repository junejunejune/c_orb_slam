/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

void Viewer_init(Viewer* pViewer, System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath)
{    
    pViewer->mpSystem=pSystem; 
    pViewer->mpFrameDrawer=pFrameDrawer;
    pViewer->mpMapDrawer=pMapDrawer; 
    pViewer->mpTracker=pTracking;
    pViewer->mbFinishRequested=false; 
    pViewer->mbFinished=true; 
    pViewer->mbStopped=true;
    pViewer->mbStopRequested=false;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    pViewer->mT = 1e3/fps;

    pViewer->mImageWidth = fSettings["Camera.width"];
    pViewer->mImageHeight = fSettings["Camera.height"];
    if(pViewer->mImageWidth<1 || pViewer->mImageHeight<1)
    {
        pViewer->mImageWidth = 640;
        pViewer->mImageHeight = 480;
    }

    pViewer->mViewpointX = fSettings["Viewer.ViewpointX"];
    pViewer->mViewpointY = fSettings["Viewer.ViewpointY"];
    pViewer->mViewpointZ = fSettings["Viewer.ViewpointZ"];
    pViewer->mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer_Run(Viewer* pViewer)
{
    pViewer->mbFinished = false;
    pViewer->mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,pViewer->mViewpointF,pViewer->mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(pViewer->mViewpointX,pViewer->mViewpointY,pViewer->mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        MapDrawer_GetCurrentOpenGLCameraMatrix(pViewer->mpMapDrawer,Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(pViewer->mViewpointX,pViewer->mViewpointY,pViewer->mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            System_ActivateLocalizationMode(pViewer->mpSystem);
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            System_DeactivateLocalizationMode(pViewer->mpSystem);
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        MapDrawer_DrawCurrentCamera(pViewer->mpMapDrawer,Twc);
        if(menuShowKeyFrames || menuShowGraph)
            MapDrawer_DrawKeyFrames(pViewer->mpMapDrawer,menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            MapDrawer_DrawMapPoints(pViewer->mpMapDrawer);

        pangolin::FinishFrame();

        cv::Mat im = FrameDrawer_DrawFrame(pViewer->mpFrameDrawer);
        cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::waitKey(pViewer->mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                System_DeactivateLocalizationMode(pViewer->mpSystem);
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            System_Reset(pViewer->mpSystem);
            menuReset = false;
        }

        if(Viewer_Stop(pViewer))
        {
            while(Viewer_isStopped(pViewer))
            {
                usleep(3000);
            }
        }

        if(Viewer_CheckFinish(pViewer))
            break;
    }

    Viewer_SetFinish(pViewer);
}

void Viewer_RequestFinish(Viewer* pViewer)
{
    unique_lock<mutex> lock(pViewer->mMutexFinish);
    pViewer->mbFinishRequested = true;
}

bool Viewer_CheckFinish(Viewer* pViewer)
{
    unique_lock<mutex> lock(pViewer->mMutexFinish);
    return pViewer->mbFinishRequested;
}

void Viewer_SetFinish(Viewer* pViewer)
{
    unique_lock<mutex> lock(pViewer->mMutexFinish);
    pViewer->mbFinished = true;
}

bool Viewer_isFinished(Viewer* pViewer)
{
    unique_lock<mutex> lock(pViewer->mMutexFinish);
    return pViewer->mbFinished;
}

void Viewer_RequestStop(Viewer* pViewer)
{
    unique_lock<mutex> lock(pViewer->mMutexStop);
    if(!pViewer->mbStopped)
        pViewer->mbStopRequested = true;
}

bool Viewer_isStopped(Viewer* pViewer)
{
    unique_lock<mutex> lock(pViewer->mMutexStop);
    return pViewer->mbStopped;
}

bool Viewer_Stop(Viewer* pViewer)
{
    unique_lock<mutex> lock(pViewer->mMutexStop);
    unique_lock<mutex> lock2(pViewer->mMutexFinish);

    if(pViewer->mbFinishRequested)
        return false;
    else if(pViewer->mbStopRequested)
    {
        pViewer->mbStopped = true;
        pViewer->mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer_Release(Viewer* pViewer)
{
    unique_lock<mutex> lock(pViewer->mMutexStop);
    pViewer->mbStopped = false;
}

}
