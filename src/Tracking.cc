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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

void Tracking_init(Tracking* pTr,System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor)
{  
    pTr->mState=Tracking::NO_IMAGES_YET; 
    pTr->mSensor=sensor;
    pTr->mbOnlyTracking=false; 
    pTr->mbVO=false; 
    pTr->mpORBVocabulary=pVoc;
    pTr->mpKeyFrameDB=pKFDB; 
    pTr->mpInitializer=static_cast<Initializer*>(NULL); 
    pTr->mpSystem=pSys; 
    pTr->mpViewer=NULL;
    pTr->mpFrameDrawer=pFrameDrawer; 
    pTr->mpMapDrawer=pMapDrawer; 
    pTr->mpMap=pMap; 
    pTr->mnLastRelocFrameId=0;

    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(pTr->mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(pTr->mDistCoef);

    pTr->mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    pTr->mMinFrames = 0;
    pTr->mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    pTr->mbRGB = nRGB;

    if(pTr->mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    pTr->mpORBextractorLeft = new ORBextractor;
    ORBextractor_init(pTr->mpORBextractorLeft,nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
    {    pTr->mpORBextractorRight = new ORBextractor;
         ORBextractor_init(pTr->mpORBextractorRight,nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    
    }
    if(sensor==System::MONOCULAR)
    {    pTr->mpIniORBextractor = new ORBextractor;
         ORBextractor_init(pTr->mpIniORBextractor, 2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
   
    }

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        pTr->mThDepth = pTr->mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << pTr->mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        pTr->mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(pTr->mDepthMapFactor)<1e-5)
            pTr->mDepthMapFactor=1;
        else
            pTr->mDepthMapFactor = 1.0f/pTr->mDepthMapFactor;
    }
}

void Tracking_SetLocalMapper(Tracking* pTr,LocalMapping *pLocalMapper)
{
    pTr->mpLocalMapper=pLocalMapper;
}

void Tracking_SetLoopClosing(Tracking* pTr,LoopClosing *pLoopClosing)
{
    pTr->mpLoopClosing=pLoopClosing;
}

void Tracking_SetViewer(Tracking* pTr,Viewer *pViewer)
{
    pTr->mpViewer=pViewer;
}


cv::Mat Tracking_GrabImageStereo(Tracking* pTr,const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    pTr->mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(pTr->mImGray.channels()==3)
    {
        if(pTr->mbRGB)
        {
            cvtColor(pTr->mImGray,pTr->mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(pTr->mImGray,pTr->mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(pTr->mImGray.channels()==4)
    {
        if(pTr->mbRGB)
        {
            cvtColor(pTr->mImGray,pTr->mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(pTr->mImGray,pTr->mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    pTr->mCurrentFrame=new Frame();
    Frame_init_2(pTr->mCurrentFrame,pTr->mImGray,imGrayRight,timestamp,pTr->mpORBextractorLeft,pTr->mpORBextractorRight,pTr->mpORBVocabulary,pTr->mK,pTr->mDistCoef,pTr->mbf,pTr->mThDepth);

    Tracking_Track(pTr);

    return pTr->mCurrentFrame->mTcw.clone();
}


cv::Mat Tracking_GrabImageRGBD(Tracking* pTr,const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    pTr->mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(pTr->mImGray.channels()==3)
    {
        if(pTr->mbRGB)
            cvtColor(pTr->mImGray,pTr->mImGray,CV_RGB2GRAY);
        else
            cvtColor(pTr->mImGray,pTr->mImGray,CV_BGR2GRAY);
    }
    else if(pTr->mImGray.channels()==4)
    {
        if(pTr->mbRGB)
            cvtColor(pTr->mImGray,pTr->mImGray,CV_RGBA2GRAY);
        else
            cvtColor(pTr->mImGray,pTr->mImGray,CV_BGRA2GRAY);
    }

    if((fabs(pTr->mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,pTr->mDepthMapFactor);

    pTr->mCurrentFrame=new Frame();
    Frame_init_3(pTr->mCurrentFrame,pTr->mImGray,imDepth,timestamp,pTr->mpORBextractorLeft,pTr->mpORBVocabulary,pTr->mK,pTr->mDistCoef,pTr->mbf,pTr->mThDepth);

    Tracking_Track(pTr);

    return pTr->mCurrentFrame->mTcw.clone();
}


cv::Mat Tracking_GrabImageMonocular(Tracking* pTr,const cv::Mat &im, const double &timestamp)
{
    pTr->mImGray = im;

    if(pTr->mImGray.channels()==3)
    {
        if(pTr->mbRGB)
            cvtColor(pTr->mImGray,pTr->mImGray,CV_RGB2GRAY);
        else
            cvtColor(pTr->mImGray,pTr->mImGray,CV_BGR2GRAY);
    }
    else if(pTr->mImGray.channels()==4)
    {
        if(pTr->mbRGB)
            cvtColor(pTr->mImGray,pTr->mImGray,CV_RGBA2GRAY);
        else
            cvtColor(pTr->mImGray,pTr->mImGray,CV_BGRA2GRAY);
    }

    if(pTr->mState==Tracking::NOT_INITIALIZED || pTr->mState==Tracking::NO_IMAGES_YET)
    {
        pTr->mCurrentFrame=new Frame();
        Frame_init_4(pTr->mCurrentFrame,pTr->mImGray,timestamp,pTr->mpIniORBextractor,pTr->mpORBVocabulary,pTr->mK,pTr->mDistCoef,pTr->mbf,pTr->mThDepth);
    }
    else
    {
        pTr->mCurrentFrame=new Frame();
        Frame_init_4(pTr->mCurrentFrame,pTr->mImGray,timestamp,pTr->mpORBextractorLeft,pTr->mpORBVocabulary,pTr->mK,pTr->mDistCoef,pTr->mbf,pTr->mThDepth);
    }
    Tracking_Track(pTr);

    return pTr->mCurrentFrame->mTcw.clone();
}

void Tracking_Track(Tracking* pTr)
{
    if(pTr->mState==Tracking::NO_IMAGES_YET)
    {
        pTr->mState = Tracking::NOT_INITIALIZED;
    }

    pTr->mLastProcessedState=pTr->mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pTr->mpMap->mMutexMapUpdate);

    if(pTr->mState==Tracking::NOT_INITIALIZED)
    {
        if(pTr->mSensor==System::STEREO || pTr->mSensor==System::RGBD)
            Tracking_StereoInitialization(pTr);
        else
            Tracking_MonocularInitialization(pTr);

        FrameDrawer_Update(pTr->mpFrameDrawer,pTr);

        if(pTr->mState!=Tracking::OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!pTr->mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(pTr->mState==Tracking::OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                Tracking_CheckReplacedInLastFrame(pTr);

                if(pTr->mVelocity.empty() || pTr->mCurrentFrame->mnId<pTr->mnLastRelocFrameId+2)
                {
                    bOK = Tracking_TrackReferenceKeyFrame(pTr);
                }
                else
                {
                    bOK = Tracking_TrackWithMotionModel(pTr);
                    if(!bOK)
                        bOK = Tracking_TrackReferenceKeyFrame(pTr);
                }
            }
            else
            {
                bOK = Tracking_Relocalization(pTr);
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(pTr->mState==Tracking::LOST)
            {
                bOK = Tracking_Relocalization(pTr);
            }
            else
            {
                if(!pTr->mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!pTr->mVelocity.empty())
                    {
                        bOK = Tracking_TrackWithMotionModel(pTr);
                    }
                    else
                    {
                        bOK = Tracking_TrackReferenceKeyFrame(pTr);
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!pTr->mVelocity.empty())
                    {
                        bOKMM = Tracking_TrackWithMotionModel(pTr);
                        vpMPsMM = pTr->mCurrentFrame->mvpMapPoints;
                        vbOutMM = pTr->mCurrentFrame->mvbOutlier;
                        TcwMM = pTr->mCurrentFrame->mTcw.clone();
                    }
                    bOKReloc = Tracking_Relocalization(pTr);

                    if(bOKMM && !bOKReloc)
                    {
                        Frame_SetPose(pTr->mCurrentFrame,TcwMM);
                        pTr->mCurrentFrame->mvpMapPoints = vpMPsMM;
                        pTr->mCurrentFrame->mvbOutlier = vbOutMM;

                        if(pTr->mbVO)
                        {
                            for(int i =0; i<pTr->mCurrentFrame->N; i++)
                            {
                                if(pTr->mCurrentFrame->mvpMapPoints[i] && !pTr->mCurrentFrame->mvbOutlier[i])
                                {
                                    MapPoint_IncreaseFound(pTr->mCurrentFrame->mvpMapPoints[i]);
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        pTr->mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        pTr->mCurrentFrame->mpReferenceKF = pTr->mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!pTr->mbOnlyTracking)
        {
            if(bOK)
                bOK = Tracking_TrackLocalMap(pTr);
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !pTr->mbVO)
                bOK = Tracking_TrackLocalMap(pTr);
        }

        if(bOK)
            pTr->mState = Tracking::OK;
        else
            pTr->mState=Tracking::LOST;

        // Update drawer
        FrameDrawer_Update(pTr->mpFrameDrawer, pTr);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!pTr->mLastFrame->mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                Frame_GetRotationInverse(pTr->mLastFrame).copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                Frame_GetCameraCenter(pTr->mLastFrame).copyTo(LastTwc.rowRange(0,3).col(3));
                pTr->mVelocity = pTr->mCurrentFrame->mTcw*LastTwc;
            }
            else
                pTr->mVelocity = cv::Mat();

            MapDrawer_SetCurrentCameraPose(pTr->mpMapDrawer,pTr->mCurrentFrame->mTcw);

            // Clean VO matches
            for(int i=0; i<pTr->mCurrentFrame->N; i++)
            {
                MapPoint* pMP = pTr->mCurrentFrame->mvpMapPoints[i];
                if(pMP)
                    if(MapPoint_Observations(pMP)<1)
                    {
                        pTr->mCurrentFrame->mvbOutlier[i] = false;
                        pTr->mCurrentFrame->mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = pTr->mlpTemporalPoints.begin(), lend =  pTr->mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            pTr->mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(Tracking_NeedNewKeyFrame(pTr))
                Tracking_CreateNewKeyFrame(pTr);

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<pTr->mCurrentFrame->N;i++)
            {
                if(pTr->mCurrentFrame->mvpMapPoints[i] && pTr->mCurrentFrame->mvbOutlier[i])
                    pTr->mCurrentFrame->mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(pTr->mState==Tracking::LOST)
        {
            if(Map_KeyFramesInMap(pTr->mpMap)<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                System_Reset(pTr->mpSystem);
                return;
            }
        }

        if(!pTr->mCurrentFrame->mpReferenceKF)
            pTr->mCurrentFrame->mpReferenceKF = pTr->mpReferenceKF;

        pTr->mLastFrame=new Frame();
        Frame_init_1(pTr->mLastFrame,pTr->mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!pTr->mCurrentFrame->mTcw.empty())
    {
        cv::Mat Tcr = pTr->mCurrentFrame->mTcw*KeyFrame_GetPoseInverse(pTr->mCurrentFrame->mpReferenceKF);
        pTr->mlRelativeFramePoses.push_back(Tcr);
        pTr->mlpReferences.push_back(pTr->mpReferenceKF);
        pTr->mlFrameTimes.push_back(pTr->mCurrentFrame->mTimeStamp);
        pTr->mlbLost.push_back(pTr->mState==Tracking::LOST);
    }
    else
    {
        // This can happen if tracking is lost
        pTr->mlRelativeFramePoses.push_back(pTr->mlRelativeFramePoses.back());
        pTr->mlpReferences.push_back(pTr->mlpReferences.back());
        pTr->mlFrameTimes.push_back(pTr->mlFrameTimes.back());
        pTr->mlbLost.push_back(pTr->mState==Tracking::LOST);
    }
}


void Tracking_StereoInitialization(Tracking* pTr)
{
    if(pTr->mCurrentFrame->N>500)
    {
        // Set Frame pose to the origin
        Frame_SetPose(pTr->mCurrentFrame,cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        struct KeyFrame* pKFini = new struct KeyFrame();
        KeyFrame_init(pKFini, pTr->mCurrentFrame, pTr->mpMap, pTr->mpKeyFrameDB);

        // Insert KeyFrame in the map
        Map_AddKeyFrame(pTr->mpMap,pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<pTr->mCurrentFrame->N;i++)
        {
            float z = pTr->mCurrentFrame->mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = Frame_UnprojectStereo(pTr->mCurrentFrame,i);
                MapPoint* pNewMP = new MapPoint;
                MapPoint_init_1(pNewMP, x3D,pKFini,pTr->mpMap);
                MapPoint_AddObservation(pNewMP,pKFini,i);
                KeyFrame_AddMapPoint(pKFini, pNewMP,i);
                MapPoint_ComputeDistinctiveDescriptors(pNewMP);
                MapPoint_UpdateNormalAndDepth(pNewMP);
                Map_AddMapPoint(pTr->mpMap, pNewMP);

                pTr->mCurrentFrame->mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << Map_MapPointsInMap(pTr->mpMap) << " points" << endl;

        LocalMapping_InsertKeyFrame(pTr->mpLocalMapper,pKFini);

        pTr->mLastFrame=new Frame();
        Frame_init_1(pTr->mLastFrame,pTr->mCurrentFrame);
        pTr->mnLastKeyFrameId=pTr->mCurrentFrame->mnId;
        pTr->mpLastKeyFrame = pKFini;

        pTr->mvpLocalKeyFrames.push_back(pKFini);
        pTr->mvpLocalMapPoints=Map_GetAllMapPoints(pTr->mpMap);
        pTr->mpReferenceKF = pKFini;
        pTr->mCurrentFrame->mpReferenceKF = pKFini;

        Map_SetReferenceMapPoints(pTr->mpMap,pTr->mvpLocalMapPoints);

        pTr->mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        MapDrawer_SetCurrentCameraPose(pTr->mpMapDrawer,pTr->mCurrentFrame->mTcw);

        pTr->mState=Tracking::OK;
    }
}

void Tracking_MonocularInitialization(Tracking* pTr)
{

    if(!pTr->mpInitializer)
    {
        // Set Reference Frame
        if(pTr->mCurrentFrame->mvKeys.size()>100)
        {
            pTr->mInitialFrame=new Frame();
            pTr->mLastFrame=new Frame();
            Frame_init_1(pTr->mInitialFrame,pTr->mCurrentFrame);
            Frame_init_1(pTr->mLastFrame,pTr->mCurrentFrame);
            pTr->mvbPrevMatched.resize(pTr->mCurrentFrame->mvKeysUn.size());
            for(size_t i=0; i<pTr->mCurrentFrame->mvKeysUn.size(); i++)
                pTr->mvbPrevMatched[i]=pTr->mCurrentFrame->mvKeysUn[i].pt;

            if(pTr->mpInitializer)
                delete pTr->mpInitializer;

            pTr->mpInitializer= new Initializer;
            Initializer_init(pTr->mpInitializer, pTr->mCurrentFrame,1.0,200);

            fill(pTr->mvIniMatches.begin(),pTr->mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)pTr->mCurrentFrame->mvKeys.size()<=100)
        {
            delete pTr->mpInitializer;
            pTr->mpInitializer = static_cast<Initializer*>(NULL);
            fill(pTr->mvIniMatches.begin(),pTr->mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher* matcher = new ORBmatcher();
        ORBmatcher_init(matcher, 0.9, true);

        int nmatches = ORBmatcher_SearchForInitialization(matcher, pTr->mInitialFrame,pTr->mCurrentFrame,pTr->mvbPrevMatched,pTr->mvIniMatches,100);
        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete pTr->mpInitializer;
            pTr->mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(Initializer_Initialize(pTr->mpInitializer,pTr->mCurrentFrame, pTr->mvIniMatches, Rcw, tcw, pTr->mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=pTr->mvIniMatches.size(); i<iend;i++)
            {
                if(pTr->mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    pTr->mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            Frame_SetPose(pTr->mInitialFrame,cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            Frame_SetPose(pTr->mCurrentFrame,Tcw);

            Tracking_CreateInitialMapMonocular(pTr);
        }
    }
}

void Tracking_CreateInitialMapMonocular(Tracking* pTr)
{
    // Create KeyFrames
    struct KeyFrame* pKFini =new struct KeyFrame();
    KeyFrame_init(pKFini, pTr->mInitialFrame, pTr->mpMap, pTr->mpKeyFrameDB);

    struct KeyFrame* pKFcur =new struct KeyFrame();
    KeyFrame_init(pKFcur, pTr->mCurrentFrame, pTr->mpMap, pTr->mpKeyFrameDB);


    KeyFrame_ComputeBoW(pKFini);
    KeyFrame_ComputeBoW(pKFcur);

    // Insert KFs in the map
    Map_AddKeyFrame(pTr->mpMap,pKFini);
    Map_AddKeyFrame(pTr->mpMap,pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<pTr->mvIniMatches.size();i++)
    {
        if(pTr->mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(pTr->mvIniP3D[i]);

        MapPoint* pMP = new MapPoint;
        MapPoint_init_1(pMP,worldPos,pKFcur,pTr->mpMap);

        KeyFrame_AddMapPoint(pKFini, pMP,i);
        KeyFrame_AddMapPoint(pKFcur,pMP,pTr->mvIniMatches[i]);

        MapPoint_AddObservation(pMP, pKFini,i);
        MapPoint_AddObservation(pMP,pKFcur,pTr->mvIniMatches[i]);

        MapPoint_ComputeDistinctiveDescriptors(pMP);
        MapPoint_UpdateNormalAndDepth(pMP);

        //Fill Current Frame structure
        pTr->mCurrentFrame->mvpMapPoints[pTr->mvIniMatches[i]] = pMP;
        pTr->mCurrentFrame->mvbOutlier[pTr->mvIniMatches[i]] = false;

        //Add to Map
        Map_AddMapPoint(pTr->mpMap,pMP);
    }

    // Update Connections
    KeyFrame_UpdateConnections(pKFini);
    KeyFrame_UpdateConnections(pKFcur);

    // Bundle Adjustment
    cout << "New Map created with " << Map_MapPointsInMap(pTr->mpMap) << " points" << endl;

    Optimizer_GlobalBundleAdjustemnt(pTr->mpMap,20);

    // Set median depth to 1
    float medianDepth = KeyFrame_ComputeSceneMedianDepth(pKFini, 2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || KeyFrame_TrackedMapPoints(pKFcur, 1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        cout<<"medianDepth: "<<medianDepth<<endl;
        Tracking_Reset(pTr);
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = KeyFrame_GetPose(pKFcur);
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    KeyFrame_SetPose(pKFcur, Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = KeyFrame_GetMapPointMatches(pKFini);
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            MapPoint_SetWorldPos(pMP, MapPoint_GetWorldPos(pMP)*invMedianDepth);
        }
    }

    LocalMapping_InsertKeyFrame(pTr->mpLocalMapper,pKFini);
    LocalMapping_InsertKeyFrame(pTr->mpLocalMapper,pKFcur);

    Frame_SetPose(pTr->mCurrentFrame,KeyFrame_GetPose(pKFcur));
    pTr->mnLastKeyFrameId=pTr->mCurrentFrame->mnId;
    pTr->mpLastKeyFrame = pKFcur;

    pTr->mvpLocalKeyFrames.push_back(pKFcur);
    pTr->mvpLocalKeyFrames.push_back(pKFini);
    pTr->mvpLocalMapPoints=Map_GetAllMapPoints(pTr->mpMap);
    pTr->mpReferenceKF = pKFcur;
    pTr->mCurrentFrame->mpReferenceKF = pKFcur;

    pTr->mLastFrame=new Frame();
    Frame_init_1(pTr->mLastFrame,pTr->mCurrentFrame);

    Map_SetReferenceMapPoints(pTr->mpMap,pTr->mvpLocalMapPoints);

    MapDrawer_SetCurrentCameraPose(pTr->mpMapDrawer,KeyFrame_GetPose(pKFcur));

    pTr->mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    pTr->mState=Tracking::OK;
}

void Tracking_CheckReplacedInLastFrame(Tracking* pTr)
{
    for(int i =0; i<pTr->mLastFrame->N; i++)
    {
        MapPoint* pMP = pTr->mLastFrame->mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = MapPoint_GetReplaced(pMP);
            if(pRep)
            {
                pTr->mLastFrame->mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking_TrackReferenceKeyFrame(Tracking* pTr)
{
    // Compute Bag of Words vector
    Frame_ComputeBoW(pTr->mCurrentFrame);

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
     ORBmatcher* matcher = new ORBmatcher();
     ORBmatcher_init(matcher, 0.7, true);
   
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = ORBmatcher_SearchByBoW(matcher, pTr->mpReferenceKF,pTr->mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    pTr->mCurrentFrame->mvpMapPoints = vpMapPointMatches;
    Frame_SetPose(pTr->mCurrentFrame,pTr->mLastFrame->mTcw);

    Optimizer_PoseOptimization(pTr->mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<pTr->mCurrentFrame->N; i++)
    {
        if(pTr->mCurrentFrame->mvpMapPoints[i])
        {
            if(pTr->mCurrentFrame->mvbOutlier[i])
            {
                MapPoint* pMP = pTr->mCurrentFrame->mvpMapPoints[i];

                pTr->mCurrentFrame->mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                pTr->mCurrentFrame->mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = pTr->mCurrentFrame->mnId;
                nmatches--;
            }
            else if(MapPoint_Observations(pTr->mCurrentFrame->mvpMapPoints[i])>0)
                nmatchesMap++;
        }
    }
    return nmatchesMap>=10;
}

void Tracking_UpdateLastFrame(Tracking* pTr)
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = pTr->mLastFrame->mpReferenceKF;
    cv::Mat Tlr = pTr->mlRelativeFramePoses.back();

    Frame_SetPose(pTr->mLastFrame,Tlr*KeyFrame_GetPose(pRef));

    if(pTr->mnLastKeyFrameId==pTr->mLastFrame->mnId || pTr->mSensor==System::MONOCULAR || !pTr->mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(pTr->mLastFrame->N);
    for(int i=0; i<pTr->mLastFrame->N;i++)
    {
        float z = pTr->mLastFrame->mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = pTr->mLastFrame->mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(MapPoint_Observations(pMP)<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = Frame_UnprojectStereo(pTr->mLastFrame,i);
            MapPoint* pNewMP = new MapPoint;
            MapPoint_init_2(pNewMP, x3D,pTr->mpMap,pTr->mLastFrame,i);

            pTr->mLastFrame->mvpMapPoints[i]=pNewMP;

            pTr->mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>pTr->mThDepth && nPoints>100)
            break;
    }
}

bool Tracking_TrackWithMotionModel(Tracking* pTr)
{
    ORBmatcher* matcher=new ORBmatcher();
    ORBmatcher_init(matcher, 0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    Tracking_UpdateLastFrame(pTr);

    Frame_SetPose(pTr->mCurrentFrame,pTr->mVelocity*pTr->mLastFrame->mTcw);

    fill(pTr->mCurrentFrame->mvpMapPoints.begin(),pTr->mCurrentFrame->mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(pTr->mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = ORBmatcher_SearchByProjection(matcher, pTr->mCurrentFrame,pTr->mLastFrame,th,pTr->mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(pTr->mCurrentFrame->mvpMapPoints.begin(),pTr->mCurrentFrame->mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = ORBmatcher_SearchByProjection(matcher, pTr->mCurrentFrame,pTr->mLastFrame,2*th,pTr->mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer_PoseOptimization(pTr->mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<pTr->mCurrentFrame->N; i++)
    {
        if(pTr->mCurrentFrame->mvpMapPoints[i])
        {
            if(pTr->mCurrentFrame->mvbOutlier[i])
            {
                MapPoint* pMP = pTr->mCurrentFrame->mvpMapPoints[i];

                pTr->mCurrentFrame->mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                pTr->mCurrentFrame->mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = pTr->mCurrentFrame->mnId;
                nmatches--;
            }
            else if(MapPoint_Observations(pTr->mCurrentFrame->mvpMapPoints[i])>0)
                nmatchesMap++;
        }
    }    

    if(pTr->mbOnlyTracking)
    {
        pTr->mbVO = nmatchesMap<10;
        return nmatches>20;
    }
    return nmatchesMap>=10;
}

bool Tracking_TrackLocalMap(Tracking* pTr)
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    Tracking_UpdateLocalMap(pTr);

    Tracking_SearchLocalPoints(pTr);

    // Optimize Pose
    Optimizer_PoseOptimization(pTr->mCurrentFrame);
    pTr->mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<pTr->mCurrentFrame->N; i++)
    {
        if(pTr->mCurrentFrame->mvpMapPoints[i])
        {
            if(!pTr->mCurrentFrame->mvbOutlier[i])
            {
                MapPoint_IncreaseFound(pTr->mCurrentFrame->mvpMapPoints[i]);
                if(!pTr->mbOnlyTracking)
                {
                    if(MapPoint_Observations(pTr->mCurrentFrame->mvpMapPoints[i])>0)
                        pTr->mnMatchesInliers++;
                }
                else
                    pTr->mnMatchesInliers++;
            }
            else if(pTr->mSensor==System::STEREO)
                pTr->mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(pTr->mCurrentFrame->mnId<pTr->mnLastRelocFrameId+pTr->mMaxFrames && pTr->mnMatchesInliers<50)
        return false;

    if(pTr->mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking_NeedNewKeyFrame(Tracking* pTr)
{
    if(pTr->mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(LocalMapping_isStopped(pTr->mpLocalMapper) || LocalMapping_stopRequested(pTr->mpLocalMapper))
        return false;

    const int nKFs = Map_KeyFramesInMap(pTr->mpMap);

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(pTr->mCurrentFrame->mnId<pTr->mnLastRelocFrameId+pTr->mMaxFrames && nKFs>pTr->mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = KeyFrame_TrackedMapPoints(pTr->mpReferenceKF, nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle =LocalMapping_AcceptKeyFrames( pTr->mpLocalMapper);

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(pTr->mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<pTr->mCurrentFrame->N; i++)
        {
            if(pTr->mCurrentFrame->mvDepth[i]>0 && pTr->mCurrentFrame->mvDepth[i]<pTr->mThDepth)
            {
                if(pTr->mCurrentFrame->mvpMapPoints[i] && !pTr->mCurrentFrame->mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(pTr->mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = pTr->mCurrentFrame->mnId>=pTr->mnLastKeyFrameId+pTr->mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (pTr->mCurrentFrame->mnId>=pTr->mnLastKeyFrameId+pTr->mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  pTr->mSensor!=System::MONOCULAR && (pTr->mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((pTr->mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && pTr->mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            LocalMapping_InterruptBA(pTr->mpLocalMapper);
            if(pTr->mSensor!=System::MONOCULAR)
            {
                if(LocalMapping_KeyframesInQueue(pTr->mpLocalMapper)<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking_CreateNewKeyFrame(Tracking* pTr)
{
    if(!LocalMapping_SetNotStop(pTr->mpLocalMapper,true))
        return;

    struct KeyFrame *pKF = new struct KeyFrame();
    KeyFrame_init(pKF, pTr->mCurrentFrame, pTr->mpMap, pTr->mpKeyFrameDB);

    pTr->mpReferenceKF = pKF;
    pTr->mCurrentFrame->mpReferenceKF = pKF;

    if(pTr->mSensor!=System::MONOCULAR)
    {
        Frame_UpdatePoseMatrices(pTr->mCurrentFrame);

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(pTr->mCurrentFrame->N);
        for(int i=0; i<pTr->mCurrentFrame->N; i++)
        {
            float z = pTr->mCurrentFrame->mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = pTr->mCurrentFrame->mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(MapPoint_Observations(pMP)<1)
                {
                    bCreateNew = true;
                    pTr->mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = Frame_UnprojectStereo(pTr->mCurrentFrame,i);
                    MapPoint* pNewMP = new MapPoint;
                    MapPoint_init_1(pNewMP,x3D,pKF,pTr->mpMap);
                    MapPoint_AddObservation(pNewMP,pKF,i);
                    KeyFrame_AddMapPoint(pKF,pNewMP,i);
                    MapPoint_ComputeDistinctiveDescriptors(pNewMP);
                    MapPoint_UpdateNormalAndDepth(pNewMP);
                    Map_AddMapPoint(pTr->mpMap,pNewMP);

                    pTr->mCurrentFrame->mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>pTr->mThDepth && nPoints>100)
                    break;
            }
        }
    }

    LocalMapping_InsertKeyFrame(pTr->mpLocalMapper,pKF);

    LocalMapping_SetNotStop(pTr->mpLocalMapper,false);

    pTr->mnLastKeyFrameId = pTr->mCurrentFrame->mnId;
    pTr->mpLastKeyFrame = pKF;
}

void Tracking_SearchLocalPoints(Tracking* pTr)
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=pTr->mCurrentFrame->mvpMapPoints.begin(), vend=pTr->mCurrentFrame->mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(MapPoint_isBad(pMP))
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                MapPoint_IncreaseVisible(pMP);
                pMP->mnLastFrameSeen = pTr->mCurrentFrame->mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=pTr->mvpLocalMapPoints.begin(), vend=pTr->mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == pTr->mCurrentFrame->mnId)
            continue;
        if(MapPoint_isBad(pMP))
            continue;
        // Project (this fills MapPoint variables for matching)
        if(Frame_isInFrustum(pTr->mCurrentFrame,pMP,0.5))
        {
            MapPoint_IncreaseVisible(pMP);
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher* matcher= new ORBmatcher();
        ORBmatcher_init(matcher, 0.8, true);

        int th = 1;
        if(pTr->mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(pTr->mCurrentFrame->mnId<pTr->mnLastRelocFrameId+2)
            th=5;
        ORBmatcher_SearchByProjection(matcher, pTr->mCurrentFrame,pTr->mvpLocalMapPoints,th);
    }
}

void Tracking_UpdateLocalMap(Tracking* pTr)
{
    // This is for visualization
    Map_SetReferenceMapPoints(pTr->mpMap,pTr->mvpLocalMapPoints);

    // Update
    Tracking_UpdateLocalKeyFrames(pTr);
    Tracking_UpdateLocalPoints(pTr);
}

void Tracking_UpdateLocalPoints(Tracking* pTr)
{
    pTr->mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=pTr->mvpLocalKeyFrames.begin(), itEndKF=pTr->mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = KeyFrame_GetMapPointMatches(pKF);

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==pTr->mCurrentFrame->mnId)
                continue;
            if(!MapPoint_isBad(pMP))
            {
                pTr->mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=pTr->mCurrentFrame->mnId;
            }
        }
    }
}


void Tracking_UpdateLocalKeyFrames(Tracking* pTr)
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<pTr->mCurrentFrame->N; i++)
    {
        if(pTr->mCurrentFrame->mvpMapPoints[i])
        {
            MapPoint* pMP = pTr->mCurrentFrame->mvpMapPoints[i];
            if(!MapPoint_isBad(pMP))
            {
                const map<KeyFrame*,size_t> observations = MapPoint_GetObservations(pMP);
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                pTr->mCurrentFrame->mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    pTr->mvpLocalKeyFrames.clear();
    pTr->mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(KeyFrame_isBad(pKF))
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        pTr->mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = pTr->mCurrentFrame->mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=pTr->mvpLocalKeyFrames.begin(), itEndKF=pTr->mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(pTr->mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = KeyFrame_GetBestCovisibilityKeyFrames(pKF,10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!KeyFrame_isBad(pNeighKF))
            {
                if(pNeighKF->mnTrackReferenceForFrame!=pTr->mCurrentFrame->mnId)
                {
                    pTr->mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=pTr->mCurrentFrame->mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = KeyFrame_GetChilds(pKF);
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!KeyFrame_isBad(pChildKF))
            {
                if(pChildKF->mnTrackReferenceForFrame!=pTr->mCurrentFrame->mnId)
                {
                    pTr->mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=pTr->mCurrentFrame->mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = KeyFrame_GetParent(pKF);
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=pTr->mCurrentFrame->mnId)
            {
                pTr->mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=pTr->mCurrentFrame->mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        pTr->mpReferenceKF = pKFmax;
        pTr->mCurrentFrame->mpReferenceKF = pTr->mpReferenceKF;
    }
}

bool Tracking_Relocalization(Tracking* pTr)
{
    // Compute Bag of Words Vector
    Frame_ComputeBoW(pTr->mCurrentFrame);

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = KeyFrameDatabase_DetectRelocalizationCandidates(pTr->mpKeyFrameDB,pTr->mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher* matcher= new ORBmatcher();
    ORBmatcher_init(matcher,0.75, true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(KeyFrame_isBad(pKF))
            vbDiscarded[i] = true;
        else
        {
            int nmatches = ORBmatcher_SearchByBoW(matcher, pKF,pTr->mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver;
                PnPsolver_init(pSolver, pTr->mCurrentFrame,vvpMapPointMatches[i]);
                PnPsolver_SetRansacParameters(pSolver,0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;

                PnPsolver_destruct(pSolver);  
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher* matcher2 = new ORBmatcher();
    ORBmatcher_init(matcher2, 0.9, true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = PnPsolver_iterate(pSolver,5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(pTr->mCurrentFrame->mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        pTr->mCurrentFrame->mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        pTr->mCurrentFrame->mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer_PoseOptimization(pTr->mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<pTr->mCurrentFrame->N; io++)
                    if(pTr->mCurrentFrame->mvbOutlier[io])
                        pTr->mCurrentFrame->mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =ORBmatcher_SearchByProjection(matcher2, pTr->mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer_PoseOptimization(pTr->mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<pTr->mCurrentFrame->N; ip++)
                                if(pTr->mCurrentFrame->mvpMapPoints[ip])
                                    sFound.insert(pTr->mCurrentFrame->mvpMapPoints[ip]);
                            nadditional =ORBmatcher_SearchByProjection(matcher2,pTr->mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer_PoseOptimization(pTr->mCurrentFrame);

                                for(int io =0; io<pTr->mCurrentFrame->N; io++)
                                    if(pTr->mCurrentFrame->mvbOutlier[io])
                                        pTr->mCurrentFrame->mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }

                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        pTr->mnLastRelocFrameId = pTr->mCurrentFrame->mnId;
        return true;
    }

}

void Tracking_Reset(Tracking* pTr)
{

    cout << "System Reseting" << endl;
    if(pTr->mpViewer)
    {
       // Viewer_RequestStop(mpViewer);
       // while(!Viewer_isStopped(mpViewer))
        Viewer_RequestStop(pTr->mpViewer);
        while(!Viewer_isStopped(pTr->mpViewer))
          usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    LocalMapping_RequestReset(pTr->mpLocalMapper);
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    LoopClosing_RequestReset(pTr->mpLoopClosing);
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    KeyFrameDatabase_clear(pTr->mpKeyFrameDB);
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    Map_clear(pTr->mpMap);

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    pTr->mState = Tracking::NO_IMAGES_YET;

    if(pTr->mpInitializer)
    {
        delete pTr->mpInitializer;
        pTr->mpInitializer = static_cast<Initializer*>(NULL);
    }

    pTr->mlRelativeFramePoses.clear();
    pTr->mlpReferences.clear();
    pTr->mlFrameTimes.clear();
    pTr->mlbLost.clear();

    if(pTr->mpViewer)
      Viewer_Release(pTr->mpViewer);
}

void Tracking_ChangeCalibration(Tracking* pTr,const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(pTr->mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(pTr->mDistCoef);

    pTr->mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking_InformOnlyTracking(Tracking* pTr,const bool &flag)
{
    pTr->mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
