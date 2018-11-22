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


#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{

struct FrameDrawer sFD;
struct MapDrawer sMD;

void System_init(System *pSystem, const string &strVocFile, const string &strSettingsFile, const System::eSensor sensor,
               const bool bUseViewer)
{
  
    pSystem->mSensor=sensor;
    pSystem->mpViewer=static_cast<Viewer*>(NULL); 
    pSystem->mbReset=false;
    pSystem->mbActivateLocalizationMode=false;
    pSystem->mbDeactivateLocalizationMode=false;

    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(pSystem->mSensor==System::MONOCULAR)
        cout << "Monocular" << endl;
    else if(pSystem->mSensor==System::STEREO)
        cout << "Stereo" << endl;
    else if(pSystem->mSensor==System::RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    pSystem->mpVocabulary = new ORBVocabulary();
    bool bVocLoad = pSystem->mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    pSystem->mpKeyFrameDatabase = new KeyFrameDatabase;
    KeyFrameDatabase_init(pSystem->mpKeyFrameDatabase,*(pSystem->mpVocabulary));
    //Create the Map
    pSystem->mpMap = new Map();
 
    //Create Drawers. These are used by the Viewer
    pSystem->mpFrameDrawer = &sFD;
    FrameDrawer_init(pSystem->mpFrameDrawer, pSystem->mpMap);

    pSystem->mpMapDrawer = &sMD;
    MapDrawer_init(pSystem->mpMapDrawer, pSystem->mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    pSystem->mpTracker = new Tracking();
    Tracking_init(pSystem->mpTracker,pSystem, pSystem->mpVocabulary, pSystem->mpFrameDrawer, pSystem->mpMapDrawer,
                             pSystem->mpMap, pSystem->mpKeyFrameDatabase, strSettingsFile, pSystem->mSensor);

    //Initialize the Local Mapping thread and launch
    pSystem->mpLocalMapper = new LocalMapping();
    LocalMapping_init(pSystem->mpLocalMapper,pSystem->mpMap, pSystem->mSensor==System::MONOCULAR);
    pSystem->mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping_Run,pSystem->mpLocalMapper);
 
    //Initialize the Loop Closing thread and launch
    pSystem->mpLoopCloser = new LoopClosing();
    LoopClosing_init(pSystem->mpLoopCloser,pSystem->mpMap, pSystem->mpKeyFrameDatabase, pSystem->mpVocabulary, pSystem->mSensor!=System::MONOCULAR);
    pSystem->mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing_Run, pSystem->mpLoopCloser);
 
    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        pSystem->mpViewer = new Viewer;
        Viewer_init(pSystem->mpViewer,pSystem, pSystem->mpFrameDrawer,pSystem->mpMapDrawer,pSystem->mpTracker,strSettingsFile);
        
        pSystem->mptViewer = new thread(&Viewer_Run, pSystem->mpViewer);
        Tracking_SetViewer(pSystem->mpTracker,pSystem->mpViewer);
    }

   //Set pointers between threads
    Tracking_SetLocalMapper(pSystem->mpTracker,pSystem->mpLocalMapper);
    Tracking_SetLoopClosing(pSystem->mpTracker,pSystem->mpLoopCloser);

    LocalMapping_SetTracker(pSystem->mpLocalMapper,pSystem->mpTracker);
    LocalMapping_SetLoopCloser(pSystem->mpLocalMapper,pSystem->mpLoopCloser);

    LoopClosing_SetTracker(pSystem->mpLoopCloser,pSystem->mpTracker);
    LoopClosing_SetLocalMapper(pSystem->mpLoopCloser,pSystem->mpLocalMapper);
}

cv::Mat System_TrackStereo(System *pSystem,const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(pSystem->mSensor!=System::STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(pSystem->mMutexMode);
        if(pSystem->mbActivateLocalizationMode)
        {
            LocalMapping_RequestStop(pSystem->mpLocalMapper);

            // Wait until Local Mapping has effectively stopped
            while(!LocalMapping_isStopped(pSystem->mpLocalMapper))
            {
                usleep(1000);
            }

            Tracking_InformOnlyTracking(pSystem->mpTracker, true);
            pSystem->mbActivateLocalizationMode = false;
        }
        if(pSystem->mbDeactivateLocalizationMode)
        {
            Tracking_InformOnlyTracking(pSystem->mpTracker,false);
            LocalMapping_Release(pSystem->mpLocalMapper);
            pSystem->mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(pSystem->mMutexReset);
    if(pSystem->mbReset)
    {
        Tracking_Reset(pSystem->mpTracker);
        pSystem->mbReset = false;
    }
    }

    cv::Mat Tcw = Tracking_GrabImageStereo(pSystem->mpTracker,imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(pSystem->mMutexState);
    pSystem->mTrackingState = pSystem->mpTracker->mState;
    pSystem->mTrackedMapPoints = pSystem->mpTracker->mCurrentFrame->mvpMapPoints;
    pSystem->mTrackedKeyPointsUn = pSystem->mpTracker->mCurrentFrame->mvKeysUn;
    return Tcw;
}

cv::Mat System_TrackRGBD(System *pSystem,const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(pSystem->mSensor!=System::RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(pSystem->mMutexMode);
        if(pSystem->mbActivateLocalizationMode)
        {
            LocalMapping_RequestStop(pSystem->mpLocalMapper);

            // Wait until Local Mapping has effectively stopped
            while(!LocalMapping_isStopped(pSystem->mpLocalMapper))
            {
                usleep(1000);
            }

            Tracking_InformOnlyTracking(pSystem->mpTracker,true);
            pSystem->mbActivateLocalizationMode = false;
        }
        if(pSystem->mbDeactivateLocalizationMode)
        {
            Tracking_InformOnlyTracking(pSystem->mpTracker,false);
            LocalMapping_Release(pSystem->mpLocalMapper);
            pSystem->mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(pSystem->mMutexReset);
    if(pSystem->mbReset)
    {
        Tracking_Reset(pSystem->mpTracker);
        pSystem->mbReset = false;
    }
    }

    cv::Mat Tcw = Tracking_GrabImageRGBD(pSystem->mpTracker,im,depthmap,timestamp);

    unique_lock<mutex> lock2(pSystem->mMutexState);
    pSystem->mTrackingState = pSystem->mpTracker->mState;
    pSystem->mTrackedMapPoints = pSystem->mpTracker->mCurrentFrame->mvpMapPoints;
    pSystem->mTrackedKeyPointsUn = pSystem->mpTracker->mCurrentFrame->mvKeysUn;
    return Tcw;
}

cv::Mat System_TrackMonocular(System *pSystem,const cv::Mat &im, const double &timestamp)
{
    if(pSystem->mSensor!=System::MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(pSystem->mMutexMode);
        if(pSystem->mbActivateLocalizationMode)
        {
            LocalMapping_RequestStop(pSystem->mpLocalMapper);

            // Wait until Local Mapping has effectively stopped
            while(!LocalMapping_isStopped(pSystem->mpLocalMapper))
            {
                usleep(1000);
            }

            Tracking_InformOnlyTracking(pSystem->mpTracker,true);
            pSystem->mbActivateLocalizationMode = false;
        }
        if(pSystem->mbDeactivateLocalizationMode)
        {
            Tracking_InformOnlyTracking(pSystem->mpTracker,false);
            LocalMapping_Release(pSystem->mpLocalMapper);
            pSystem->mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(pSystem->mMutexReset);
    if(pSystem->mbReset)
    {
        Tracking_Reset(pSystem->mpTracker);
        pSystem->mbReset = false;
    }
    }

    cv::Mat Tcw = Tracking_GrabImageMonocular(pSystem->mpTracker,im,timestamp);

    unique_lock<mutex> lock2(pSystem->mMutexState);
    pSystem->mTrackingState = pSystem->mpTracker->mState;
    pSystem->mTrackedMapPoints = pSystem->mpTracker->mCurrentFrame->mvpMapPoints;
    pSystem->mTrackedKeyPointsUn = pSystem->mpTracker->mCurrentFrame->mvKeysUn;

    return Tcw;
}

void System_ActivateLocalizationMode(System *pSystem)
{
    unique_lock<mutex> lock(pSystem->mMutexMode);
    pSystem->mbActivateLocalizationMode = true;
}

void System_DeactivateLocalizationMode(System *pSystem)
{
    unique_lock<mutex> lock(pSystem->mMutexMode);
    pSystem->mbDeactivateLocalizationMode = true;
}

bool System_MapChanged(System *pSystem)
{
    static int n=0;
    int curn = Map_GetLastBigChangeIdx(pSystem->mpMap);
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System_Reset(System *pSystem)
{
    unique_lock<mutex> lock(pSystem->mMutexReset);
    pSystem->mbReset = true;
}

void System_Shutdown(System *pSystem)
{
    LocalMapping_RequestFinish(pSystem->mpLocalMapper);
    LoopClosing_RequestFinish(pSystem->mpLoopCloser);
    if(pSystem->mpViewer)
    {
     // Viewer_RequestFinish(mpViewer);
     //   while(!Viewer_isFinished(mpViewer))
      Viewer_RequestFinish(pSystem->mpViewer);
      while(!Viewer_isFinished(pSystem->mpViewer))
        usleep(5000);
    }

    delete pSystem->mpViewer;
    pSystem->mpViewer = static_cast<Viewer*>(NULL);

    // Wait until all thread have effectively stopped
    while(!LocalMapping_isFinished(pSystem->mpLocalMapper) || !LoopClosing_isFinished(pSystem->mpLoopCloser) || LoopClosing_isRunningGBA(pSystem->mpLoopCloser))
    {
        usleep(5000);
    }

    if(pSystem->mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");

}

void System_SaveTrajectoryTUM(System *pSystem,const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(pSystem->mSensor==System::MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = Map_GetAllKeyFrames(pSystem->mpMap);
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame_lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = KeyFrame_GetPoseInverse(vpKFs[0]);

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = pSystem->mpTracker->mlpReferences.begin();
    list<double>::iterator lT = pSystem->mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = pSystem->mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=pSystem->mpTracker->mlRelativeFramePoses.begin(),
        lend=pSystem->mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(KeyFrame_isBad(pKF))
        {
            Trw = Trw*pKF->mTcp;
            pKF = KeyFrame_GetParent(pKF);
        }

        Trw = Trw*KeyFrame_GetPose(pKF)*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter_toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System_SaveKeyFrameTrajectoryTUM(System *pSystem,const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = Map_GetAllKeyFrames(pSystem->mpMap);
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame_lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(KeyFrame_isBad(pKF))
            continue;

        cv::Mat R = KeyFrame_GetRotation(pKF).t();
        vector<float> q = Converter_toQuaternion(R);
        cv::Mat t = KeyFrame_GetCameraCenter(pKF);
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System_SaveTrajectoryKITTI(System *pSystem,const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(pSystem->mSensor==System::MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = Map_GetAllKeyFrames(pSystem->mpMap);
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame_lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = KeyFrame_GetPoseInverse(vpKFs[0]);

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = pSystem->mpTracker->mlpReferences.begin();
    list<double>::iterator lT = pSystem->mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=pSystem->mpTracker->mlRelativeFramePoses.begin(), lend=pSystem->mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(KeyFrame_isBad(pKF))
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = KeyFrame_GetParent(pKF);
        }

        Trw = Trw*KeyFrame_GetPose(pKF)*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System_GetTrackingState(System *pSystem)
{
    unique_lock<mutex> lock(pSystem->mMutexState);
    return pSystem->mTrackingState;
}

vector<MapPoint*> System_GetTrackedMapPoints(System *pSystem)
{
    unique_lock<mutex> lock(pSystem->mMutexState);
    return pSystem->mTrackedMapPoints;
}

vector<cv::KeyPoint> System_GetTrackedKeyPointsUn(System *pSystem)
{
    unique_lock<mutex> lock(pSystem->mMutexState);
    return pSystem->mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
