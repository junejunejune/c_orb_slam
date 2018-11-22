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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

struct Tracking;
struct LoopClosing;
struct Map;

struct LocalMapping
{
    bool mbMonocular;

    bool mbResetRequested;
    std::mutex mMutexReset;

    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};
    void LocalMapping_init(LocalMapping* pLM,Map* pMap, const float bMonocular);

    void LocalMapping_SetLoopCloser(LocalMapping* pLM,LoopClosing* pLoopCloser);

    void LocalMapping_SetTracker(LocalMapping* pLM,Tracking* pTracker);

    // Main function
    void LocalMapping_Run(LocalMapping* pLM);

    void LocalMapping_InsertKeyFrame(LocalMapping* pLM,KeyFrame* pKF);

    // Thread Synch
    void LocalMapping_RequestStop(LocalMapping* pLM);
    void LocalMapping_RequestReset(LocalMapping* pLM);
    bool LocalMapping_Stop(LocalMapping* pLM);
    void LocalMapping_Release(LocalMapping* pLM);
    bool LocalMapping_isStopped(LocalMapping* pLM);
    bool LocalMapping_stopRequested(LocalMapping* pLM);
    bool LocalMapping_AcceptKeyFrames(LocalMapping* pLM);
    void LocalMapping_SetAcceptKeyFrames(LocalMapping* pLM,bool flag);
    bool LocalMapping_SetNotStop(LocalMapping* pLM,bool flag);

    void LocalMapping_InterruptBA(LocalMapping* pLM);

    void LocalMapping_RequestFinish(LocalMapping* pLM);
    bool LocalMapping_isFinished(LocalMapping* pLM);

    int LocalMapping_KeyframesInQueue(LocalMapping* pLM);

    bool LocalMapping_CheckNewKeyFrames(LocalMapping* pLM);
    void LocalMapping_ProcessNewKeyFrame(LocalMapping* pLM);
    void LocalMapping_CreateNewMapPoints(LocalMapping* pLM);

    void LocalMapping_MapPointCulling(LocalMapping* pLM);
    void LocalMapping_SearchInNeighbors(LocalMapping* pLM);

    void LocalMapping_KeyFrameCulling(LocalMapping* pLM);

    cv::Mat LocalMapping_ComputeF12(LocalMapping* pLM,KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat LocalMapping_SkewSymmetricMatrix(LocalMapping* pLM,const cv::Mat &v);

    void LocalMapping_ResetIfRequested(LocalMapping* pLM);
    bool LocalMapping_CheckFinish(LocalMapping* pLM);
    void LocalMapping_SetFinish(LocalMapping* pLM);
 
} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
