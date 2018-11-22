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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

struct Tracking;
struct LocalMapping;
struct KeyFrameDatabase;

struct LoopClosing
{
    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;


    bool mbResetRequested;
    std::mutex mMutexReset;

    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;
    bool mnFullBAIdx;
};

    void LoopClosing_init(LoopClosing* pLC,Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void LoopClosing_SetTracker(LoopClosing* pLC,Tracking* pTracker);

    void LoopClosing_SetLocalMapper(LoopClosing* pLC,LocalMapping* pLocalMapper);

    // Main function
    void LoopClosing_Run(LoopClosing* pLC);

    void LoopClosing_InsertKeyFrame(LoopClosing* pLC,KeyFrame *pKF);

    void LoopClosing_RequestReset(LoopClosing* pLC);

    // This function will run in a separate thread
    void LoopClosing_RunGlobalBundleAdjustment(LoopClosing* pLC,unsigned long nLoopKF);

    bool LoopClosing_isRunningGBA(LoopClosing* pLC);

    bool LoopClosing_isFinishedGBA(LoopClosing* pLC);

    void LoopClosing_RequestFinish(LoopClosing* pLC);

    bool LoopClosing_isFinished(LoopClosing* pLC);

    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool LoopClosing_CheckNewKeyFrames(LoopClosing* pLC);

    bool LoopClosing_DetectLoop(LoopClosing* pLC);

    bool LoopClosing_ComputeSim3(LoopClosing* pLC);

    void LoopClosing_SearchAndFuse(LoopClosing* pLC,LoopClosing::KeyFrameAndPose &CorrectedPosesMap);

    void LoopClosing_CorrectLoop(LoopClosing* pLC);

    void LoopClosing_ResetIfRequested(LoopClosing* pLC);
     
    bool LoopClosing_CheckFinish(LoopClosing* pLC);
    
    void LoopClosing_SetFinish(LoopClosing* pLC);
 
} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
