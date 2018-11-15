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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

struct KeyFrame;
struct Map;
class Frame;


struct MapPoint
{
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;

    static std::mutex mGlobalMutex;

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};
    void MapPoint_init_1(MapPoint* pMPT,const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    void MapPoint_init_2(MapPoint* pMPT,const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void MapPoint_SetWorldPos(MapPoint* pMPT,const cv::Mat &Pos);
    cv::Mat MapPoint_GetWorldPos(MapPoint* pMPT);

    cv::Mat MapPoint_GetNormal(MapPoint* pMPT);
    KeyFrame* MapPoint_GetReferenceKeyFrame(MapPoint* pMPT);

    std::map<KeyFrame*,size_t> MapPoint_GetObservations(MapPoint* pMPT);
    int MapPoint_Observations(MapPoint* pMPT);

    void MapPoint_AddObservation(MapPoint* pMPT,KeyFrame* pKF,size_t idx);
    void MapPoint_EraseObservation(MapPoint* pMPT,KeyFrame* pKF);

    int MapPoint_GetIndexInKeyFrame(MapPoint* pMPT,KeyFrame* pKF);
    bool MapPoint_IsInKeyFrame(MapPoint* pMPT,KeyFrame* pKF);

    void MapPoint_SetBadFlag(MapPoint* pMPT);
    bool MapPoint_isBad(MapPoint* pMPT);

    void MapPoint_Replace(MapPoint* pMPT,MapPoint* pMP);    
    MapPoint* MapPoint_GetReplaced(MapPoint* pMPT);

    void MapPoint_IncreaseVisible(MapPoint* pMPT,int n=1);
    void MapPoint_IncreaseFound(MapPoint* pMPT,int n=1);
    float MapPoint_GetFoundRatio(MapPoint* pMPT);
    inline int MapPoint_GetFound(MapPoint* pMPT){
        return pMPT->mnFound;
    }

    void MapPoint_ComputeDistinctiveDescriptors(MapPoint* pMPT);

    cv::Mat MapPoint_GetDescriptor(MapPoint* pMPT);

    void MapPoint_UpdateNormalAndDepth(MapPoint* pMPT);

    float MapPoint_GetMinDistanceInvariance(MapPoint* pMPT);
    float MapPoint_GetMaxDistanceInvariance(MapPoint* pMPT);
    int MapPoint_PredictScale(MapPoint* pMPT,const float &currentDist, KeyFrame*pKF);
    int MapPoint_PredictScale(MapPoint* pMPT,const float &currentDist, Frame* pF);


} //namespace ORB_SLAM

#endif // MAPPOINT_H
