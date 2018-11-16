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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

void MapPoint_init_1(MapPoint* pMPT,const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap)
{
    pMPT->mnFirstKFid=pRefKF->mnId;
    pMPT->mnFirstFrame=pRefKF->mnFrameId; 
    pMPT->nObs=0;
    pMPT->mnTrackReferenceForFrame=0;
    pMPT->mnLastFrameSeen=0; 
    pMPT->mnBALocalForKF=0; 
    pMPT->mnFuseCandidateForKF=0;
    pMPT->mnLoopPointForKF=0; 
    pMPT->mnCorrectedByKF=0;
    pMPT->mnCorrectedReference=0;
    pMPT->mnBAGlobalForKF=0; 
    pMPT->mpRefKF=pRefKF;
    pMPT->mnVisible=1; 
    pMPT->mnFound=1; 
    pMPT->mbBad=false;
    pMPT->mpReplaced=static_cast<MapPoint*>(NULL); 
    pMPT->mfMinDistance=0; 
    pMPT->mfMaxDistance=0; 
    pMPT->mpMap=pMap;

    Pos.copyTo(pMPT->mWorldPos);
    pMPT->mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(pMPT->mpMap->mMutexPointCreation);
    pMPT->mnId=pMPT->nNextId++;
}

void MapPoint_init_2(MapPoint* pMPT, const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF)
{
    pMPT->mnFirstKFid=-1;
    pMPT->mnFirstFrame=pFrame->mnId;
    pMPT->nObs=0;
    pMPT->mnTrackReferenceForFrame=0; 
    pMPT->mnLastFrameSeen=0;
    pMPT->mnBALocalForKF=0; 
    pMPT->mnFuseCandidateForKF=0;
    pMPT->mnLoopPointForKF=0;
    pMPT->mnCorrectedByKF=0;
    pMPT->mnCorrectedReference=0; 
    pMPT->mnBAGlobalForKF=0; 
    pMPT->mpRefKF=static_cast<KeyFrame*>(NULL); 
    pMPT->mnVisible=1;
    pMPT->mnFound=1; 
    pMPT->mbBad=false; 
    pMPT->mpReplaced=NULL; 
    pMPT->mpMap=pMap;

    Pos.copyTo(pMPT->mWorldPos);
    cv::Mat Ow = Frame_GetCameraCenter(pFrame);
    pMPT->mNormalVector = pMPT->mWorldPos - Ow;
    pMPT->mNormalVector = pMPT->mNormalVector/cv::norm(pMPT->mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    pMPT->mfMaxDistance = dist*levelScaleFactor;
    pMPT->mfMinDistance = pMPT->mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(pMPT->mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(pMPT->mpMap->mMutexPointCreation);
    pMPT->mnId=pMPT->nNextId++;
}

void MapPoint_SetWorldPos(MapPoint* pMPT,const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(pMPT->mGlobalMutex);
    unique_lock<mutex> lock(pMPT->mMutexPos);
    Pos.copyTo(pMPT->mWorldPos);
}

cv::Mat MapPoint_GetWorldPos(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexPos);
    return pMPT->mWorldPos.clone();
}

cv::Mat MapPoint_GetNormal(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexPos);
    return pMPT->mNormalVector.clone();
}

KeyFrame* MapPoint_GetReferenceKeyFrame(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    return pMPT->mpRefKF;
}

void MapPoint_AddObservation(MapPoint* pMPT,KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    if(pMPT->mObservations.count(pKF))
        return;
    pMPT->mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        pMPT->nObs+=2;
    else
        pMPT->nObs++;
}

void MapPoint_EraseObservation(MapPoint* pMPT,KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(pMPT->mMutexFeatures);
        if(pMPT->mObservations.count(pKF))
        {
            int idx = pMPT->mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                pMPT->nObs-=2;
            else
                pMPT->nObs--;

            pMPT->mObservations.erase(pKF);

            if(pMPT->mpRefKF==pKF)
                pMPT->mpRefKF=pMPT->mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(pMPT->nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        MapPoint_SetBadFlag(pMPT);
}

map<KeyFrame*, size_t> MapPoint_GetObservations(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    return pMPT->mObservations;
}

int MapPoint_Observations(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    return pMPT->nObs;
}

void MapPoint_SetBadFlag(MapPoint* pMPT)
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(pMPT->mMutexFeatures);
        unique_lock<mutex> lock2(pMPT->mMutexPos);
        pMPT->mbBad=true;
        obs = pMPT->mObservations;
        pMPT->mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        KeyFrame_EraseMapPointMatch(pKF,mit->second);
    }

    Map_EraseMapPoint(pMPT->mpMap,pMPT);
}

MapPoint* MapPoint_GetReplaced(MapPoint* pMPT)
{
    unique_lock<mutex> lock1(pMPT->mMutexFeatures);
    unique_lock<mutex> lock2(pMPT->mMutexPos);
    return pMPT->mpReplaced;
}

void MapPoint_Replace(MapPoint* pMPT,MapPoint* pMP)
{
    if(pMP->mnId==pMPT->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(pMPT->mMutexFeatures);
        unique_lock<mutex> lock2(pMPT->mMutexPos);
        obs=pMPT->mObservations;
        pMPT->mObservations.clear();
        pMPT->mbBad=true;
        nvisible = pMPT->mnVisible;
        nfound = pMPT->mnFound;
        pMPT->mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!MapPoint_IsInKeyFrame(pMP,pKF))
        {
            KeyFrame_ReplaceMapPointMatch(pKF, mit->second, pMP);
            MapPoint_AddObservation(pMP,pKF,mit->second);
        }
        else
        {
            KeyFrame_EraseMapPointMatch(pKF,mit->second);
        }
    }
    MapPoint_IncreaseFound(pMP,nfound);
    MapPoint_IncreaseVisible(pMP,nvisible);
    MapPoint_ComputeDistinctiveDescriptors(pMP);

    Map_EraseMapPoint(pMPT->mpMap,pMPT);
}

bool MapPoint_isBad(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    unique_lock<mutex> lock2(pMPT->mMutexPos);
    return pMPT->mbBad;
}

void MapPoint_IncreaseVisible(MapPoint* pMPT,int n)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    pMPT->mnVisible+=n;
}

void MapPoint_IncreaseFound(MapPoint* pMPT,int n)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    pMPT->mnFound+=n;
}

float MapPoint_GetFoundRatio(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    return static_cast<float>(pMPT->mnFound)/pMPT->mnVisible;
}

void MapPoint_ComputeDistinctiveDescriptors(MapPoint* pMPT)
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(pMPT->mMutexFeatures);
        if(pMPT->mbBad)
            return;
        observations=pMPT->mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!KeyFrame_isBad(pKF))
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher_DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(pMPT->mMutexFeatures);
        pMPT->mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint_GetDescriptor(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    return pMPT->mDescriptor.clone();
}

int MapPoint_GetIndexInKeyFrame(MapPoint* pMPT,KeyFrame *pKF)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    if(pMPT->mObservations.count(pKF))
        return pMPT->mObservations[pKF];
    else
        return -1;
}

bool MapPoint_IsInKeyFrame(MapPoint* pMPT,KeyFrame *pKF)
{
    unique_lock<mutex> lock(pMPT->mMutexFeatures);
    return (pMPT->mObservations.count(pKF));
}

void MapPoint_UpdateNormalAndDepth(MapPoint* pMPT)
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(pMPT->mMutexFeatures);
        unique_lock<mutex> lock2(pMPT->mMutexPos);
        if(pMPT->mbBad)
            return;
        observations=pMPT->mObservations;
        pRefKF=pMPT->mpRefKF;
        Pos = pMPT->mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = KeyFrame_GetCameraCenter(pKF);
        cv::Mat normali = pMPT->mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - KeyFrame_GetCameraCenter(pRefKF);
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(pMPT->mMutexPos);
        pMPT->mfMaxDistance = dist*levelScaleFactor;
        pMPT->mfMinDistance = pMPT->mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        pMPT->mNormalVector = normal/n;
    }
}

float MapPoint_GetMinDistanceInvariance(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexPos);
    return 0.8f*pMPT->mfMinDistance;
}

float MapPoint_GetMaxDistanceInvariance(MapPoint* pMPT)
{
    unique_lock<mutex> lock(pMPT->mMutexPos);
    return 1.2f*pMPT->mfMaxDistance;
}

int MapPoint_PredictScale(MapPoint* pMPT,const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(pMPT->mMutexPos);
        ratio = pMPT->mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint_PredictScale(MapPoint* pMPT,const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(pMPT->mMutexPos);
        ratio = pMPT->mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}



} //namespace ORB_SLAM
