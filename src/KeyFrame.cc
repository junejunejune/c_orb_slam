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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;
    
bool KeyFrame_weightComp( int a, int b){
    return a>b;
}

bool KeyFrame_lId(KeyFrame* pKF1, KeyFrame* pKF2){
    return pKF1->mnId<pKF2->mnId;
}


void KeyFrame_init(KeyFrame *pKeyFrame, Frame &F, Map *pMap, KeyFrameDatabase *pKFDB)
{
    pKeyFrame->mnFrameId=F.mnId;  pKeyFrame->mTimeStamp=F.mTimeStamp; pKeyFrame->mnGridCols=FRAME_GRID_COLS; pKeyFrame->mnGridRows=FRAME_GRID_ROWS;
    pKeyFrame->mfGridElementWidthInv=F.mfGridElementWidthInv; pKeyFrame->mfGridElementHeightInv=F.mfGridElementHeightInv;
    pKeyFrame->mnTrackReferenceForFrame=0; pKeyFrame->mnFuseTargetForKF=0; pKeyFrame->mnBALocalForKF=0; pKeyFrame->mnBAFixedForKF=0;
    pKeyFrame->mnLoopQuery=0; pKeyFrame->mnLoopWords=0; pKeyFrame->mnRelocQuery=0; pKeyFrame->mnRelocWords=0; pKeyFrame->mnBAGlobalForKF=0;
    pKeyFrame->fx=F.fx; pKeyFrame->fy=F.fy; pKeyFrame->cx=F.cx; pKeyFrame->cy=F.cy; pKeyFrame->invfx=F.invfx; pKeyFrame->invfy=F.invfy;
    pKeyFrame->mbf=F.mbf; 
    pKeyFrame->mb=F.mb; 
    pKeyFrame->mThDepth=F.mThDepth; 
    pKeyFrame->N=F.N; 
    
    pKeyFrame->mvKeys=F.mvKeys;
    pKeyFrame->mvKeysUn=F.mvKeysUn;
    pKeyFrame->mvuRight=F.mvuRight; pKeyFrame->mvDepth=F.mvDepth; pKeyFrame->mDescriptors=F.mDescriptors.clone();
    pKeyFrame->mBowVec=F.mBowVec; pKeyFrame->mFeatVec=F.mFeatVec; pKeyFrame->mnScaleLevels=F.mnScaleLevels; pKeyFrame->mfScaleFactor=F.mfScaleFactor;
    pKeyFrame->mfLogScaleFactor=F.mfLogScaleFactor; pKeyFrame->mvScaleFactors=F.mvScaleFactors; pKeyFrame->mvLevelSigma2=F.mvLevelSigma2;
    pKeyFrame->mvInvLevelSigma2=F.mvInvLevelSigma2; pKeyFrame->mnMinX=F.mnMinX; pKeyFrame->mnMinY=F.mnMinY; pKeyFrame->mnMaxX=F.mnMaxX;
    pKeyFrame->mnMaxY=F.mnMaxY; pKeyFrame->mK=F.mK; pKeyFrame->mvpMapPoints=F.mvpMapPoints; pKeyFrame->mpKeyFrameDB=pKFDB;
    pKeyFrame->mpORBvocabulary=F.mpORBvocabulary; pKeyFrame->mbFirstConnection=true; pKeyFrame->mpParent=NULL; pKeyFrame->mbNotErase=false;
    pKeyFrame->mbToBeErased=false; pKeyFrame->mbBad=false; pKeyFrame->mHalfBaseline=F.mb/2; pKeyFrame->mpMap=pMap;

    pKeyFrame->mnId=KeyFrame::nNextId++;

    pKeyFrame->mGrid.resize(pKeyFrame->mnGridCols);
    for(int i=0; i<pKeyFrame->mnGridCols;i++)
    {
        pKeyFrame->mGrid[i].resize(pKeyFrame->mnGridRows);
        for(int j=0; j<pKeyFrame->mnGridRows; j++)
            pKeyFrame->mGrid[i][j] = F.mGrid[i][j];
    }

    KeyFrame_SetPose(pKeyFrame, F.mTcw);    
}

void KeyFrame_ComputeBoW(KeyFrame *pKeyFrame)
{
    if(pKeyFrame->mBowVec.empty() || pKeyFrame->mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter_toDescriptorVector(pKeyFrame->mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        pKeyFrame->mpORBvocabulary->transform(vCurrentDesc,pKeyFrame->mBowVec,pKeyFrame->mFeatVec,4);
    }
}

void KeyFrame_SetPose(KeyFrame *pKeyFrame, const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexPose);
    Tcw_.copyTo(pKeyFrame->Tcw);
    cv::Mat Rcw = pKeyFrame->Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = pKeyFrame->Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    pKeyFrame->Ow = -Rwc*tcw;

    pKeyFrame->Twc = cv::Mat::eye(4,4,pKeyFrame->Tcw.type());
    Rwc.copyTo(pKeyFrame->Twc.rowRange(0,3).colRange(0,3));
    pKeyFrame->Ow.copyTo(pKeyFrame->Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << pKeyFrame->mHalfBaseline, 0 , 0, 1);
    pKeyFrame->Cw = pKeyFrame->Twc*center;
}

cv::Mat KeyFrame_GetPose(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexPose);
    return pKeyFrame->Tcw.clone();
}

cv::Mat KeyFrame_GetPoseInverse(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexPose);
    return pKeyFrame->Twc.clone();
}

cv::Mat KeyFrame_GetCameraCenter(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexPose);
    return pKeyFrame->Ow.clone();
}

cv::Mat KeyFrame_GetStereoCenter(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexPose);
    return pKeyFrame->Cw.clone();
}


cv::Mat KeyFrame_GetRotation(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexPose);
    return pKeyFrame->Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame_GetTranslation(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexPose);
    return pKeyFrame->Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame_AddConnection(KeyFrame *pKeyFrame,KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
        if(!pKeyFrame->mConnectedKeyFrameWeights.count(pKF))
            pKeyFrame->mConnectedKeyFrameWeights[pKF]=weight;
        else if(pKeyFrame->mConnectedKeyFrameWeights[pKF]!=weight)
            pKeyFrame->mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    KeyFrame_UpdateBestCovisibles(pKeyFrame);
}

void KeyFrame_UpdateBestCovisibles(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(pKeyFrame->mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=pKeyFrame->mConnectedKeyFrameWeights.begin(), mend=pKeyFrame->mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    pKeyFrame->mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    pKeyFrame->mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<KeyFrame*> KeyFrame_GetConnectedKeyFrames(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=pKeyFrame->mConnectedKeyFrameWeights.begin();mit!=pKeyFrame->mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame_GetVectorCovisibleKeyFrames(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
    return pKeyFrame->mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame_GetBestCovisibilityKeyFrames(KeyFrame *pKeyFrame,const int &N)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
    if((int)pKeyFrame->mvpOrderedConnectedKeyFrames.size()<N)
        return pKeyFrame->mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(pKeyFrame->mvpOrderedConnectedKeyFrames.begin(),pKeyFrame->mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame_GetCovisiblesByWeight(KeyFrame *pKeyFrame,const int &w)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexConnections);

    if(pKeyFrame->mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(pKeyFrame->mvOrderedWeights.begin(),pKeyFrame->mvOrderedWeights.end(),w,KeyFrame_weightComp);
    if(it==pKeyFrame->mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-pKeyFrame->mvOrderedWeights.begin();
        return vector<KeyFrame*>(pKeyFrame->mvpOrderedConnectedKeyFrames.begin(), pKeyFrame->mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame_GetWeight(KeyFrame *pKeyFrame,KeyFrame *pKF)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
    if(pKeyFrame->mConnectedKeyFrameWeights.count(pKF))
        return pKeyFrame->mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame_AddMapPoint(KeyFrame *pKeyFrame, MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexFeatures);
    pKeyFrame->mvpMapPoints[idx]=pMP;
}

void KeyFrame_EraseMapPointMatch(KeyFrame *pKeyFrame,const size_t &idx)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexFeatures);
    pKeyFrame->mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame_EraseMapPointMatch(KeyFrame *pKeyFrame,MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(pKeyFrame);
    if(idx>=0)
        pKeyFrame->mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame_ReplaceMapPointMatch(KeyFrame *pKeyFrame,const size_t &idx, MapPoint* pMP)
{
    pKeyFrame->mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame_GetMapPoints(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=pKeyFrame->mvpMapPoints.size(); i<iend; i++)
    {
        if(!pKeyFrame->mvpMapPoints[i])
            continue;
        MapPoint* pMP = pKeyFrame->mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame_TrackedMapPoints(KeyFrame *pKeyFrame, const int &minObs)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<pKeyFrame->N; i++)
    {
        MapPoint* pMP = pKeyFrame->mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(pKeyFrame->mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame_GetMapPointMatches(KeyFrame *pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexFeatures);
    return pKeyFrame->mvpMapPoints;
}

MapPoint* KeyFrame_GetMapPoint(KeyFrame *pKeyFrame,const size_t &idx)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexFeatures);
    return pKeyFrame->mvpMapPoints[idx];
}

void KeyFrame_UpdateConnections(KeyFrame *pKeyFrame)
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(pKeyFrame->mMutexFeatures);
        vpMP = pKeyFrame->mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==pKeyFrame->mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            KeyFrame_AddConnection((mit->first),pKeyFrame,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        KeyFrame_AddConnection(pKFmax, pKeyFrame,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(pKeyFrame->mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        pKeyFrame->mConnectedKeyFrameWeights = KFcounter;
        pKeyFrame->mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        pKeyFrame->mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(pKeyFrame->mbFirstConnection && pKeyFrame->mnId!=0)
        {
            pKeyFrame->mpParent = pKeyFrame->mvpOrderedConnectedKeyFrames.front();
            KeyFrame_AddChild(pKeyFrame->mpParent, pKeyFrame);
            pKeyFrame->mbFirstConnection = false;
        }

    }
}

void KeyFrame_AddChild(KeyFrame* pKeyFrame, KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(pKeyFrame->mMutexConnections);
    pKeyFrame->mspChildrens.insert(pKF);
}

void KeyFrame_EraseChild(KeyFrame* pKeyFrame,KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(pKeyFrame->mMutexConnections);
    pKeyFrame->mspChildrens.erase(pKF);
}

void KeyFrame_ChangeParent(KeyFrame* pKeyFrame,KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(pKeyFrame->mMutexConnections);
    pKeyFrame->mpParent = pKF;
    KeyFrame_AddChild(pKF,pKeyFrame);
}

set<KeyFrame*> KeyFrame_GetChilds(KeyFrame* pKeyFrame)
{
    unique_lock<mutex> lockCon(pKeyFrame->mMutexConnections);
    return pKeyFrame->mspChildrens;
}

KeyFrame* KeyFrame_GetParent(KeyFrame* pKeyFrame)
{
    unique_lock<mutex> lockCon(pKeyFrame->mMutexConnections);
    return pKeyFrame->mpParent;
}

bool KeyFrame_hasChild(KeyFrame* pKeyFrame,KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(pKeyFrame->mMutexConnections);
    return pKeyFrame->mspChildrens.count(pKF);
}

void KeyFrame_AddLoopEdge(KeyFrame* pKeyFrame,KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(pKeyFrame->mMutexConnections);
    pKeyFrame->mbNotErase = true;
    pKeyFrame->mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame_GetLoopEdges(KeyFrame* pKeyFrame)
{
    unique_lock<mutex> lockCon(pKeyFrame->mMutexConnections);
    return pKeyFrame->mspLoopEdges;
}

void KeyFrame_SetNotErase(KeyFrame* pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
    pKeyFrame->mbNotErase = true;
}

void KeyFrame_SetErase(KeyFrame* pKeyFrame)
{
    {
        unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
        if(pKeyFrame->mspLoopEdges.empty())
        {
            pKeyFrame->mbNotErase = false;
        }
    }

    if(pKeyFrame->mbToBeErased)
    {
        KeyFrame_SetBadFlag(pKeyFrame);
    }
}

void KeyFrame_SetBadFlag(KeyFrame* pKeyFrame)
{   
    {
        unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
        if(pKeyFrame->mnId==0)
            return;
        else if(pKeyFrame->mbNotErase)
        {
            pKeyFrame->mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = pKeyFrame->mConnectedKeyFrameWeights.begin(), mend=pKeyFrame->mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        KeyFrame_EraseConnection(mit->first,pKeyFrame);

    for(size_t i=0; i<pKeyFrame->mvpMapPoints.size(); i++)
        if(pKeyFrame->mvpMapPoints[i])
            pKeyFrame->mvpMapPoints[i]->EraseObservation(pKeyFrame);
    {
        unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
        unique_lock<mutex> lock1(pKeyFrame->mMutexFeatures);

        pKeyFrame->mConnectedKeyFrameWeights.clear();
        pKeyFrame->mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(pKeyFrame->mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!pKeyFrame->mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=pKeyFrame->mspChildrens.begin(), send=pKeyFrame->mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(KeyFrame_isBad(pKF))
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = KeyFrame_GetVectorCovisibleKeyFrames(pKF);
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = KeyFrame_GetWeight(pKF,vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                KeyFrame_ChangeParent(pC,pP);
                sParentCandidates.insert(pC);
                pKeyFrame->mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!pKeyFrame->mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=pKeyFrame->mspChildrens.begin(); sit!=pKeyFrame->mspChildrens.end(); sit++)
            {
                KeyFrame_ChangeParent((*sit),pKeyFrame->mpParent);
            }

        KeyFrame_EraseChild(pKeyFrame->mpParent, pKeyFrame);
        pKeyFrame->mTcp = pKeyFrame->Tcw*KeyFrame_GetPoseInverse(pKeyFrame->mpParent);
        pKeyFrame->mbBad = true;
    }


    Map_EraseKeyFrame(pKeyFrame->mpMap,pKeyFrame);
    pKeyFrame->mpKeyFrameDB->erase(pKeyFrame);
}

bool KeyFrame_isBad(KeyFrame* pKeyFrame)
{
    unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
    return pKeyFrame->mbBad;
}

void KeyFrame_EraseConnection(KeyFrame* pKeyFrame, KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(pKeyFrame->mMutexConnections);
        if(pKeyFrame->mConnectedKeyFrameWeights.count(pKF))
        {
            pKeyFrame->mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        KeyFrame_UpdateBestCovisibles(pKeyFrame);
}

vector<size_t> KeyFrame_GetFeaturesInArea(KeyFrame* pKeyFrame, const float &x, const float &y, const float &r) 
{
    vector<size_t> vIndices;
    vIndices.reserve(pKeyFrame->N);

    const int nMinCellX = max(0,(int)floor((x-pKeyFrame->mnMinX-r)*pKeyFrame->mfGridElementWidthInv));
    if(nMinCellX>=pKeyFrame->mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)pKeyFrame->mnGridCols-1,(int)ceil((x-pKeyFrame->mnMinX+r)*pKeyFrame->mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-pKeyFrame->mnMinY-r)*pKeyFrame->mfGridElementHeightInv));
    if(nMinCellY>=pKeyFrame->mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)pKeyFrame->mnGridRows-1,(int)ceil((y-pKeyFrame->mnMinY+r)*pKeyFrame->mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = pKeyFrame->mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = pKeyFrame->mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame_IsInImage(KeyFrame* pKeyFrame,const float &x, const float &y) 
{
    return (x>=pKeyFrame->mnMinX && x<pKeyFrame->mnMaxX && y>=pKeyFrame->mnMinY && y<pKeyFrame->mnMaxY);
}

cv::Mat KeyFrame_UnprojectStereo(KeyFrame* pKeyFrame, int i)
{
    const float z = pKeyFrame->mvDepth[i];
    if(z>0)
    {
        const float u = pKeyFrame->mvKeys[i].pt.x;
        const float v = pKeyFrame->mvKeys[i].pt.y;
        const float x = (u-pKeyFrame->cx)*z*pKeyFrame->invfx;
        const float y = (v-pKeyFrame->cy)*z*pKeyFrame->invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(pKeyFrame->mMutexPose);
        return pKeyFrame->Twc.rowRange(0,3).colRange(0,3)*x3Dc+pKeyFrame->Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame_ComputeSceneMedianDepth(KeyFrame* pKeyFrame,const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(pKeyFrame->mMutexFeatures);
        unique_lock<mutex> lock2(pKeyFrame->mMutexPose);
        vpMapPoints = pKeyFrame->mvpMapPoints;
        Tcw_ = pKeyFrame->Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(pKeyFrame->N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<pKeyFrame->N; i++)
    {
        if(pKeyFrame->mvpMapPoints[i])
        {
            MapPoint* pMP = pKeyFrame->mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
