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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

void Map_init(Map *pMap)
{
  pMap->mnMaxKFid=0;
  pMap->mnBigChangeIdx=0;
}

void Map_AddKeyFrame(Map *pMap, KeyFrame *pKF)
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    pMap->mspKeyFrames.insert(pKF);
    if(pKF->mnId>pMap->mnMaxKFid)
        pMap->mnMaxKFid=pKF->mnId;
}

void Map_AddMapPoint(Map *pMap, MapPoint *pMP)
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    pMap->mspMapPoints.insert(pMP);
}

void Map_EraseMapPoint(Map *pMap, MapPoint *pMP)
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    pMap->mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map_EraseKeyFrame(Map *pMap, KeyFrame *pKF)
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    pMap->mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map_SetReferenceMapPoints(Map *pMap, const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    pMap->mvpReferenceMapPoints = vpMPs;
}

void Map_InformNewBigChange(Map *pMap )
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    pMap->mnBigChangeIdx++;
}

int Map_GetLastBigChangeIdx(Map *pMap )
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    return pMap->mnBigChangeIdx;
}

vector<KeyFrame*> Map_GetAllKeyFrames(Map *pMap )
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    return vector<KeyFrame*>(pMap->mspKeyFrames.begin(),pMap->mspKeyFrames.end());
}

vector<MapPoint*> Map_GetAllMapPoints(Map *pMap )
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    return vector<MapPoint*>(pMap->mspMapPoints.begin(),pMap->mspMapPoints.end());
}

long unsigned int Map_MapPointsInMap(Map *pMap )
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    return pMap->mspMapPoints.size();
}

long unsigned int Map_KeyFramesInMap(Map *pMap )
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    return pMap->mspKeyFrames.size();
}

vector<MapPoint*> Map_GetReferenceMapPoints(Map *pMap)
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    return pMap->mvpReferenceMapPoints;
}

long unsigned int Map_GetMaxKFid(Map *pMap)
{
    unique_lock<mutex> lock(pMap->mMutexMap);
    return pMap->mnMaxKFid;
}

void Map_clear(Map *pMap)
{
    for(set<MapPoint*>::iterator sit=pMap->mspMapPoints.begin(), send=pMap->mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=pMap->mspKeyFrames.begin(), send=pMap->mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    pMap->mspMapPoints.clear();
    pMap->mspKeyFrames.clear();
    pMap->mnMaxKFid = 0;
    pMap->mvpReferenceMapPoints.clear();
    pMap->mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
