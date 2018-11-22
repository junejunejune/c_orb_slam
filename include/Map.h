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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace ORB_SLAM2
{

struct MapPoint;
struct KeyFrame;

struct Map
{
    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};

    void Map_init(Map *pMap);

    void Map_AddKeyFrame(Map *pMap, KeyFrame* pKF);
    void Map_AddMapPoint(Map *pMap, MapPoint* pMP);
    void Map_EraseMapPoint(Map *pMap, MapPoint* pMP);
    void Map_EraseKeyFrame(Map *pMap, KeyFrame* pKF);
    void Map_SetReferenceMapPoints(Map *pMap, const std::vector<MapPoint*> &vpMPs);
    void Map_InformNewBigChange(Map *pMap);
    int Map_GetLastBigChangeIdx(Map *pMap);

    std::vector<KeyFrame*> Map_GetAllKeyFrames(Map *pMap);
    std::vector<MapPoint*> Map_GetAllMapPoints(Map *pMap);
    std::vector<MapPoint*> Map_GetReferenceMapPoints(Map *pMap);

    long unsigned int Map_MapPointsInMap(Map *pMap);
    long unsigned  Map_KeyFramesInMap(Map *pMap);

    long unsigned int Map_GetMaxKFid(Map *pMap);

    void Map_clear(Map *pMap);
} //namespace ORB_SLAM

#endif // MAP_H
