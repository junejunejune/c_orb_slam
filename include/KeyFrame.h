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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

struct Map;
struct MapPoint;
class Frame;
struct KeyFrameDatabase;

struct KeyFrame
{

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
    static long unsigned int nNextId;
    long unsigned int mnId;
    long unsigned int mnFrameId;

    double mTimeStamp;

    // Grid (to speed up feature matching)
    int mnGridCols;
    int mnGridRows;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<float> mvuRight; // negative value for monocular points
    std::vector<float> mvDepth; // negative value for monocular points
    cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    int mnMinX;
    int mnMinY;
    int mnMaxX;
    int mnMaxY;
    cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};
    void KeyFrame_init(KeyFrame *pKeyFrame, Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void KeyFrame_SetPose(KeyFrame *pKeyFrame,const cv::Mat &Tcw);
    cv::Mat KeyFrame_GetPose(KeyFrame *pKeyFrame);
    cv::Mat KeyFrame_GetPoseInverse(KeyFrame *pKeyFrame);
    cv::Mat KeyFrame_GetCameraCenter(KeyFrame *pKeyFrame);
    cv::Mat KeyFrame_GetStereoCenter(KeyFrame *pKeyFrame);
    cv::Mat KeyFrame_GetRotation(KeyFrame *pKeyFrame);
    cv::Mat KeyFrame_GetTranslation(KeyFrame *pKeyFrame);

    // Bag of Words Representation
    void KeyFrame_ComputeBoW(KeyFrame *pKeyFrame);

    // Covisibility graph functions
    void KeyFrame_AddConnection(KeyFrame *pKeyFrame,KeyFrame* pKF, const int &weight);
    void KeyFrame_EraseConnection(KeyFrame *pKeyFrame,KeyFrame* pKF);
    void KeyFrame_UpdateConnections(KeyFrame *pKeyFrame);
    void KeyFrame_UpdateBestCovisibles(KeyFrame *pKeyFrame);
    std::set<KeyFrame *> KeyFrame_GetConnectedKeyFrames(KeyFrame *pKeyFrame);
    std::vector<KeyFrame* > KeyFrame_GetVectorCovisibleKeyFrames(KeyFrame *pKeyFrame);
    std::vector<KeyFrame*> KeyFrame_GetBestCovisibilityKeyFrames(KeyFrame *pKeyFrame,const int &N);
    std::vector<KeyFrame*> KeyFrame_GetCovisiblesByWeight(KeyFrame *pKeyFrame,const int &w);
    int KeyFrame_GetWeight(KeyFrame *pKeyFrame,KeyFrame* pKF);

    // Spanning tree functions
    void KeyFrame_AddChild(KeyFrame *pKeyFrame,KeyFrame* pKF);
    void KeyFrame_EraseChild(KeyFrame *pKeyFrame,KeyFrame* pKF);
    void KeyFrame_ChangeParent(KeyFrame *pKeyFrame,KeyFrame* pKF);
    std::set<KeyFrame*> KeyFrame_GetChilds(KeyFrame *pKeyFrame);
    KeyFrame* KeyFrame_GetParent(KeyFrame *pKeyFrame);
    bool KeyFrame_hasChild(KeyFrame *pKeyFrame,KeyFrame* pKF);

    // Loop Edges
    void KeyFrame_AddLoopEdge(KeyFrame *pKeyFrame,KeyFrame* pKF);
    std::set<KeyFrame*> KeyFrame_GetLoopEdges(KeyFrame *pKeyFrame);

    // MapPoint observation functions
    void KeyFrame_AddMapPoint(KeyFrame *pKeyFrame,MapPoint* pMP, const size_t &idx);
    void KeyFrame_EraseMapPointMatch(KeyFrame *pKeyFrame,const size_t &idx);
    void KeyFrame_EraseMapPointMatch(KeyFrame *pKeyFrame,MapPoint* pMP);
    void KeyFrame_ReplaceMapPointMatch(KeyFrame *pKeyFrame,const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> KeyFrame_GetMapPoints(KeyFrame *pKeyFrame);
    std::vector<MapPoint*> KeyFrame_GetMapPointMatches(KeyFrame *pKeyFrame);
    int KeyFrame_TrackedMapPoints(KeyFrame *pKeyFrame,const int &minObs);
    MapPoint* KeyFrame_GetMapPoint(KeyFrame *pKeyFrame,const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> KeyFrame_GetFeaturesInArea(KeyFrame *pKeyFrame,const float &x, const float  &y, const float  &r);
    cv::Mat KeyFrame_UnprojectStereo(KeyFrame *pKeyFrame,int i);

    // Image
    bool KeyFrame_IsInImage(KeyFrame *pKeyFrame,const float &x, const float &y);

    // Enable/Disable bad flag changes
    void KeyFrame_SetNotErase(KeyFrame *pKeyFrame);
    void KeyFrame_SetErase(KeyFrame *pKeyFrame);

    // Set/check bad flag
    void KeyFrame_SetBadFlag(KeyFrame *pKeyFrame);
    bool KeyFrame_isBad(KeyFrame *pKeyFrame);

    // Compute Scene Depth (q=2 median). Used in monocular.
    float KeyFrame_ComputeSceneMedianDepth(KeyFrame *pKeyFrame,const int q);

    bool KeyFrame_weightComp( int a, int b);

    bool KeyFrame_lId(KeyFrame* pKF1, KeyFrame* pKF2);


} //namespace ORB_SLAM

#endif // KEYFRAME_H
