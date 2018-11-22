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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

//Copy Constructor
void Frame_init_1(Frame* pF,const Frame *frame)
{
     pF->mpORBvocabulary=frame->mpORBvocabulary;
     pF->mpORBextractorLeft=frame->mpORBextractorLeft;
     pF->mpORBextractorRight=frame->mpORBextractorRight;
     pF->mTimeStamp=frame->mTimeStamp;
     pF->mK=frame->mK.clone();
     pF->mDistCoef=frame->mDistCoef.clone();
     pF->mbf=frame->mbf;
     pF->mb=frame->mb;
     pF->mThDepth=frame->mThDepth;
     pF->N=frame->N;
     pF->mvKeys=frame->mvKeys;
     pF->mvKeysRight=frame->mvKeysRight;
     pF->mvKeysUn=frame->mvKeysUn;  
     pF->mvuRight=frame->mvuRight;
     pF->mvDepth=frame->mvDepth;
     pF->mBowVec=frame->mBowVec;
     pF->mFeatVec=frame->mFeatVec;
     pF->mDescriptors=frame->mDescriptors.clone();
     pF->mDescriptorsRight=frame->mDescriptorsRight.clone();
     pF->mvpMapPoints=frame->mvpMapPoints;
     pF->mvbOutlier=frame->mvbOutlier;
     pF->mnId=frame->mnId;
     pF->mpReferenceKF=frame->mpReferenceKF;
     pF->mnScaleLevels=frame->mnScaleLevels;
     pF->mfScaleFactor=frame->mfScaleFactor;
     pF->mfLogScaleFactor=frame->mfLogScaleFactor;
     pF->mvScaleFactors=frame->mvScaleFactors; 
     pF->mvInvScaleFactors=frame->mvInvScaleFactors;
     pF->mvLevelSigma2=frame->mvLevelSigma2;
     pF->mvInvLevelSigma2=frame->mvInvLevelSigma2;

    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            pF->mGrid[i][j]=frame->mGrid[i][j];

    if(!frame->mTcw.empty())
        Frame_SetPose(pF, frame->mTcw);
}


void Frame_init_2(Frame* pF,const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
{
    pF->mpORBvocabulary=voc;
    pF->mpORBextractorLeft=extractorLeft;
    pF->mpORBextractorRight=extractorRight;
    pF->mTimeStamp=timeStamp;
    pF->mK=K.clone();
    pF->mDistCoef=distCoef.clone(); 
    pF->mbf=bf;
    pF->mThDepth=thDepth;
    pF->mpReferenceKF=static_cast<KeyFrame*>(NULL);

    // Frame ID
    pF->mnId=pF->nNextId++;

    // Scale Level Info
    pF->mnScaleLevels = ORBextractor_GetLevels(pF->mpORBextractorLeft);
    pF->mfScaleFactor = ORBextractor_GetScaleFactor(pF->mpORBextractorLeft);
    pF->mfLogScaleFactor = log(pF->mfScaleFactor);
    pF->mvScaleFactors = ORBextractor_GetScaleFactors(pF->mpORBextractorLeft);
    pF->mvInvScaleFactors = ORBextractor_GetInverseScaleFactors(pF->mpORBextractorLeft);
    pF->mvLevelSigma2 = ORBextractor_GetScaleSigmaSquares(pF->mpORBextractorLeft);
    pF->mvInvLevelSigma2 = ORBextractor_GetInverseScaleSigmaSquares(pF->mpORBextractorLeft);

    // ORB extraction
    thread threadLeft(&Frame_ExtractORB,pF,0,imLeft);
    thread threadRight(&Frame_ExtractORB,pF,1,imRight);
    threadLeft.join();
    threadRight.join();

    pF->N = pF->mvKeys.size();

    if(pF->mvKeys.empty())
        return;

    Frame_UndistortKeyPoints(pF);

    Frame_ComputeStereoMatches(pF);

    pF->mvpMapPoints = vector<MapPoint*>(pF->N,static_cast<MapPoint*>(NULL));    
    pF->mvbOutlier = vector<bool>(pF->N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(pF->mbInitialComputations)
    {
        Frame_ComputeImageBounds(pF,imLeft);

        pF->mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(pF->mnMaxX-pF->mnMinX);
        pF->mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(pF->mnMaxY-pF->mnMinY);

        pF->fx = K.at<float>(0,0);
        pF->fy = K.at<float>(1,1);
        pF->cx = K.at<float>(0,2);
        pF->cy = K.at<float>(1,2);
        pF->invfx = 1.0f/pF->fx;
        pF->invfy = 1.0f/pF->fy;

        pF->mbInitialComputations=false;
    }

    pF->mb = pF->mbf/pF->fx;

    Frame_AssignFeaturesToGrid(pF);
}

void Frame_init_3(Frame* pF,const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
{
    pF->mpORBvocabulary=voc;
    pF->mpORBextractorLeft=extractor;
    pF->mpORBextractorRight=static_cast<ORBextractor*>(NULL);
    pF->mTimeStamp=timeStamp;
    pF->mK=K.clone();
    pF->mDistCoef=distCoef.clone();
    pF->mbf=bf;
    pF->mThDepth=thDepth;

    // Frame ID
    pF->mnId=pF->nNextId++;

    // Scale Level Info
    pF->mnScaleLevels = ORBextractor_GetLevels(pF->mpORBextractorLeft);
    pF->mfScaleFactor = ORBextractor_GetScaleFactor(pF->mpORBextractorLeft);    
    pF->mfLogScaleFactor = log(pF->mfScaleFactor);
    pF->mvScaleFactors = ORBextractor_GetScaleFactors(pF->mpORBextractorLeft);
    pF->mvInvScaleFactors = ORBextractor_GetInverseScaleFactors(pF->mpORBextractorLeft);
    pF->mvLevelSigma2 = ORBextractor_GetScaleSigmaSquares(pF->mpORBextractorLeft);
    pF->mvInvLevelSigma2 = ORBextractor_GetInverseScaleSigmaSquares(pF->mpORBextractorLeft);

    // ORB extraction
    Frame_ExtractORB(pF,0,imGray);

    pF->N = pF->mvKeys.size();

    if(pF->mvKeys.empty())
        return;

    Frame_UndistortKeyPoints(pF);

    Frame_ComputeStereoFromRGBD(pF,imDepth);

    pF->mvpMapPoints = vector<MapPoint*>(pF->N,static_cast<MapPoint*>(NULL));
    pF->mvbOutlier = vector<bool>(pF->N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(pF->mbInitialComputations)
    {
        Frame_ComputeImageBounds(pF,imGray);

        pF->mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(pF->mnMaxX-pF->mnMinX);
        pF->mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(pF->mnMaxY-pF->mnMinY);

        pF->fx = K.at<float>(0,0);
        pF->fy = K.at<float>(1,1);
        pF->cx = K.at<float>(0,2);
        pF->cy = K.at<float>(1,2);
        pF->invfx = 1.0f/pF->fx;
        pF->invfy = 1.0f/pF->fy;

        pF->mbInitialComputations=false;
    }

    pF->mb = pF->mbf/pF->fx;

    Frame_AssignFeaturesToGrid(pF);
}


void Frame_init_4(Frame* pF,const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
{
    pF->mpORBvocabulary=voc;
    pF->mpORBextractorLeft=extractor;
    pF->mpORBextractorRight=static_cast<ORBextractor*>(NULL);
    pF->mTimeStamp=timeStamp;
    pF->mK=K.clone();
    pF->mDistCoef=distCoef.clone();
    pF->mbf=bf;
    pF->mThDepth=thDepth;

    // Frame ID
    pF->mnId=pF->nNextId++;

    // Scale Level Info
    pF->mnScaleLevels = ORBextractor_GetLevels(pF->mpORBextractorLeft);
    pF->mfScaleFactor = ORBextractor_GetScaleFactor(pF->mpORBextractorLeft);
    pF->mfLogScaleFactor = log(pF->mfScaleFactor);
    pF->mvScaleFactors = ORBextractor_GetScaleFactors(pF->mpORBextractorLeft);
    pF->mvInvScaleFactors = ORBextractor_GetInverseScaleFactors(pF->mpORBextractorLeft);
    pF->mvLevelSigma2 = ORBextractor_GetScaleSigmaSquares(pF->mpORBextractorLeft);
    pF->mvInvLevelSigma2 = ORBextractor_GetInverseScaleSigmaSquares(pF->mpORBextractorLeft);

    // ORB extraction
    Frame_ExtractORB(pF,0,imGray);

    pF->N = pF->mvKeys.size();

    if(pF->mvKeys.empty())
        return;

    Frame_UndistortKeyPoints(pF);

    // Set no stereo information
    pF->mvuRight = vector<float>(pF->N,-1);
    pF->mvDepth = vector<float>(pF->N,-1);

    pF->mvpMapPoints = vector<MapPoint*>(pF->N,static_cast<MapPoint*>(NULL));
    pF->mvbOutlier = vector<bool>(pF->N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(pF->mbInitialComputations)
    {
        Frame_ComputeImageBounds(pF,imGray);

        pF->mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(pF->mnMaxX-pF->mnMinX);
        pF->mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(pF->mnMaxY-pF->mnMinY);

        pF->fx = K.at<float>(0,0);
        pF->fy = K.at<float>(1,1);
        pF->cx = K.at<float>(0,2);
        pF->cy = K.at<float>(1,2);
        pF->invfx = 1.0f/pF->fx;
        pF->invfy = 1.0f/pF->fy;

        pF->mbInitialComputations=false;
    }

    pF->mb = pF->mbf/pF->fx;

    Frame_AssignFeaturesToGrid(pF);
}

void Frame_AssignFeaturesToGrid(Frame* pF)
{
    int nReserve = 0.5f*pF->N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            pF->mGrid[i][j].reserve(nReserve);

    for(int i=0;i<pF->N;i++)
    {
        const cv::KeyPoint &kp = pF->mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(Frame_PosInGrid(pF,kp,nGridPosX,nGridPosY))
            pF->mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame_ExtractORB(Frame* pF,int flag, const cv::Mat &im)
{
    /*if(flag==0)
        (*pF->mpORBextractorLeft,im,cv::Mat(),pF->mvKeys,pF->mDescriptors);
    else
        (*pF->mpORBextractorRight,im,cv::Mat(),pF->mvKeysRight,pF->mDescriptorsRight);
    */
    if(flag==0)
        ORBextractor_operator(pF->mpORBextractorLeft,im,cv::Mat(),pF->mvKeys,pF->mDescriptors);
    else
        ORBextractor_operator(pF->mpORBextractorRight,im,cv::Mat(),pF->mvKeysRight,pF->mDescriptorsRight);
    
}

void Frame_SetPose(Frame* pF,cv::Mat Tcw)
{
    pF->mTcw = Tcw.clone();
    Frame_UpdatePoseMatrices(pF);
}

void Frame_UpdatePoseMatrices(Frame* pF)
{ 
    pF->mRcw = pF->mTcw.rowRange(0,3).colRange(0,3);
    pF->mRwc = pF->mRcw.t();
    pF->mtcw = pF->mTcw.rowRange(0,3).col(3);
    pF->mOw = -pF->mRcw.t()*pF->mtcw;
}

bool Frame_isInFrustum(Frame* pF,MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = MapPoint_GetWorldPos(pMP); 

    // 3D in camera coordinates
    const cv::Mat Pc = pF->mRcw*P+pF->mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=pF->fx*PcX*invz+pF->cx;
    const float v=pF->fy*PcY*invz+pF->cy;

    if(u<pF->mnMinX || u>pF->mnMaxX)
        return false;
    if(v<pF->mnMinY || v>pF->mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = MapPoint_GetMaxDistanceInvariance(pMP);
    const float minDistance = MapPoint_GetMinDistanceInvariance(pMP);
    const cv::Mat PO = P-pF->mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = MapPoint_GetNormal(pMP);

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = MapPoint_PredictScale(pMP,dist,pF);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - pF->mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame_GetFeaturesInArea(Frame* pF,const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) 
{
    vector<size_t> vIndices;
    vIndices.reserve(pF->N);

    const int nMinCellX = max(0,(int)floor((x-pF->mnMinX-r)*pF->mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-pF->mnMinX+r)*pF->mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-pF->mnMinY-r)*pF->mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-pF->mnMinY+r)*pF->mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = pF->mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = pF->mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame_PosInGrid(Frame* pF,const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-pF->mnMinX)*pF->mfGridElementWidthInv);
    posY = round((kp.pt.y-pF->mnMinY)*pF->mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame_ComputeBoW(Frame* pF)
{
    if(pF->mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter_toDescriptorVector(pF->mDescriptors);
        pF->mpORBvocabulary->transform(vCurrentDesc,pF->mBowVec,pF->mFeatVec,4);
    }
}

void Frame_UndistortKeyPoints(Frame* pF)
{
    if(pF->mDistCoef.at<float>(0)==0.0)
    {
        pF->mvKeysUn=pF->mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(pF->N,2,CV_32F);
    for(int i=0; i<pF->N; i++)
    {
        mat.at<float>(i,0)=pF->mvKeys[i].pt.x;
        mat.at<float>(i,1)=pF->mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,pF->mK,pF->mDistCoef,cv::Mat(),pF->mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    pF->mvKeysUn.resize(pF->N);
    for(int i=0; i<pF->N; i++)
    {
        cv::KeyPoint kp = pF->mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        pF->mvKeysUn[i]=kp;
    }
}

void Frame_ComputeImageBounds(Frame* pF,const cv::Mat &imLeft)
{
    if(pF->mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,pF->mK,pF->mDistCoef,cv::Mat(),pF->mK);
        mat=mat.reshape(1);

        pF->mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        pF->mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        pF->mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        pF->mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        pF->mnMinX = 0.0f;
        pF->mnMaxX = imLeft.cols;
        pF->mnMinY = 0.0f;
        pF->mnMaxY = imLeft.rows;
    }
}

void Frame_ComputeStereoMatches(Frame* pF)
{
    pF->mvuRight = vector<float>(pF->N,-1.0f);
    pF->mvDepth = vector<float>(pF->N,-1.0f);

    const int thOrbDist = (TH_HIGH+TH_LOW)/2;

    const int nRows = pF->mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = pF->mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = pF->mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*pF->mvScaleFactors[pF->mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = pF->mb;
    const float minD = 0;
    const float maxD = pF->mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(pF->N);

    for(int iL=0; iL<pF->N; iL++)
    {
        const cv::KeyPoint &kpL = pF->mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = pF->mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = pF->mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = pF->mDescriptorsRight.row(iR);
                const int dist = ORBmatcher_DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = pF->mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = pF->mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = pF->mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= pF->mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = pF->mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = pF->mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                pF->mvDepth[iL]=pF->mbf/disparity;
                pF->mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            pF->mvuRight[vDistIdx[i].second]=-1;
            pF->mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


void Frame_ComputeStereoFromRGBD(Frame* pF,const cv::Mat &imDepth)
{
    pF->mvuRight = vector<float>(pF->N,-1);
    pF->mvDepth = vector<float>(pF->N,-1);

    for(int i=0; i<pF->N; i++)
    {
        const cv::KeyPoint &kp = pF->mvKeys[i];
        const cv::KeyPoint &kpU = pF->mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            pF->mvDepth[i] = d;
            pF->mvuRight[i] = kpU.pt.x-pF->mbf/d;
        }
    }
}

cv::Mat Frame_UnprojectStereo(Frame* pF,const int &i)
{
    const float z = pF->mvDepth[i];
    if(z>0)
    {
        const float u = pF->mvKeysUn[i].pt.x;
        const float v = pF->mvKeysUn[i].pt.y;
        const float x = (u-pF->cx)*z*pF->invfx;
        const float y = (v-pF->cy)*z*pF->invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return pF->mRwc*x3Dc+pF->mOw;
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM
