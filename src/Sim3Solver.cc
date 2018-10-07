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


#include "Sim3Solver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM2
{


void Sim3Solver_init(Sim3Solver* mpSim3Solver,KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12, const bool bFixScale)
{  
  
    mpSim3Solver->mnIterations=0;
    mpSim3Solver->mnBestInliers=0;
    mpSim3Solver->mbFixScale=bFixScale;

    mpSim3Solver->mpKF1 = pKF1;
    mpSim3Solver->mpKF2 = pKF2;

    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    mpSim3Solver->mN1 = vpMatched12.size();

    mpSim3Solver->mvpMapPoints1.reserve(mpSim3Solver->mN1);
    mpSim3Solver->mvpMapPoints2.reserve(mpSim3Solver->mN1);
    mpSim3Solver->mvpMatches12 = vpMatched12;
    mpSim3Solver->mvnIndices1.reserve(mpSim3Solver->mN1);
    mpSim3Solver->mvX3Dc1.reserve(mpSim3Solver->mN1);
    mpSim3Solver->mvX3Dc2.reserve(mpSim3Solver->mN1);

    cv::Mat Rcw1 = pKF1->GetRotation();
    cv::Mat tcw1 = pKF1->GetTranslation();
    cv::Mat Rcw2 = pKF2->GetRotation();
    cv::Mat tcw2 = pKF2->GetTranslation();

    mpSim3Solver->mvAllIndices.reserve(mpSim3Solver->mN1);

    size_t idx=0;
    for(int i1=0; i1<mpSim3Solver->mN1; i1++)
    {
        if(vpMatched12[i1])
        {
            MapPoint* pMP1 = vpKeyFrameMP1[i1];
            MapPoint* pMP2 = vpMatched12[i1];

            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
            int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

            if(indexKF1<0 || indexKF2<0)
                continue;

            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];

            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];

            mpSim3Solver->mvnMaxError1.push_back(9.210*sigmaSquare1);
            mpSim3Solver->mvnMaxError2.push_back(9.210*sigmaSquare2);

            mpSim3Solver->mvpMapPoints1.push_back(pMP1);
            mpSim3Solver->mvpMapPoints2.push_back(pMP2);
            mpSim3Solver->mvnIndices1.push_back(i1);

            cv::Mat X3D1w = pMP1->GetWorldPos();
            mpSim3Solver->mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);

            cv::Mat X3D2w = pMP2->GetWorldPos();
            mpSim3Solver->mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            mpSim3Solver->mvAllIndices.push_back(idx);
            idx++;
        }
    }

    mpSim3Solver->mK1 = pKF1->mK;
    mpSim3Solver->mK2 = pKF2->mK;

    Sim3Solver_FromCameraToImage(mpSim3Solver,mpSim3Solver->mvX3Dc1,mpSim3Solver->mvP1im1,mpSim3Solver->mK1);
    Sim3Solver_FromCameraToImage(mpSim3Solver,mpSim3Solver->mvX3Dc2,mpSim3Solver->mvP2im2,mpSim3Solver->mK2);

    Sim3Solver_SetRansacParameters(mpSim3Solver);
}

void Sim3Solver_SetRansacParameters(Sim3Solver* mpSim3Solver,double probability, int minInliers, int maxIterations)
{
    mpSim3Solver->mRansacProb = probability;
    mpSim3Solver->mRansacMinInliers = minInliers;
    mpSim3Solver->mRansacMaxIts = maxIterations;    

    mpSim3Solver->N = mpSim3Solver->mvpMapPoints1.size(); // number of correspondences

    mpSim3Solver->mvbInliersi.resize(mpSim3Solver->N);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)mpSim3Solver->mRansacMinInliers/mpSim3Solver->N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mpSim3Solver->mRansacMinInliers==mpSim3Solver->N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mpSim3Solver->mRansacProb)/log(1-pow(epsilon,3)));

    mpSim3Solver->mRansacMaxIts = max(1,min(nIterations,mpSim3Solver->mRansacMaxIts));

    mpSim3Solver->mnIterations = 0;
}

cv::Mat Sim3Solver_iterate(Sim3Solver* mpSim3Solver,int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers = vector<bool>(mpSim3Solver->mN1,false);
    nInliers=0;

    if(mpSim3Solver->N<mpSim3Solver->mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    vector<size_t> vAvailableIndices;

    cv::Mat P3Dc1i(3,3,CV_32F);
    cv::Mat P3Dc2i(3,3,CV_32F);

    int nCurrentIterations = 0;
    while(mpSim3Solver->mnIterations<mpSim3Solver->mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mpSim3Solver->mnIterations++;

        vAvailableIndices = mpSim3Solver->mvAllIndices;

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            mpSim3Solver->mvX3Dc1[idx].copyTo(P3Dc1i.col(i));
            mpSim3Solver->mvX3Dc2[idx].copyTo(P3Dc2i.col(i));

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        Sim3Solver_ComputeSim3(mpSim3Solver,P3Dc1i,P3Dc2i);

        Sim3Solver_CheckInliers(mpSim3Solver);

        if(mpSim3Solver->mnInliersi>=mpSim3Solver->mnBestInliers)
        {
            mpSim3Solver->mvbBestInliers = mpSim3Solver->mvbInliersi;
            mpSim3Solver->mnBestInliers = mpSim3Solver->mnInliersi;
            mpSim3Solver->mBestT12 = mpSim3Solver->mT12i.clone();
            mpSim3Solver->mBestRotation = mpSim3Solver->mR12i.clone();
            mpSim3Solver->mBestTranslation = mpSim3Solver->mt12i.clone();
            mpSim3Solver->mBestScale = mpSim3Solver->ms12i;

            if(mpSim3Solver->mnInliersi>mpSim3Solver->mRansacMinInliers)
            {
                nInliers = mpSim3Solver->mnInliersi;
                for(int i=0; i<mpSim3Solver->N; i++)
                    if(mpSim3Solver->mvbInliersi[i])
                        vbInliers[mpSim3Solver->mvnIndices1[i]] = true;
                return mpSim3Solver->mBestT12;
            }
        }
    }

    if(mpSim3Solver->mnIterations>=mpSim3Solver->mRansacMaxIts)
        bNoMore=true;

    return cv::Mat();
}

cv::Mat Sim3Solver_find(Sim3Solver* mpSim3Solver,vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return Sim3Solver_iterate(mpSim3Solver, mpSim3Solver->mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

void Sim3Solver_ComputeCentroid(Sim3Solver* mpSim3Solver,cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    cv::reduce(P,C,1,CV_REDUCE_SUM);
    C = C/P.cols;

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;
    }
}

void Sim3Solver_ComputeSim3(Sim3Solver* mpSim3Solver,cv::Mat &P1, cv::Mat &P2)
{
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Step 1: Centroid and relative coordinates

    cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
    cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
    cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
    cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

    Sim3Solver_ComputeCentroid(mpSim3Solver,P1,Pr1,O1);
    Sim3Solver_ComputeCentroid(mpSim3Solver,P2,Pr2,O2);

    // Step 2: Compute M matrix

    cv::Mat M = Pr2*Pr1.t();

    // Step 3: Compute N matrix

    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);


    // Step 4: Eigenvector of the highest eigenvalue

    cv::Mat eval, evec;

    cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

    cv::Mat vec(1,3,evec.type());
    (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    double ang=atan2(norm(vec),evec.at<float>(0,0));

    vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

    mpSim3Solver->mR12i.create(3,3,P1.type());

    cv::Rodrigues(vec,mpSim3Solver->mR12i); // computes the rotation matrix from angle-axis

    // Step 5: Rotate set 2

    cv::Mat P3 = mpSim3Solver->mR12i*Pr2;

    // Step 6: Scale

    if(!mpSim3Solver->mbFixScale)
    {
        double nom = Pr1.dot(P3);
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        cv::pow(P3,2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        mpSim3Solver->ms12i = nom/den;
    }
    else
        mpSim3Solver->ms12i = 1.0f;

    // Step 7: Translation

    mpSim3Solver->mt12i.create(1,3,P1.type());
    mpSim3Solver->mt12i = O1 - mpSim3Solver->ms12i*mpSim3Solver->mR12i*O2;

    // Step 8: Transformation

    // Step 8.1 T12
    mpSim3Solver->mT12i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sR = mpSim3Solver->ms12i*mpSim3Solver->mR12i;

    sR.copyTo(mpSim3Solver->mT12i.rowRange(0,3).colRange(0,3));
    mpSim3Solver->mt12i.copyTo(mpSim3Solver->mT12i.rowRange(0,3).col(3));

    // Step 8.2 T21

    mpSim3Solver->mT21i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sRinv = (1.0/mpSim3Solver->ms12i)*mpSim3Solver->mR12i.t();

    sRinv.copyTo(mpSim3Solver->mT21i.rowRange(0,3).colRange(0,3));
    cv::Mat tinv = -sRinv*mpSim3Solver->mt12i;
    tinv.copyTo(mpSim3Solver->mT21i.rowRange(0,3).col(3));
}


void Sim3Solver_CheckInliers(Sim3Solver* mpSim3Solver)
{
    vector<cv::Mat> vP1im2, vP2im1;
    Sim3Solver_Project(mpSim3Solver,mpSim3Solver->mvX3Dc2,vP2im1,mpSim3Solver->mT12i,mpSim3Solver->mK1);
    Sim3Solver_Project(mpSim3Solver,mpSim3Solver->mvX3Dc1,vP1im2,mpSim3Solver->mT21i,mpSim3Solver->mK2);

    mpSim3Solver->mnInliersi=0;

    for(size_t i=0; i<mpSim3Solver->mvP1im1.size(); i++)
    {
        cv::Mat dist1 = mpSim3Solver->mvP1im1[i]-vP2im1[i];
        cv::Mat dist2 = vP1im2[i]-mpSim3Solver->mvP2im2[i];

        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        if(err1<mpSim3Solver->mvnMaxError1[i] && err2<mpSim3Solver->mvnMaxError2[i])
        {
            mpSim3Solver->mvbInliersi[i]=true;
            mpSim3Solver->mnInliersi++;
        }
        else
            mpSim3Solver->mvbInliersi[i]=false;
    }
}


cv::Mat Sim3Solver_GetEstimatedRotation(Sim3Solver* mpSim3Solver)
{
    return mpSim3Solver->mBestRotation.clone();
}

cv::Mat Sim3Solver_GetEstimatedTranslation(Sim3Solver* mpSim3Solver)
{
    return mpSim3Solver->mBestTranslation.clone();
}

float Sim3Solver_GetEstimatedScale(Sim3Solver* mpSim3Solver)
{
    return mpSim3Solver->mBestScale;
}

void Sim3Solver_Project(Sim3Solver* mpSim3Solver,const vector<cv::Mat> &vP3Dw, vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K)
{
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        cv::Mat P3Dc = Rcw*vP3Dw[i]+tcw;
        const float invz = 1/(P3Dc.at<float>(2));
        const float x = P3Dc.at<float>(0)*invz;
        const float y = P3Dc.at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

void Sim3Solver_FromCameraToImage(Sim3Solver* mpSim3Solver,const vector<cv::Mat> &vP3Dc, vector<cv::Mat> &vP2D, cv::Mat K)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const float invz = 1/(vP3Dc[i].at<float>(2));
        const float x = vP3Dc[i].at<float>(0)*invz;
        const float y = vP3Dc[i].at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

} //namespace ORB_SLAM
