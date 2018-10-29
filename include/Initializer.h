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
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
struct Initializer
{
    typedef pair<int,int> Match;

    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;

    // Current pair<int,int>es from Reference to Current
    vector<pair<int,int>> mvmatches12;
    vector<bool> mvbmatched1;

    // Calibration
    cv::Mat mK;

    // Standard Deviation and Variance
    float mSigma, mSigma2;

    // Ransac max iterations
    int mMaxIterations;

    // Ransac sets
    vector<vector<size_t> > mvSets;   

};
    // Fix the reference frame
    void Initializer_init(Initializer *pIni, const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initializer_Initialize(Initializer *pIni,const Frame &CurrentFrame, const vector<int> &vmatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);


    void Initializer_FindHomography(Initializer *pIni,vector<bool> &vbmatchesInliers, float &score, cv::Mat &H21);
    void Initializer_FindFundamental(Initializer *pIni,vector<bool> &vbInliers, float &score, cv::Mat &F21);

    cv::Mat Initializer_ComputeH21(Initializer *pIni,const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    cv::Mat Initializer_ComputeF21(Initializer *pIni,const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    float Initializer_CheckHomography(Initializer *pIni,const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbmatchesInliers, float sigma);

    float Initializer_CheckFundamental(Initializer *pIni,const cv::Mat &F21, vector<bool> &vbmatchesInliers, float sigma);

    bool Initializer_ReconstructF(Initializer *pIni,vector<bool> &vbmatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    bool Initializer_ReconstructH(Initializer *pIni,vector<bool> &vbmatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    void Initializer_Triangulate(Initializer *pIni,const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    void Initializer_Normalize(Initializer *pIni,const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    int Initializer_CheckRT(Initializer *pIni,const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<pair<int,int>> &vmatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    void Initializer_DecomposeE(Initializer *pIni,const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


} //namespace ORB_SLAM

#endif // INITIALIZER_H
