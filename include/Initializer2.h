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

    // Fix the reference frame
     // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference to Current
    vector<Match> mvMatches12;
    vector<bool> mvbMatched1;

    // Calibration
    cv::Mat mK;

    // Standard Deviation and Variance
    float mSigma, mSigma2;

    // Ransac max iterations
    int mMaxIterations;

    // Ransac sets
    vector<vector<size_t> > mvSets;   

};
    void Initializer_init(Initializer* mpInitializer, const Frame &ReferenceFrame, float sigma =1.0, int iterations=200 );

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initializer_Initialize(Initializer* mpInitializer, const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);

    void Initializer_FindHomography(Initializer* mpInitializer, vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    void Initializer_FindFundamental(Initializer* mpInitializer, vector<bool> &vbInliers, float &score, cv::Mat &F21);

    cv::Mat Initializer_ComputeH21(Initializer* mpInitializer, const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    cv::Mat Initializer_ComputeF21(Initializer* mpInitializer, const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    float Initializer_CheckHomography(Initializer* mpInitializer, const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

    float Initializer_CheckFundamental(Initializer* mpInitializer, const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    bool Initializer_ReconstructF(Initializer* mpInitializer, vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    bool Initializer_ReconstructH(Initializer* mpInitializer, vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    void Initializer_Triangulate(Initializer* mpInitializer, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    void Initializer_Normalize(Initializer* mpInitializer, const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    int Initializer_CheckRT(Initializer* mpInitializer, const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<pair<int,int>> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    void Initializer_DecomposeE(Initializer* mpInitializer, const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);



} //namespace ORB_SLAM

#endif // INITIALIZER_H
