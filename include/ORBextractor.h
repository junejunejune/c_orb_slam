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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

struct ExtractorNode
{
    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};
void ExtractorNode_init(ExtractorNode* pEN);

void ExtractorNode_DivideNode(ExtractorNode* pEN,ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);


struct ORBextractor
{

    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    std::vector<cv::Mat> mvImagePyramid;

   std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};
    void ORBextractor_init(ORBextractor* mpORBe, int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void ORBextractor_operator(ORBextractor* mpORBe,cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);


    int inline ORBextractor_GetLevels(ORBextractor* mpORBe){
        return mpORBe->nlevels;}

    float inline ORBextractor_GetScaleFactor(ORBextractor* mpORBe){
        return mpORBe->scaleFactor;}

    std::vector<float> inline ORBextractor_GetScaleFactors(ORBextractor* mpORBe){
        return mpORBe->mvScaleFactor;
    }

    std::vector<float> inline ORBextractor_GetInverseScaleFactors(ORBextractor* mpORBe){
        return mpORBe->mvInvScaleFactor;
    }

    std::vector<float> inline ORBextractor_GetScaleSigmaSquares(ORBextractor* mpORBe){
        return mpORBe->mvLevelSigma2;
    }

    std::vector<float> inline ORBextractor_GetInverseScaleSigmaSquares(ORBextractor* mpORBe){
        return mpORBe->mvInvLevelSigma2;
    }

    void ORBextractor_ComputePyramid(ORBextractor* mpORBe,cv::Mat image);

    void ORBextractor_ComputeKeyPointsOctTree(ORBextractor* mpORBe,std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    
    std::vector<cv::KeyPoint> ORBextractor_DistributeOctTree(ORBextractor* mpORBe,const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ORBextractor_ComputeKeyPointsOld(ORBextractor* mpORBe,std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
 
} //namespace ORB_SLAM

#endif

