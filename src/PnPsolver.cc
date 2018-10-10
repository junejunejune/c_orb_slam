/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
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

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#include <iostream>

#include "PnPsolver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include "Thirdparty/DBoW2/DUtils/Random.h"
#include <algorithm>

using namespace std;

namespace ORB_SLAM2
{


void PnPsolver_init(PnPsolver *pSolver, const Frame &F, const vector<MapPoint*> &vpMapPointMatches)
{  
  
    pSolver->pws=0; pSolver->us=0; pSolver->alphas=0; pSolver->pcs=0; pSolver->maximum_number_of_correspondences=0; pSolver->number_of_correspondences=0; pSolver->mnInliersi=0;
    pSolver->mnIterations=0; pSolver->mnBestInliers=0; pSolver->N=0;
    pSolver->mvpMapPointMatches = vpMapPointMatches;
    pSolver->mvP2D.reserve(F.mvpMapPoints.size());
    pSolver->mvSigma2.reserve(F.mvpMapPoints.size());
    pSolver->mvP3Dw.reserve(F.mvpMapPoints.size());
    pSolver->mvKeyPointIndices.reserve(F.mvpMapPoints.size());
    pSolver->mvAllIndices.reserve(F.mvpMapPoints.size());

    int idx=0;
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];

        if(pMP)
        {
            if(!pMP->isBad())
            {
                const cv::KeyPoint &kp = F.mvKeysUn[i];

                pSolver->mvP2D.push_back(kp.pt);
                pSolver->mvSigma2.push_back(F.mvLevelSigma2[kp.octave]);

                cv::Mat Pos = pMP->GetWorldPos();
                pSolver->mvP3Dw.push_back(cv::Point3f(Pos.at<float>(0),Pos.at<float>(1), Pos.at<float>(2)));

                pSolver->mvKeyPointIndices.push_back(i);
                pSolver->mvAllIndices.push_back(idx);               

                idx++;
            }
        }
    }

    // Set camera calibration parameters
    pSolver->fu = F.fx;
    pSolver->fv = F.fy;
    pSolver->uc = F.cx;
    pSolver->vc = F.cy;

   PnPsolver_SetRansacParameters(pSolver);
}
/*
PnPsolver_~PnPsolver()
{
  delete [] pws;
  delete [] us;
  delete [] alphas;
  delete [] pcs;
}
*/

void PnPsolver_SetRansacParameters(PnPsolver *pSolver, double probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2)
{
    pSolver->mRansacProb = probability;
    pSolver->mRansacMinInliers = minInliers;
    pSolver->mRansacMaxIts = maxIterations;
    pSolver->mRansacEpsilon = epsilon;
    pSolver->mRansacMinSet = minSet;

    pSolver->N = pSolver->mvP2D.size(); // number of correspondences

    pSolver->mvbInliersi.resize(pSolver->N);

    // Adjust Parameters according to number of correspondences
    int nMinInliers = pSolver->N*pSolver->mRansacEpsilon;
    if(nMinInliers<pSolver->mRansacMinInliers)
        nMinInliers=pSolver->mRansacMinInliers;
    if(nMinInliers<minSet)
        nMinInliers=minSet;
    pSolver->mRansacMinInliers = nMinInliers;

    if(pSolver->mRansacEpsilon<(float)pSolver->mRansacMinInliers/pSolver->N)
        pSolver->mRansacEpsilon=(float)pSolver->mRansacMinInliers/pSolver->N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(pSolver->mRansacMinInliers==pSolver->N)
        nIterations=1;
    else
        nIterations = ceil(log(1-pSolver->mRansacProb)/log(1-pow(pSolver->mRansacEpsilon,3)));

    pSolver->mRansacMaxIts = max(1,min(nIterations,pSolver->mRansacMaxIts));

    pSolver->mvMaxError.resize(pSolver->mvSigma2.size());
    for(size_t i=0; i<pSolver->mvSigma2.size(); i++)
        pSolver->mvMaxError[i] = pSolver->mvSigma2[i]*th2;
}

cv::Mat PnPsolver_find(PnPsolver *pSolver, vector<bool> &vbInliers, int &nInliers)
{
    bool bFlag;
    return PnPsolver_iterate(pSolver, pSolver->mRansacMaxIts,bFlag,vbInliers,nInliers);    
}

cv::Mat PnPsolver_iterate(PnPsolver *pSolver, int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers.clear();
    nInliers=0;

    PnPsolver_set_maximum_number_of_correspondences(pSolver,pSolver->mRansacMinSet);

    if(pSolver->N<pSolver->mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    vector<size_t> vAvailableIndices;

    int nCurrentIterations = 0;
    while(pSolver->mnIterations<pSolver->mRansacMaxIts || nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        pSolver->mnIterations++;
        PnPsolver_reset_correspondences(pSolver);

        vAvailableIndices = pSolver->mvAllIndices;

        // Get min set of points
        for(short i = 0; i < pSolver->mRansacMinSet; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            PnPsolver_add_correspondence(pSolver,pSolver->mvP3Dw[idx].x,pSolver->mvP3Dw[idx].y,pSolver->mvP3Dw[idx].z,pSolver->mvP2D[idx].x,pSolver->mvP2D[idx].y);

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // Compute camera pose
        PnPsolver_compute_pose(pSolver,pSolver->mRi, pSolver->mti);

        // Check inliers
        PnPsolver_CheckInliers(pSolver);

        if(pSolver->mnInliersi>=pSolver->mRansacMinInliers)
        {
            // If it is the best solution so far, save it
            if(pSolver->mnInliersi>pSolver->mnBestInliers)
            {
                pSolver->mvbBestInliers = pSolver->mvbInliersi;
                pSolver->mnBestInliers = pSolver->mnInliersi;

                cv::Mat Rcw(3,3,CV_64F,pSolver->mRi);
                cv::Mat tcw(3,1,CV_64F,pSolver->mti);
                Rcw.convertTo(Rcw,CV_32F);
                tcw.convertTo(tcw,CV_32F);
                pSolver->mBestTcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(pSolver->mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(pSolver->mBestTcw.rowRange(0,3).col(3));
            }

            if(PnPsolver_Refine(pSolver))
            {
                nInliers = pSolver->mnRefinedInliers;
                vbInliers = vector<bool>(pSolver->mvpMapPointMatches.size(),false);
                for(int i=0; i<pSolver->N; i++)
                {
                    if(pSolver->mvbRefinedInliers[i])
                        vbInliers[pSolver->mvKeyPointIndices[i]] = true;
                }
                return pSolver->mRefinedTcw.clone();
            }

        }
    }

    if(pSolver->mnIterations>=pSolver->mRansacMaxIts)
    {
        bNoMore=true;
        if(pSolver->mnBestInliers>=pSolver->mRansacMinInliers)
        {
            nInliers=pSolver->mnBestInliers;
            vbInliers = vector<bool>(pSolver->mvpMapPointMatches.size(),false);
            for(int i=0; i<pSolver->N; i++)
            {
                if(pSolver->mvbBestInliers[i])
                    vbInliers[pSolver->mvKeyPointIndices[i]] = true;
            }
            return pSolver->mBestTcw.clone();
        }
    }

    return cv::Mat();
}

bool PnPsolver_Refine(PnPsolver *pSolver)
{
    vector<int> vIndices;
    vIndices.reserve(pSolver->mvbBestInliers.size());

    for(size_t i=0; i<pSolver->mvbBestInliers.size(); i++)
    {
        if(pSolver->mvbBestInliers[i])
        {
            vIndices.push_back(i);
        }
    }

    PnPsolver_set_maximum_number_of_correspondences(pSolver, vIndices.size());

    PnPsolver_reset_correspondences(pSolver);

    for(size_t i=0; i<vIndices.size(); i++)
    {
        int idx = vIndices[i];
        PnPsolver_add_correspondence(pSolver, pSolver->mvP3Dw[idx].x,pSolver->mvP3Dw[idx].y,pSolver->mvP3Dw[idx].z,pSolver->mvP2D[idx].x,pSolver->mvP2D[idx].y);
    }

    // Compute camera pose
    PnPsolver_compute_pose(pSolver, pSolver->mRi, pSolver->mti);

    // Check inliers
    PnPsolver_CheckInliers(pSolver);

    pSolver->mnRefinedInliers =pSolver->mnInliersi;
    pSolver->mvbRefinedInliers = pSolver->mvbInliersi;

    if(pSolver->mnInliersi>pSolver->mRansacMinInliers)
    {
        cv::Mat Rcw(3,3,CV_64F,pSolver->mRi);
        cv::Mat tcw(3,1,CV_64F,pSolver->mti);
        Rcw.convertTo(Rcw,CV_32F);
        tcw.convertTo(tcw,CV_32F);
        pSolver->mRefinedTcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(pSolver->mRefinedTcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(pSolver->mRefinedTcw.rowRange(0,3).col(3));
        return true;
    }

    return false;
}


void PnPsolver_CheckInliers(PnPsolver *pSolver )
{
    pSolver->mnInliersi=0;

    for(int i=0; i<pSolver->N; i++)
    {
        cv::Point3f P3Dw = pSolver->mvP3Dw[i];
        cv::Point2f P2D = pSolver->mvP2D[i];

        float Xc = pSolver->mRi[0][0]*P3Dw.x+pSolver->mRi[0][1]*P3Dw.y+pSolver->mRi[0][2]*P3Dw.z+pSolver->mti[0];
        float Yc = pSolver->mRi[1][0]*P3Dw.x+pSolver->mRi[1][1]*P3Dw.y+pSolver->mRi[1][2]*P3Dw.z+pSolver->mti[1];
        float invZc = 1/(pSolver->mRi[2][0]*P3Dw.x+pSolver->mRi[2][1]*P3Dw.y+pSolver->mRi[2][2]*P3Dw.z+pSolver->mti[2]);

        double ue = pSolver->uc + pSolver->fu * Xc * invZc;
        double ve = pSolver->vc + pSolver->fv * Yc * invZc;

        float distX = P2D.x-ue;
        float distY = P2D.y-ve;

        float error2 = distX*distX+distY*distY;

        if(error2<pSolver->mvMaxError[i])
        {
            pSolver->mvbInliersi[i]=true;
            pSolver->mnInliersi++;
        }
        else
        {
            pSolver->mvbInliersi[i]=false;
        }
    }
}


void PnPsolver_set_maximum_number_of_correspondences(PnPsolver *pSolver, int n)
{
  if (pSolver->maximum_number_of_correspondences < n) {
    if (pSolver->pws != 0) delete [] pSolver->pws;
    if (pSolver->us != 0) delete [] pSolver->us;
    if (pSolver->alphas != 0) delete [] pSolver->alphas;
    if (pSolver->pcs != 0) delete [] pSolver->pcs;

    pSolver->maximum_number_of_correspondences = n;
    pSolver->pws = new double[3 * pSolver->maximum_number_of_correspondences];
    pSolver->us = new double[2 * pSolver->maximum_number_of_correspondences];
    pSolver->alphas = new double[4 * pSolver->maximum_number_of_correspondences];
    pSolver->pcs = new double[3 * pSolver->maximum_number_of_correspondences];
  }
}

void PnPsolver_reset_correspondences(PnPsolver *pSolver)
{
  pSolver->number_of_correspondences = 0;
}

void PnPsolver_add_correspondence(PnPsolver *pSolver, double X, double Y, double Z, double u, double v)
{
  pSolver->pws[3 * pSolver->number_of_correspondences    ] = X;
  pSolver->pws[3 * pSolver->number_of_correspondences + 1] = Y;
  pSolver->pws[3 * pSolver->number_of_correspondences + 2] = Z;

  pSolver->us[2 * pSolver->number_of_correspondences    ] = u;
  pSolver->us[2 * pSolver->number_of_correspondences + 1] = v;

  pSolver->number_of_correspondences++;
}

void PnPsolver_choose_control_points(PnPsolver *pSolver)
{
  // Take C0 as the reference points centroid:
  pSolver->cws[0][0] = pSolver->cws[0][1] = pSolver->cws[0][2] = 0;
  for(int i = 0; i < pSolver->number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      pSolver->cws[0][j] += pSolver->pws[3 * i + j];

  for(int j = 0; j < 3; j++)
    pSolver->cws[0][j] /= pSolver->number_of_correspondences;


  // Take C1, C2, and C3 from PCA on the reference points:
  CvMat * PW0 = cvCreateMat(pSolver->number_of_correspondences, 3, CV_64F);

  double pw0tpw0[3 * 3], dc[3], uct[3 * 3];
  CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0);
  CvMat DC      = cvMat(3, 1, CV_64F, dc);
  CvMat UCt     = cvMat(3, 3, CV_64F, uct);

  for(int i = 0; i < pSolver->number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      PW0->data.db[3 * i + j] = pSolver->pws[3 * i + j] - pSolver->cws[0][j];

  cvMulTransposed(PW0, &PW0tPW0, 1);
  cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

  cvReleaseMat(&PW0);

  for(int i = 1; i < 4; i++) {
    double k = sqrt(dc[i - 1] / pSolver->number_of_correspondences);
    for(int j = 0; j < 3; j++)
      pSolver->cws[i][j] = pSolver->cws[0][j] + k * uct[3 * (i - 1) + j];
  }
}

void PnPsolver_compute_barycentric_coordinates(PnPsolver *pSolver)
{
  double cc[3 * 3], cc_inv[3 * 3];
  CvMat CC     = cvMat(3, 3, CV_64F, cc);
  CvMat CC_inv = cvMat(3, 3, CV_64F, cc_inv);

  for(int i = 0; i < 3; i++)
    for(int j = 1; j < 4; j++)
      cc[3 * i + j - 1] = pSolver->cws[j][i] - pSolver->cws[0][i];

  cvInvert(&CC, &CC_inv, CV_SVD);
  double * ci = cc_inv;
  for(int i = 0; i < pSolver->number_of_correspondences; i++) {
    double * pi = pSolver->pws + 3 * i;
    double * a = pSolver->alphas + 4 * i;

    for(int j = 0; j < 3; j++)
      a[1 + j] =
	ci[3 * j    ] * (pi[0] - pSolver->cws[0][0]) +
	ci[3 * j + 1] * (pi[1] - pSolver->cws[0][1]) +
	ci[3 * j + 2] * (pi[2] - pSolver->cws[0][2]);
    a[0] = 1.0f - a[1] - a[2] - a[3];
  }
}

void PnPsolver_fill_M(PnPsolver *pSolver, CvMat * M,
		  const int row, const double * as, const double u, const double v)
{
  double * M1 = M->data.db + row * 12;
  double * M2 = M1 + 12;

  for(int i = 0; i < 4; i++) {
    M1[3 * i    ] = as[i] * pSolver->fu;
    M1[3 * i + 1] = 0.0;
    M1[3 * i + 2] = as[i] * (pSolver->uc - u);

    M2[3 * i    ] = 0.0;
    M2[3 * i + 1] = as[i] * pSolver->fv;
    M2[3 * i + 2] = as[i] * (pSolver->vc - v);
  }
}

void PnPsolver_compute_ccs(PnPsolver *pSolver, const double * betas, const double * ut)
{
  for(int i = 0; i < 4; i++)
    pSolver->ccs[i][0] = pSolver->ccs[i][1] = pSolver->ccs[i][2] = 0.0f;

  for(int i = 0; i < 4; i++) {
    const double * v = ut + 12 * (11 - i);
    for(int j = 0; j < 4; j++)
      for(int k = 0; k < 3; k++)
	pSolver->ccs[j][k] += betas[i] * v[3 * j + k];
  }
}

void PnPsolver_compute_pcs(PnPsolver *pSolver)
{
  for(int i = 0; i < pSolver->number_of_correspondences; i++) {
    double * a = pSolver->alphas + 4 * i;
    double * pc = pSolver->pcs + 3 * i;

    for(int j = 0; j < 3; j++)
      pc[j] = a[0] * pSolver->ccs[0][j] + a[1] * pSolver->ccs[1][j] + a[2] * pSolver->ccs[2][j] + a[3] * pSolver->ccs[3][j];
  }
}

double PnPsolver_compute_pose(PnPsolver *pSolver, double R[3][3], double t[3])
{
  PnPsolver_choose_control_points(pSolver);
  PnPsolver_compute_barycentric_coordinates(pSolver);

  CvMat * M = cvCreateMat(2 * pSolver->number_of_correspondences, 12, CV_64F);

  for(int i = 0; i < pSolver->number_of_correspondences; i++)
   PnPsolver_fill_M(pSolver,M, 2 * i, pSolver->alphas + 4 * i, pSolver->us[2 * i], pSolver->us[2 * i + 1]);

  double mtm[12 * 12], d[12], ut[12 * 12];
  CvMat MtM = cvMat(12, 12, CV_64F, mtm);
  CvMat D   = cvMat(12,  1, CV_64F, d);
  CvMat Ut  = cvMat(12, 12, CV_64F, ut);

  cvMulTransposed(M, &MtM, 1);
  cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
  cvReleaseMat(&M);

  double l_6x10[6 * 10], rho[6];
  CvMat L_6x10 = cvMat(6, 10, CV_64F, l_6x10);
  CvMat Rho    = cvMat(6,  1, CV_64F, rho);

  PnPsolver_compute_L_6x10(pSolver, ut, l_6x10);
  PnPsolver_compute_rho(pSolver, rho);

  double Betas[4][4], rep_errors[4];
  double Rs[4][3][3], ts[4][3];

  PnPsolver_find_betas_approx_1(pSolver,&L_6x10, &Rho, Betas[1]);
  PnPsolver_gauss_newton(pSolver,&L_6x10, &Rho, Betas[1]);
  rep_errors[1] = PnPsolver_compute_R_and_t(pSolver,ut, Betas[1], Rs[1], ts[1]);

  PnPsolver_find_betas_approx_2(pSolver,&L_6x10, &Rho, Betas[2]);
  PnPsolver_gauss_newton(pSolver,&L_6x10, &Rho, Betas[2]);
  rep_errors[2] = PnPsolver_compute_R_and_t(pSolver,ut, Betas[2], Rs[2], ts[2]);

  PnPsolver_find_betas_approx_3(pSolver,&L_6x10, &Rho, Betas[3]);
  PnPsolver_gauss_newton(pSolver,&L_6x10, &Rho, Betas[3]);
  rep_errors[3] = PnPsolver_compute_R_and_t(pSolver,ut, Betas[3], Rs[3], ts[3]);

  int N = 1;
  if (rep_errors[2] < rep_errors[1]) N = 2;
  if (rep_errors[3] < rep_errors[N]) N = 3;

  PnPsolver_copy_R_and_t(pSolver,Rs[N], ts[N], R, t);

  return rep_errors[N];
}

void PnPsolver_copy_R_and_t(PnPsolver *pSolver, const double R_src[3][3], const double t_src[3],
			double R_dst[3][3], double t_dst[3])
{
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++)
      R_dst[i][j] = R_src[i][j];
    t_dst[i] = t_src[i];
  }
}

double PnPsolver_dist2(PnPsolver *pSolver, const double * p1, const double * p2)
{
  return
    (p1[0] - p2[0]) * (p1[0] - p2[0]) +
    (p1[1] - p2[1]) * (p1[1] - p2[1]) +
    (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

double PnPsolver_dot(PnPsolver *pSolver, const double * v1, const double * v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

double PnPsolver_reprojection_error(PnPsolver *pSolver, const double R[3][3], const double t[3])
{
  double sum2 = 0.0;

  for(int i = 0; i < pSolver->number_of_correspondences; i++) {
    double * pw = pSolver->pws + 3 * i;
    double Xc = PnPsolver_dot(pSolver,R[0], pw) + t[0];
    double Yc = PnPsolver_dot(pSolver,R[1], pw) + t[1];
    double inv_Zc = 1.0 / (PnPsolver_dot(pSolver, R[2], pw) + t[2]);
    double ue = pSolver->uc + pSolver->fu * Xc * inv_Zc;
    double ve = pSolver->vc + pSolver->fv * Yc * inv_Zc;
    double u = pSolver->us[2 * i], v = pSolver->us[2 * i + 1];

    sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
  }

  return sum2 / pSolver->number_of_correspondences;
}

void PnPsolver_estimate_R_and_t(PnPsolver *pSolver, double R[3][3], double t[3])
{
  double pc0[3], pw0[3];

  pc0[0] = pc0[1] = pc0[2] = 0.0;
  pw0[0] = pw0[1] = pw0[2] = 0.0;

  for(int i = 0; i < pSolver->number_of_correspondences; i++) {
    const double * pc = pSolver->pcs + 3 * i;
    const double * pw = pSolver->pws + 3 * i;

    for(int j = 0; j < 3; j++) {
      pc0[j] += pc[j];
      pw0[j] += pw[j];
    }
  }
  for(int j = 0; j < 3; j++) {
    pc0[j] /= pSolver->number_of_correspondences;
    pw0[j] /= pSolver->number_of_correspondences;
  }

  double abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
  CvMat ABt   = cvMat(3, 3, CV_64F, abt);
  CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);
  CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);
  CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);

  cvSetZero(&ABt);
  for(int i = 0; i < pSolver->number_of_correspondences; i++) {
    double * pc = pSolver->pcs + 3 * i;
    double * pw = pSolver->pws + 3 * i;

    for(int j = 0; j < 3; j++) {
      abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
      abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
      abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
    }
  }

  cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      R[i][j] = PnPsolver_dot(pSolver,abt_u + 3 * i, abt_v + 3 * j);

  const double det =
    R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
    R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

  if (det < 0) {
    R[2][0] = -R[2][0];
    R[2][1] = -R[2][1];
    R[2][2] = -R[2][2];
  }

  t[0] = pc0[0] - PnPsolver_dot(pSolver, R[0], pw0);
  t[1] = pc0[1] - PnPsolver_dot(pSolver, R[1], pw0);
  t[2] = pc0[2] - PnPsolver_dot(pSolver, R[2], pw0);
}

void PnPsolver_print_pose(PnPsolver *pSolver, const double R[3][3], const double t[3])
{
  cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
  cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
  cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}

void PnPsolver_solve_for_sign(PnPsolver *pSolver)
{
  if (pSolver->pcs[2] < 0.0) {
    for(int i = 0; i < 4; i++)
      for(int j = 0; j < 3; j++)
	pSolver->ccs[i][j] = -pSolver->ccs[i][j];

    for(int i = 0; i < pSolver->number_of_correspondences; i++) {
      pSolver->pcs[3 * i    ] = -pSolver->pcs[3 * i];
      pSolver->pcs[3 * i + 1] = -pSolver->pcs[3 * i + 1];
      pSolver->pcs[3 * i + 2] = -pSolver->pcs[3 * i + 2];
    }
  }
}

double PnPsolver_compute_R_and_t(PnPsolver *pSolver, const double * ut, const double * betas,
			     double R[3][3], double t[3])
{
  PnPsolver_compute_ccs(pSolver,betas, ut);
  PnPsolver_compute_pcs(pSolver);

  PnPsolver_solve_for_sign(pSolver);

  PnPsolver_estimate_R_and_t(pSolver,R, t);

  return PnPsolver_reprojection_error(pSolver, R, t);
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

void PnPsolver_find_betas_approx_1(PnPsolver *pSolver, const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  double l_6x4[6 * 4], b4[4];
  CvMat L_6x4 = cvMat(6, 4, CV_64F, l_6x4);
  CvMat B4    = cvMat(4, 1, CV_64F, b4);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x4, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x4, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x4, i, 2, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x4, i, 3, cvmGet(L_6x10, i, 6));
  }

  cvSolve(&L_6x4, Rho, &B4, CV_SVD);

  if (b4[0] < 0) {
    betas[0] = sqrt(-b4[0]);
    betas[1] = -b4[1] / betas[0];
    betas[2] = -b4[2] / betas[0];
    betas[3] = -b4[3] / betas[0];
  } else {
    betas[0] = sqrt(b4[0]);
    betas[1] = b4[1] / betas[0];
    betas[2] = b4[2] / betas[0];
    betas[3] = b4[3] / betas[0];
  }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void PnPsolver_find_betas_approx_2(PnPsolver *pSolver, const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  double l_6x3[6 * 3], b3[3];
  CvMat L_6x3  = cvMat(6, 3, CV_64F, l_6x3);
  CvMat B3     = cvMat(3, 1, CV_64F, b3);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x3, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x3, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x3, i, 2, cvmGet(L_6x10, i, 2));
  }

  cvSolve(&L_6x3, Rho, &B3, CV_SVD);

  if (b3[0] < 0) {
    betas[0] = sqrt(-b3[0]);
    betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
  } else {
    betas[0] = sqrt(b3[0]);
    betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
  }

  if (b3[1] < 0) betas[0] = -betas[0];

  betas[2] = 0.0;
  betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

void PnPsolver_find_betas_approx_3(PnPsolver *pSolver, const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  double l_6x5[6 * 5], b5[5];
  CvMat L_6x5 = cvMat(6, 5, CV_64F, l_6x5);
  CvMat B5    = cvMat(5, 1, CV_64F, b5);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x5, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x5, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x5, i, 2, cvmGet(L_6x10, i, 2));
    cvmSet(&L_6x5, i, 3, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x5, i, 4, cvmGet(L_6x10, i, 4));
  }

  cvSolve(&L_6x5, Rho, &B5, CV_SVD);

  if (b5[0] < 0) {
    betas[0] = sqrt(-b5[0]);
    betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
  } else {
    betas[0] = sqrt(b5[0]);
    betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
  }
  if (b5[1] < 0) betas[0] = -betas[0];
  betas[2] = b5[3] / betas[0];
  betas[3] = 0.0;
}

void PnPsolver_compute_L_6x10(PnPsolver *pSolver, const double * ut, double * l_6x10)
{
  const double * v[4];

  v[0] = ut + 12 * 11;
  v[1] = ut + 12 * 10;
  v[2] = ut + 12 *  9;
  v[3] = ut + 12 *  8;

  double dv[4][6][3];

  for(int i = 0; i < 4; i++) {
    int a = 0, b = 1;
    for(int j = 0; j < 6; j++) {
      dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];
      dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
      dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

      b++;
      if (b > 3) {
	a++;
	b = a + 1;
      }
    }
  }

  for(int i = 0; i < 6; i++) {
    double * row = l_6x10 + 10 * i;

    row[0] =        PnPsolver_dot(pSolver, dv[0][i], dv[0][i]);
    row[1] = 2.0f * PnPsolver_dot(pSolver, dv[0][i], dv[1][i]);
    row[2] =        PnPsolver_dot(pSolver, dv[1][i], dv[1][i]);
    row[3] = 2.0f * PnPsolver_dot(pSolver, dv[0][i], dv[2][i]);
    row[4] = 2.0f * PnPsolver_dot(pSolver, dv[1][i], dv[2][i]);
    row[5] =        PnPsolver_dot(pSolver, dv[2][i], dv[2][i]);
    row[6] = 2.0f * PnPsolver_dot(pSolver, dv[0][i], dv[3][i]);
    row[7] = 2.0f * PnPsolver_dot(pSolver, dv[1][i], dv[3][i]);
    row[8] = 2.0f * PnPsolver_dot(pSolver, dv[2][i], dv[3][i]);
    row[9] =        PnPsolver_dot(pSolver, dv[3][i], dv[3][i]);
  }
}

void PnPsolver_compute_rho(PnPsolver *pSolver, double * rho)
{
  rho[0] = PnPsolver_dist2(pSolver, pSolver->cws[0], pSolver->cws[1]);
  rho[1] = PnPsolver_dist2(pSolver, pSolver->cws[0], pSolver->cws[2]);
  rho[2] = PnPsolver_dist2(pSolver, pSolver->cws[0], pSolver->cws[3]);
  rho[3] = PnPsolver_dist2(pSolver, pSolver->cws[1], pSolver->cws[2]);
  rho[4] = PnPsolver_dist2(pSolver, pSolver->cws[1], pSolver->cws[3]);
  rho[5] = PnPsolver_dist2(pSolver, pSolver->cws[2], pSolver->cws[3]);
}

void PnPsolver_compute_A_and_b_gauss_newton(PnPsolver *pSolver, const double * l_6x10, const double * rho,
					double betas[4], CvMat * A, CvMat * b)
{
  for(int i = 0; i < 6; i++) {
    const double * rowL = l_6x10 + i * 10;
    double * rowA = A->data.db + i * 4;

    rowA[0] = 2 * rowL[0] * betas[0] +     rowL[1] * betas[1] +     rowL[3] * betas[2] +     rowL[6] * betas[3];
    rowA[1] =     rowL[1] * betas[0] + 2 * rowL[2] * betas[1] +     rowL[4] * betas[2] +     rowL[7] * betas[3];
    rowA[2] =     rowL[3] * betas[0] +     rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +     rowL[8] * betas[3];
    rowA[3] =     rowL[6] * betas[0] +     rowL[7] * betas[1] +     rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

    cvmSet(b, i, 0, rho[i] -
	   (
	    rowL[0] * betas[0] * betas[0] +
	    rowL[1] * betas[0] * betas[1] +
	    rowL[2] * betas[1] * betas[1] +
	    rowL[3] * betas[0] * betas[2] +
	    rowL[4] * betas[1] * betas[2] +
	    rowL[5] * betas[2] * betas[2] +
	    rowL[6] * betas[0] * betas[3] +
	    rowL[7] * betas[1] * betas[3] +
	    rowL[8] * betas[2] * betas[3] +
	    rowL[9] * betas[3] * betas[3]
	    ));
  }
}

void PnPsolver_gauss_newton(PnPsolver *pSolver, const CvMat * L_6x10, const CvMat * Rho,
			double betas[4])
{
  const int iterations_number = 5;

  double a[6*4], b[6], x[4];
  CvMat A = cvMat(6, 4, CV_64F, a);
  CvMat B = cvMat(6, 1, CV_64F, b);
  CvMat X = cvMat(4, 1, CV_64F, x);

  for(int k = 0; k < iterations_number; k++) {
    PnPsolver_compute_A_and_b_gauss_newton(pSolver,L_6x10->data.db, Rho->data.db,
				 betas, &A, &B);
    PnPsolver_qr_solve(pSolver,&A, &B, &X);

    for(int i = 0; i < 4; i++)
      betas[i] += x[i];
  }
}

void PnPsolver_qr_solve(PnPsolver *pSolver, CvMat * A, CvMat * b, CvMat * X)
{
  static int max_nr = 0;
  static double * A1, * A2;

  const int nr = A->rows;
  const int nc = A->cols;

  if (max_nr != 0 && max_nr < nr) {
    delete [] A1;
    delete [] A2;
  }
  if (max_nr < nr) {
    max_nr = nr;
    A1 = new double[nr];
    A2 = new double[nr];
  }

  double * pA = A->data.db, * ppAkk = pA;
  for(int k = 0; k < nc; k++) {
    double * ppAik = ppAkk, eta = fabs(*ppAik);
    for(int i = k + 1; i < nr; i++) {
      double elt = fabs(*ppAik);
      if (eta < elt) eta = elt;
      ppAik += nc;
    }

    if (eta == 0) {
      A1[k] = A2[k] = 0.0;
      cerr << "God damnit, A is singular, this shouldn't happen." << endl;
      return;
    } else {
      double * ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
      for(int i = k; i < nr; i++) {
	*ppAik *= inv_eta;
	sum += *ppAik * *ppAik;
	ppAik += nc;
      }
      double sigma = sqrt(sum);
      if (*ppAkk < 0)
	sigma = -sigma;
      *ppAkk += sigma;
      A1[k] = sigma * *ppAkk;
      A2[k] = -eta * sigma;
      for(int j = k + 1; j < nc; j++) {
	double * ppAik = ppAkk, sum = 0;
	for(int i = k; i < nr; i++) {
	  sum += *ppAik * ppAik[j - k];
	  ppAik += nc;
	}
	double tau = sum / A1[k];
	ppAik = ppAkk;
	for(int i = k; i < nr; i++) {
	  ppAik[j - k] -= tau * *ppAik;
	  ppAik += nc;
	}
      }
    }
    ppAkk += nc + 1;
  }

  // b <- Qt b
  double * ppAjj = pA, * pb = b->data.db;
  for(int j = 0; j < nc; j++) {
    double * ppAij = ppAjj, tau = 0;
    for(int i = j; i < nr; i++)	{
      tau += *ppAij * pb[i];
      ppAij += nc;
    }
    tau /= A1[j];
    ppAij = ppAjj;
    for(int i = j; i < nr; i++) {
      pb[i] -= tau * *ppAij;
      ppAij += nc;
    }
    ppAjj += nc + 1;
  }

  // X = R-1 b
  double * pX = X->data.db;
  pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
  for(int i = nc - 2; i >= 0; i--) {
    double * ppAij = pA + i * nc + (i + 1), sum = 0;

    for(int j = i + 1; j < nc; j++) {
      sum += *ppAij * pX[j];
      ppAij++;
    }
    pX[i] = (pb[i] - sum) / A2[i];
  }
}



void PnPsolver_relative_error(PnPsolver *pSolver, double & rot_err, double & transl_err,
			  const double Rtrue[3][3], const double ttrue[3],
			  const double Rest[3][3],  const double test[3])
{
  double qtrue[4], qest[4];

  PnPsolver_mat_to_quat(pSolver,Rtrue, qtrue);
  PnPsolver_mat_to_quat(pSolver,Rest, qest);

  double rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
			 (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
			 (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
			 (qtrue[3] - qest[3]) * (qtrue[3] - qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  double rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
			 (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
			 (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
			 (qtrue[3] + qest[3]) * (qtrue[3] + qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  rot_err = min(rot_err1, rot_err2);

  transl_err =
    sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
	 (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
	 (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
    sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}

void PnPsolver_mat_to_quat(PnPsolver *pSolver, const double R[3][3], double q[4])
{
  double tr = R[0][0] + R[1][1] + R[2][2];
  double n4;

  if (tr > 0.0f) {
    q[0] = R[1][2] - R[2][1];
    q[1] = R[2][0] - R[0][2];
    q[2] = R[0][1] - R[1][0];
    q[3] = tr + 1.0f;
    n4 = q[3];
  } else if ( (R[0][0] > R[1][1]) && (R[0][0] > R[2][2]) ) {
    q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
    q[1] = R[1][0] + R[0][1];
    q[2] = R[2][0] + R[0][2];
    q[3] = R[1][2] - R[2][1];
    n4 = q[0];
  } else if (R[1][1] > R[2][2]) {
    q[0] = R[1][0] + R[0][1];
    q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
    q[2] = R[2][1] + R[1][2];
    q[3] = R[2][0] - R[0][2];
    n4 = q[1];
  } else {
    q[0] = R[2][0] + R[0][2];
    q[1] = R[2][1] + R[1][2];
    q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
    q[3] = R[0][1] - R[1][0];
    n4 = q[2];
  }
  double scale = 0.5f / double(sqrt(n4));

  q[0] *= scale;
  q[1] *= scale;
  q[2] *= scale;
  q[3] *= scale;
}

} //namespace ORB_SLAM
