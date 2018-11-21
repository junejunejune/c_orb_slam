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

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2
{

struct PnPsolver 
{

  double uc, vc, fu, fv;

  double * pws, * us, * alphas, * pcs;
  int maximum_number_of_correspondences;
  int number_of_correspondences;

  double cws[4][3], ccs[4][3];
  double cws_determinant;

  vector<MapPoint*> mvpMapPointMatches;

  // 2D Points
  vector<cv::Point2f> mvP2D;
  vector<float> mvSigma2;

  // 3D Points
  vector<cv::Point3f> mvP3Dw;

  // Index in Frame
  vector<size_t> mvKeyPointIndices;

  // Current Estimation
  double mRi[3][3];
  double mti[3];
  cv::Mat mTcwi;
  vector<bool> mvbInliersi;
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  vector<bool> mvbBestInliers;
  int mnBestInliers;
  cv::Mat mBestTcw;

  // Refined
  cv::Mat mRefinedTcw;
  vector<bool> mvbRefinedInliers;
  int mnRefinedInliers;

  // Number of Correspondences
  int N;

  // Indices for random selection [0 .. N-1]
  vector<size_t> mvAllIndices;

  // RANSAC probability
  double mRansacProb;

  // RANSAC min inliers
  int mRansacMinInliers;

  // RANSAC max iterations
  int mRansacMaxIts;

  // RANSAC expected inliers/total ratio
  float mRansacEpsilon;

  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float mRansacTh;

  // RANSAC Minimun Set used at each iteration
  int mRansacMinSet;

  // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
  vector<float> mvMaxError;

};
  void PnPsolver_init(PnPsolver* pPnP,const Frame *F, const vector<MapPoint*> &vpMapPointMatches);

  void PnPsolver_SetRansacParameters(PnPsolver* pPnP,double probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4,
                           float th2 = 5.991);

  cv::Mat PnPsolver_find(PnPsolver* pPnP,vector<bool> &vbInliers, int &nInliers);

  cv::Mat PnPsolver_iterate(PnPsolver* pPnP,int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

  void PnPsolver_CheckInliers(PnPsolver* pPnP);
  bool PnPsolver_Refine(PnPsolver* pPnP);

  // Functions from the original EPnP code
  void PnPsolver_set_maximum_number_of_correspondences(PnPsolver* pPnP,const int n);
  void PnPsolver_reset_correspondences(PnPsolver* pPnP);
  void PnPsolver_add_correspondence(PnPsolver* pPnP,const double X, const double Y, const double Z,
              const double u, const double v);

  double PnPsolver_compute_pose(PnPsolver* pPnP,double R[3][3], double T[3]);

  void PnPsolver_relative_error(PnPsolver* pPnP,double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  void PnPsolver_print_pose(PnPsolver* pPnP,const double R[3][3], const double t[3]);
  double PnPsolver_reprojection_error(PnPsolver* pPnP,const double R[3][3], const double t[3]);

  void PnPsolver_choose_control_points(PnPsolver* pPnP);
  void PnPsolver_compute_barycentric_coordinates(PnPsolver* pPnP);
  void PnPsolver_fill_M(PnPsolver* pPnP,CvMat * M, const int row, const double * alphas, const double u, const double v);
  void PnPsolver_compute_ccs(PnPsolver* pPnP,const double * betas, const double * ut);
  void PnPsolver_compute_pcs(PnPsolver* pPnP);

  void PnPsolver_solve_for_sign(PnPsolver* pPnP);

  void PnPsolver_find_betas_approx_1(PnPsolver* pPnP,const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void PnPsolver_find_betas_approx_2(PnPsolver* pPnP,const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void PnPsolver_find_betas_approx_3(PnPsolver* pPnP,const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void PnPsolver_qr_solve(PnPsolver* pPnP,CvMat * A, CvMat * b, CvMat * X);

  double PnPsolver_dot(PnPsolver* pPnP,const double * v1, const double * v2);
  double PnPsolver_dist2(PnPsolver* pPnP,const double * p1, const double * p2);

  void PnPsolver_compute_rho(PnPsolver* pPnP,double * rho);
  void PnPsolver_compute_L_6x10(PnPsolver* pPnP,const double * ut, double * l_6x10);

  void PnPsolver_gauss_newton(PnPsolver* pPnP,const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  void PnPsolver_compute_A_and_b_gauss_newton(PnPsolver* pPnP,const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  double PnPsolver_compute_R_and_t(PnPsolver* pPnP,const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  void PnPsolver_estimate_R_and_t(PnPsolver* pPnP,double R[3][3], double t[3]);

  void PnPsolver_copy_R_and_t(PnPsolver* pPnP,const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  void PnPsolver_mat_to_quat(PnPsolver* pPnP,const double R[3][3], double q[4]);


} //namespace ORB_SLAM

#endif //PNPSOLVER_H
