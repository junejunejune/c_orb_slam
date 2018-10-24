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

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

    std::vector<cv::Mat> Converter_toDescriptorVector(const cv::Mat &Descriptors);

    g2o::SE3Quat Converter_toSE3Quat(const cv::Mat &cvT);
    g2o::SE3Quat Converter_toSE3Quat(const g2o::Sim3 &gSim3);

    cv::Mat Converter_toCvMat(const g2o::SE3Quat &SE3);
    cv::Mat Converter_toCvMat(const g2o::Sim3 &Sim3);
    cv::Mat Converter_toCvMat(const Eigen::Matrix<double,4,4> &m);
    cv::Mat Converter_toCvMat(const Eigen::Matrix3d &m);
    cv::Mat Converter_toCvMat(const Eigen::Matrix<double,3,1> &m);
    cv::Mat Converter_toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    Eigen::Matrix<double,3,1> Converter_toVector3d(const cv::Mat &cvVector);
    Eigen::Matrix<double,3,1> Converter_toVector3d(const cv::Point3f &cvPoint);
    Eigen::Matrix<double,3,3> Converter_toMatrix3d(const cv::Mat &cvMat3);

    std::vector<float> Converter_toQuaternion(const cv::Mat &M);

}// namespace ORB_SLAM

#endif // CONVERTER_H
