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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

/*#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"*/
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


    enum SENSOR{
        STEREO,
        MONOCULAR
    };

cv::Mat PnP_pose(cv::Mat pointbuf,cv::Mat M1);
    int PoseOptimization(SENSOR s, const cv::Mat& P2, cv::Mat& pointbuf, cv::Mat initial_=cv::Mat::eye(4,4,CV_64F), cv::Mat pointbuf1=cv::Mat());


#endif // OPTIMIZER_H
