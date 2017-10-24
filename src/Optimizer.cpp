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
#include<iostream>
#include "../include/Optimizer.h"
#include<vector>
#include "../g2o/core/block_solver.h"
#include "../g2o/core/optimization_algorithm_levenberg.h"
#include "../g2o/core/robust_kernel_impl.h"
#include "../g2o/solvers/linear_solver_eigen.h"
#include "../g2o/solvers/linear_solver_dense.h"
#include "../g2o/types/types_six_dof_expmap.h"
#include "../g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "../include/Converter.h"

#include<mutex>

using namespace std;
using namespace cv;

/**
 * @brief Pose Only Optimization
 * 
 * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n
 * 只优化Frame的Tcw，不优化MapPoints的坐标
 * 
 * 1. Vertex: g2o::VertexSE3Expmap()，即当前帧的Tcw
 * 2. Edge:
 *     - g2o::EdgeSE3ProjectXYZOnlyPose()，BaseUnaryEdge
 *         + Vertex：待优化当前帧的Tcw
 *         + measurement：MapPoint在当前帧中的二维位置(u,v)
 *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *     - g2o::EdgeStereoSE3ProjectXYZOnlyPose()，BaseUnaryEdge
 *         + Vertex：待优化当前帧的Tcw
 *         + measurement：MapPoint在当前帧中的二维位置(ul,v,ur)
 *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *
 * @param   pFrame Frame
 * @return  inliers数量
 */

void toStandard(Mat &src,Mat& n){
    for(int i=0;i<src.rows;i++){
        n.at<double>(0,i)=src.at<Point2f>(i).x;
        n.at<double>(1,i)=src.at<Point2f>(i).y;
    }
}


Mat mergeCols(Mat A, Mat B)
{
    int totalCols = A.cols + B.cols;
    Mat mergedDescriptors( A.rows, totalCols,A.type());
    Mat submat = mergedDescriptors.colRange(0, A.cols);
    A.copyTo(submat);
    submat = mergedDescriptors.colRange(A.cols, totalCols);
    B.copyTo(submat);
    return mergedDescriptors;
}

Mat mergeRows(Mat A, Mat B)
{
    int totalRows = A.rows + B.rows;
    Mat mergedDescriptors(  totalRows,A.cols,A.type());
    Mat submat = mergedDescriptors.rowRange(0, A.rows);
    A.copyTo(submat);
    submat = mergedDescriptors.rowRange(A.rows, totalRows);
    B.copyTo(submat);
    return mergedDescriptors;
}

Mat PnP_pose(Mat pointbuf,Mat M1)
{
    Mat point3d_(49,3,CV_64F);
        int row=-1;
        for(int i=0;i<point3d_.rows;i++){
            point3d_.at<double>(i,0)=(i%7)*8;
            point3d_.at<double>(i,1)=((i%7)?row:++row)*8;
            point3d_.at<double>(i,2)=0;
        }
    Mat pointbuf_(2,49,CV_64F);
    toStandard(pointbuf,pointbuf_);
    Mat r_m,rvec,tvec;//cv::Mat()
    cv::solvePnPRansac(point3d_,pointbuf_.t(),M1,cv::Mat(),rvec,tvec,SOLVEPNP_EPNP);
    cv::Rodrigues(rvec,r_m);
    Mat C=(Mat_<double>(1,4) << 0, 0, 0, 1);
    return mergeRows(mergeCols(r_m,tvec),C);
}

int PoseOptimization(SENSOR s, const cv::Mat& P2, cv::Mat& pointbuf, Mat initial_, Mat pointbuf1)
{
    // Construct g2o optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex Tcw, transformation from the world coordinates to the current frame camera coordinate.
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(initial_));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = 49;
    std::vector<bool> mvbOutlier;
    mvbOutlier.reserve(N);

    // for Monocular
    std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    std::vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    // for Stereo
    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);

    // Add unary edge
    {
    int row=-1;
    for(int i=0; i<N; i++)
    {
        //MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(s==MONOCULAR)
        {
            // Monocular observation
            // 单目情况
                nInitialCorrespondences++;
                mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
//                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << pointbuf.at<Point2f>(i).x, pointbuf.at<Point2f>(i).y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
//                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity());

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = P2.at<double>(0,0);
                e->fy = P2.at<double>(1,1);
                e->cx = P2.at<double>(0,2);
                e->cy = P2.at<double>(1,2);
                //cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = (float)( (i%7)* 8);//8mm each with 7 circles a row
                e->Xw[1] = (float)(i%7?row:++row)*8;
                e->Xw[2] = (float)(0);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
          }
          else  // Stereo observation
          {
                nInitialCorrespondences++;
                mvbOutlier[i] = false;

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
//                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
//                const float &kp_ur = pFrame->mvuRight[i];
                obs << pointbuf.at<Point2f>(i).x, pointbuf.at<Point2f>(i).y, pointbuf1.at<Point2f>(i).x;
                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
//                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity();
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);

                e->fx = P2.at<double>(0,0);
                e->fy = P2.at<double>(1,1);
                e->cx = P2.at<double>(0,2);
                e->cy = P2.at<double>(1,2);
                e->bf = abs(P2.at<double>(0,3));
                e->Xw[0] = (float)( (i%7)* 8);//8mm each with 7 circles a row
                e->Xw[1] = (float)(i%7?row:++row)*8;
                e->Xw[2] = (float)(0);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
        }
    }
    }
    cout<<"Correspondences "<<nInitialCorrespondences<<endl;

//    if(nInitialCorrespondences<3)
//        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.

    //Threshold from the Chi2 distribution
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={1,30,50,15000000};

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(initial_));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        double sum =0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(mvbOutlier[idx])
            {
                e->computeError(); // NOTE add errors from outliers
            }

            const float chi2 = e->chi2();
            sum+=chi2;
//            if(chi2>chi2Mono[it])
//            {
//                mvbOutlier[idx]=true;
//                e->setLevel(1);                 // 设置为outlier
//                nBad++;
//            }
//            else
//            {
//                mvbOutlier[idx]=false;
//                e->setLevel(0);                 // 设置为inlier
//            }
cout<<"Error of node "<<i<<": "<<chi2<<endl;
            if(it==2)
                e->setRobustKernel(0); // 除了前两次优化需要RobustKernel以外, 其余的优化都不需要
        }
//assert()
//        eigen::a
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();
            sum+=chi2;
//            if(chi2>chi2Stereo[it])
//            {
//                mvbOutlier[idx]=true;
//                e->setLevel(1);
//                nBad++;
//            }
//            else
//            {
//                e->setLevel(0);
//                mvbOutlier[idx]=false;
//            }
cout<<"Error of node "<<i<<": "<<chi2<<endl;
            if(it==2)
                e->setRobustKernel(0);
        }
        cout<<"Iteration times :"<<its[it]<<" Sum of error :"<< sum<<endl;
        if(optimizer.edges().size()<10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    if(pose.empty())cout <<"Result Not Available!"<<endl;
    else
    cout<<"Optimized Pose: "<<pose<<endl;
//    cout<<"cross product: "<<pose.colRange(1,1)<<endl;
//    cout<<"cross product: "<<pose.colRange(2,3).dot(pose.colRange(2,3))<<endl;
//    cout<<"Optimized Pose: "<<pose<<endl;
    //pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

