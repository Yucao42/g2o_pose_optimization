//#include "include/Converter.h"
#include"../include/Optimizer.h"
#include <iostream>
#include <opencv2/core/opengl.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/viz.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace std;
using namespace cv;
using namespace cv::viz;

std::string intrinsic_filename = "../data/zed_final/intrinsics.yml";
std::string extrinsic_filename = "../data/zed_final/extrinsics.yml";
Mat R, T, R1, P1, R2, P2,Q;
Mat M1, D1, M2, D2;
Size boardSize(7,7);

void save_points3d(Mat& points3d){
    ;
}

void chessboard(string file,Mat& pointbuf_,bool left_cam){
    Mat frame=imread(file,IMREAD_GRAYSCALE),pointbuf;
    bool found = findCirclesGrid( frame, boardSize, pointbuf );
    if(found)
        drawChessboardCorners( frame, boardSize, pointbuf, found );
    else cout<<"F*"<<endl;
    if(left_cam)
        undistortPoints(pointbuf,pointbuf_,M1,D1,R1,P1);
    else
        undistortPoints(pointbuf,pointbuf_,M2,D2,R2,P2);
}

void rectify()
{
    float scale =1;
    if( !intrinsic_filename.empty() )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, FileStorage::READ);

        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        M1 *= scale;
        M2 *= scale;

        fs.open(extrinsic_filename, FileStorage::READ);

        fs["R"] >> R;
        fs["T"] >> T;
        fs["R1"] >> R1;
        fs["R2"] >> R2;
        fs["P1"] >> P1;
        fs["P2"] >> P2;
    }
}


int main(){

    string left="../data/4/Left08.png",right="../data/4/Right08.png",test="../data/4/Left07.png";
    Mat pointbuf1,pointbuf2,pointbuf3;
    rectify();
    chessboard(right,pointbuf2,false);
    chessboard(test,pointbuf3,true);
    chessboard(left,pointbuf1,true);
    double start = double(getTickCount());
    Mat Guess = PnP_pose(pointbuf1,P1.rowRange(0,3).colRange(0,3));
    double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
    std::cout << "It took " << duration_ms << " ms." << std::endl;
    Mat Guess1 = PnP_pose(pointbuf3,P1.rowRange(0,3).colRange(0,3));
    cout<<"Initial Pose: "<<Guess1.rowRange(0,3)<<endl;
    PoseOptimization(MONOCULAR,P1,pointbuf1,Guess1);
    cout<<"PnP Pose: "<<Guess.rowRange(1,2)<<endl;
              
    // Create the pointcloud
    std::vector<cv::Vec3d> point_cloud;
    int row =-1;
    for (int i = 0; i < 49; ++i) {
      cv::Vec3d point3d(             (double)( (i%7)* 8),//8mm each with 7 circles a row
          (double)(i%7?row:++row)*8,0);
      point_cloud.push_back(point3d);
    }
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.setBackgroundColor(viz::Color::white()); // black by default

    viz::WCloud cloud_widget(point_cloud, viz::Color::green());
    Vec3d t1(Guess.rowRange(0,3).colRange(3,4));
    Vec3d t2(Guess1.rowRange(0,3).colRange(3,4));
    Affine3d campos=Affine3d(Guess.rowRange(0,3).colRange(0,3).t(),-t1);
    Affine3d campos1=Affine3d(Guess1.rowRange(0,3).colRange(0,3).t(),-t2);
    Matx33d K = Matx33d( 500, 0, 640,
                       0, 500, 360,
                       0, 0,  1);


    for (size_t i = 0; i < point_cloud.size(); ++i)
    {
      Vec3d point = point_cloud[i];
      Affine3d point_pose(Mat::eye(3,3,CV_64F), point);

      char buffer[50];
      sprintf (buffer, "%d", static_cast<int>(i));

      viz::WCube cube_widget(Point3f(1,1,0.0), Point3f(0.0,0.0,1), true, viz::Color::red());
                 cube_widget.setRenderingProperty(viz::LINE_WIDTH, 3.0);
      myWindow.showWidget("Cube"+string(buffer), cube_widget, point_pose);
    }


    viz::WCameraPosition cpw,cpw1; // Coordinate axes
    viz::WCameraPosition cpw_frustum(K, 10, viz::Color::blue()),cpw_frustum1(K, 10, viz::Color::blue()); // Camera frustum
    vector<Affine3d> path_est;path_est.push_back(campos);path_est.push_back(campos1);
    myWindow.showWidget("CPW", cpw, campos);
    myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, campos);
    myWindow.showWidget("CPW1", cpw1, campos1);
    myWindow.showWidget("CPW_FRUSTUM1", cpw_frustum1, campos1);
    myWindow.showWidget("cloud", cloud_widget);
    myWindow.showWidget("cameras_frames_and_lines_est", viz::WTrajectory(path_est, viz::WTrajectory::PATH, 1.0, viz::Color::green()));
    myWindow.spin();

    return 0;
}
