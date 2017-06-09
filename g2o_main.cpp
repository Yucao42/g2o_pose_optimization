//#include "include/Converter.h"
#include"include/Optimizer.h"
#include <iostream>
#include <opencv2/core/opengl.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace std;
using namespace cv;

std::string intrinsic_filename = "zed_final/intrinsics.yml";
std::string extrinsic_filename = "zed_final/extrinsics.yml";
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
    imshow("yes1",frame);
    if(left_cam)
        undistortPoints(pointbuf,pointbuf_,M1,D1,R1,P1);
    else
        undistortPoints(pointbuf,pointbuf_,M2,D2,R2,P2);

//    cout<<pointbuf_<<endl;
}

//Mat mergeCols(Mat A, Mat B)
//{
//    int totalCols = A.cols + B.cols;
//    Mat mergedDescriptors( A.rows, totalCols,A.type());
//    Mat submat = mergedDescriptors.colRange(0, A.cols);
//    A.copyTo(submat);
//    submat = mergedDescriptors.colRange(A.cols, totalCols);
//    B.copyTo(submat);
//    return mergedDescriptors;
//}

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
    string left="4/Left08.png",right="4/Right08.png",test="4/Left06.png";
    Mat pointbuf1,pointbuf2,pointbuf3;
    rectify();
   // eigen::assert()
    chessboard(right,pointbuf2,false);
    chessboard(test,pointbuf3,true);
    chessboard(left,pointbuf1,true);
        double start = double(getTickCount());
    Mat Guess = PnP_pose(pointbuf1,P1.rowRange(0,3).colRange(0,3));
    double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
    std::cout << "It took " << duration_ms << " ms." << std::endl;
    Mat Guess1 = PnP_pose(pointbuf3,P1.rowRange(0,3).colRange(0,3));
    cout<<"Initial Pose: "<<Guess.rowRange(0,3)<<endl;
//    PoseOptimization(MONOCULAR,P1,pointbuf1,Guess);
        PoseOptimization(MONOCULAR,P1,pointbuf1,Guess);
//    PoseOptimization(MONOCULAR,P1,pointbuf1,Mat::eye(4,4,CV_64F));
//    PoseOptimization(STEREO,P2,pointbuf1,Guess1,pointbuf2);
//        PoseOptimization(STEREO,P2,pointbuf1,Mat::eye(4,4,CV_64F),pointbuf2);
//    cout<<pointbuf1.size()<<endl;
    waitKey(0);
    return 0;
}
