//#include "include/Converter.h"
//#include"include/Optimizer.h"
//#include <iostream>
//#include <opencv2/core/opengl.hpp>
//#include "opencv2/opencv.hpp"
//#include "opencv2/features2d.hpp"
//#include <string>
//#include <vector>
//#include <algorithm>
//#include <numeric>

//using namespace std;
//using namespace cv;

//std::string intrinsic_filename = "zed_final/intrinsics.yml";
//std::string extrinsic_filename = "zed_final/extrinsics.yml";
//Mat R, T, R1, P1, R2, P2,Q;
//Mat M1, D1, M2, D2;
//Size boardSize(7,7);

//void save_points3d(Mat& points3d){
//    ;
//}

//void chessboard(string file,Mat& pointbuf_,bool left_cam){
//    Mat frame=imread(file,IMREAD_GRAYSCALE),pointbuf;
////    bool found = findChessboardCorners( frame, boardSize, pointbuf,
////        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
//    bool found = findCirclesGrid( frame, boardSize, pointbuf );
//    if(found)//return 0;
//                drawChessboardCorners( frame, boardSize, pointbuf, found );
//    else cout<<"F*"<<endl;
//    imshow("yes1",frame);
//    if(left_cam)
//        undistortPoints(pointbuf,pointbuf_,M1,D1,R1,P1);
//    else
//        undistortPoints(pointbuf,pointbuf_,M2,D2,R2,P2);

//    cout<<pointbuf_<<endl;
//}

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

//void rectify(Mat& img1,Mat& img2 ){
//    Rect roi1, roi2;
//    Size img_size = img1.size();
//    float scale =1;
//    if( !intrinsic_filename.empty() )
//    {
//        // reading intrinsic parameters
//        FileStorage fs(intrinsic_filename, FileStorage::READ);

//        fs["M1"] >> M1;
//        fs["D1"] >> D1;
//        fs["M2"] >> M2;
//        fs["D2"] >> D2;

//        M1 *= scale;
//        M2 *= scale;

//        fs.open(extrinsic_filename, FileStorage::READ);

//        fs["R"] >> R;
//        fs["T"] >> T;

//        Rect validRoi[2];
//        //                                                                CALIB_ZERO_DISPARITY
//        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &validRoi[0], &validRoi[1] );
//        Mat map11, map12, map21, map22;
//        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
//        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
//        Mat img1r, img2r;
//        cv::remap(img1, img1r, map11, map12, INTER_LINEAR);
//        cv::remap(img2, img2r, map21, map22, INTER_LINEAR);
//    }
//}

//void print_distance(Mat & points3d)
//{
//    float q;
//    Point3f d;
//    for (int i=0;i<47;i++){
//        d=Point3f(points3d.at<float>(0,i),points3d.at<float>(1,i),points3d.at<float>(2,i))-Point3f(points3d.at<float>(0,i+1),points3d.at<float>(1,i+1),points3d.at<float>(2,i+1));
//        q=sqrt(d.dot(d));
//        cout<<"distance between the "<<i<<" th and the "<< i+1<<" th point is "<<q<<" mm."<<endl;

//    }
////    cout<<endl;
//}

//Mat reconstruct_stereo(string left,string right)
//{
////#define matlab__
//        Mat frame1=imread(left,IMREAD_GRAYSCALE);//filenamer005...data/filenamer005.jpg
//        Mat frame2=imread(right,IMREAD_GRAYSCALE);
//        rectify(frame1,frame2);
//        imshow("y1",frame1);
//        Mat points3d;
//       Mat pointbuf,pointbuf_,pointbuf1_,pointbuf1;
//       bool found = findCirclesGrid( frame1, boardSize, pointbuf );
//       bool found1 = findCirclesGrid( frame2, boardSize, pointbuf1);//,
////           CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
//               if(found)//return 0;
//                   drawChessboardCorners( frame1, boardSize, pointbuf, found );
//               else cout<<"F*"<<endl;
//               if(found1)//return 0;
//                   drawChessboardCorners( frame2, boardSize, pointbuf1, found1 );
////        cout<<pointbuf<<endl;
//               imshow("yes",frame2);imshow("yes1",frame1);
//#ifndef matlab__
//        cout<<pointbuf.size()<<endl;
//    undistortPoints(pointbuf,pointbuf_,M1,D1,R1,P1);
//    undistortPoints(pointbuf1,pointbuf1_,M2,D2,R2,P2);
//    triangulatePoints(P1,P2,pointbuf_,pointbuf1_,points3d);
//    for (int i=0;i<48;i++){
//        points3d.at<float>(0,i)=points3d.at<float>(0,i)/points3d.at<float>(3,i);
//        points3d.at<float>(1,i)=points3d.at<float>(1,i)/points3d.at<float>(3,i);
//        points3d.at<float>(2,i)=points3d.at<float>(2,i)/points3d.at<float>(3,i);
//points3d.at<float>(3,i)=1;
////        points3d(i,1)=0;        points3d(i,0)=0; points3d(i,2)=0; points3d(i,3)=0;
//    }

//    Mat point3d=points3d.rowRange(0,3);
////    cout<<endl;
////    cout<<point3d<<endl;
////    print_distance(points3d);
////waitKey(0);
//#else
//    Mat I=Mat::eye(3,3,CV_64F),z=Mat::zeros(3,1,CV_64F);
//    cout<<mergeCols(R,T)<<endl;
//    Mat var2=mergeCols(R,T);
//    Mat var1=mergeCols(I,z);
//    Mat P11=M1*var1;
//    Mat P22=M2*var2;
//    cout<<P11<<endl;
//    undistortPoints(pointbuf,pointbuf_,M1,D1);
//    undistortPoints(pointbuf1,pointbuf1_,M2,D2);
//    triangulatePoints(P11,P22,pointbuf_,pointbuf1_,points3d);
//    for (int i=0;i<48;i++){
//        points3d.at<float>(0,i)=points3d.at<float>(0,i)/points3d.at<float>(3,i);
//        points3d.at<float>(1,i)=points3d.at<float>(1,i)/points3d.at<float>(3,i);
//        points3d.at<float>(2,i)=points3d.at<float>(2,i)/points3d.at<float>(3,i);
//points3d.at<float>(3,i)=1;
////        points3d(i,1)=0;        points3d(i,0)=0; points3d(i,2)=0; points3d(i,3)=0;
//    }
//    cout<<points3d<<endl;
//    cout<<endl;
//    print_distance(points3d);
//    Mat point3d=points3d.colRange(0,2);
//#endif
//    return point3d;
//}

//void toStandard(Mat &src,Mat& n){
//    for(int i=0;i<src.rows;i++){
//        n.at<double>(0,i)=src.at<Point2f>(i).x;
//        n.at<double>(1,i)=src.at<Point2f>(i).y;
//    }
//}

//int main(){

//    string left="4/Left00.png",right="4/Right00.png";
//    Mat point3d=reconstruct_stereo(left,right);
//    Mat pointbuf1,pointbuf2,pointbuf2_(2,49,CV_64F),pointbuf1_(2,49,CV_64F),K1=P1.rowRange(0,3).colRange(0,3);
//    chessboard("4/Left01.png",pointbuf1,true);chessboard("4/Right01.png",pointbuf2,true);
////    pointbuf1_=pointbuf1.t();
//    toStandard(pointbuf1,pointbuf1_);
//    toStandard(pointbuf2,pointbuf2_);
////    cout<<pointbuf1_<<endl;
////transpose has some problems

//    cout<<point3d.size()<<endl;
//    Mat point3d_(49,3,CV_64F);
//    int row=-1;
//    for(int i=0;i<point3d_.rows;i++){
//        point3d_.at<double>(i,0)=(i%7)*8;
//        point3d_.at<double>(i,1)=((i%7)?row:++row)*8;
//        point3d_.at<double>(i,2)=0;
//    }
//    cout<<point3d_<<endl;cout<<pointbuf1_.t()<<endl;
//    cout<<R1*R2.t()<<endl;
//    Mat r_m,rvec,tvec,r_m1,rvec1,tvec1;//cv::Mat()
//        solvePnPRansac(point3d_,pointbuf1_.t(),M1,D1,rvec,tvec,SOLVEPNP_EPNP);
//        solvePnPRansac(point3d_,pointbuf2_.t(),M1,D1,rvec1,tvec1,SOLVEPNP_EPNP);
////        solvePnPRansac(point3d.t(),pointbuf1_.t(),M1,D1,rvec,tvec,SOLVEPNP_EPNP);
////        solvePnPRansac(point3d.t(),pointbuf2_.t(),M1,D1,rvec1,tvec1,SOLVEPNP_EPNP);


//        //    solvePnP(point3d.t(),pointbuf1_.t(),K1,cv::Mat(),rvec,tvec,false,SOLVEPNP_EPNP);
//    Rodrigues(rvec,r_m);
//    cout<<r_m<<tvec<<cv::norm(tvec)<<endl;
//    cout<<"\n hehe "<<tvec1-tvec<<cv::norm(tvec1-tvec)<<endl;

//    waitKey(0);
//    return 0;
//}
