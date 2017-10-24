/*
* starter_video.cpp
*
*  Created on: Nov 23, 2010
*      Author: Ethan Rublee
*
* A starter sample for using opencv, get a video stream and display the images
* easy as CV_PI right?
*/
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include<string>
#include <stdio.h>

using namespace cv;
using namespace std;


int main(int ac, char** av) {
    //int n=0;
    char filename[30],filename1[30],filename2[30],filename3[30];
    for(int n=0;n<24;n++)
    {
	sprintf(filename,"Left%.2d.png",n);
        sprintf(filename1,"Right%.2d.png",n);
	sprintf(filename2,"new/Left%.2d.png",n);
        sprintf(filename3,"new/Right%.2d.png",n);
        Mat a=imread(filename),b=imread(filename1);
        imwrite(filename3,a);
        imwrite(filename2,b);
    }
    return 0;
}
