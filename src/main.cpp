#include <iostream>
#include <sstream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "src/main.h"

using namespace std;
using namespace cv;

int main( int argc, char** argv )
{
    StereoVision *stereoVision;

    stereoVision= new StereoVision;

    Mat leftImage;
    Mat rightImage;
    Mat leftImage2;
    Mat rightImage2;
    bool calibrationStatus;
    int x=9;
    int y=6;
    stereoVision->setPatternSize(x,y);
    String url= "/home/wiktor/datasets/stereovision/Calibration/";

   for(int i=1; i<9;i++){
        ostringstream ss;
        ss<<i;
        string i2=ss.str();
        String left=url+"left0"+i2+".jpg";
        String right=url+"right0"+i2+".jpg";



        leftImage = imread(left,CV_LOAD_IMAGE_GRAYSCALE);
        rightImage = imread(right,CV_LOAD_IMAGE_GRAYSCALE);
        //For later changes--------------------------
        if(i==1){
            stereoVision->setImageSize(leftImage.size());
        }
        //-------------------------------------------
        stereoVision->addSampleToCalibration(leftImage,rightImage);
    }




    stereoVision->setImageSize(leftImage.size());
    calibrationStatus = stereoVision->calibrateStereoCameras();
    cout<<calibrationStatus;

    //Rectify and 3d
    if(calibrationStatus == true){
        String left=url+"left05.jpg";
        String right=url+"right05.jpg";


        leftImage = imread(left,CV_LOAD_IMAGE_GRAYSCALE);
        rightImage = imread(right,CV_LOAD_IMAGE_GRAYSCALE);

       stereoVision->rectifyStereoCameras(leftImage,rightImage,leftImage2,rightImage2);


       // undistortPoints( stereoVision->imagePoints[0][4], stereoVision->imagePoints[0][4],stereoVision->cameraMatrix[0], stereoVision->distCoeffs[0],noArray(),stereoVision->cameraMatrix[0]);
       // undistortPoints( stereoVision->imagePoints[1][4], stereoVision->imagePoints[1][4],stereoVision->cameraMatrix[1], stereoVision->distCoeffs[1],noArray(),stereoVision->cameraMatrix[1]);
       // drawChessboardCorners(leftImage2, Size(9,6), stereoVision->imagePoints[0][4], true);
       // drawChessboardCorners(rightImage2, Size(9,6), stereoVision->imagePoints[1][4], true);
         cout<<"works";
       imshow("leftAfter",leftImage2);
       imshow("rightAfter",rightImage2);
       waitKey(0);
    }









    return 0;
}
