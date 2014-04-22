#include "stereovision.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#define CUSTOM_REPROJECT

StereoVision::StereoVision()

{
    squareSize=0.05;
   // cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
   // cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 2.0 );
  viewer->initCameraParameters ();
  return (viewer);
}



bool StereoVision::addSampleToCalibration(Mat &leftImage, Mat &rightImage){

    samplesCounter++;

    vector<Point2f> corners[2];

    bool result;

    result = findChessboardCorners(leftImage,
                                   patternSize,
                                   corners[0],
                                   CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE );
    std::cout<<result<<"\n";
    if(result)
        result = findChessboardCorners(rightImage,
                                       patternSize,
                                       corners[1],
                                       CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE );

    //We accept only samples with full visibility of corners
    if(result)
        goodSamplesCounter++;
    else
        return false;

    //If we found corners then we need to do some subpix corners interpolation

    cornerSubPix(leftImage, corners[0], Size(11, 11), Size(-1, -1),
                 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


    cornerSubPix(rightImage, corners[1], Size(11, 11), Size(-1, -1),

                 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    //For Check
    //

    imagePoints[0].push_back(corners[0]);
    imagePoints[1].push_back(corners[1]);

    return true;

}

bool StereoVision::calibrateStereoCameras(){
    minSamples=5;
    std::cout<<"calibration"<<"\n";
    std::cout<<goodSamplesCounter<<"\n";
    std::cout<<minSamples<<"\n";

    if(goodSamplesCounter<minSamples){
        std::cout<<"more samples"<<"\n";
        return false;

    }

    //Assign values to objectPoints in object coordinate space
    for(int i = 0; i <goodSamplesCounter ; i++ ){
        objectPoints.push_back(vector<Point3f>());
        {
            for(int j = 0; j < patternSize.height; j++ )
                for(int k = 0; k < patternSize.width; k++ )
                    objectPoints[i].push_back(Point3f(float(j*squareSize), float(k*squareSize), 0));

        }
    }

    //For Check
  //  for(int i=0;i<objectPoints.size();i++)
    //    for(int j=0;j<objectPoints[i].size();j++)
      //      qDebug()<<objectPoints[i][j].x<<objectPoints[i][j].y<<objectPoints[i][j].z;
    //
    vector<Mat> rvecs[2], tvecs[2];

    //Calibrate first then second camera separately
    calibrateCamera(objectPoints,imagePoints[0],imageSize,cameraMatrix[0],distCoeffs[0],rvecs[0],tvecs[1]);
    calibrateCamera(objectPoints,imagePoints[1],imageSize,cameraMatrix[1],distCoeffs[1],rvecs[0],tvecs[1]);



    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                        cameraMatrix[0], distCoeffs[0],
                        cameraMatrix[1], distCoeffs[1],
                        imageSize, R, T, E, F,
                        TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5),
                        CV_CALIB_FIX_INTRINSIC);




    //qDebug()<<"OBJECT POINTS";
    //for(unsigned int i=0;i<objectPoints.size();i++)
        //for(unsigned int j=0;j<objectPoints[i].size();j++)
          //  qDebug()<<"ObjectPoints.size "<<objectPoints.size()<<" ObjectPoints["<<i<<"] size "<< objectPoints[i].size()<<" value "<<j<<" :"
            //       <<objectPoints[i][j].x<<objectPoints[i][j].y<<objectPoints[i][j].z;


    // from Calibration Matrixes perform Rectification
    stereoRectify(cameraMatrix[0],distCoeffs[0],cameraMatrix[1],distCoeffs[1],imageSize,R,T,R1,R2,P1,P2,Q,CALIB_ZERO_DISPARITY,0,imageSize,0,0);
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));// check cameras mutual position

    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);// computes undistortion and rectify maps for camera #1
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);// computes undistortion and rectify maps for camera #2



    FileStorage fs("test.xml", FileStorage::WRITE);
    fs << "cameraMatrix0"<< cameraMatrix[0] << "cameraMatrix1"<< cameraMatrix[1]<<"distCoeffs0"<<distCoeffs[1]<<"distCoeffs1"<<distCoeffs[1]<<"R"<<R<<"T"<<T<<"E"<<E<<"F"<<F;
    fs.release();

return true;
}
bool StereoVision::rectifyStereoCameras(Mat &leftImage, Mat &rightImage,Mat &leftImage2, Mat &rightImage2){

    Mat disparityMap;
    Mat disparityMap2;
    Mat mapa3d;
    Mat perspective3d;

    remap(leftImage,leftImage2,rmap[0][0],rmap[0][1],INTER_LINEAR,BORDER_TRANSPARENT,0);// applies undistortion and rectify maps to input image from camera #1
    remap(rightImage,rightImage2,rmap[1][0],rmap[1][1],INTER_LINEAR,BORDER_TRANSPARENT,0);// applies undistortion and rectify maps to input image from camera #2

    sgbm.SADWindowSize = 2;
    sgbm.numberOfDisparities = 256;
    sgbm.preFilterCap = 4;
    sgbm.minDisparity = 0;
    sgbm.uniquenessRatio = 1;
    sgbm.speckleWindowSize = 150;
    sgbm.speckleRange = 2;
    sgbm.disp12MaxDiff = 10;
    sgbm.fullDP = true;
    sgbm.P1 = 600;
    sgbm.P2 = 2400;

    sgbm(leftImage2,rightImage2,disparityMap);
    normalize(disparityMap,disparityMap2,0,255,CV_MINMAX,CV_8U);

    imshow("Disparity",disparityMap2);


    //If size of Q is not 4x4 exit
    if (Q.cols != 4 || Q.rows != 4)
    {
      std::cerr << "ERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
      return 1;
    }

  #ifdef CUSTOM_REPROJECT
    //Get the interesting parameters from Q
    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0,3);
    Q13 = Q.at<double>(1,3);
    Q23 = Q.at<double>(2,3);
    Q32 = Q.at<double>(3,2);
    Q33 = Q.at<double>(3,3);

    std::cout << "Q(0,3) = "<< Q03 <<"; Q(1,3) = "<< Q13 <<"; Q(2,3) = "<< Q23 <<"; Q(3,2) = "<< Q32 <<"; Q(3,3) = "<< Q33 <<";" << std::endl;

  #endif


   // std::cout << "Read matrix in file " << argv[3] << std::endl;

    //Show the values inside Q (for debug purposes)
    /*
    for (int y = 0; y < Q.rows; y++)
    {
      const double* Qy = Q.ptr<double>(y);
      for (int x = 0; x < Q.cols; x++)
      {
        std::cout << "Q(" << x << "," << y << ") = " << Qy[x] << std::endl;
      }
    }
    */

    //Load rgb-image
    Mat img_rgb = leftImage;
    if (img_rgb.data == NULL)
    {
      std::cerr << "ERROR: Could not read rgb-image: " << "rgb-image.ppm" << std::endl;
      return 1;
    }

    //Load disparity image
    Mat img_disparity = disparityMap2;
    if (img_disparity.data == NULL)
    {
      std::cerr << "ERROR: Could not read disparity-image: " << "disparity-image.pgm" << std::endl;
      return 1;
    }

    //Both images must be same size
    if (img_rgb.size() != img_disparity.size())
    {
      std::cerr << "ERROR: rgb-image and disparity-image have different sizes " << std::endl;
      return 1;
    }

    //Show both images (for debug purposes)
    cv::namedWindow("rgb-image");
    cv::namedWindow("disparity-image");
    cv::imshow("rbg-image", img_rgb);
    cv::imshow("disparity-image", img_disparity);
    std::cout << "Press a key to continue..." << std::endl;
    cv::waitKey(0);
    cv::destroyWindow("rgb-image");
    cv::destroyWindow("disparity-image");

  #ifndef CUSTOM_REPROJECT
    //Create matrix that will contain 3D corrdinates of each pixel
    cv::Mat recons3D(img_disparity.size(), CV_32FC3);

    //Reproject image to 3D
    std::cout << "Reprojecting image to 3D..." << std::endl;
    cv::reprojectImageTo3D( img_disparity, recons3D, Q, false, CV_32F );
  #endif
    //Create point cloud and fill it
    std::cout << "Creating Point Cloud..." <<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    double px, py, pz;
    uchar pr, pg, pb;

    for (int i = 0; i < img_rgb.rows; i++)
    {
      uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
  #ifdef CUSTOM_REPROJECT
      uchar* disp_ptr = img_disparity.ptr<uchar>(i);
  #else
      double* recons_ptr = recons3D.ptr<double>(i);
  #endif
      for (int j = 0; j < img_rgb.cols; j++)
      {
        //Get 3D coordinates
  #ifdef CUSTOM_REPROJECT
        uchar d = disp_ptr[j];
        if ( d == 0 ) continue; //Discard bad pixels
        double pw = -1.0 * static_cast<double>(d) * Q32 + Q33;
        px = static_cast<double>(j) + Q03;
        py = static_cast<double>(i) + Q13;
        pz = Q23;

        px = px/pw;
        py = py/pw;
        pz = pz/pw;

  #else
        px = recons_ptr[3*j];
        py = recons_ptr[3*j+1];
        pz = recons_ptr[3*j+2];
  #endif

        //Get RGB info
        pb = rgb_ptr[3*j];
        pg = rgb_ptr[3*j+1];
        pr = rgb_ptr[3*j+2];

        //Insert info into point cloud structure
        pcl::PointXYZRGB point;
        point.x = px;
        point.y = py;
        point.z = pz;
        uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
        point.rgb = *reinterpret_cast<float*>(&rgb);
        point_cloud_ptr->points.push_back (point);
      }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    //Create visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = createVisualizer( point_cloud_ptr );

    //Main loop
    while ( !viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }




}

double StereoVision::getCameraValueAt(int x,int y, char camera) const {
    //Doesnt check if calibration was done or not

    if(x>3||x<0||y>3||y<0)
        return -1;\
    if(camera == 'L')
        return cameraMatrix[0].at<double>(x,y);
    if(camera == 'R')
        return cameraMatrix[1].at<double>(x,y);

    return -1;
}
