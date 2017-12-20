#include <iostream>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/features2d/features2d.hpp"
<<<<<<< HEAD
//#include "opencv2/nonfree/features2d.hpp"
=======
#include "opencv2/nonfree/features2d.hpp"
>>>>>>> 4611257807c3b3d30c3c75a2b645c8e7b21ccccc


#include "camera_camera_calib/objectPose.h"

using namespace std;
using namespace cv;

void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                  Settings::Pattern patternType = Settings::CHESSBOARD)
{
	corners.clear();

	switch(patternType)
	{
	case Settings::CHESSBOARD:
	case Settings::CIRCLES_GRID:
	  for( int i = 0; i < boardSize.height; ++i )
	    for( int j = 0; j < boardSize.width; ++j )
	        corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
	  break;

	case Settings::ASYMMETRIC_CIRCLES_GRID:
	  for( int i = 0; i < boardSize.height; i++ )
	     for( int j = 0; j < boardSize.width; j++ )
	        corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
	  break;
	}
}

bool findBoardCorner(Mat gray, Size patternsize, vector<Point2f>& corners, bool drawcorner=true)
{
	bool patternfound = findChessboardCorners(gray, patternsize, corners, 
                CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                + CALIB_CB_FAST_CHECK);

    if (patternfound){
        cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1),
           TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    else{
    	// cout << "Chess board corners are not found!" << endl;
    	return false;
    }

    if (drawcorner){
    	drawChessboardCorners(gray, patternsize, Mat(corners), patternfound);
    	imshow("Chessboard corners", gray);
        waitKey(1);
    }

    return true;
}

void featureMatching(const Mat& img_1, const Mat& img_2, std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2)
{
  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;


  // //SurfFeatureDetector detector( minHessian );
  // Ptr<FeatureDetector> detector = ORB::create();

  // std::vector<KeyPoint> keypoints_1, keypoints_2;

  // detector->detect( img_1, keypoints_1 );
  // detector->detect( img_2, keypoints_2 );

  // //-- Step 2: Calculate descriptors (feature vectors)
  // //SurfDescriptorExtractor extractor;
  // Ptr<DescriptorExtractor> extractor = ORB::create();

  // Mat descriptors_1, descriptors_2;

  // extractor->compute( img_1, keypoints_1, descriptors_1 );
  // extractor->compute( img_2, keypoints_2, descriptors_2 );

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_1, keypoints_2;

  detector.detect( img_1, keypoints_1 );
  detector.detect( img_2, keypoints_2 );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_1, descriptors_2;

  extractor.compute( img_1, keypoints_1, descriptors_1 );
  extractor.compute( img_2, keypoints_2, descriptors_2 );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { 
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  // printf("-- Max dist : %f \n", max_dist );
  // printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_1.rows; i++ )
  { 
    if( matches[i].distance <= max(2*min_dist, 0.02) )
    { good_matches.push_back( matches[i]); }
  }

  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Show detected matches
  imshow( "Good Matches", img_matches );

  for( int i = 0; i < (int)good_matches.size(); i++ )
  { 
    // printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
    kp1.push_back(keypoints_1[good_matches[i].queryIdx]);
    kp2.push_back(keypoints_2[good_matches[i].trainIdx]);

  }

  waitKey(1);
}
