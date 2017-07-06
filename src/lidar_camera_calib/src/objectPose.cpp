#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "lidar_camera_calib/objectPose.h"

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

void findBoardCorner(Mat gray, Size patternsize, vector<Point2f>& corners, bool drawcorner=true)
{
	bool patternfound = findChessboardCorners(gray, patternsize, corners, 
                CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                + CALIB_CB_FAST_CHECK);

    if (patternfound){
        cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
           TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    else{
    	cout << "Chess board corners are not found!" << endl;
    }

    if (drawcorner){
    	drawChessboardCorners(gray, patternsize, Mat(corners), patternfound);
    	imshow("Chessboard corners", gray);
        waitKey(0);
    }
}
